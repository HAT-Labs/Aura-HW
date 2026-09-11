#include "nrf52840_bitfields.h"
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include <sys/_types.h>
#define HEADNOD_SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define HEADNOD_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#include <math.h>

static BLEService headNodService(HEADNOD_SERVICE_UUID);
static BLECharacteristic headNodChar(HEADNOD_CHARACTERISTIC_UUID, BLERead | BLENotify, 5);

static void notifyHeadNod(int algoId, unsigned long ts) {
    Serial.print("DETECTION algo");
    Serial.print(algoId);
    Serial.print(" t=");
    Serial.println(ts);

    uint8_t payload[5];
    payload[0] = (uint8_t)algoId;
    payload[1] = (uint8_t)(ts & 0xFF);
    payload[2] = (uint8_t)((ts >> 8) & 0xFF);
    payload[3] = (uint8_t)((ts >> 16) & 0xFF);
    payload[4] = (uint8_t)((ts >> 24) & 0xFF);
    headNodChar.writeValue(payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// ALGORITHM 1 -- commented out (matches algorithms.txt's "#DO NOT REMOVE"
// block): fixed 100-sample trailing window (window = primary[i-100 .. i],
// i.e. up to 101 samples once the stream is warmed up) gated on variance +
// peak-to-peak. rolling_rms(window, K_WIN) is computed and discarded, same
// as in the original -- kept here for fidelity even though it doesn't
// affect any detection.
// ---------------------------------------------------------------------------
/*
#define K1_WIN 100

static double k1_buf[K1_WIN + 1];
static int k1_count = 0;
static int k1_idx = 0;
static bool k1_armed = true;
static unsigned long k1_t_prev = 0;  // mirrors Python's `t1 = 0.0` before the loop

static double k1_variance(const double *v, int n) {
    if (n < 2) return 0.0;
    double m = 0.0;
    for (int i = 0; i < n; i++) m += v[i];
    m /= n;
    double s = 0.0;
    for (int i = 0; i < n; i++) s += (v[i] - m) * (v[i] - m);
    return s / (n - 1);
}

static double k1_peak_to_peak(const double *v, int n) {
    if (n == 0) return 0.0;
    double mn = v[0], mx = v[0];
    for (int i = 1; i < n; i++) {
        if (v[i] < mn) mn = v[i];
        if (v[i] > mx) mx = v[i];
    }
    return mx - mn;
}

// mirrors Ops.rolling_rms(window, K_WIN): RMS of each sample's own trailing
// K_WIN-sample sub-window. Result is discarded by the caller, same as the
// original -- see the header note above.
static void k1_rolling_rms(const double *v, int n, int window, double *out) {
    for (int i = 0; i < n; i++) {
        int start = i - window + 1;
        if (start < 0) start = 0;
        double s = 0.0;
        for (int j = start; j <= i; j++) s += v[j] * v[j];
        out[i] = sqrt(s / (i - start + 1));
    }
}

bool detect(double primary, double confirm, unsigned long ts) {
    (void)confirm;  // threaded through for parity; not gated on (see file header)

    k1_buf[k1_idx] = primary;
    k1_idx = (k1_idx + 1) % (K1_WIN + 1);
    if (k1_count < K1_WIN + 1) k1_count++;

    double window_var = k1_variance(k1_buf, k1_count);
    double ptp = k1_peak_to_peak(k1_buf, k1_count);
    static double k1_rms_out[K1_WIN + 1];
    k1_rolling_rms(k1_buf, k1_count, K1_WIN, k1_rms_out);  // unused, mirrors original

    bool gated = (window_var > K1_WIN * 1.5) && (ptp > K1_WIN * 0.45);

    // Throttled diagnostic print (every 200ms) -- thresholds are ported
    // verbatim from algorithms.txt, so use this to see how close live
    // gyro motion is getting to the var>150 && ptp>45 gate.
    static unsigned long k1_last_print = 0;
    if (ts - k1_last_print >= 200) {
        Serial.print("algo1 diag gz=");
        Serial.print(primary);
        Serial.print(" var=");
        Serial.print(window_var);
        Serial.print(" ptp=");
        Serial.print(ptp);
        Serial.print(" gated=");
        Serial.print(gated);
        Serial.print(" armed=");
        Serial.println(k1_armed);
        k1_last_print = ts;
    }

    bool fired = false;
    unsigned long fire_ts = k1_t_prev;  // Python appends t1 (the PREVIOUS ts), not ts[i]
    if (gated && k1_armed) {
        fired = true;
        k1_armed = false;
    } else if (!gated) {
        k1_armed = true;
    }
    k1_t_prev = ts;  // t1 = ts[i], updated unconditionally at the end of the iteration

    if (fired) {
        notifyHeadNod(1, fire_ts);
    }
    return fired;
}


/*
// ---------------------------------------------------------------------------
// ALGORITHM 2 -- ACTIVE: batch 25 samples, run one STFT frame over the
// batch (Hann-windowed, zero-padded to frame_size=50, real DFT -- Ops.stft's
// defaults, since algorithms.txt's call site only overrides window_size),
// gate on summed magnitude in the 2.3-3.5 Hz band. With fs=50 and
// frame_size=50, bins land on exact integer Hz (0, 1, 2, ... 25), so only
// bin k=3 (3 Hz) actually falls inside [2.3, 3.5] -- the "band sum" is
// really just that one bin's magnitude.
// ---------------------------------------------------------------------------
#define K2_WIN 25
#define FRAME_SIZE 50
#define FS2 50.0
#define BAND_LOW 2.3
#define BAND_HIGH 3.5
#define BAND_THRESHOLD 500.00

static double k2_buf[K2_WIN];
static int k2_wlen = 0;
static bool k2_armed = true;
static double k2_hann[K2_WIN];
static bool k2_hann_init = false;

static void k2_init_hann() {
    for (int i = 0; i < K2_WIN; i++)
        k2_hann[i] = 0.5 - 0.5 * cos(2.0 * PI * i / (K2_WIN - 1));
    k2_hann_init = true;
}

// naive real-DFT magnitude, bins 0..N/2 -- stands in for scipy's rfft;
// N (FRAME_SIZE) is a small fixed constant so this is cheap (~1.3k mult-adds).
static void k2_dft_magnitude(const double *x, int N, double *mag) {
    int half = N / 2;
    for (int k = 0; k <= half; k++) {
        double re = 0.0, im = 0.0;
        for (int t = 0; t < N; t++) {
            double ang = -2.0 * PI * k * t / N;
            re += x[t] * cos(ang);
            im += x[t] * sin(ang);
        }
        mag[k] = sqrt(re * re + im * im);
    }
}

bool detect(double primary, double confirm, unsigned long ts) {
    (void)confirm;  // threaded through for parity; not gated on (see file header)

    if (!k2_hann_init) k2_init_hann();

    k2_buf[k2_wlen++] = primary;
    bool fired = false;

    if (k2_wlen == K2_WIN) {
        double padded[FRAME_SIZE];
        for (int k = 0; k < K2_WIN; k++) padded[k] = k2_buf[k] * k2_hann[k];
        for (int k = K2_WIN; k < FRAME_SIZE; k++) padded[k] = 0.0;

        double mag[FRAME_SIZE / 2 + 1];
        k2_dft_magnitude(padded, FRAME_SIZE, mag);

        double band_total = 0.0;
        for (int k = 0; k <= FRAME_SIZE / 2; k++) {
            double freq = (double)k * FS2 / FRAME_SIZE;
            if (freq >= BAND_LOW && freq <= BAND_HIGH) band_total += mag[k];
        }
        k2_wlen = 0;

        if (band_total > BAND_THRESHOLD && k2_armed) {
            fired = true;
            k2_armed = false;
        }
    } else {
        // mirrors the original: armed is force-reset True on every sample
        // that isn't the K_WIN-th, so the gate never actually holds
        // between checks -- every completed-window check starts armed.
        k2_armed = true;
    }

    if (fired) {
        notifyHeadNod(2, ts);
    }
    return fired;
}
*/
 
// ---------------------------------------------------------------------------
// ALGORITHM 3 -- commented out: UP/DOWN/ZERO/IDLE state machine. `window` in
// the source is always exactly the contiguous run of samples appended since
// the last release (append is unconditional every sample; pop(0) only
// removes from the front during a release in ZERO state), so it's tracked
// here as primary[left..i] via a ring buffer instead of a real list.
//
// K3_CAP bounds how far `left` can lag `i` -- unbounded on the desktop
// (worst case O(n^2), per the original's own complexity note), but real
// hardware has finite RAM. K3_CAP=4096 is far larger than any realistic
// UP/DOWN/ZERO cycle on live gyro data; if it's ever exceeded the oldest
// sample is dropped rather than overflowing memory (only reachable if the
// signal spends an unrealistically long stretch outside ZERO state).
// ---------------------------------------------------------------------------
#define PARAM 200
#define CAP 4096
#define REFRACTORY_MS 280
static int n = 0;

// Phase increment per sample (rad/sample) = 2*pi*f / fs for each tone,
// fs = 100 Hz (matches IMU.gyroscopeSampleRate() / LPF_Init/HPF_Init above).
// f = 2.3, 2.9, 3.5 Hz -> 2*pi*f = 14.4, 18.22, 21.99.

#define win_len 32
struct store{
  double ref;
  double reff_shifted;
  double inv_ref;
  double xt;
  double z[win_len];
  double e[win_len];
  int i = 0;
  int k = 0;
  double phase_inc[3] = {14.4/100, 18.22/100, 21.99/100};
  double phase1 = 0.0, phase2 = 0.0, phase3 = 0.0;
};

struct store master;
struct store *data = &master;

#define LOG_EPS 1e-6f
static inline float safe_log(float x) {
  return log(x > LOG_EPS ? x : LOG_EPS);
}


bool detect(double w0, double a0, unsigned long ts) {
  
  //data->xt = safe_log(data->xt);
  data->xt = (1.0 / 5.0)*cbrt(w0);

  // Serial.print(data -> xt);
  // Serial.print(" ");
  // Serial.println(a0);
  
  data->phase1 += data->phase_inc[0];
  if (data->phase1 > 2.0*PI) data->phase1 -= 2.0*PI;
  data->phase2 += data->phase_inc[1];
  if (data->phase2 > 2.0*PI) data->phase2 -= 2.0*PI;
  data->phase3 += data->phase_inc[2];
  if (data->phase3 > 2.0*PI) data->phase3 -= 2.0*PI;
  
  data->ref = sin(data->phase1);
  data->reff_shifted = cos(data->phase2);
  data->inv_ref = -sin(data->phase3);

  data ->e[data->i++] = (0.5*cos(n + (PI / 4)))*(data->ref + data->reff_shifted * data->inv_ref);
  data->z[data->i] = data->xt;
  
  double sum = 0.0;
  if (data -> i > win_len) {
    for (int i = 0; i < win_len; i++) {
      sum += fabs(data->z[i] - data->e[i]);
    }
    sum = sum * a0;
    if (sum > 12.50 && sum < 14.50 && (a0 < 1 && a0 > 0)) notifyHeadNod(2, ts);
    //Serial.println(sum);
    data ->i = 0;
  }
  
  
  // double dot_product = 0.0;
  // for (int i = 0; i < 50; ++i) {
  //   dot_product += data->z[i] * data->e[i];
  // }
  bool nodded = false;
  // if (dot_product < 0.85 && dot_product > 0.65) {
  //   notifyHeadNod(2, ts); 
  //   nodded = true; 
  // }
  // data->i++;
  return nodded;
  // if (dot_product > 0 && dot_product < 0.7) {
  //   notifyHeadNod(2, ts);
  //   return true;
  // }

  // if (data->i >= 50) {
  //   double best_corr = -1e9;
  //   for (int shift = 0; shift < 50; shift += 8) { 
  //     double corr = 0.0;
  //     for (int i = 0; i < 50; ++i) {
  //       corr += data->z[i] * data->e[(i + shift) % 50];
  //     }
  //     if (corr > best_corr) best_corr = corr;
  //   }
  //   Serial.println(best_corr);
  //   if (best_corr < 4 && best_corr > 2.30) {  
  //     notifyHeadNod(2, ts);
  //   }
  //   data->i = 0;
  // }
  
}


/*
typedef enum { UP = 0, DOWN = 1, ZERO = 2} state_t;

static long left = 0;   // logical index of the window's start ("left")
static long right = 0;  // logical index one past the most recent sample
static bool armed = false;  // mirrors "armed = False" set right before the loop
static state_t state = UP;
static int m = 0;
static double buf[CAP];
static unsigned long down_entry_ts = 0;
static int N = 0;
static unsigned long start_ms = 0;
static double accumulate = 0;
inline void detect(double main, double side, unsigned long ts) {
  // threaded through for parity; not gated on (see file header)
  double gz = main;
  double ax = side;

  if (right - left >= CAP) left++;  // embedded-only safety valve, see header
  buf[right % CAP] = gz;
  right++;

  double threshold = PARAM * gz;
  
  switch (state) {
    case UP: 

      if (gz > 250 && ax >= -0.15 && ax < 0.25) {
        down_entry_ts = ts;
        state = DOWN;
      }

      else {
        N = 0;
      }
    
    break;
    case DOWN:
      if (gz < 0.0) {
        state = ZERO;
      } else if (ts - down_entry_ts >= REFRACTORY_MS) {
        state = UP;
        N = 0;
      }
    break;
    case ZERO:
      if (m < PARAM) {
        double ers = 0.0;
        for (long p = left; p < right; p++) ers += buf[p % CAP];
        if (ers >= threshold && armed) {
          armed = false;
          Serial.println("HEAD NOD");
          Serial.println(ts);
        // notifyHeadNod(2, ts);
        }
        else {
          state = UP;
          m = 0;
          armed = true;
        }
        ++m;
      }
    break;

  }
}


// Claude Alg A
#define WIN_LAG   4      // samples, lag-1..lag-4 autocovariance
#define VALID_LEN 32
#define THETA_PRE 300    // tune from noise floor
#define THETA_VAL 1800

typedef enum { IDLE, PRELIM, VALID } nod_state_t;

typedef struct {
    int16_t buf[VALID_LEN];
    uint8_t idx;
    int32_t mean_acc;
    nod_state_t state;
    uint8_t valid_cnt;
    int32_t cov_acc;
} abacv_t;

abacv_t abc;
abacv_t *s = &abc;

static void abacv_push(int16_t x, unsigned long ts) {
    s->buf[s->idx] = x;
    s->idx = (s->idx + 1) % VALID_LEN;

    int32_t cov = 0;
    for (uint8_t k = 1; k <= WIN_LAG; k++) {
        uint8_t j = (s->idx + VALID_LEN - k) % VALID_LEN;
        cov += (int32_t)s->buf[s->idx == 0 ? VALID_LEN-1 : s->idx-1] * s->buf[j];
    }

    switch (s->state) {
    case IDLE:
        if (cov > THETA_PRE) { s->state = PRELIM; s->valid_cnt = 0; s->cov_acc = 0; }
        break;
    case PRELIM:
        s->cov_acc += cov;
        if (++s->valid_cnt >= VALID_LEN) {
            s->state = (s->cov_acc > THETA_VAL) ? VALID : IDLE;
        }
        break;
    case VALID:
        // NOD_DETECTED — latch/output, then reset
        //notifyHeadNod(1, ts);
        Serial.println("Headnod");
        s->state = IDLE;
        break;
    }
}
*/


static unsigned long led_off_at = 0;

void setup() {
  Serial.begin(9600);
  unsigned long wait_start = millis();
  while (!Serial && millis() - wait_start < 2000) { }  // don't hang if untethered

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1) { }
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1) { }
  }
  BLE.setLocalName("HeadNod");
  BLE.setAdvertisedService(headNodService);
  headNodService.addCharacteristic(headNodChar);
  BLE.addService(headNodService);
  uint8_t initial[5] = {0, 0, 0, 0, 0};
  headNodChar.writeValue(initial, sizeof(initial));
  BLE.advertise();
  Serial.println("BLE advertising as \"HeadNod\"");
}



void loop() {
  BLE.poll();

  unsigned long benchStart = micros();
  unsigned long benchEnd = micros();
  unsigned long overhead = benchEnd - benchStart;
  
  unsigned long startTime = micros();
  unsigned long ts = millis();
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);
  float ax = 0, ay = 0, az = 0;
  IMU.readAcceleration(ax, ay, az);
  detect(gz, ax, ts);
  //abacv_push(gz, ts);
  unsigned long endTime = micros();
  unsigned long executionTime = (endTime - startTime) - overhead;
  static int i = 0;
  if (i++ < 2) {
    Serial.println(executionTime);
    delay(1000);
    i = 0;
  }
    

   
    
  

}
