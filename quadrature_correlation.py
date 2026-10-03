"""Quadrature reference correlation -- head-nod detector for AURA.

One-to-one port of algorithm3_quadratureCorrelation from the SocialMonitorNode
firmware (src/main.cpp). Every constant, index quirk and gate threshold is kept
identical so this flags the same instants the nRF52840 node does.

How it works, per sample (w0 = gyro, a0 = accel):
  1. Compress the gyro sample:  xt = (1/5) * cbrt(w0).
  2. Advance three phase accumulators by 2*pi*f/fs rad, fs = 100 Hz, wrapping
     once past 2*pi. The tones are f = 2.29, 2.90, 3.50 Hz (the /100 bakes in
     the firmware's 100 Hz gyro rate -- kept literal for an exact match).
  3. Build a synthetic reference from those tones and store it in a ring e[].
  4. Store the compressed gyro sample xt in a ring z[].
  5. Once per full window (every 33 calls) compare the two rings:
        sum = a0 * sum(|z[j] - e[j]|)  over the window
     and flag a nod when 5 < sum < 7.5 and 0 < a0 < 0.5.

The firmware reported detections through notifyHeadNod(); here that becomes the
return value of push, as AURA requires. The detector never modifies the data.

Reads two channels at once (section 3 of the spec): "motion" from the gyro,
"tilt" from the accelerometer. The session profile picks the exact channel for
each; push is handed {"motion": ..., "tilt": ...}.

Self-contained: standard library only.
"""

import math

NAME = "Quadrature correlation"

DESCRIPTION = "Flags head nods by correlating gyro against a three-tone quadrature reference, gated by accel tilt."

# Two named inputs -> push receives {"motion": <gyro>, "tilt": <accel>}.
INPUTS = {
    "motion": ("gyro",),   # w0 in the firmware -- the signal being correlated
    "tilt": ("accel",),    # a0 in the firmware -- the gate and window scaler
}

# The four numbers in the firmware's fire condition, exposed so they can be
# tuned per session. Everything else (the tone frequencies, the window length,
# the 1/5 compression) defines the detector itself and is kept as a constant.
PARAMS = {
    "sum_low": {
        "default": 5,
        "min": 0.0,
        "max": 1000.0,
        "help": "Lower bound the correlation distance must exceed to flag a nod.",
    },
    "sum_high": {
        "default": 11,
        "min": 0.0,
        "max": 1000.0,
        "help": "Upper bound the correlation distance must stay under to flag a nod.",
    },
    "accel_low": {
        "default": 0,
        "min": -100.0,
        "max": 100.0,
        "help": "Tilt (accel) must be strictly above this to flag a nod.",
    },
    "accel_high": {
        "default": -1,
        "min": -100.0,
        "max": 100.0,
        "help": "Tilt (accel) must be strictly below this to flag a nod.",
    },
}

# --- Detector constants, ported verbatim from the firmware -----------------
WIN_LEN = 32                                  # #define win_len 32
PHASE_INC = (14.4 / 50.0, 18.22 / 50.0, 21.99 / 50.0)  # 2*pi*f / fs, fs=100
TWO_PI = 2.0 * math.pi
XT_SCALE = 1.0 / 5.0                          # gyro compression gain
# 0.5 * cos(n + pi/4) with the firmware's global n, which is never incremented
# (stays 0), so this is a constant amplitude on the reference.
REF_AMP = 0.5 * math.cos(0 + math.pi / 4.0)


def _cbrt(v):
    # Real cube root, sign-preserving -- matches C's cbrt() for negatives.
    return math.copysign(abs(v) ** (1.0 / 3.0), v)


def init(params, info):
    return {
        "sum_low": float(params["sum_low"]),
        "sum_high": float(params["sum_high"]),
        "accel_low": float(params["accel_low"]),
        "accel_high": float(params["accel_high"]),
        # Rings sized WIN_LEN + 2 so the firmware's post-increment writes
        # (e[32], z[33]) land in real slots instead of running off the end.
        # The window sum only ever reads indices 0..WIN_LEN-1, and z[0] is
        # never written -- it stays 0.0, exactly as in the firmware.
        "z": [0.0] * (WIN_LEN + 2),
        "e": [0.0] * (WIN_LEN + 2),
        "i": 0,
        "phase1": 0.0,
        "phase2": 0.0,
        "phase3": 0.0,
    }


def push(state, values, ts):
    w0 = values["motion"]   # gyro
    a0 = values["tilt"]     # accel

    a0 *= -1
    xt = XT_SCALE * _cbrt(w0)

    # Advance the three phase accumulators, wrapping once past 2*pi.
    p1 = state["phase1"] + PHASE_INC[0]
    if p1 > TWO_PI:
        p1 -= TWO_PI
    p2 = state["phase2"] + PHASE_INC[1]
    if p2 > TWO_PI:
        p2 -= TWO_PI
    p3 = state["phase3"] + PHASE_INC[2]
    if p3 > TWO_PI:
        p3 -= TWO_PI
    state["phase1"], state["phase2"], state["phase3"] = p1, p2, p3

    ref = math.sin(p1)
    reff_shifted = math.cos(p2)
    inv_ref = -math.sin(p3)

    # e[i++] = ...; z[i] = xt  -- the post-increment leaves z[0] untouched.
    i = state["i"]
    state["e"][i] = REF_AMP * (ref + reff_shifted * inv_ref)
    i += 1
    state["i"] = i
    state["z"][i] = xt

    result = None
    if i > WIN_LEN:
        total = 0.0
        z, e = state["z"], state["e"]
        for j in range(WIN_LEN):
            total += abs(z[j] - e[j])
        total *= a0 * -1

        if (total > state["sum_low"] and total < state["sum_high"]
                and a0 > state["accel_high"] and a0 < state["accel_low"]):
            result = "Head nod"

        state["i"] = 0

    return result
