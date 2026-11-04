#ifndef INTERVAL_DECODER_H
#define INTERVAL_DECODER_H

#include <stdint.h>

// Interval-code frame decoder. No Arduino or mbed dependencies, so the same code can be compiled on a PC and
// checked against the offline decoder (tools/e4_analyze.py, selfcal_pair).
//
// A frame is two marks of markCycles carrier cycles; the ID is in the spacing of their leading edges,
// 2 * mark + step * ID. The receiver stretches each mark and delays the second one by an amount that depends on
// signal strength (+37 to +88 us between 30° and 0° in E2). The first mark's stretch predicts that delay:
// delay = 0.66 x stretch + 20 us (E2, 2026-10-07, all three levels, correlation 0.76). So each frame corrects
// itself, and no stored offset is needed.
namespace IntervalDecoder {

const int CARRIER_PERIOD_US = 28;
const int64_t SLOPE_X100 = 66;  // 0.66, kept in integers
const int64_t OFFSET_US = 20;
const int MAX_IDS = 16;         // irLookedBitmask is 16 bits

// ID of the frame formed by two consecutive marks (start times from micros(), widths in us), or -1 if they don't
// form a frame: a mark outside 12 cycles -3/+6 (a collision or noise), or a spacing outside IDs 0 .. MAX_IDS-1.
inline int decodePair(uint32_t t1, uint32_t w1, uint32_t t2, uint32_t w2, int markCycles, int stepUs) {
  int64_t mark = (int64_t)markCycles * CARRIER_PERIOD_US;
  int64_t lo = mark - 3 * CARRIER_PERIOD_US, hi = mark + 6 * CARRIER_PERIOD_US;
  if ((int64_t)w1 < lo || (int64_t)w1 > hi || (int64_t)w2 < lo || (int64_t)w2 > hi) return -1;
  int64_t spacing = (int64_t)(uint32_t)(t2 - t1); // unsigned difference: correct across the micros() rollover
  // Spacing beyond ID 0, minus the delay predicted from the first mark's stretch; all x100
  int64_t fromId0 = 100 * (spacing - 2 * mark) - (SLOPE_X100 * ((int64_t)w1 - mark) + 100 * OFFSET_US);
  int64_t d = 100 * (int64_t)stepUs;
  int64_t id = fromId0 >= 0 ? (fromId0 + d / 2) / d : -((-fromId0 + d / 2) / d); // nearest ID, halves away from 0
  return (id >= 0 && id < MAX_IDS) ? (int)id : -1;
}

} // namespace IntervalDecoder

#endif
