#include "hbridge_encode.h"

#include <stdlib.h>

#define PAT_A_FWD   0x1u
#define PAT_A_REV   0x2u
#define PAT_A_BRAKE 0x3u
#define PAT_B_FWD   0x4u
#define PAT_B_REV   0x8u
#define PAT_B_BRAKE 0xcu

static uint32_t segment(uint32_t pattern, int32_t len) {
    return pattern | ((uint32_t)len << 4);
}

void hbridge_encode_period(int32_t span, int32_t duty_a, int32_t duty_b, uint32_t *w0, uint32_t *w1) {
    int32_t ma = abs(duty_a), mb = abs(duty_b);
    if (ma > span) ma = span;
    if (mb > span) mb = span;

    uint32_t drive_a = ma == 0 ? PAT_A_BRAKE : duty_a > 0 ? PAT_A_FWD : PAT_A_REV;
    uint32_t drive_b = mb == 0 ? PAT_B_BRAKE : duty_b > 0 ? PAT_B_FWD : PAT_B_REV;

    // Segment 1: both coils driven for the shorter duty.
    // Segment 2: only the longer-duty coil driven.
    // Segments 3-4: both braked for the rest (split so lengths fit 12 bits).
    int32_t lo = ma < mb ? ma : mb;
    int32_t hi = ma < mb ? mb : ma;
    uint32_t seg2_pat = ma >= mb ? (drive_a | PAT_B_BRAKE) : (PAT_A_BRAKE | drive_b);
    int32_t rest = span - hi;

    *w0 = segment(drive_a | drive_b, lo) | segment(seg2_pat, hi - lo) << 16;
    *w1 = segment(PAT_A_BRAKE | PAT_B_BRAKE, rest / 2) |
          segment(PAT_A_BRAKE | PAT_B_BRAKE, rest - rest / 2) << 16;
}
