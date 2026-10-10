// SPDX-License-Identifier: MIT
// ABBA measurements compare trial 0+3 against trial 1+2.
// Use trial POSITION, never boolean truthiness of the runtime mode:
// e.g. an ABBA comparison of mode 2 vs mode 3 has two nonzero modes.
#pragma once
#include <stdint.h>

typedef struct b3Ps2ABBAStats
{
    uint64_t baselineTicks;
    uint64_t targetTicks;
    double speedup;
} b3Ps2ABBAStats;

static inline b3Ps2ABBAStats b3Ps2SummarizeABBA( const uint64_t ticks[4] )
{
    b3Ps2ABBAStats s;
    s.baselineTicks = ticks[0] + ticks[3];
    s.targetTicks = ticks[1] + ticks[2];
    s.speedup = s.targetTicks != 0
        ? (double)s.baselineTicks / (double)s.targetTicks : 0.0;
    return s;
}
