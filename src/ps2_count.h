// SPDX-License-Identifier: MIT
// Software extension of 32-bit CP0 Count, only safe for a single EE thread.
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct b3Ps2CountState
{
    uint32_t lastCount;
    uint64_t elapsedTicks;
    bool initialized;
} b3Ps2CountState;

static inline uint64_t b3Ps2ExtendCount( b3Ps2CountState* state, uint32_t now )
{
    if ( !state->initialized )
    {
        state->lastCount = now;
        state->initialized = true;
        return 0;
    }

    // Handles one wrap. Full revolutions missed between reads cannot be detected.
    state->elapsedTicks += (uint32_t)( now - state->lastCount );
    state->lastCount = now;
    return state->elapsedTicks;
}
