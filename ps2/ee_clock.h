// SPDX-License-Identifier: MIT
// EE Core Manual CPR[0,09]: increments once per EE clock.
// Read-only Count: do not steal PS2SDK's Compare interrupt or reset Count.
// Single sample interval must remain shorter than 2^31 cycles (~7.28s).
#pragma once
#include <stdint.h>

static inline uint32_t b3Ps2ReadCpuCycles( void )
{
    uint32_t count;
    __asm__ __volatile__(
        ".set push\n\t"
        ".set noreorder\n\t"
        "mfc0 %0, $9\n\t"
        "nop\n\t"
        ".set pop"
        : "=r"( count ) : : "memory" );
    return count;
}

static inline uint32_t b3Ps2DeltaCpuCycles( uint32_t start, uint32_t end )
{
    return (uint32_t)( end - start );
}
