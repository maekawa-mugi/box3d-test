// SPDX-License-Identifier: MIT
// Optional 128-bit integer MMI primitives for R5900 / PS2SDK.
// These are NOT VU floating-point SIMD. Runtime switch for A/B benchmarks.
#pragma once

#include <stddef.h>
#include <stdint.h>

// Single EE worker only; change mode only between simulations, not in a step.
void b3Ps2SetMMIEnabled( int enabled );
int b3Ps2GetMMIEnabled( void );

// Zero arbitrary bytes; OR complete 64-bit words (size must be multiple of 8).
// Aligned 16-byte blocks use MMI on PS2; all other bytes use portable fallback.
void b3Ps2ClearBytes( void* dst, size_t byteCount );
void b3Ps2OrWords( uint64_t* dst, const uint64_t* src, size_t wordCount );
