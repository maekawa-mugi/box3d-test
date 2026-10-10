// SPDX-License-Identifier: MIT
// First VU0 macro-mode experiment: four independent vec3 dot products.
// VU0 is a COP2 floating point unit, not integer MMI or VU1 micro mode.
// No multithreaded EE use, interrupt-time COP2, or simultaneous VU0 microcode.
#pragma once

// The 112 byte scratch is 16-byte aligned. Every LQC2/SQC2 address is
// aligned, including when a b3Vec3W4 passed to the wrapper is unaligned.
typedef struct b3Ps2VU0Dot4Data
{
    _Alignas(16) float ax[4];
    float bx[4];
    float ay[4];
    float by[4];
    float az[4];
    float bz[4];
    float out[4];
} b3Ps2VU0Dot4Data;

// Runtime switch applies to b3DotW4 used by the width-4 contact solver.
// Disabled by default. Do not toggle while b3World_Step is in progress.
void b3Ps2SetVU0Enabled( int enabled );
int b3Ps2GetVU0Enabled( void );

// Always executes VU0 on PS2, scalar fallback on other targets. Used by
// both the physics path and the differential correctness microbenchmark.
void b3Ps2VU0Dot4( b3Ps2VU0Dot4Data* data );

// Four independent DotW4 operations in one call. Input is an array of
// four 16-byte-aligned records. Batching is required to amortize transfers.
// This kernel alone does not alter the current per-contact solver layout.
void b3Ps2VU0Dot4Batch4( b3Ps2VU0Dot4Data data[4] );
