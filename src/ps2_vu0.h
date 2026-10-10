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

// 0: original scalar COP1; 1: legacy single-dot VU0 path;
// 2: original paired VU0 dot kernels (single dots stay scalar);
// 3: same paired mode plus preparation-stage paired dots.
// This isolates additional scheduling/data-transfer gains from mode 2.
// Mode 0 remains default. Change only between completed world steps.
void b3Ps2SetVU0Enabled( int enabled );
int b3Ps2GetVU0Enabled( void );

// Always executes VU0 on PS2, scalar fallback on other targets. Used by
// both the physics path and the differential correctness microbenchmark.
void b3Ps2VU0Dot4( b3Ps2VU0Dot4Data* data );

// Four independent DotW4 operations in one call. Input is an array of
// four 16-byte-aligned records. Batching is required to amortize transfers.
// This kernel alone does not alter the current per-contact solver layout.
void b3Ps2VU0Dot4Batch4( b3Ps2VU0Dot4Data data[4] );

// Two independent dot4 jobs fused in one macro-mode invocation. The
// input layout is a pair of 16-byte-aligned 112-byte scratch records.
typedef struct b3Ps2VU0Dot4PairData
{
    b3Ps2VU0Dot4Data pair[2];
} b3Ps2VU0Dot4PairData;

// Interleaves independent VU0 multiplies and adds to cover pipeline stalls.
void b3Ps2VU0Dot4Pair( b3Ps2VU0Dot4PairData* data );

// Instrumentation counters to ensure the physics A/B exercises paired VU0.
void b3Ps2ResetVU0PairCount( void );
unsigned b3Ps2GetVU0PairCount( void );

// Instrument only extra preparation-stage pair sites so mode 3 A/B tests
// cannot report a speedup when the new code path was not reached.
void b3Ps2RecordVU0ExtendedPair( void );
unsigned b3Ps2GetVU0ExtendedPairCount( void );
