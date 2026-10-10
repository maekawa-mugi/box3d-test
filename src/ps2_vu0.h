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
// 4: mode 3 plus shared-input fused two-dot VU0 friction calculations.
// 5: mode 4 plus a fused normal contact impulse/update macro kernel.
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

// Nine distinct input float4s and two output float4s, all aligned.
// The old generic pair uses 12 input float4s and two outputs.
// Batch 'shared dot tangent1' and 'shared dot tangent2' inside one VU0
// macro invocation with a shared 3D vector retained in VF registers.
typedef struct b3Ps2VU0SharedDotData
{
    _Alignas(16) float sharedX[4], sharedY[4], sharedZ[4];
    float b0x[4], b0y[4], b0z[4];
    float b1x[4], b1y[4], b1z[4];
    float out0[4], out1[4];
} b3Ps2VU0SharedDotData;

void b3Ps2VU0SharedDot4( b3Ps2VU0SharedDotData* data );
unsigned b3Ps2GetVU0SharedDotCount( void );

// Eight float4 inputs and five float4 outputs, all 16B aligned.
// VU0 owns intermediate bias/mass/clamp and accumulator values.
typedef struct b3Ps2VU0NormalSolveData
{
    _Alignas(16) float oldImpulse[4];
    float normalMass[4];
    float normalSpeed[4];
    float velocityBias[4];
    float oldPointTotal[4];
    float oldManifoldTotal[4];
    float oldTwistLimit[4];
    float leverArm[4];
    float newImpulse[4];
    float deltaImpulse[4];
    float newPointTotal[4];
    float newManifoldTotal[4];
    float newTwistLimit[4];
} b3Ps2VU0NormalSolveData;

void b3Ps2VU0NormalSolve( b3Ps2VU0NormalSolveData* data );
unsigned b3Ps2GetVU0NormalSolveCount( void );

// Compute-only saturation probe: 8 independent VF accumulators, each with
// four float32 lanes, stay in COP2 for 64 iterations of vector addition.
// The 17 memory transfers are amortized over 512 vector instructions.
// This is a diagnostic roofline *experiment*, not an optimized solver path.
#define B3_PS2_VU0_PROBE_ROUNDS 64
#define B3_PS2_VU0_PROBE_STREAMS 8

typedef struct b3Ps2VU0ALUProbeData
{
    _Alignas(16) float sum[B3_PS2_VU0_PROBE_STREAMS][4];
    float add[4];
} b3Ps2VU0ALUProbeData;

void b3Ps2VU0ALUProbe( b3Ps2VU0ALUProbeData* data );
