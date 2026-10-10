// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "simd.h"

// Macros to avoid name clashes between W4 and W8 functions existing in the same code base.
// At the same time this helps to avoid code duplication.

// Help Intellisense from being confused.
#if defined( __INTELLISENSE__ ) && !defined( B3_SIMD_WIDTH )
#define B3_SIMD_WIDTH 8
#endif

#if B3_SIMD_WIDTH == 8
#define B3_WIDE( name ) name##W8
#elif B3_SIMD_WIDTH == 4
#define B3_WIDE( name ) name##W4
#else
#error "B3_SIMD_WIDTH must be 4 or 8"
#endif

#define B3_WIDE_ALIGNMENT ( 4 * B3_SIMD_WIDTH )

typedef B3_WIDE( b3Float ) b3FloatW;
typedef B3_WIDE( b3Vec3 ) b3Vec3W;

#define b3ZeroW B3_WIDE( b3Zero )
#define b3SplatW B3_WIDE( b3Splat )
#define b3SetW B3_WIDE( b3Set )
#define b3LoadW B3_WIDE( b3Load )
#define b3StoreW B3_WIDE( b3Store )
#define b3NegW B3_WIDE( b3Neg )
#define b3AbsW B3_WIDE( b3Abs )
#define b3AddW B3_WIDE( b3Add )
#define b3SubW B3_WIDE( b3Sub )
#define b3MulW B3_WIDE( b3Mul )
#define b3DivW B3_WIDE( b3Div )
#define b3SqrtW B3_WIDE( b3Sqrt )
#define b3MulAddW B3_WIDE( b3MulAdd )
#define b3MinW B3_WIDE( b3Min )
#define b3MaxW B3_WIDE( b3Max )
#define b3SymClampW B3_WIDE( b3SymClamp )
#define b3AndW B3_WIDE( b3And )
#define b3OrW B3_WIDE( b3Or )
#define b3AndNotW B3_WIDE( b3AndNot )
#define b3SoftMaskW B3_WIDE( b3SoftMask )
#define b3GreaterThanW B3_WIDE( b3GreaterThan )
#define b3LessThanW B3_WIDE( b3LessThan )
#define b3EqualsW B3_WIDE( b3Equals )
#define b3AllZeroW B3_WIDE( b3AllZero )
#define b3AnyTrueW B3_WIDE( b3AnyTrue )
#define b3BlendW B3_WIDE( b3Blend )
#define b3EmbedIndexW B3_WIDE( b3EmbedIndex )
#define b3MinIndexW B3_WIDE( b3MinIndex )
#define b3TransposeW B3_WIDE( b3Transpose )
#define b3LoadVW B3_WIDE( b3LoadV )
#define b3StoreVW B3_WIDE( b3StoreV )
#define b3SplatVW B3_WIDE( b3SplatV )
#define b3NegVW B3_WIDE( b3NegV )
#define b3MulSVW B3_WIDE( b3MulSV )
#define b3MulSubSVW B3_WIDE( b3MulSubSV )
#define b3MulAddSVW B3_WIDE( b3MulAddSV )
#define b3SubVW B3_WIDE( b3SubV )
#define b3AddVW B3_WIDE( b3AddV )
#define b3DotW B3_WIDE( b3Dot )
#define b3CrossW B3_WIDE( b3Cross )

// Two independent dot products. Runtime modes 2 and 3 fuse the work into
// one pipelined VU0 macro invocation. Other builds and modes retain the
// original two scalar/legacy dots with identical arithmetic association.
static inline void b3Dot2W( b3Vec3W a0, b3Vec3W b0, b3Vec3W a1, b3Vec3W b1,
                            b3FloatW* out0, b3FloatW* out1 )
{
#if defined( BOX3D_PS2 ) && B3_SIMD_WIDTH == 4
    if ( b3Ps2GetVU0Enabled() >= 2 )
    {
        b3DotPairW4( a0, b0, a1, b1, out0, out1 );
        return;
    }
#endif
    *out0 = b3DotW( a0, b0 );
    *out1 = b3DotW( a1, b1 );
}


// Additional preparation-stage independent dot pairs, enabled only in
// experimental mode 3. Mode 2 remains bit-for-bit the previous PS2
// paired-backend implementation to allow direct A/B timing without a
// different scalar baseline or unrelated microbenchmark overhead.
static inline void b3Dot2WExtended( b3Vec3W a0, b3Vec3W b0, b3Vec3W a1, b3Vec3W b1,
                                    b3FloatW* out0, b3FloatW* out1 )
{
#if defined( BOX3D_PS2 ) && B3_SIMD_WIDTH == 4
    if ( b3Ps2GetVU0Enabled() >= 3 )
    {
        b3Ps2RecordVU0ExtendedPair();
        b3DotPairW4( a0, b0, a1, b1, out0, out1 );
        return;
    }
#endif
    *out0 = b3DotW( a0, b0 );
    *out1 = b3DotW( a1, b1 );
}

// Many friction operations dot the *same* relative velocity against two
// tangent directions. For mode 4 load velocity XYZ only once into VU0;
// other modes retain the previous generic pair (or scalar) path.
static inline void b3Dot2SharedW( b3Vec3W shared, b3Vec3W b0, b3Vec3W b1,
                                  b3FloatW* out0, b3FloatW* out1 )
{
#if defined( BOX3D_PS2 ) && B3_SIMD_WIDTH == 4
    if ( b3Ps2GetVU0Enabled() == 4 )
    {
        b3Ps2VU0SharedDotData s;
        b3StoreW4( s.sharedX, shared.X );
        b3StoreW4( s.sharedY, shared.Y );
        b3StoreW4( s.sharedZ, shared.Z );
        b3StoreW4( s.b0x, b0.X );
        b3StoreW4( s.b0y, b0.Y );
        b3StoreW4( s.b0z, b0.Z );
        b3StoreW4( s.b1x, b1.X );
        b3StoreW4( s.b1y, b1.Y );
        b3StoreW4( s.b1z, b1.Z );
        b3Ps2VU0SharedDot4( &s );
        *out0 = b3LoadW4( s.out0 );
        *out1 = b3LoadW4( s.out1 );
        return;
    }
#endif
    b3Dot2W( shared, b0, shared, b1, out0, out1 );
}
