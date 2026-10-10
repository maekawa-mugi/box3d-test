// SPDX-License-Identifier: MIT
// PS2 EE VU0 COP2 macro-mode, reference: supplied VU User Manual chapter 5
// and ps2sdk ee/math3d/src/math3d.c for GNU assembler syntax.
#include "ps2_vu0.h"

#include <stddef.h>
#include <stdint.h>

_Static_assert( _Alignof(b3Ps2VU0Dot4Data) >= 16, "VU0 scratch alignment" );
_Static_assert( sizeof(b3Ps2VU0Dot4Data) == 112, "VU0 scratch expected layout" );
_Static_assert( offsetof(b3Ps2VU0Dot4Data, bx) == 16, "VU0 bx offset" );
_Static_assert( offsetof(b3Ps2VU0Dot4Data, ay) == 32, "VU0 ay offset" );
_Static_assert( offsetof(b3Ps2VU0Dot4Data, by) == 48, "VU0 by offset" );
_Static_assert( offsetof(b3Ps2VU0Dot4Data, az) == 64, "VU0 az offset" );
_Static_assert( offsetof(b3Ps2VU0Dot4Data, bz) == 80, "VU0 bz offset" );
_Static_assert( offsetof(b3Ps2VU0Dot4Data, out) == 96, "VU0 output offset" );

_Static_assert( sizeof(b3Ps2VU0Dot4Data) % 16 == 0, "VU0 record stride" );

static int s_vu0Mode;
static unsigned s_vu0PairCount;
static unsigned s_vu0ExtendedPairCount;

void b3Ps2ResetVU0PairCount( void )
{
    s_vu0PairCount = 0;
    s_vu0ExtendedPairCount = 0;
}

unsigned b3Ps2GetVU0PairCount( void )
{
    return s_vu0PairCount;
}

void b3Ps2RecordVU0ExtendedPair( void )
{
    ++s_vu0ExtendedPairCount;
}

unsigned b3Ps2GetVU0ExtendedPairCount( void )
{
    return s_vu0ExtendedPairCount;
}

void b3Ps2SetVU0Enabled( int mode )
{
    s_vu0Mode = mode == 3 ? 3 : ( mode == 2 ? 2 : ( mode != 0 ? 1 : 0 ) );
}

int b3Ps2GetVU0Enabled( void )
{
    return s_vu0Mode;
}

void b3Ps2VU0Dot4( b3Ps2VU0Dot4Data* data )
{
#if defined(BOX3D_PS2) && (defined(__mips__) || defined(__mips))
    // LQC2/SQC2 have a hard 16-byte alignment requirement on the EE.
    // The struct alignment and offsets above are part of the function ABI.
    // Six loads, three independent VMUL, two VADD and one store:
    // ((ax*bx) + (ay*by)) + az*bz in each of the four lanes. Deliberately
    // avoid VMADD/ACC so floating-point fusion does not change association.
    //
    // VF1..VF8 are dedicated scratch COP2 registers. GCC uses COP1 for C
    // expressions; like PS2SDK's math3d, this inline asm owns the VU0 regs.
    // Do not use while any other library/VU0 microprogram owns COP2 state.
    __asm__ __volatile__(
        "lqc2 $vf1, 0(%0)\n\t"
        "lqc2 $vf2, 16(%0)\n\t"
        "lqc2 $vf3, 32(%0)\n\t"
        "lqc2 $vf4, 48(%0)\n\t"
        "lqc2 $vf5, 64(%0)\n\t"
        "lqc2 $vf6, 80(%0)\n\t"
        "vmul.xyzw $vf7, $vf1, $vf2\n\t"
        "vmul.xyzw $vf8, $vf3, $vf4\n\t"
        "vadd.xyzw $vf7, $vf7, $vf8\n\t"
        "vmul.xyzw $vf8, $vf5, $vf6\n\t"
        "vadd.xyzw $vf7, $vf7, $vf8\n\t"
        "sqc2 $vf7, 96(%0)"
        : : "r"(data) : "memory");
#else
    for (int lane = 0; lane < 4; ++lane)
    {
        data->out[lane] = (data->ax[lane] * data->bx[lane] +
                           data->ay[lane] * data->by[lane]) +
                          data->az[lane] * data->bz[lane];
    }
#endif
}

// Four independent dot4 records. The block structure allows multiple
// VU0 multiplies to be issued before consuming their products, without
// round-tripping intermediate floats through EE COP1. The first product
// is not read immediately after its VMUL (issue distance through the
// second VMUL and subsequent load), and the third product is computed
// before the first VADD. See EE VU0 pipeline/manual restrictions.
//
// Keep VF1..VF10 private. Unrolling avoids short backward branch loops.
// Scratch is 16-byte aligned and record stride is exactly 112 bytes.
void b3Ps2VU0Dot4Batch4( b3Ps2VU0Dot4Data data[4] )
{
#if defined(BOX3D_PS2) && (defined(__mips__) || defined(__mips))
    for (int i = 0; i < 4; ++i)
    {
        float* row = data[i].ax;
        __asm__ __volatile__(
            "lqc2 $vf1, 0(%0)\n\t"
            "lqc2 $vf2, 16(%0)\n\t"
            "lqc2 $vf3, 32(%0)\n\t"
            "lqc2 $vf4, 48(%0)\n\t"
            "vmul.xyzw $vf7, $vf1, $vf2\n\t"
            "lqc2 $vf5, 64(%0)\n\t"
            "lqc2 $vf6, 80(%0)\n\t"
            "vmul.xyzw $vf8, $vf3, $vf4\n\t"
            "vmul.xyzw $vf9, $vf5, $vf6\n\t"
            "vadd.xyzw $vf7, $vf7, $vf8\n\t"
            "vadd.xyzw $vf7, $vf7, $vf9\n\t"
            "sqc2 $vf7, 96(%0)"
            : : "r"(row) : "memory" );
    }
#else
    for (int i = 0; i < 4; ++i)
    {
        b3Ps2VU0Dot4( &data[i] );
    }
#endif
}

_Static_assert( sizeof(b3Ps2VU0Dot4PairData) == 224, "VU0 paired layout" );
_Static_assert( _Alignof(b3Ps2VU0Dot4PairData) >= 16, "VU0 paired alignment" );
_Static_assert( offsetof(b3Ps2VU0Dot4PairData, pair[1]) == 112, "VU0 paired offset" );

void b3Ps2VU0Dot4Pair( b3Ps2VU0Dot4PairData* data )
{
    s_vu0PairCount++;
#if defined(BOX3D_PS2) && (defined(__mips__) || defined(__mips))
    // Both jobs have independent dependencies. Put all multiplies before
    // either add, then stagger the two separate chains. This allows VU0's
    // multiply/add pipeline to overlap while waiting for results.
    //
    // 0..96: first dot; 112..208: second. Base + every offset is 16B
    // aligned. Stack layout is statically checked above.
    __asm__ __volatile__(
        "lqc2 $vf1, 0(%0)\n\t"
        "lqc2 $vf2, 16(%0)\n\t"
        "lqc2 $vf3, 32(%0)\n\t"
        "lqc2 $vf4, 48(%0)\n\t"
        "lqc2 $vf5, 64(%0)\n\t"
        "lqc2 $vf6, 80(%0)\n\t"
        "lqc2 $vf7, 112(%0)\n\t"
        "lqc2 $vf8, 128(%0)\n\t"
        "lqc2 $vf9, 144(%0)\n\t"
        "lqc2 $vf10, 160(%0)\n\t"
        "lqc2 $vf11, 176(%0)\n\t"
        "lqc2 $vf12, 192(%0)\n\t"
        "vmul.xyzw $vf13, $vf1, $vf2\n\t"
        "vmul.xyzw $vf14, $vf7, $vf8\n\t"
        "vmul.xyzw $vf15, $vf3, $vf4\n\t"
        "vmul.xyzw $vf16, $vf9, $vf10\n\t"
        "vmul.xyzw $vf17, $vf5, $vf6\n\t"
        "vmul.xyzw $vf18, $vf11, $vf12\n\t"
        "vadd.xyzw $vf13, $vf13, $vf15\n\t"
        "vadd.xyzw $vf14, $vf14, $vf16\n\t"
        "vadd.xyzw $vf13, $vf13, $vf17\n\t"
        "vadd.xyzw $vf14, $vf14, $vf18\n\t"
        "sqc2 $vf13, 96(%0)\n\t"
        "sqc2 $vf14, 208(%0)"
        : : "r"(data) : "memory");
#else
    b3Ps2VU0Dot4( &data->pair[0] );
    b3Ps2VU0Dot4( &data->pair[1] );
#endif
}
