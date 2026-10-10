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

static int s_vu0Enabled;

void b3Ps2SetVU0Enabled( int enabled )
{
    s_vu0Enabled = enabled != 0;
}

int b3Ps2GetVU0Enabled( void )
{
    return s_vu0Enabled;
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
