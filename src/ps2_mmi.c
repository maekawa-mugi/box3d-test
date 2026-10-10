// SPDX-License-Identifier: MIT
// EE MMI block clear/bitset OR. Float physics stays in the scalar COP1 path.
// LQ/SQ silently round an unaligned address down on R5900: NEVER issue them
// except when both pointers have verified 16-byte alignment.
#include "ps2_mmi.h"

#include <stdint.h>
#include <string.h>

// Default OFF preserves the established PCSx2 scalar physics smoke.
// Explicitly opt in per trial with b3Ps2SetMMIEnabled(1).
static int s_mmiEnabled;

void b3Ps2SetMMIEnabled( int enabled )
{
    s_mmiEnabled = enabled != 0;
}

int b3Ps2GetMMIEnabled( void )
{
    return s_mmiEnabled;
}

void b3Ps2ClearBytes( void* dst, size_t byteCount )
{
    if ( byteCount == 0 )
    {
        return;
    }

#if defined( BOX3D_PS2 ) && ( defined( __mips__ ) || defined( __mips ) )
    if ( s_mmiEnabled && ( (uintptr_t)dst & 15u ) == 0 && byteCount >= 32 )
    {
        unsigned char* p = (unsigned char*)dst;
        size_t i = 0;
        for ( ; i + 16 <= byteCount; i += 16 )
        {
            // Zero GPR0 is 128-bit zero on EE. Straight-line MMI load/store.
            __asm__ __volatile__( "sq $0, 0(%0)" : : "r"( p + i ) : "memory" );
        }
        if ( i != byteCount )
        {
            memset( p + i, 0, byteCount - i );
        }
        return;
    }
#endif
    memset( dst, 0, byteCount );
}

void b3Ps2OrWords( uint64_t* dst, const uint64_t* src, size_t wordCount )
{
    if ( wordCount == 0 )
    {
        return;
    }

    size_t i = 0;
#if defined( BOX3D_PS2 ) && ( defined( __mips__ ) || defined( __mips ) )
    if ( s_mmiEnabled &&
         ( ( (uintptr_t)dst | (uintptr_t)src ) & 15u ) == 0 &&
         wordCount >= 4 )
    {
        // Two adjacent 64-bit words per 128-bit MMI OR.
        // Explicitly clobber caller-saved EE GPRs, no manual branch/delay slot.
        for ( ; i + 1 < wordCount; i += 2 )
        {
            __asm__ __volatile__(
                "lq $8, 0(%0)\n\t"
                "lq $9, 0(%1)\n\t"
                "por $8, $8, $9\n\t"
                "sq $8, 0(%0)"
                : : "r"( dst + i ), "r"( src + i )
                : "$8", "$9", "memory" );
        }
    }
#endif
    for ( ; i < wordCount; ++i )
    {
        dst[i] |= src[i];
    }
}
