// SPDX-License-Identifier: MIT
// Portable regression for the EE CP0 Count software extension.
#include "ps2_count.h"
#include <stdio.h>

int main( void )
{
    b3Ps2CountState state = { 0 };
    const uint32_t samples[] = { 0xfffffff0u, 0xfffffff8u, 0x00000008u, 0x00000018u, 0x00000018u };
    const uint64_t expected[] = { 0, 8, 24, 40, 40 };
    for ( unsigned i = 0; i < sizeof( samples ) / sizeof( samples[0] ); ++i )
    {
        uint64_t actual = b3Ps2ExtendCount( &state, samples[i] );
        if ( actual != expected[i] )
        {
            printf( "CP0 Count FAIL at %u: expected %llu, got %llu\n", i,
                    (unsigned long long)expected[i], (unsigned long long)actual );
            return 1;
        }
    }
    b3Ps2CountState other = { 0 };
    if ( b3Ps2ExtendCount( &other, 1234u ) != 0 || b3Ps2ExtendCount( &other, 1235u ) != 1 )
    {
        puts( "CP0 Count initialization FAIL" );
        return 1;
    }
    puts( "PS2 CP0 Count wrap: PASS" );
    return 0;
}
