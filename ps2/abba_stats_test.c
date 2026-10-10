// SPDX-License-Identifier: MIT
// Host-only regression: all ABBA mode comparisons, including 2 vs 3.
#include "abba_stats.h"
#include <assert.h>

int main( void )
{
    // Equal totals despite unequal individual samples.
    const uint64_t equal[4] = { 200, 100, 130, 30 };
    const b3Ps2ABBAStats e = b3Ps2SummarizeABBA( equal );
    assert( e.baselineTicks == 230 && e.targetTicks == 230 && e.speedup == 1.0 );

    // Example mode2->mode3 values: baseline must NOT be zero.
    const uint64_t faster[4] = { 2000, 1000, 1000, 2000 };
    const b3Ps2ABBAStats f = b3Ps2SummarizeABBA( faster );
    assert( f.baselineTicks == 4000 && f.targetTicks == 2000 && f.speedup == 2.0 );

    const uint64_t slower[4] = { 1000, 2000, 2000, 1000 };
    const b3Ps2ABBAStats s = b3Ps2SummarizeABBA( slower );
    assert( s.baselineTicks == 2000 && s.targetTicks == 4000 && s.speedup == 0.5 );

    const uint64_t empty[4] = { 1, 0, 0, 1 };
    const b3Ps2ABBAStats z = b3Ps2SummarizeABBA( empty );
    assert( z.baselineTicks == 2 && z.targetTicks == 0 && z.speedup == 0.0 );
    return 0;
}
