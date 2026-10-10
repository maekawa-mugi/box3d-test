// SPDX-License-Identifier: MIT
// Host C17 regression for the ONLY supported 480p PCSS sample pages.
#include "unified_demo.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    assert(B3_PS2_DEMO_SCENES==12);
    assert(B3_PS2_DEMO_MAX_BODIES>=1+400);
    const int expected[4]={64,144,256,400};
    for(int i=0;i<B3_PS2_STRESS_PRESETS;++i)
    {
        int side=b3Ps2DemoStressSide(i);
        assert(side*side==expected[i]);
    }
    assert(b3Ps2DemoStressSide(-1)==0);
    assert(b3Ps2DemoStressSide(4)==0);
    assert(b3Ps2DemoNextPage(0,-1)==11);
    assert(b3Ps2DemoNextPage(11,1)==0);
    for(int i=0;i<12;++i)
    {
        assert(b3Ps2DemoNextPage(i,1)==(i+1)%12);
        assert(b3Ps2DemoNextPage(i,-1)==(i+11)%12);
    }
    puts("PASS: unified 12 pages, D-pad wrap, stress bodies 64/144/256/400");
    return 0;
}
