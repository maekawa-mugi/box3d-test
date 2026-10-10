// SPDX-License-Identifier: MIT
#include "pcss_filter_tiers.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    assert(b3Ps2PcssDownsampleLevel(1)==0);
    assert(b3Ps2PcssDownsampleLevel(2)==1);
    assert(b3Ps2PcssDownsampleLevel(3)==1);
    assert(b3Ps2PcssDownsampleLevel(4)==2);
    assert(b3Ps2PcssDownsampleLevel(5)==2);
    for(int r=1;r<=5;++r)assert(b3Ps2PcssFinalShift(r)<=1);
    assert(b3Ps2PcssDirectShift(2)==0);
    assert(b3Ps2PcssDirectShift(3)==1);
    assert(b3Ps2PcssDirectShift(5)==1);
    for(int r=2;r<=5;++r)
    {
        int total=0;
        for(int y=-1;y<=1;++y)
        for(int x=-1;x<=1;++x)
            total+=b3Ps2PcssDirectWeight(r,x,y);
        assert(total==96);
    }
    puts("PASS: PCSS direct full-resolution 3x3 tap weights and tiers");
    return 0;
}
