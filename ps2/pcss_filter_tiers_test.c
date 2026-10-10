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
    puts("PASS: PCSS full/half/quarter tiers, nonduplicating +/-1px taps");
    return 0;
}
