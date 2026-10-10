// SPDX-License-Identifier: MIT
// Portable regression for normal-constraint ABI/layout and scalar fallback.
#include "../src/ps2_vu0.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
int main(void)
{
    b3Ps2VU0NormalSolveData d = {0};
    assert(((uintptr_t)&d % 16)==0);
    for(int k=0;k<4;++k)
    {
        d.oldImpulse[k]=1.0f;
        d.normalMass[k]=0.5f;
        d.normalSpeed[k]=(float)(k-2);
        d.velocityBias[k]=0.25f;
        d.oldPointTotal[k]=2.0f;
        d.oldManifoldTotal[k]=3.0f;
        d.oldTwistLimit[k]=4.0f;
        d.leverArm[k]=0.5f;
    }
    b3Ps2ResetVU0PairCount();
    b3Ps2VU0NormalSolve(&d);
    assert(b3Ps2GetVU0NormalSolveCount()==1);
    for(int k=0;k<4;++k)
    {
        float raw=1.0f-0.5f*((float)(k-2)+0.25f);
        float next=raw>=0.0f?raw:0.0f;
        assert(fabsf(d.newImpulse[k]-next)<0.000001f);
        assert(fabsf(d.deltaImpulse[k]-(next-1.0f))<0.000001f);
        assert(fabsf(d.newPointTotal[k]-(next+2.0f))<0.000001f);
        assert(fabsf(d.newManifoldTotal[k]-(next+3.0f))<0.000001f);
        assert(fabsf(d.newTwistLimit[k]-(4.0f+0.5f*next))<0.000001f);
    }
    return 0;
}
