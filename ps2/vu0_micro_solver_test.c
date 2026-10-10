// SPDX-License-Identifier: MIT
// Host regression: VU0 4-lane/16-record DMEM ABI and scalar math contract.
// This test does NOT execute PS2 microcode; PCSX2 runs the real differential.
#include "vu0_micro_solver.h"
#include "../src/ps2_vu0.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    assert(sizeof(b3Vu0MicroRecord)==208u);
    assert(sizeof(b3Vu0MicroBlock)==3344u);
    assert(offsetof(b3Vu0MicroBlock,record)==16u);
    assert(offsetof(b3Vu0MicroRecord,newImpulse)==128u);
    assert(offsetof(b3Vu0MicroRecord,deltaImpulse)==144u);
    assert(offsetof(b3Vu0MicroRecord,newPointTotal)==160u);
    assert(offsetof(b3Vu0MicroRecord,newManifoldTotal)==176u);
    assert(offsetof(b3Vu0MicroRecord,newTwistLimit)==192u);
    assert(B3_VU0_MICRO_TOTAL_QWS<256u);
    // Mode 6 must never run legacy COP2 macro dot kernels while the
    // VU0 micro engine owns the same vector unit.
    assert(!b3Ps2VU0MacroModeMatches(0,2));
    assert(!b3Ps2VU0MacroModeMatches(1,2));
    assert(b3Ps2VU0MacroModeMatches(2,2));
    assert(b3Ps2VU0MacroModeMatches(3,3));
    assert(b3Ps2VU0MacroModeMatches(4,4));
    assert(b3Ps2VU0MacroModeMatches(5,2));
    assert(!b3Ps2VU0MacroModeMatches(6,2));
    assert(!b3Ps2VU0MacroModeMatches(6,3));
    assert(!b3Ps2VU0MacroModeMatches(6,4));
    assert(B3_VU0_MICRO_MAX_LIVE_PER_STEP>0);
    assert(B3_PS2_VU0_BATCH_GROUPS==B3_VU0_MICRO_GROUPS);
    assert(B3_VU0_MICRO_MAX_BATCHES_PER_STEP>=B3_VU0_MICRO_MAX_LIVE_PER_STEP);
    b3Vu0MicroBlock block={0};
    assert(((uintptr_t)&block&15u)==0u);
    for(unsigned seed=0;seed<64u;++seed)
    {
        for(unsigned g=0;g<16u;++g)
        for(unsigned l=0;l<4u;++l)
        {
            b3Vu0MicroRecord* r=&block.record[g];
            const float x=(float)((seed*31u+g*7u+l*19u)%49u)*0.125f;
            r->oldImpulse[l]=x;
            r->normalMass[l]=0.125f+(float)(l+1u)*0.125f;
            r->normalSpeed[l]=(float)((int)((seed+g*3u+l)%17u)-8)*0.25f;
            r->velocityBias[l]=(float)(g%3u)*0.125f;
            r->oldPointTotal[l]=x*0.5f;
            r->oldManifoldTotal[l]=x*0.25f;
            r->oldTwistLimit[l]=0.75f;
            r->leverArm[l]=0.5f;
        }
        b3Vu0MicroReference(&block);
        for(unsigned g=0;g<16u;++g)
        for(unsigned l=0;l<4u;++l)
        {
            b3Vu0MicroRecord* r=&block.record[g];
            float next=r->oldImpulse[l]-
                r->normalMass[l]*(r->normalSpeed[l]+r->velocityBias[l]);
            if(next<0.0f)next=0.0f;
            assert(fabsf(r->newImpulse[l]-next)<0.00001f);
            assert(fabsf(r->deltaImpulse[l]-(next-r->oldImpulse[l]))<0.00001f);
            assert(fabsf(r->newPointTotal[l]-
                         (r->oldPointTotal[l]+next))<0.00001f);
            assert(fabsf(r->newManifoldTotal[l]-
                         (r->oldManifoldTotal[l]+next))<0.00001f);
            assert(fabsf(r->newTwistLimit[l]-
                         (r->oldTwistLimit[l]+r->leverArm[l]*next))<0.00001f);
        }
    }
    puts("PASS: 16x4 VU0 micro DMEM ABI and 64 scalar differential batches");
    return 0;
}
