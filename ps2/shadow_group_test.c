// SPDX-License-Identifier: MIT
// Host regression for eliminating overlapping PCSS double shadows.
#include "shadow_group.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    b3Ps2ShadowBounds in[40]={0},out[40]={0};
    int parent[40];
    assert(b3Ps2ShadowGroup(in,40,parent,out)==0);
    in[0]=(b3Ps2ShadowBounds){10,10,13,13,1,1};
    in[1]=(b3Ps2ShadowBounds){34,10,38,13,3,1};
    in[2]=(b3Ps2ShadowBounds){58,10,61,13,2,1};
    assert(b3Ps2ShadowGroup(in,40,parent,out)==1);
    assert(out[0].radius==3 && out[0].x0==10 && out[0].x1==61);
    in[2].x0=280;in[2].x1=285;
    assert(b3Ps2ShadowGroup(in,40,parent,out)==2);
    assert(out[0].radius==3 && out[2].radius==2);
    for(int i=0;i<40;++i)
        in[i]=(b3Ps2ShadowBounds){3*i,10,3*i+1,11,
                                   (unsigned char)(1+i%5),1};
    assert(b3Ps2ShadowGroup(in,40,parent,out)==1);
    assert(out[0].radius==5);
    for(int i=0;i<40;++i)assert(parent[i]==0);
    // Scattered casters: every valid rectangle should remain independent,
    // including casters beyond the 640-pixel view and with negative X.
    b3Ps2ShadowBounds wide[400]={0},groups[400]={0};
    int links[400];
    for(int i=0;i<400;++i)
        wide[i]=(b3Ps2ShadowBounds){i*100-300,-100,i*100-298,-98,1,1};
    assert(b3Ps2ShadowGroup(wide,400,links,groups)==400);
    for(int i=0;i<400;++i)assert(links[i]==i);
    // Transitive overlap with deliberately shuffled X positions.
    for(int i=0;i<40;++i)
        in[i]=(b3Ps2ShadowBounds){(39-i)*7,5,(39-i)*7+9,20,1,1};
    assert(b3Ps2ShadowGroup(in,40,parent,out)==1);
    for(int i=0;i<40;++i)assert(parent[i]==0);
    puts("PASS: broadphase union preserves disjoint, overlap and transitive groups");
    return 0;
}
