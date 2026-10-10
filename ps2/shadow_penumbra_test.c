// SPDX-License-Identifier: MIT
// Host-only C17 regression for height-dependent GS shadow filter.
#include "shadow_penumbra.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    assert(b3Ps2ShadowRadius(0.0f,10.0f)==1);
    assert(b3Ps2ShadowRadius(-2.0f,10.0f)==1);
    assert(b3Ps2ShadowRadius(2.0f,10.0f)==3);
    assert(b3Ps2ShadowRadius(4.0f,10.0f)==4);
    assert(b3Ps2ShadowRadius(5.0f,10.0f)==5);
    assert(b3Ps2ShadowRadius(400.0f,10.0f)==5);
    assert(b3Ps2ShadowRadius(1.0f,5.0f)==1);
    int total=0,nonzero=0;
    for(int y=-2;y<=2;++y)
    for(int x=-2;x<=2;++x)
    {
        int weight=b3Ps2ShadowWeight5(x,y);
        assert(weight>=0 && weight<128);
        total+=weight;
        nonzero+=weight!=0;
        assert(b3Ps2ShadowOffset5(x,3)>=-3 &&
               b3Ps2ShadowOffset5(x,3)<=3);
    }
    assert(total==98);
    assert(nonzero==21);
    unsigned groups[6]={0,0,0,0,0,40};
    assert(b3Ps2ShadowGroupCount(groups)==1); // former 40 composites
    groups[1]=12;groups[2]=7;groups[4]=3;
    assert(b3Ps2ShadowGroupCount(groups)==4);
    groups[3]=1;
    assert(b3Ps2ShadowGroupCount(groups)==5);
    for(int r=1;r<=5;++r)groups[r]=0;
    assert(b3Ps2ShadowGroupCount(groups)==0);
    assert(b3Ps2ShadowCompressMinor(5)==3);
    assert(b3Ps2ShadowCompressMinor(-5)==-3);
    assert(b3Ps2ShadowCompressMinor(3)==2);
    assert(b3Ps2ShadowCompressMinor(-3)==-2);
    assert(b3Ps2ShadowCompressMinor(0)==0);
    assert(b3Ps2ShadowOffset5(-2,3)==-3);
    assert(b3Ps2ShadowOffset5(-1,3)==-2);
    assert(b3Ps2ShadowOffset5( 1,3)== 2);
    assert(b3Ps2ShadowOffset5( 2,3)== 3);
    puts("PASS: height-dependent PS2 GS penumbra radius and 21-tap kernel");
    return 0;
}
