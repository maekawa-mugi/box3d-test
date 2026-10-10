// SPDX-License-Identifier: MIT
// Fast X broadphase must preserve EXACT connected components vs all-pairs.
#include "shadow_group.h"
#include <stdint.h>
#include <stdio.h>

enum { N=400 };
static uint32_t state=UINT32_C(0x317abc9d);
static uint32_t nextRand(void)
{
    state^=state<<13;state^=state>>17;state^=state<<5;
    return state;
}
int main(void)
{
    b3Ps2ShadowBounds input[N],out[N];
    int fast[N],reference[N];
    for(int trial=0;trial<250;++trial)
    {
        int count=trial%3==0?N:(int)(nextRand()%(N+1));
        for(int i=0;i<count;++i)
        {
            int x=(int)(nextRand()%2000)-450;
            int y=(int)(nextRand()%1400)-340;
            int w=(int)(nextRand()%80),h=(int)(nextRand()%60);
            input[i]=(b3Ps2ShadowBounds){
                x,y,x+w,y+h,
                (unsigned char)(1+nextRand()%5),
                (unsigned char)(nextRand()%8!=0)
            };
            reference[i]=input[i].valid?i:-1;
        }
        int nFast=b3Ps2ShadowGroup(input,count,fast,out),nRef=0;
        for(int i=0;i<count;++i)
        {
            if(!input[i].valid)continue;
            for(int j=i+1;j<count;++j)
            {
                if(!b3Ps2ShadowTouch(&input[i],&input[j]))continue;
                int a=b3Ps2ShadowRoot(reference,i);
                int b=b3Ps2ShadowRoot(reference,j);
                if(a!=b)
                {
                    if(a<b)reference[b]=a;
                    else reference[a]=b;
                }
            }
        }
        for(int i=0;i<count;++i)
        {
            if(!input[i].valid)continue;
            reference[i]=b3Ps2ShadowRoot(reference,i);
            nRef+=(reference[i]==i);
            if(reference[i]!=fast[i])
            {
                printf("FAIL: trial=%d caster=%d got=%d expected=%d\n",
                       trial,i,fast[i],reference[i]);
                return 1;
            }
        }
        if(nFast!=nRef)
        {
            printf("FAIL: trial=%d components=%d expected=%d\n",
                   trial,nFast,nRef);
            return 1;
        }
    }
    puts("PASS: PCSS broadphase matches 250 brute-force scenes (N<=400)");
    return 0;
}
