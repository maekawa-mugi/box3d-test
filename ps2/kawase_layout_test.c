// SPDX-License-Identifier: MIT
// Portable verification of native GS TEX0 dimensions and VRAM
// budgets for the extended Dual Kawase pyramid.
#include <assert.h>
#include <stdio.h>
static int tw(int v)
{
    int p=1,e=0;
    while(p<v && e<10){p<<=1;++e;}
    return e;
}
static unsigned pageSize(unsigned w,unsigned height,unsigned bpp)
{
    unsigned stride=((w+63u)/64u)*64u;
    unsigned bytes=stride*height*bpp/8u;
    return (bytes+8191u)&~8191u;
}
static unsigned layout(unsigned w,unsigned height,int progressive)
{
    unsigned sum=0;
    unsigned colorBpp=progressive?16u:32u;
    unsigned depthBpp=progressive?16u:32u;
    sum+=2u*pageSize(w,height,colorBpp);
    sum+=pageSize(w,height,depthBpp);
    sum+=pageSize(w,height,32u); // RAW shadow mask
    for(unsigned shrink=2;shrink<=8;shrink*=2)
        sum+=2u*pageSize(w/shrink,height/shrink,32u);
    return sum;
}
int main(void)
{
    // Source: 320x224, half, quarter, eighth.
    assert(tw(320)==9 && tw(224)==8);
    assert(tw(160)==8 && tw(112)==7);
    assert(tw(80)==7 && tw(56)==6);
    assert(tw(40)==6 && tw(28)==5);
    // 480p must not use the old 320x224 exponent pair!
    assert(tw(640)==10 && tw(480)==9);
    assert(tw(320)==9 && tw(240)==8);
    assert(tw(160)==8 && tw(120)==7);
    assert(tw(80)==7 && tw(60)==6);
    assert(pageSize(40,28,32)==8192u);
    assert(layout(320,224,0)==991232u);
    assert(layout(640,480,1)==3956736u);
    assert(layout(640,480,1)<4u*1024u*1024u);
    printf("PASS: TEX0 640x480 TW/TH 10/9, 40x28 8KiB pages, 480p wide VRAM %u/4194304 bytes\n",
           layout(640,480,1));
    return 0;
}
