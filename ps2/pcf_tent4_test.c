// SPDX-License-Identifier: MIT
// Portable C17 reference for exact four-bilinear 3x3 separable tent.
// Every 0.5-texel interpolated tap covers 2x2 pixels. In the interior
// (ignoring GS fixed-point rounding), 4 taps yield all nine 3x3 terms.
#include <assert.h>
#include <math.h>
#include <stdio.h>
static float bilinear(const float a[3][3],float x,float y)
{
    int ix=(int)floorf(x),iy=(int)floorf(y);
    float tx=x-(float)ix,ty=y-(float)iy;
    return (1-tx)*(1-ty)*a[iy][ix]+tx*(1-ty)*a[iy][ix+1]
         +(1-tx)*ty*a[iy+1][ix]+tx*ty*a[iy+1][ix+1];
}
int main(void)
{
    unsigned state=1;
    const int c[3]={1,2,1};
    for(int trial=0;trial<2048;++trial)
    {
        float texels[3][3];
        for(int y=0;y<3;++y)
        for(int x=0;x<3;++x)
        {
            state=state*1664525u+1013904223u;
            texels[y][x]=(float)((state>>16)&255)/255.0f;
        }
        float nine=0, four=0;
        for(int y=0;y<3;++y)
        for(int x=0;x<3;++x)
            nine+=(float)(c[x]*c[y])*texels[y][x]/16.0f;
        // +/-0.5 around center (1,1) in each axis.
        for(int y=0;y<2;++y)
        for(int x=0;x<2;++x)
            four+=bilinear(texels,0.5f+(float)x,0.5f+(float)y)*0.25f;
        assert(fabsf(four-nine)<=0.000001f);
    }
    // GS VRAM reservation at 640x480 PSMCT16x2 + Z16 + RAW32.
    // Sizes in words: graph_vram_size returns bytes via x4.
    const unsigned w=640u,h=480u;
    const unsigned color16=w*h*2u;
    const unsigned depth16=w*h*2u;
    const unsigned raw32=w*h*4u;
    assert(2u*color16+depth16+raw32 == 3072000u);
    assert(2u*color16+depth16+raw32 < 4194304u);
    // GS blitter UV 4.4 half-offset, TBW=10 and 640x480 dimensions.
    assert((w<<4)+16 < 16384u && (h<<4)+16 < 16384u);
    puts("PASS: 2048 4-vs-9 tap 3x3 tent cases; 640x480p VRAM budget");
    return 0;
}
