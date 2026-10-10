// SPDX-License-Identifier: MIT
// Contact-hardening penumbra approximation for GS-native PS2 masks.
// The EE computes caster/receiver separation; the GS samples mask RGB.
// This is PCSS-inspired, not depth-map blocker-search PCSS.
#ifndef B3_PS2_SHADOW_PENUMBRA_H
#define B3_PS2_SHADOW_PENUMBRA_H

// The source emits one pixel of penumbra at zero separation, and the
// effective GS sample radius grows with gap times projected pixel density.
// A 5px clamp bounds GIF packet, fill and aliasing costs on R5900/GS.
// Arguments must be finite; the caller derives these from Box3D transforms.
static inline int b3Ps2ShadowRadius(float casterGap,float pixelsPerMeter)
{
    if(casterGap<=0.0f || pixelsPerMeter<=0.0f)return 1;
    float r=1.0f+0.08f*casterGap*pixelsPerMeter;
    if(r>=5.0f)return 5;
    int rounded=(int)(r+0.5f);
    return rounded<1?1:rounded;
}

// Fixed-function GS FIX units for a normalized 5x5 binomial kernel.
// The 5x5 sum is 98, very close to the 3x3 baseline's 96.
// Corners round to zero; the renderer omits those four taps entirely.
static inline int b3Ps2ShadowWeight5(int x,int y)
{
    static const int c[5]={1,4,6,4,1};
    return (96*c[x+2]*c[y+2]+128)/256;
}

// Reduce only the minor axis of a projected directional penumbra.
// Integer arithmetic is symmetrical for negative and positive offsets
// under C17 truncation toward zero. Radius 5 yields +/-3px minor axis.
static inline int b3Ps2ShadowCompressMinor(int offset)
{
    return (offset*2)/3;
}

// If 40 overlapping casters quantize to the same 5px radius, one
// unioned coverage mask is enough. Thus only populated groups consume
// a GS filter/composite pass, rather than N per-body darkening passes.
static inline int b3Ps2ShadowGroupCount(const unsigned groupCasters[6])
{
    int count=0;
    for(int r=1;r<=5;++r)
        count+=(groupCasters[r]!=0);
    return count;
}

static inline int b3Ps2ShadowOffset5(int index,int radius)
{
    if(index==-2)return -radius;
    if(index== 2)return  radius;
    if(index==-1)return -(radius+1)/2;
    if(index== 1)return (radius+1)/2;
    return 0;
}
#endif
