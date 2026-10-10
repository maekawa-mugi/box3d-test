// SPDX-License-Identifier: MIT
#ifndef B3_PS2_PCSS_TIERS_H
#define B3_PS2_PCSS_TIERS_H
// Real GS 2:1 downsampling provides connected blur support.
// The final screen-space 3x3 kernel NEVER exceeds +/-1 pixel.
// 1=full + 4-bilinear PCF, 2/3=half + 9tap, 4/5=quarter + 9tap.
static inline int b3Ps2PcssDownsampleLevel(int radius)
{
    return radius<=1?0:radius<=3?1:2;
}
static inline int b3Ps2PcssFinalShift(int radius)
{
    return radius<=1?0:1;
}
// Safe full-resolution PCSS: 3x3 bilinear tent, no downsample passes.
// The draw path uses +/-0.5 texel UV for a zero XY shift and +/-1.5
// texels for a one-pixel XY shift.
static inline int b3Ps2PcssDirectShift(int radius)
{
    return radius<=2?0:1;
}
static inline int b3Ps2PcssDirectWeight(int radius,int x,int y)
{
    if(radius>=4)return (x==0 && y==0)?16:10;
    if(x==0 && y==0)return 24;
    return (x==0 || y==0)?12:6;
}
#endif
