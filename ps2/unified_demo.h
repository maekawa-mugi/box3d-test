// SPDX-License-Identifier: MIT
// Host-compilable canonical PS2 demo scene/pagination configuration.
#ifndef B3_PS2_UNIFIED_DEMO_H
#define B3_PS2_UNIFIED_DEMO_H
#define B3_PS2_DEMO_SCENES 12
#define B3_PS2_STRESS_PRESETS 4
#define B3_PS2_DEMO_MAX_BODIES 410

static inline int b3Ps2DemoStressSide(int index)
{
    static const int side[B3_PS2_STRESS_PRESETS]={8,12,16,20};
    return (index>=0 && index<B3_PS2_STRESS_PRESETS)?side[index]:0;
}
static inline int b3Ps2DemoNextPage(int current,int direction)
{
    // Supports LEFT / RIGHT, including wrapping 0 -> final stress tier.
    int next=current+direction;
    if(next<0)next+=B3_PS2_DEMO_SCENES;
    if(next>=B3_PS2_DEMO_SCENES)next-=B3_PS2_DEMO_SCENES;
    return next;
}
#endif
