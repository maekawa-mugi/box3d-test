// SPDX-License-Identifier: MIT
// Overlap-aware screen-space PCSS grouping for Box3D PS2.
#ifndef B3_PS2_SHADOW_GROUP_H
#define B3_PS2_SHADOW_GROUP_H

typedef struct b3Ps2ShadowBounds
{
    int x0,y0,x1,y1;
    unsigned char radius,valid;
} b3Ps2ShadowBounds;

// Compare the silhouettes expanded by their GS PCSS kernel support.
// Two masks that touch in this expanded screen region MUST use one
// union mask and one subtractive composite, not twice-darkened pixels.
static inline int b3Ps2ShadowTouch(const b3Ps2ShadowBounds* a,
                                    const b3Ps2ShadowBounds* b)
{
    if(!a->valid || !b->valid)return 0;
    int pa=(int)a->radius*3+6,pb=(int)b->radius*3+6;
    return a->x0-pa<=b->x1+pb && b->x0-pb<=a->x1+pa &&
           a->y0-pa<=b->y1+pb && b->y0-pb<=a->y1+pa;
}
static inline int b3Ps2ShadowRoot(int parent[],int i)
{
    while(parent[i]!=i)
    {
        parent[i]=parent[parent[i]];
        i=parent[i];
    }
    return i;
}
// Connected components of projected shadows. Each union has one mask,
// one max radius and one filtered GS composite. Separate shadows still
// retain distinct radii. Deterministic O(N*N) integer bounds comparisons
// replace multiple GPU over-darkening passes.
static inline int b3Ps2ShadowGroup(const b3Ps2ShadowBounds input[],
                                    int count,int parent[],
                                    b3Ps2ShadowBounds output[])
{
    for(int i=0;i<count;++i)
    {
        parent[i]=input[i].valid?i:-1;
        output[i].valid=0;
    }
    for(int i=0;i<count;++i)
    {
        if(!input[i].valid)continue;
        for(int j=i+1;j<count;++j)
        {
            if(!b3Ps2ShadowTouch(&input[i],&input[j]))continue;
            int a=b3Ps2ShadowRoot(parent,i);
            int b=b3Ps2ShadowRoot(parent,j);
            if(a!=b)
            {
                if(a<b)parent[b]=a;
                else parent[a]=b;
            }
        }
    }
    int num=0;
    for(int i=0;i<count;++i)
    {
        if(!input[i].valid)continue;
        int root=b3Ps2ShadowRoot(parent,i);
        parent[i]=root;
        if(!output[root].valid)
        {
            output[root]=input[i];
            ++num;
        }
        else
        {
            b3Ps2ShadowBounds* g=&output[root];
            if(input[i].x0<g->x0)g->x0=input[i].x0;
            if(input[i].y0<g->y0)g->y0=input[i].y0;
            if(input[i].x1>g->x1)g->x1=input[i].x1;
            if(input[i].y1>g->y1)g->y1=input[i].y1;
            if(input[i].radius>g->radius)g->radius=input[i].radius;
        }
    }
    return num;
}
#endif
