// SPDX-License-Identifier: MIT
#include "shadow_project.h"
#include <math.h>
#include <string.h>

static b3ShadowVec3 add(b3ShadowVec3 a,b3ShadowVec3 b)
{
    b3ShadowVec3 r={a.x+b.x,a.y+b.y,a.z+b.z};return r;
}
static b3ShadowVec3 sub(b3ShadowVec3 a,b3ShadowVec3 b)
{
    b3ShadowVec3 r={a.x-b.x,a.y-b.y,a.z-b.z};return r;
}
static b3ShadowVec3 mul(b3ShadowVec3 a,float b)
{
    b3ShadowVec3 r={a.x*b,a.y*b,a.z*b};return r;
}
static float dot(b3ShadowVec3 a,b3ShadowVec3 b)
{
    return a.x*b.x+a.y*b.y+a.z*b.z;
}
static float cross2(b3ShadowVec2 a,b3ShadowVec2 b,b3ShadowVec2 c)
{
    return (b.u-a.u)*(c.v-a.v)-(b.v-a.v)*(c.u-a.u);
}
static int less(b3ShadowVec2 a,b3ShadowVec2 b)
{
    return a.u<b.u || (a.u==b.u && a.v<b.v);
}

// Monotone chain convex hull of eight 2D points. The return winding is
// consistently counterclockwise in (u,v), regardless of box orientation.
static int convexHull(b3ShadowVec2 input[20],int count,
                      b3ShadowVec2 output[B3_SHADOW_MAX_VERTS])
{
    for(int i=1;i<count;++i)
    {
        b3ShadowVec2 value=input[i];
        int j=i;
        while(j>0 && less(value,input[j-1]))
        {
            input[j]=input[j-1];--j;
        }
        input[j]=value;
    }
    int n=0;
    for(int i=0;i<count;++i)
    {
        while(n>=2 && cross2(output[n-2],output[n-1],input[i])<=0.00001f)
            --n;
        output[n++]=input[i];
    }
    int lower=n;
    for(int i=count-2;i>=0;--i)
    {
        while(n>lower && cross2(output[n-2],output[n-1],input[i])<=0.00001f)
            --n;
        output[n++]=input[i];
    }
    if(n>1)--n; // duplicate first point
    return n>=3?n:0;
}
static float component(b3ShadowVec2 p,int side)
{
    return (side&1)?p.v:p.u;
}
static int clip(b3ShadowVec2 in[B3_SHADOW_MAX_VERTS],int n,
                b3ShadowVec2 out[B3_SHADOW_MAX_VERTS],
                int side,float limit,int upper)
{
    if(n==0)return 0;
    int k=0;
    b3ShadowVec2 prev=in[n-1];
    float a=component(prev,side)-limit;
    int insidePrev=upper?(a<=0.0f):(a>=0.0f);
    for(int i=0;i<n;++i)
    {
        b3ShadowVec2 curr=in[i];
        float b=component(curr,side)-limit;
        int inside=upper?(b<=0.0f):(b>=0.0f);
        if(inside!=insidePrev)
        {
            float denom=a-b;
            if(fabsf(denom)>1e-8f)
            {
                float t=a/denom;
                b3ShadowVec2 intersection={
                    prev.u+t*(curr.u-prev.u),
                    prev.v+t*(curr.v-prev.v)};
                if(k>=B3_SHADOW_MAX_VERTS)return 0;
                out[k++]=intersection;
            }
        }
        if(inside)
        {
            if(k>=B3_SHADOW_MAX_VERTS)return 0;
            out[k++]=curr;
        }
        prev=curr;a=b;insidePrev=inside;
    }
    return k;
}

int b3ShadowProjectBox(const b3ShadowVec3 corners[8],
                       b3ShadowVec3 lightRay,
                       const b3ShadowReceiver* surface,
                       b3ShadowPolygon* result)
{
    if(!result)return 0;
    result->count=0;
    if(!corners || !surface || surface->halfU<=0.0f || surface->halfV<=0.0f)
        return 0;
    float dn=dot(lightRay,surface->normal);
    // Shadow only when the directional light points into the receiving
    // side of the plane. A backfacing receiver is never shaded.
    if(dn >= -0.00001f)return 0;

    // First clip the caster cuboid against the receiver half-space.
    // Keeping only its above-plane original vertices was incorrect: if
    // the box interpenetrates the ground, the missing *edge-plane
    // intersection* points can make the projected hull collapse or
    // disappear entirely. A cuboid has 8 corners and 12 edges.
    static const int edges[12][2]={
        {0,1},{0,2},{0,4},{1,3},{1,5},{2,3},
        {2,6},{3,7},{4,5},{4,6},{5,7},{6,7}
    };
    b3ShadowVec3 candidates[20];
    float h[8];
    int candidatesCount=0;
    for(int i=0;i<8;++i)
    {
        h[i]=dot(sub(corners[i],surface->origin),surface->normal);
        if(h[i]>=-0.00001f)
            candidates[candidatesCount++]=corners[i];
    }
    for(int i=0;i<12;++i)
    {
        const int a=edges[i][0],b=edges[i][1];
        if((h[a]>0.0f && h[b]<0.0f) ||
           (h[b]>0.0f && h[a]<0.0f))
        {
            float t=h[a]/(h[a]-h[b]);
            candidates[candidatesCount++]=add(corners[a],
                                             mul(sub(corners[b],corners[a]),t));
        }
    }
    if(candidatesCount<3)return 0;
    b3ShadowVec2 projected[20];
    int n=0;
    for(int i=0;i<candidatesCount;++i)
    {
        b3ShadowVec3 delta=sub(candidates[i],surface->origin);
        float height=dot(delta,surface->normal);
        float t=height>0.0f ? height/(-dn):0.0f;
        b3ShadowVec3 p=add(candidates[i],mul(lightRay,t));
        b3ShadowVec3 relative=sub(p,surface->origin);
        float u=dot(relative,surface->axisU);
        float v=dot(relative,surface->axisV);
        if(!isfinite(u)||!isfinite(v))continue;
        projected[n++]=(b3ShadowVec2){u,v};
    }
    if(n<3)return 0;
    b3ShadowVec2 a[B3_SHADOW_MAX_VERTS]={0};
    b3ShadowVec2 b[B3_SHADOW_MAX_VERTS]={0};
    n=convexHull(projected,n,a);
    if(n<3)return 0;
    n=clip(a,n,b,0,-surface->halfU,0);
    n=clip(b,n,a,0, surface->halfU,1);
    n=clip(a,n,b,1,-surface->halfV,0);
    n=clip(b,n,a,1, surface->halfV,1);
    if(n<3)return 0;
    memcpy(result->vertices,a,(unsigned)n*sizeof(a[0]));
    result->count=n;
    return n;
}

b3ShadowVec3 b3ShadowOnReceiver(b3ShadowVec2 p,
                                const b3ShadowReceiver* surface,
                                float normalBias)
{
    return add(add(add(surface->origin,mul(surface->axisU,p.u)),
                   mul(surface->axisV,p.v)),mul(surface->normal,normalBias));
}
