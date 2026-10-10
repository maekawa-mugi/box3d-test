// SPDX-License-Identifier: MIT
#include "shadow_project.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static b3ShadowVec3 v(float x,float y,float z){return (b3ShadowVec3){x,y,z};}
static void cube(b3ShadowVec3 out[8],float x,float y,float z)
{
    for(int i=0;i<8;i++)
        out[i]=v(x+((i&1)?1.0f:-1.0f),
                 y+((i&2)?1.0f:-1.0f),
                 z+((i&4)?1.0f:-1.0f));
}
static b3ShadowReceiver floorAt(float half)
{
    return (b3ShadowReceiver){
        .origin={0,0,0},.axisU={1,0,0},.axisV={0,0,1},
        .normal={0,1,0},.halfU=half,.halfV=half};
}
static float polygonArea(const b3ShadowPolygon* p)
{
    float area=0.0f;
    for(int i=0;i<p->count;++i)
    {
        b3ShadowVec2 a=p->vertices[i],b=p->vertices[(i+1)%p->count];
        area+=a.u*b.v-a.v*b.u;
    }
    return area*0.5f;
}
int main(void)
{
    b3ShadowVec3 points[8];
    b3ShadowPolygon shadow;
    b3ShadowReceiver floor=floorAt(32.0f);
    cube(points,0,2,0);
    // Vertical light projects both the top and bottom box faces into
    // one complete quad. This must be nondegenerate and correctly wound.
    assert(b3ShadowProjectBox(points,v(0,-1,0),&floor,&shadow)==4);
    assert(fabsf(polygonArea(&shadow)-4.0f)<0.0001f);
    // Oblique light + elevated cube: silhouette includes *all* vertices,
    // enlarging the footprint beyond the bottom-face-only projection.
    assert(b3ShadowProjectBox(points,v(-0.5f,-1,0),&floor,&shadow)>=4);
    assert(polygonArea(&shadow)>5.9f);
    // Receiver-clipping check.
    b3ShadowReceiver small=floorAt(0.5f);
    assert(b3ShadowProjectBox(points,v(0,-1,0),&small,&shadow)==4);
    assert(fabsf(polygonArea(&shadow)-1.0f)<0.0001f);
    cube(points,40,2,40);
    assert(b3ShadowProjectBox(points,v(0,-1,0),&small,&shadow)==0);
    // No cast toward a receiver facing away from the light.
    assert(b3ShadowProjectBox(points,v(0,1,0),&floor,&shadow)==0);
    // A tilted surface accepts box shadows, respects its finite bounds.
    cube(points,0,8,0);
    const float sn=0.5646424734f,cs=0.8253356149f;
    b3ShadowReceiver ramp={
        .origin={0,5,0},.axisU={1,0,0},.axisV={0,-sn,cs},
        .normal={0,cs,sn},.halfU=8,.halfV=5
    };
    assert(b3ShadowProjectBox(points,v(-0.44f,-0.82f,-0.36f),&ramp,&shadow)>=3);
    for(int i=0;i<shadow.count;++i)
    {
        assert(fabsf(shadow.vertices[i].u)<=8.0001f);
        assert(fabsf(shadow.vertices[i].v)<=5.0001f);
        b3ShadowVec3 position=b3ShadowOnReceiver(shadow.vertices[i],&ramp,0.03f);
        float dist=(position.x-ramp.origin.x)*ramp.normal.x +
                   (position.y-ramp.origin.y)*ramp.normal.y +
                   (position.z-ramp.origin.z)*ramp.normal.z;
        assert(fabsf(dist-0.03f)<0.0001f);
    }
    puts("PASS: PS2 convex shadow projection, clipped ground and tilted ramp");
    return 0;
}
