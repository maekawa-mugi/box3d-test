// SPDX-License-Identifier: MIT
// First native-GS port of Box3D sample scenarios, using C17/PS2SDK:
//   Stacking / Box Stack (12 cubes; original uses 40)
//   Shapes / Inclined Plane (5 cubes with varying friction)
//   Bodies / Spinning Books (original 3 angular-velocity axes)
// Scene construction mirrors samples/sample_stacking.cpp,
// samples/sample_shapes.cpp and samples/sample_bodies.cpp. Original
// desktop C++ UI, Sokol shaders and ImGui are intentionally omitted.
#include "box3d/box3d.h"
#include "box3d/math_functions.h"
#include "gs_native.h"
#if defined(BOX3D_GS_RECEIVER_SHADOWS)
#include "shadow_project.h"
#endif

#include <kernel.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define SCENE_COUNT 3
#if defined(BOX3D_GS_OFFICIAL)
#define MAX_BOXES 48
#else
#define MAX_BOXES 20
#endif
#define SCENE_FRAMES 600

typedef struct SceneBody
{
    b3BodyId id;
    b3GsPoint3 half;
    uint8_t r,g,b;
} SceneBody;

typedef struct Scene
{
    b3WorldId world;
    SceneBody objects[MAX_BOXES];
    int count;
    int kind;
    b3GsCamera camera;
} Scene;

static Scene s_scene;
static uint32_t s_frame;

static b3GsPoint3 pt(float x,float y,float z)
{
    b3GsPoint3 result={x,y,z};
    return result;
}
static b3GsPoint3 fromVec3(b3Vec3 v){return pt(v.x,v.y,v.z);}

static int fail_assert(const char* expr,const char* path,int line)
{
    printf("BOX3D_SAMPLE_ASSERT,%s,%s,%d\n",expr,path,line);
    for(;;) SleepThread();
    return 0;
}
static void log_message(const char* message)
{
    printf("BOX3D_SAMPLE_LOG,%s\n",message);
}

static b3BodyId add_box(int dynamic,b3GsPoint3 position,b3GsPoint3 half,
                         b3Quat rotation,b3Vec3 spin,float friction,
                         uint8_t r,uint8_t g,uint8_t b)
{
    b3BodyDef def=b3DefaultBodyDef();
    def.type=dynamic?b3_dynamicBody:b3_staticBody;
    def.position=(b3Pos){position.x,position.y,position.z};
    def.rotation=rotation;
    def.angularVelocity=spin;
    b3BodyId id=b3CreateBody(s_scene.world,&def);
    b3BoxHull shape=b3MakeBoxHull(half.x,half.y,half.z);
    b3ShapeDef sd=b3DefaultShapeDef();
    sd.density=1.0f;
    sd.baseMaterial.friction=friction;
    sd.baseMaterial.rollingResistance=0.05f;
    b3CreateHullShape(id,&sd,&shape.base);

    // Keep the visible geometry bound to the exact body ID, not to an
    // approximate simulated position. Quaternions are read each frame.
    if(s_scene.count<MAX_BOXES)
    {
        SceneBody* out=&s_scene.objects[s_scene.count++];
        out->id=id;
        out->half=half;
        out->r=r;out->g=g;out->b=b;
    }
    return id;
}

static void scene_create(int kind)
{
    s_scene.count=0;
    s_scene.kind=kind;
    b3WorldDef wd=b3DefaultWorldDef();
    wd.workerCount=1;
    s_scene.world=b3CreateWorld(&wd);
    const b3Quat identity={ {0,0,0},1 };
    const b3Vec3 noSpin={0,0,0};

    if(kind==0)
    {
        // Stacking / Box Stack: first 12 of the desktop sample's
        // vertically stacked 40 boxes. Same half-extent and spacing.
#if defined(BOX3D_GS_OFFICIAL)
        // Match the upstream Box Stack's 40m ground and full 40-box setup.
        add_box(0,pt(0,-1,0),pt(40,1,40),identity,noSpin,0.7f,65,95,112);
        const int stackCount=40;
#else
        add_box(0,pt(0,-1,0),pt(20,1,20),identity,noSpin,0.7f,65,95,112);
        const int stackCount=12;
#endif
        for(int i=0;i<stackCount;++i)
        {
            float y=0.75f+1.25f*(float)i;
#if defined(BOX3D_GS_OFFICIAL)
            // Official debug adapter shades dynamic geometry using physically
            // lit material colors, rather than the saturated rainbow palette.
            add_box(1,pt(0,y,0),pt(0.5f,0.5f,0.5f),identity,noSpin,0.7f,
                    (uint8_t)(173+5*(i%3)),(uint8_t)(188+3*(i%4)),
                    (uint8_t)(203+3*(i%4)));
#else
            add_box(1,pt(0,y,0),pt(0.5f,0.5f,0.5f),identity,noSpin,0.7f,
                    (uint8_t)(90+i*10),(uint8_t)(190-i*6),230);
#endif
        }
#if defined(BOX3D_GS_OFFICIAL)
        // Upstream SetView(40deg yaw, 15deg pitch, radius 50,
        // target=(0,20,0)). Approximate with the same spherical pose.
        s_scene.camera=b3GsLookAt(pt(31.05f,32.94f,36.99f),
                                  pt(0,20,0),200);
        puts("BOX3D_SAMPLE_SCENE,Stacking/BoxStack,boxes=40,look=upstream");
#else
        s_scene.camera=b3GsLookAt(pt(18,13,22),pt(0,6.5f,0),200);
        puts("BOX3D_SAMPLE_SCENE,Stacking/BoxStack,boxes=12");
#endif
    }
    else if(kind==1)
    {
        // Shapes / Inclined Plane: static sloping platform with five
        // independently friction-tuned dynamic cubes.
        add_box(0,pt(0,-1,0),pt(25,1,25),identity,noSpin,0.7f,65,95,112);
        b3Quat incline=b3MakeQuatFromAxisAngle((b3Vec3){1,0,0},0.60f);
#if defined(BOX3D_GS_OFFICIAL)
        add_box(0,pt(0,5,-1),pt(8,0.4f,5),incline,noSpin,1.0f,169,178,190);
#else
        add_box(0,pt(0,5,-1),pt(8,0.4f,5),incline,noSpin,1.0f,116,156,190);
#endif
        for(int i=0;i<5;++i)
        {
            add_box(1,pt(-5.0f+2.5f*i,11.3f,-4.0f),pt(0.5f,0.5f,0.5f),
                    identity,noSpin,0.04f*(float)((i+1)*(i+1)),
#if defined(BOX3D_GS_OFFICIAL)
                    (uint8_t)(190+8*i),(uint8_t)(175+2*i),
                    (uint8_t)(148+4*i));
#else
                    240,(uint8_t)(210-24*i),(uint8_t)(70+24*i));
#endif
        }
        s_scene.camera=b3GsLookAt(pt(18,13,20),pt(0,4,0),195);
        puts("BOX3D_SAMPLE_SCENE,Shapes/InclinedPlane,boxes=5");
    }
    else
    {
        // Bodies / Spinning Books: rotate each thin box about a different
        // axis without gravity, matching the desktop sample.
        add_box(0,pt(0,-1,0),pt(20,1,20),identity,noSpin,0.7f,65,95,112);
        const b3Vec3 spins[3]={
            {5.0f,0.01f,0.01f},{0.01f,5.0f,0.01f},{0.01f,0.01f,-5.0f}
        };
        for(int i=0;i<3;++i)
        {
            b3BodyDef def=b3DefaultBodyDef();
            def.type=b3_dynamicBody;
            def.gravityScale=0.0f;
            def.position=(b3Pos){(float)(i-1)*3.0f,2.0f,0.0f};
            def.angularVelocity=spins[i];
            b3BodyId id=b3CreateBody(s_scene.world,&def);
            b3BoxHull hull=b3MakeBoxHull(0.35f,0.08f,0.5f);
            b3ShapeDef sd=b3DefaultShapeDef();
            b3CreateHullShape(id,&sd,&hull.base);
            SceneBody* out=&s_scene.objects[s_scene.count++];
            out->id=id;out->half=pt(0.35f,0.08f,0.5f);
#if defined(BOX3D_GS_OFFICIAL)
            out->r=184;out->g=(uint8_t)(185+8*i);out->b=212;
#else
            out->r=240;out->g=(uint8_t)(130+40*i);out->b=90;
#endif
        }
        s_scene.camera=b3GsLookAt(pt(10,7,13),pt(0,2,0),215);
        puts("BOX3D_SAMPLE_SCENE,Bodies/SpinningBooks,books=3");
    }
}

static void draw_box(const SceneBody* body)
{
    const int endpoints[12][2]={
        {0,1},{0,2},{0,4},{1,3},{1,5},{2,3},
        {2,6},{3,7},{4,5},{4,6},{5,7},{6,7}
    };
    b3Pos origin=b3Body_GetPosition(body->id);
    b3Quat q=b3Body_GetRotation(body->id);
    b3GsPoint3 verts[8];
    for(int i=0;i<8;++i)
    {
        b3Vec3 local={
            (i&1)?body->half.x:-body->half.x,
            (i&2)?body->half.y:-body->half.y,
            (i&4)?body->half.z:-body->half.z
        };
        b3Vec3 rotated=b3RotateVector(q,local);
        verts[i]=pt((float)origin.x+rotated.x,
                    (float)origin.y+rotated.y,
                    (float)origin.z+rotated.z);
    }
#if defined(BOX3D_GS_SOLID)
    // Draw solid box sides for each dynamic body and the inclined plane.
    // The large static ground remains a wireframe to preserve the grid.
    if(body != &s_scene.objects[0])
    {
        static const int faces[6][4]={
            {0,2,3,1}, {4,5,7,6}, {0,4,6,2},
            {1,3,7,5}, {0,1,5,4}, {2,6,7,3}
        };
        static const b3Vec3 faceNormals[6]={
            {0,0,-1},{0,0,1},{-1,0,0},{1,0,0},{0,-1,0},{0,1,0}
        };
        const b3Vec3 light={0.44f,0.82f,0.36f};
        for(int face=0;face<6;++face)
        {
            b3Vec3 n=b3RotateVector(q,faceNormals[face]);
            float diffuse=n.x*light.x+n.y*light.y+n.z*light.z;
            if(diffuse<0)diffuse=0;
#if defined(BOX3D_GS_OFFICIAL)
            // Matte daylight: ambient fill plus directional Lambert light.
            // Original renderer adds PBR/IBL, this is GS flat shading.
            float shade=0.44f+0.54f*diffuse;
#else
            float shade=0.30f+0.67f*diffuse;
#endif
            const uint8_t rr=(uint8_t)((float)body->r*shade);
            const uint8_t gg=(uint8_t)((float)body->g*shade);
            const uint8_t bb=(uint8_t)((float)body->b*shade);
            const int* quad=faces[face];
            b3GsTriangle3(s_scene.camera,verts[quad[0]],verts[quad[1]],
                          verts[quad[2]],rr,gg,bb);
            b3GsTriangle3(s_scene.camera,verts[quad[0]],verts[quad[2]],
                          verts[quad[3]],rr,gg,bb);
        }
    }
#endif
    for(int j=0;j<12;++j)
    {
#if defined(BOX3D_GS_OFFICIAL)
        // Subtle bevel-like silhouettes, avoid saturated debug edges.
        b3GsLine3(s_scene.camera,verts[endpoints[j][0]],verts[endpoints[j][1]],
                  (uint8_t)((unsigned)body->r*66/100),
                  (uint8_t)((unsigned)body->g*66/100),
                  (uint8_t)((unsigned)body->b*66/100));
#else
        b3GsLine3(s_scene.camera,verts[endpoints[j][0]],verts[endpoints[j][1]],
                  body->r,body->g,body->b);
#endif
    }
}


// Official renderer approximation: soft chalk gray checker ground under the
// same Box3D ground hull. Drawn as actual GS triangles with native Z test.
#if defined(BOX3D_GS_OFFICIAL)
static void render_ground(void)
{
    const float cell=4.0f;
    const b3GsCamera camera=s_scene.camera;
    // 12x12 tiles cover the central 48x48m range. No textures or gsKit.
    for(int zi=-6;zi<6;++zi)
    {
        for(int xi=-6;xi<6;++xi)
        {
            float x=(float)xi*cell, z=(float)zi*cell;
            int checker=(xi+zi)&1;
            const uint8_t r=checker?177:183;
            const uint8_t g=checker?181:187;
            const uint8_t b=checker?182:188;
            b3GsPoint3 a=pt(x,0,z),u=pt(x+cell,0,z);
            b3GsPoint3 v=pt(x+cell,0,z+cell),w=pt(x,0,z+cell);
            b3GsTriangle3(camera,a,u,v,r,g,b);
            b3GsTriangle3(camera,a,v,w,r,g,b);
        }
    }
    // Delicate procedural grid, like the desktop ground-box material.
    // Slight Y bias prevents it from flickering with the filled surface.
    for(int i=-24;i<=24;i+=4)
    {
        b3GsLine3(camera,pt((float)i,0.012f,-24),
                  pt((float)i,0.012f,24),145,154,158);
        b3GsLine3(camera,pt(-24,0.012f,(float)i),
                  pt(24,0.012f,(float)i),145,154,158);
    }
}

// Hard-edged directional planar proxy for the desktop cascade shadow
// system. It follows the actual Box3D quaternion, with no extra collision
// geometry or expensive shadow map. Only objects near enough to the floor
// cast this first-pass shadow.
static void render_shadows(void)
{
    const float sx=0.52f, sz=0.42f;
    b3GsShadowBegin();
    for(int i=1;i<s_scene.count;++i)
    {
        const SceneBody* body=&s_scene.objects[i];
        b3Pos origin=b3Body_GetPosition(body->id);
        if(origin.y>24.0f || origin.y < -5.0f) continue;
        b3Quat q=b3Body_GetRotation(body->id);
        const float h=fmaxf((float)origin.y,0.0f);
        b3GsPoint3 xy[4];
        for(int j=0;j<4;++j)
        {
            b3Vec3 local={
                (j&1)?body->half.x:-body->half.x,
                -body->half.y,
                (j&2)?body->half.z:-body->half.z
            };
            b3Vec3 rotated=b3RotateVector(q,local);
            // Move each projected corner away from the directional sun.
            // The offset includes its actual world-space height.
            float y=fmaxf((float)origin.y+rotated.y,0.0f);
            xy[j]=pt((float)origin.x+rotated.x-sx*y,
                     0.022f,
                     (float)origin.z+rotated.z-sz*y);
        }
        b3GsTriangle3(s_scene.camera,xy[0],xy[1],xy[3],48,60,69);
        b3GsTriangle3(s_scene.camera,xy[0],xy[3],xy[2],48,60,69);
    }
    b3GsShadowEnd();
}


#if defined(BOX3D_GS_RECEIVER_SHADOWS)
// The physical receiver surfaces are the floor, and optionally the
// rotated incline. Shadow projection and convex clipping are portable
// and covered by ps2/shadow_project_test.c.
static b3ShadowVec3 shadowVec(b3Vec3 v)
{
    return (b3ShadowVec3){v.x,v.y,v.z};
}
static b3ShadowVec3 shadowPt(b3Pos p)
{
    return (b3ShadowVec3){(float)p.x,(float)p.y,(float)p.z};
}
static b3GsPoint3 toPoint3(b3ShadowVec3 v)
{
    return pt(v.x,v.y,v.z);
}

static b3ShadowReceiver makeReceiver(int index)
{
    const SceneBody* body=&s_scene.objects[index];
    b3Quat rotation=b3Body_GetRotation(body->id);
    b3ShadowReceiver p;
    p.axisU=shadowVec(b3RotateVector(rotation,(b3Vec3){1,0,0}));
    p.axisV=shadowVec(b3RotateVector(rotation,(b3Vec3){0,0,1}));
    p.normal=shadowVec(b3RotateVector(rotation,(b3Vec3){0,1,0}));
    p.origin=shadowPt(b3Body_GetPosition(body->id));
    // Ground top and inclined platform upper surface are at +half.y.
    p.origin.x+=p.normal.x*body->half.y;
    p.origin.y+=p.normal.y*body->half.y;
    p.origin.z+=p.normal.z*body->half.y;
    p.halfU=body->half.x;
    p.halfV=body->half.z;
    // Checkered floor is only generated on [-24,24] in world X/Z.
    if(index==0)
    {
        p.halfU=fminf(p.halfU,24.0f);
        p.halfV=fminf(p.halfV,24.0f);
    }
    return p;
}
static void cornersForBody(const SceneBody* body,b3ShadowVec3 out[8])
{
    b3Pos origin=b3Body_GetPosition(body->id);
    b3Quat q=b3Body_GetRotation(body->id);
    for(int v=0;v<8;++v)
    {
        b3Vec3 local={
            (v&1)?body->half.x:-body->half.x,
            (v&2)?body->half.y:-body->half.y,
            (v&4)?body->half.z:-body->half.z
        };
        b3Vec3 world=b3RotateVector(q,local);
        out[v]=(b3ShadowVec3){
            (float)origin.x+world.x,
            (float)origin.y+world.y,
            (float)origin.z+world.z
        };
    }
}
static void render_receiver_shadows(void)
{
    // Same directional-light orientation as the face lighting.
    const b3ShadowVec3 direction={-0.44f,-0.82f,-0.36f};
    const int receiverCount=s_scene.kind==1?2:1;

    // Render shadows AFTER opaque bodies, using the actual receiver Z
    // already in GS ZBUF. Shadow pass sets ZMSK=1 and source-over alpha;
    // only the visible top face of each receiver may be darkened.
    b3GsShadowBegin();
    for(int receiver=0;receiver<receiverCount;++receiver)
    {
        b3ShadowReceiver plane=makeReceiver(receiver);
        for(int caster=1;caster<s_scene.count;++caster)
        {
            if(caster==receiver)continue;
            b3ShadowVec3 corners[8];
            b3ShadowPolygon poly;
            cornersForBody(&s_scene.objects[caster],corners);
            if(b3ShadowProjectBox(corners,direction,&plane,&poly)<3)
                continue;

            // A single convex silhouette fan, rather than overlapping
            // top/bottom-face quads. This prevents alpha over-darkening
            // within one box's shadow and missing rotated silhouettes.
            b3ShadowVec3 w0=b3ShadowOnReceiver(poly.vertices[0],&plane,0.045f);
            b3GsPoint3 p0=toPoint3(w0);
            for(int t=1;t+1<poly.count;++t)
            {
                b3GsPoint3 p1=toPoint3(
                    b3ShadowOnReceiver(poly.vertices[t],&plane,0.045f));
                b3GsPoint3 p2=toPoint3(
                    b3ShadowOnReceiver(poly.vertices[t+1],&plane,0.045f));
                b3GsTriangle3(s_scene.camera,p0,p1,p2,39,48,56);
            }
        }
    }
    b3GsShadowEnd();
}
#endif

static void render_official_hud(void)
{
    const char* sceneNames[]={"BOX STACK","INCLINED PLANE","SPINNING BOOKS"};
    // A restrained top-left scene panel approximating ImGui sample
    // information, but all glyph pixels are native GS sprites.
    b3GsOverlayBegin();
    b3GsOverlayPanel(7,7,152,37);
    b3GsOverlayText(14,12,"BOX3D   PS2",239,245,247);
    b3GsOverlayText(14,25,sceneNames[s_scene.kind],189,214,223);
}
#endif
static void render(void)
{
    b3GsBegin();
#if defined(BOX3D_GS_OFFICIAL)
    // Frame layers: atmosphere -> textured-look floor -> contact shadows
    // -> lit rigid bodies -> scene panel. Native GS GIF A+D throughout.
    b3GsSkyGradient();
    render_ground();
#if defined(BOX3D_GS_RECEIVER_SHADOWS)
    for(int i=1;i<s_scene.count;++i) draw_box(&s_scene.objects[i]);
    render_receiver_shadows();
#else
    render_shadows();
    for(int i=1;i<s_scene.count;++i) draw_box(&s_scene.objects[i]);
#endif
    render_official_hud();
#else
    // Shared ground-plane axes and grid, useful as a motion reference.
    for(int i=-10;i<=10;i+=2)
    {
        b3GsLine3(s_scene.camera,pt((float)i,0,-10),pt((float)i,0,10),36,55,66);
        b3GsLine3(s_scene.camera,pt(-10,0,(float)i),pt(10,0,(float)i),36,55,66);
    }
    b3GsLine3(s_scene.camera,pt(-10,0,0),pt(10,0,0),193,64,64);
    b3GsLine3(s_scene.camera,pt(0,0,-10),pt(0,0,10),81,117,207);
    b3GsLine3(s_scene.camera,pt(0,0,0),pt(0,4,0),79,203,123);
    for(int i=0;i<s_scene.count;++i) draw_box(&s_scene.objects[i]);
#endif
    b3GsPresent();
}

int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    puts("BOX3D_SAMPLE_START,gs=native_gif,physics=Box3D");
    b3SetAssertFcn(fail_assert);
    b3SetLogFcn(log_message);
    if(!b3GsInit())
    {
        puts("BOX3D_SAMPLE_FAIL,gs_initialization");
        for(;;) SleepThread();
    }
    scene_create(0);
    for(;;)
    {
        if(s_frame>0 && s_frame%SCENE_FRAMES==0)
        {
            b3DestroyWorld(s_scene.world);
            scene_create((s_scene.kind+1)%SCENE_COUNT);
        }
        b3World_Step(s_scene.world,1.0f/60.0f,4);
        render();
        ++s_frame;
        if(s_frame%240==0)
        {
            b3Pos p=b3Body_GetPosition(s_scene.objects[s_scene.count-1].id);
            printf("BOX3D_SAMPLE_FRAME,scene=%d,frame=%lu,bodies=%d,last_y=%.4f\n",
                   s_scene.kind,(unsigned long)s_frame,s_scene.count,(double)p.y);
        }
    }
}
