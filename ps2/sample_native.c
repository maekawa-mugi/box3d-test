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
#if defined(BOX3D_GS_KAWASE_SHADOWS)
#include "ee_clock.h"
#endif
#if defined(BOX3D_GS_RECEIVER_SHADOWS)
#include "shadow_project.h"
#endif
#if defined(BOX3D_GS_PCSS_SHADOWS)
#include "shadow_penumbra.h"
#endif

#include <kernel.h>
#if defined(BOX3D_GS_PROGRESSIVE)
#include "../shared/human.h"
#endif
#if defined(BOX3D_PS2_PAD_NAV)
#include <sifrpc.h>
#include <loadfile.h>
#include <libpad.h>
#endif
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#if defined(BOX3D_GS_PROGRESSIVE)
#define SCENE_COUNT 8
#else
#define SCENE_COUNT 3
#endif
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
    // Physical capsule endpoints are often offset within their body.
    // Render the visual cuboid centered on the capsule, not body origin.
    b3GsPoint3 localCenter;
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
#if defined(BOX3D_GS_PROGRESSIVE)
static Human s_ragdoll;
#endif
#if defined(BOX3D_PS2_PAD_NAV)
static _Alignas(64) unsigned char s_padBuffer[256];
static uint16_t s_padHeld;
static int s_padReady;
static void pad_nav_init(void)
{
    sceSifInitRpc(0);
    if(SifLoadModule("rom0:SIO2MAN",0,NULL)<0)
        puts("BOX3D_PAD_WARN,SIO2MAN_load_failed");
    if(SifLoadModule("rom0:PADMAN",0,NULL)<0)
        puts("BOX3D_PAD_WARN,PADMAN_load_failed");
    if(padInit(0)==0 || padPortOpen(0,0,s_padBuffer)==0)
    {
        puts("BOX3D_PAD_WARN,pad_not_available_auto_cycle_only");
        return;
    }
    s_padReady=1;
    puts("BOX3D_PAD_READY,LEFT=previous,RIGHT=next,edge_triggered=1");
}
// Return -1 / +1 on a new digital directional press only; no auto
// repeat while held. Unplugging or initial probe never blocks physics.
static int pad_nav_poll(void)
{
    if(!s_padReady)return 0;
    int state=padGetState(0,0);
    if(state!=PAD_STATE_STABLE && state!=PAD_STATE_FINDCTP1)
    {
        s_padHeld=0;
        return 0;
    }
    struct padButtonStatus buttons;
    if(!padRead(0,0,&buttons))return 0;
    uint16_t held=(uint16_t)(~buttons.btns)&(PAD_LEFT|PAD_RIGHT);
    uint16_t pressed=(uint16_t)(held&~s_padHeld);
    s_padHeld=held;
    if(pressed&PAD_LEFT)return -1;
    if(pressed&PAD_RIGHT)return +1;
    return 0;
}
#endif
#if defined(BOX3D_GS_PROGRESSIVE)
static unsigned s_culledFaces, s_emittedFaces;
#endif

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
    // Reuse the backing array across scenes without stale local offsets.
    for(int i=0;i<MAX_BOXES;++i)s_scene.objects[i].localCenter=pt(0,0,0);
#if defined(BOX3D_GS_PROGRESSIVE)
    s_ragdoll=(Human){0};
#endif
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
    else if(kind==2)
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

#if defined(BOX3D_GS_PROGRESSIVE)
    else if(kind==3)
    {
        // Restitution/Varying, original sample supports selectable BOX
        // mode. 40 bodies with restitution 0..1 and low friction.
        add_box(0,pt(0,-1,0),pt(50,1,50),identity,noSpin,
                0.0f,175,182,184);
        for(int i=0;i<40;++i)
        {
            b3BodyDef bd=b3DefaultBodyDef();
            bd.type=b3_dynamicBody;
            bd.position=(b3Pos){-39.0f+2.0f*(float)i,40,0};
            b3BodyId id=b3CreateBody(s_scene.world,&bd);
            b3BoxHull hull=b3MakeBoxHull(0.5f,0.5f,0.5f);
            b3ShapeDef sd=b3DefaultShapeDef();
            sd.density=1.0f;
            sd.baseMaterial.friction=0;
            sd.baseMaterial.restitution=(float)i/39.0f;
            b3CreateHullShape(id,&sd,&hull.base);
            SceneBody* body=&s_scene.objects[s_scene.count++];
            body->id=id;body->half=pt(0.5f,0.5f,0.5f);
            body->r=(uint8_t)(170+i/2);
            body->g=(uint8_t)(185-i/3);body->b=210;
        }
        s_scene.camera=b3GsLookAt(pt(0,55,80),pt(0,20,0),225);
        puts("BOX3D_SAMPLE_SCENE,Restitution/Varying,boxes=40,restitution=0..1");
    }
    else if(kind==4)
    {
        // Shapes/Slide Twist: actual 20-degree static sloping platform
        // and the dynamic cube's oriented spin from the desktop sample.
        add_box(0,pt(0,-1,0),pt(50,1,50),identity,noSpin,
                0.7f,175,182,184);
        b3Quat tilt=b3MakeQuatFromAxisAngle((b3Vec3){1,0,0},
                                            20.0f*0.01745329252f);
        add_box(0,pt(0,4,0),pt(10,0.5f,10),tilt,noSpin,
                0.6f,161,170,181);
        b3Vec3 spin=b3RotateVector(tilt,(b3Vec3){0,25,0});
        add_box(1,pt(0,5,0),pt(1,0.5f,1),tilt,spin,
                0.3f,200,165,142);
        s_scene.camera=b3GsLookAt(pt(-12,13,25),pt(0,5,0),215);
        puts("BOX3D_SAMPLE_SCENE,Shapes/SlideTwist,boxes=1");
    }
    else if(kind==5)
    {
        // Shapes/Conveyor Belt: actual static shape tangent velocity.
        add_box(0,pt(0,-1,0),pt(20,1,20),identity,noSpin,
                0.7f,175,182,184);
        b3BodyDef bd=b3DefaultBodyDef();
        bd.position=(b3Pos){-5,5,0};
        bd.rotation=b3MakeQuatFromAxisAngle((b3Vec3){0,1,0},0.2f);
        b3BodyId platform=b3CreateBody(s_scene.world,&bd);
        b3ShapeDef material=b3DefaultShapeDef();
        material.baseMaterial.friction=0.8f;
        material.baseMaterial.tangentVelocity=(b3Vec3){2,0,0};
        b3BoxHull hull=b3MakeBoxHull(10,0.25f,2);
        b3CreateHullShape(platform,&material,&hull.base);
        SceneBody* support=&s_scene.objects[s_scene.count++];
        support->id=platform;support->half=pt(10,0.25f,2);
        support->r=165;support->g=172;support->b=192;
        for(int i=0;i<5;++i)
            add_box(1,pt(-10.0f+2.0f*(float)i,7,0),
                    pt(0.5f,0.5f,0.5f),identity,noSpin,
                    0.8f,197,178,152);
        s_scene.camera=b3GsLookAt(pt(0,16,35),pt(-3,3,0),235);
        puts("BOX3D_SAMPLE_SCENE,Shapes/ConveyorBelt,boxes=5");
    }
    else if(kind==6)
    {
        // World/Far Stack, default floating-point origin preset.
        add_box(0,pt(0,-1,0),pt(12,1,12),identity,noSpin,
                0.7f,175,182,184);
        for(int i=0;i<6;++i)
            add_box(1,pt((i&1)?0.02f:-0.02f,0.5f+(float)i,0),
                    pt(0.5f,0.5f,0.5f),identity,noSpin,
                    0.7f,182,194,215);
        s_scene.camera=b3GsLookAt(pt(0,9,16),pt(0,2,0),225);
        puts("BOX3D_SAMPLE_SCENE,World/FarStack,column=6,offset_km=0");
    }
    else if(kind==7)
    {
        // Actual Ragdoll/Box setup from samples/sample_ragdoll.cpp:
        // CreateHuman makes twelve Box3D capsule rigid bodies connected
        // with revolute/spherical joints and collision-filter joints.
        // The GS uses inexpensive cuboid proxies bound to the true
        // physics bone bodies (not an unjointed falling-box imitation).
        add_box(0,pt(0,-1,0),pt(20,1,20),identity,noSpin,
                0.7f,175,182,184);
        CreateHuman(&s_ragdoll,s_scene.world,(b3Pos){0,2,0},
                    5.0f,1.0f,0.7f,1,NULL,false);
        // Capsule-axis-aligned cuboid proxies for the original Human
        // bone shapes. Each localCenter follows the capsule endpoint
        // midpoint from shared/human.c, including off-axis legs/arms.
        static const b3GsPoint3 boneHalf[bone_count]={
            {0.19f,0.15f,0.15f}, // pelvis
            {0.17f,0.15f,0.16f}, // spine 01
            {0.20f,0.17f,0.18f}, // spine 03
            {0.12f,0.18f,0.12f}, // head
            {0.26f,0.11f,0.10f}, // left thigh, local X
            {0.31f,0.11f,0.10f}, // left calf+foot, local X
            {0.26f,0.11f,0.10f}, // right thigh
            {0.31f,0.11f,0.10f}, // right calf+foot
            {0.23f,0.085f,0.085f}, // left arm
            {0.21f,0.075f,0.075f},
            {0.23f,0.085f,0.085f}, // right arm
            {0.21f,0.075f,0.075f}
        };
        static const b3GsPoint3 boneCenter[bone_count]={
            {0,0,0}, {0,-0.147f,0.006f}, {0,-0.117f,-0.04f},
            {0,-0.15f,0}, {-0.214f,0,0}, {-0.22f,0,0},
            {0.214f,0,0}, {0.22f,0,0},
            {0.14f,0,-0.008f}, {0.17f,0,-0.009f},
            {-0.14f,0,0.008f}, {-0.17f,0,0.009f}
        };
        for(int i=0;i<bone_count;++i)
        {
            SceneBody* body=&s_scene.objects[s_scene.count++];
            body->id=s_ragdoll.bones[i].bodyId;
            body->half=boneHalf[i];
            body->localCenter=boneCenter[i];
            // Shirt, denim pants and skin-style palette.
            if(i==bone_pelvis||i==bone_thigh_l||i==bone_calf_l||
               i==bone_thigh_r||i==bone_calf_r)
            {
                body->r=81;body->g=135;body->b=184;
            }
            else if(i==bone_neck||i>=bone_upper_arm_l)
            {
                body->r=210;body->g=175;body->b=143;
            }
            else
            {
                body->r=76;body->g=173;body->b=162;
            }
        }
        s_scene.camera=b3GsLookAt(pt(4.2f,5.0f,7.0f),pt(0,1.4f,0),260);
        puts("BOX3D_SAMPLE_SCENE,Ragdoll/Box,bones=12,joints=11,origin=shared/human.c");
    }
#endif
#if defined(BOX3D_GS_PROGRESSIVE)
    // Desktop sample cameras use 320x224 reference focal parameters.
    // Double focal length on the high-res display to preserve FOV.
    s_scene.camera.focal*=2.0f;
#endif
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
        local.x+=body->localCenter.x;
        local.y+=body->localCenter.y;
        local.z+=body->localCenter.z;
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
#if defined(BOX3D_GS_PROGRESSIVE)
            // EE back-face culling before 2-triangle emission.
            // About half of convex box faces are not visible from a
            // camera outside the hull. Retains GS Z for occlusion.
            b3Pos center=b3Body_GetPosition(body->id);
            b3Vec3 toEye={
                s_scene.camera.eye.x-(float)center.x,
                s_scene.camera.eye.y-(float)center.y,
                s_scene.camera.eye.z-(float)center.z
            };
            if(n.x*toEye.x+n.y*toEye.y+n.z*toEye.z<=0.0f)
            {
                ++s_culledFaces;
                continue;
            }
            ++s_emittedFaces;
#endif
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
        local.x+=body->localCenter.x;
        local.y+=body->localCenter.y;
        local.z+=body->localCenter.z;
        b3Vec3 world=b3RotateVector(q,local);
        out[v]=(b3ShadowVec3){
            (float)origin.x+world.x,
            (float)origin.y+world.y,
            (float)origin.z+world.z
        };
    }
}
static void render_receiver_shadows(int receiver)
{
    unsigned emittedTriangles=0;
    unsigned projectedHulls=0;
    // Same directional-light orientation as the face lighting.
    const b3ShadowVec3 direction={-0.44f,-0.82f,-0.36f};

    // A receiver is already drawn. Shadows are clipped to its finite
    // shape. In the PCSS-inspired mode each individual caster gets a
    // separate mask/filter pass, so penumbra radius can vary per body.
#if !defined(BOX3D_GS_PCSS_SHADOWS)
    b3GsShadowBegin();
#endif
    {
        b3ShadowReceiver plane=makeReceiver(receiver);
#if defined(BOX3D_GS_PCSS_SHADOWS)
        int minRadius=5, maxRadius=0;
#endif
        for(int caster=1;caster<s_scene.count;++caster)
        {
            if(caster==receiver)continue;
            b3ShadowVec3 corners[8];
            b3ShadowPolygon poly;
            cornersForBody(&s_scene.objects[caster],corners);
            if(b3ShadowProjectBox(corners,direction,&plane,&poly)<3)
                continue;

            // A single convex silhouette fan avoids intra-caster
            // alpha overdraw. PCSS mode filters every caster separately
            // to preserve the contact-hardening property.
            ++projectedHulls;
#if defined(BOX3D_GS_PCSS_SHADOWS)
            // Distance between the nearest box corner and its receiving
            // plane. Contact/penetration -> radius 1, floating -> wider.
            float gap=1000000.0f;
            for(int v=0;v<8;++v)
            {
                const b3ShadowVec3 p=corners[v];
                float height=(p.x-plane.origin.x)*plane.normal.x+
                             (p.y-plane.origin.y)*plane.normal.y+
                             (p.z-plane.origin.z)*plane.normal.z;
                if(height<gap)gap=height;
            }
            if(gap<0.0f)gap=0.0f;

            // Projected pixel scale accounts for camera zoom/distance,
            // not just world-space height. The 0.08 scale is a tunable
            // approximation of finite-area emitter angular diameter.
            float cameraZ=
                (plane.origin.x-s_scene.camera.eye.x)*s_scene.camera.forward.x+
                (plane.origin.y-s_scene.camera.eye.y)*s_scene.camera.forward.y+
                (plane.origin.z-s_scene.camera.eye.z)*s_scene.camera.forward.z;
            if(cameraZ<2.0f)cameraZ=2.0f;
            int radius=b3Ps2ShadowRadius(gap,s_scene.camera.focal/cameraZ);
            if(radius<minRadius)minRadius=radius;
            if(radius>maxRadius)maxRadius=radius;

            // Compute a conservative, projected on-screen bbox. The GS
            // scissor limits both offscreen mask clears and 9/21 sprite
            // taps to this shadow, avoiding 40 full-screen PCF passes.
            int x0=10000,y0=10000,x1=-10000,y1=-10000,ok=1;
            for(int v=0;v<poly.count;++v)
            {
                b3ShadowVec3 world=b3ShadowOnReceiver(
                    poly.vertices[v],&plane,0.045f);
                int x,y;
                if(!b3GsProjectScreen(s_scene.camera,toPoint3(world),&x,&y))
                {
                    ok=0;break;
                }
                if(x<x0)x0=x;
                if(x>x1)x1=x;
                if(y<y0)y0=y;
                if(y>y1)y1=y;
            }
            if(!ok)
            {
                // Near-plane-clipped silhouettes can extend beyond
                // original projections; use full-screen safe fallback.
                x0=0;y0=0;x1=B3_PS2_GS_W-1;y1=B3_PS2_GS_H-1;
            }
            b3GsShadowSetPenumbra(radius,x0,y0,x1,y1);
            b3GsShadowBegin();
#endif
            b3ShadowVec3 w0=b3ShadowOnReceiver(poly.vertices[0],&plane,0.045f);
            b3GsPoint3 p0=toPoint3(w0);
            for(int t=1;t+1<poly.count;++t)
            {
                b3GsPoint3 p1=toPoint3(
                    b3ShadowOnReceiver(poly.vertices[t],&plane,0.045f));
                b3GsPoint3 p2=toPoint3(
                    b3ShadowOnReceiver(poly.vertices[t+1],&plane,0.045f));
                b3GsTriangle3(s_scene.camera,p0,p1,p2,39,48,56);
                ++emittedTriangles;
            }
#if defined(BOX3D_GS_PCSS_SHADOWS)
            b3GsShadowEnd();
#endif
        }
#if defined(BOX3D_GS_PCSS_SHADOWS)
        if(s_frame%240==0)
            printf("BOX3D_PCSS_RECEIVER,scene=%d,receiver=%d,casters=%u,radius_min=%d,radius_max=%d\n",
                   s_scene.kind,receiver,projectedHulls,
                   projectedHulls?minRadius:0,maxRadius);
#endif
    }
#if !defined(BOX3D_GS_PCSS_SHADOWS)
    b3GsShadowEnd();
#endif
    // Console diagnostics: prove complete silhouettes and both potential
    // receivers were actually processed, independently of screenshots.
    if(s_frame%240==0)
        printf("BOX3D_SHADOW_RECEIVERS,scene=%d,receiver=%d,hulls=%u,triangles=%u\n",
               s_scene.kind,receiver,projectedHulls,emittedTriangles);
}
#endif

static void render_official_hud(void)
{
#if defined(BOX3D_GS_PROGRESSIVE)
    const char* sceneNames[]={"BOX STACK","INCLINED PLANE",
                             "SPINNING BOOKS","RESTITUTION",
                             "SLIDE TWIST","CONVEYOR BELT","FAR STACK",
                             "RAGDOLL BOX"};
#else
    const char* sceneNames[]={"BOX STACK","INCLINED PLANE","SPINNING BOOKS"};
#endif
    // A restrained top-left scene panel approximating ImGui sample
    // information, but all glyph pixels are native GS sprites.
    b3GsOverlayBegin();
#if defined(BOX3D_GS_PROGRESSIVE)
    b3GsOverlayPanel(14,14,310,104);
    b3GsOverlayText(28,24,"BOX3D   PS2",239,245,247);
    b3GsOverlayText(28,53,sceneNames[s_scene.kind],189,214,223);
#else
#if defined(BOX3D_GS_KAWASE_SHADOWS)
    b3GsOverlayPanel(7,7,152,49);
#else
    b3GsOverlayPanel(7,7,152,37);
#endif
    b3GsOverlayText(14,12,"BOX3D   PS2",239,245,247);
    b3GsOverlayText(14,25,sceneNames[s_scene.kind],189,214,223);
#endif
#if defined(BOX3D_GS_PCSS_SHADOWS)
    b3GsOverlayText(14,37,"GS PCSS HEIGHT",188,226,240);
    b3GsOverlayPanel(206,184,106,30);
    b3GsOverlayText(214,195,"PCSS 3D",250,236,130);
#elif defined(BOX3D_GS_PCF_SHADOWS)
#if defined(BOX3D_GS_PCF_4TAP)
#if defined(BOX3D_GS_PROGRESSIVE)
    b3GsOverlayText(28,82,"PCF 4 TAP 480P",188,226,240);
#else
    b3GsOverlayText(14,37,"GS PCF 4 TAP",188,226,240);
#endif
#else
    b3GsOverlayText(14,37,"GS PCF 3X3",188,226,240);
#endif
    // Duplicate the build identity in the lower-right safe area.
    // Test screenshots have sometimes been cropped so that the entire
    // upper part of the HUD is outside the captured frame, leaving
    // only the dark panel's lower edge visible. A second label also
    // differentiates PCF from Kawase even when only the floor is shown.
#if defined(BOX3D_GS_PROGRESSIVE)
    b3GsOverlayPanel(B3_PS2_GS_W-210,B3_PS2_GS_H-75,198,58);
    b3GsOverlayText(B3_PS2_GS_W-192,B3_PS2_GS_H-56,
                    "480P 4TAP",250,236,130);
#else
    b3GsOverlayPanel(219,184,93,30);
    b3GsOverlayText(227,195,"PCF 3X3",250,236,130);
#endif
#elif defined(BOX3D_GS_KAWASE_SHADOWS)
    b3GsOverlayText(14,37,"SOFT GS KAWASE",188,226,240);
#endif
}
#endif
static void render(void)
{
#if defined(BOX3D_GS_PROGRESSIVE)
    s_culledFaces=0;
    s_emittedFaces=0;
#endif
    b3GsBegin();
#if defined(BOX3D_GS_OFFICIAL)
    // Frame layers: atmosphere -> textured-look floor -> contact shadows
    // -> lit rigid bodies -> scene panel. Native GS GIF A+D throughout.
    b3GsSkyGradient();
    render_ground();
#if defined(BOX3D_GS_RECEIVER_SHADOWS)
    // Layered receiver rendering prevents Z-tested shadow holes:
    // 1. floor shadows are composited onto the floor
    // 2. opaque ramp covers any floor shadows behind it
    // 3. ramp-local shadows land on the now-visible ramp
    // 4. dynamics are drawn last and occlude their own floor shadows.
    render_receiver_shadows(0);
    int firstDynamic=1;
    if(s_scene.kind==1
#if defined(BOX3D_GS_PROGRESSIVE)
       ||s_scene.kind==4||s_scene.kind==5
#endif
      )
    {
        draw_box(&s_scene.objects[1]); // static inclined plane / platform
        render_receiver_shadows(1);
        firstDynamic=2;
    }
    for(int i=firstDynamic;i<s_scene.count;++i)
        draw_box(&s_scene.objects[i]);
#if defined(BOX3D_GS_KAWASE_DEBUG)
    b3GsKawaseDebugOverlay();
#endif
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
#if defined(BOX3D_GS_PCSS_SHADOWS)
    puts("BOX3D_SHADOW_FILTER,PCSS_INSPIRED_PER_CASTER,source=320x224,radius=1..5,taps=9_or_21");
#elif defined(BOX3D_GS_PCF_SHADOWS)
    puts("BOX3D_SHADOW_FILTER,PCF_MASK_3X3,source=320x224,radius=1px,weight_sum=96");
    puts("BOX3D_PCF_ACTIVE,mode=3x3_fullres_mask,HUD=top_left+bottom_right");
#endif
    b3SetAssertFcn(fail_assert);
    b3SetLogFcn(log_message);
    if(!b3GsInit())
    {
        puts("BOX3D_SAMPLE_FAIL,gs_initialization");
        for(;;) SleepThread();
    }
#if defined(BOX3D_PS2_PAD_NAV)
    pad_nav_init();
#endif
    scene_create(0);
    for(;;)
    {
        int direction=0;
#if defined(BOX3D_PS2_PAD_NAV)
        direction=pad_nav_poll();
#endif
        if(direction!=0)
        {
            int newKind=(s_scene.kind+SCENE_COUNT+direction)%SCENE_COUNT;
            b3DestroyWorld(s_scene.world);
            scene_create(newKind);
            s_frame=0; // full duration after manually changing scenes
            printf("BOX3D_PAD_SCENE,dir=%d,scene=%d\n",direction,newKind);
        }
        else if(s_frame>0 && s_frame%SCENE_FRAMES==0)
        {
            b3DestroyWorld(s_scene.world);
            scene_create((s_scene.kind+1)%SCENE_COUNT);
            s_frame=0;
        }
#if defined(BOX3D_GS_KAWASE_SHADOWS)
        uint32_t frameStart=b3Ps2ReadCpuCycles();
#endif
        b3World_Step(s_scene.world,1.0f/60.0f,4);
#if defined(BOX3D_GS_KAWASE_SHADOWS)
        uint32_t physicsEnd=b3Ps2ReadCpuCycles();
#endif
        render();
#if defined(BOX3D_GS_PROGRESSIVE)
        if(s_frame%120==0)
            printf("BOX3D_GS_GEOMETRY,frame=%lu,culled_faces=%u,emitted_faces=%u,gif_qwords=%u,dma_submits=1\n",
                   (unsigned long)s_frame,s_culledFaces,s_emittedFaces,
                   b3GsGetLastGifQwords());
#endif
#if defined(BOX3D_GS_KAWASE_SHADOWS)
        uint32_t frameEnd=b3Ps2ReadCpuCycles();
        if(s_frame%120==0)
        {
#if defined(BOX3D_GS_PCSS_SHADOWS)
            const char* name="BOX3D_PCSS_FRAME";
#else
            const char* name="BOX3D_KAWASE_FRAME";
#endif
            printf("%s,scene=%d,frame=%lu,physics_ticks=%lu,render_vsync_ticks=%lu,total_ticks=%lu,target60_ticks=4915200\n",
                   name,s_scene.kind,(unsigned long)s_frame,
                   (unsigned long)b3Ps2DeltaCpuCycles(frameStart,physicsEnd),
                   (unsigned long)b3Ps2DeltaCpuCycles(physicsEnd,frameEnd),
                   (unsigned long)b3Ps2DeltaCpuCycles(frameStart,frameEnd));
        }
#endif
        ++s_frame;
#if defined(BOX3D_GS_PCF_SHADOWS) && !defined(BOX3D_GS_PCSS_SHADOWS)
        // Independent runtime proof for testers capturing the GS display
        // without the upper HUD visible in screenshots.
        if(s_frame%240==0)
            printf("BOX3D_PCF_FRAME,frame=%lu,scene=%d,pcf=1\n",
                   (unsigned long)s_frame,s_scene.kind);
#endif
        if(s_frame%240==0)
        {
            b3Pos p=b3Body_GetPosition(s_scene.objects[s_scene.count-1].id);
            printf("BOX3D_SAMPLE_FRAME,scene=%d,frame=%lu,bodies=%d,last_y=%.4f\n",
                   s_scene.kind,(unsigned long)s_frame,s_scene.count,(double)p.y);
        }
    }
}
