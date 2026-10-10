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
#if defined(BOX3D_PS2_UNIFIED)
#include "unified_demo.h"
#include "../src/ps2_vu0.h"
#include "vu1_boxes.h"
#include "vu0_micro_solver.h"
#endif
#if defined(BOX3D_GS_KAWASE_SHADOWS)
#include "ee_clock.h"
#endif
#if defined(BOX3D_GS_RECEIVER_SHADOWS)
#include "shadow_project.h"
#endif
#if defined(BOX3D_GS_PCSS_SHADOWS)
#include "shadow_penumbra.h"
#endif
#if defined(BOX3D_PS2_UNIFIED)
#include "shadow_group.h"
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
#define SCENE_COUNT B3_PS2_DEMO_SCENES
#else
#define SCENE_COUNT 3
#endif
#if defined(BOX3D_GS_OFFICIAL)
#if defined(BOX3D_PS2_UNIFIED)
#define MAX_BOXES B3_PS2_DEMO_MAX_BODIES
#else
#define MAX_BOXES 48
#endif
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
#if defined(BOX3D_PS2_UNIFIED)
    b3Pos cachedPosition;
    b3Quat cachedRotation;
    b3GsPoint3 cachedVertices[8];
    b3Vec3 cachedAxes[3];
#endif
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
// Unified 480p build caches Box3D poses, hulls and shadow projections
// once per frame to avoid repeated EE work under 256/400-body stress.
#if defined(BOX3D_GS_PROGRESSIVE)
static Human s_ragdoll;
#endif
#if defined(BOX3D_PS2_PAD_NAV)
static _Alignas(64) unsigned char s_padBuffer[256];
static uint16_t s_padHeld;
static int s_padReady;
#if defined(BOX3D_PS2_UNIFIED)
// User-selectable physics A/B modes; change between completed steps only.
static int s_simSubsteps=4;
static int s_stressSleeping=0; // Default: force all bodies awake in BENCH scenes
static int s_vu0Mode=0;
static int s_vu0MicroStage; // 0=none,1=armed,2=upload,3=verify,4=diff
static int s_vu0MicroResult; // most recent requested hardware stage
static int s_shadowVisualMode=0; // 0=PCSS,1=PCF,2=HARD,3=OFF,4=LEGACY
// Valid only for static-camera, fully sleeping BENCH scenes.
static int s_shadowBenchCacheValid;
static int s_vu1Ready;
// No VU1 initialization at program startup: keep the proven EE path
// usable even if MPG/MMIO verification fails on the target emulator.
// TRIANGLE stages: EE -> ACK -> PATH3 -> INIT ARM -> INIT (MPG DMA) ->
// VERIFY ARM -> VERIFY (EE readback) -> VIF -> BOOT -> VUONLY -> 1/8/400.
static int s_vu1Budget=0;
static unsigned s_trianglePressCount;
static int s_vu1InitAttempted;
static int s_vu1VerifyAttempted;
static int s_vu1VifAttempted;
static int s_vu1BootAttempted;
static int s_lastVu1Boxes;
#endif
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
    uint16_t held=(uint16_t)(~buttons.btns)&(PAD_LEFT|PAD_RIGHT
#if defined(BOX3D_PS2_UNIFIED)
                                             |PAD_UP|PAD_DOWN|PAD_SQUARE|PAD_TRIANGLE|PAD_CROSS|PAD_SELECT
#endif
                                             );
    uint16_t pressed=(uint16_t)(held&~s_padHeld);
    s_padHeld=held;
#if defined(BOX3D_PS2_UNIFIED)
    if((pressed&PAD_CROSS) && s_scene.kind>=8)
    {
        // Toggle only at frame boundaries; no scene recreation, preserve
        // VU1 selection, geometry, shadows and 4-step simulation mode.
        s_stressSleeping=!s_stressSleeping;
        b3World_EnableSleeping(s_scene.world,s_stressSleeping!=0);
        printf("BOX3D_PHYS_SLEEP,scene=%d,enabled=%d,awake=%d,substeps=%d\n",
               s_scene.kind,s_stressSleeping,
               b3World_GetAwakeBodyCount(s_scene.world),s_simSubsteps);
    }
    // SELECT is deliberately separate from DOWN's VU0 macro-mode A/B.
    // It is the ONLY entry to the new VU0 microcode/MMIO experiment,
    // always between completed Box3D world steps.
    if((pressed&PAD_SELECT) && s_scene.kind>=8)
    {
        s_vu0MicroStage=(s_vu0MicroStage+1)%5;
        s_vu0MicroResult=0;
        if(s_vu0MicroStage==1)
        {
            puts("BOX3D_VU0_PHASE,armed,no_mmio=1");
            s_vu0MicroResult=1;
        }
        else if(s_vu0MicroStage==2)
            s_vu0MicroResult=b3Vu0MicroUpload();
        else if(s_vu0MicroStage==3)
            s_vu0MicroResult=b3Vu0MicroVerify();
        else if(s_vu0MicroStage==4)
            s_vu0MicroResult=b3Vu0MicroDifferentialTest();
        printf("BOX3D_PAD_SELECT,stage=%d,result=%d,status=%s\n",
               s_vu0MicroStage,s_vu0MicroResult,b3Vu0MicroStatus());
    }
    if(pressed&PAD_UP)
    {
        s_simSubsteps=(s_simSubsteps==4)?2:4;
        printf("BOX3D_PHYS_MODE,substeps=%d,vu0=%d\n",
               s_simSubsteps,s_vu0Mode);
    }
    if(pressed&PAD_DOWN)
    {
        // Three physics paths: original EE scalar (0), legacy COP2
        // macro (5), actual VU0 micro VCALLMS (6). Mode 6 must never
        // become available until the independent 64-lane DIFF passed.
        int next=(s_vu0Mode==0)?5:(s_vu0Mode==5)?6:0;
        if(next==6 &&
           (s_scene.kind<8 || s_vu0MicroStage!=4 ||
            !s_vu0MicroResult || !b3Vu0MicroIsReady()))
        {
            puts("BOX3D_VU0_LIVE_BLOCKED,reason=select_diff_required");
            next=0;
        }
        s_vu0Mode=next;
        b3Ps2SetVU0Enabled(s_vu0Mode);
        printf("BOX3D_PHYS_MODE,substeps=%d,vu0=%d,live_micro=%d\n",
               s_simSubsteps,s_vu0Mode,s_vu0Mode==6);
    }
    if(pressed&PAD_TRIANGLE)
    {
        s_vu1Budget=(s_vu1Budget==0)?-2:
                    (s_vu1Budget==-2)?-3:
                    (s_vu1Budget==-3)?-6:
                    (s_vu1Budget==-6)?-7:
                    (s_vu1Budget==-7)?-8:
                    (s_vu1Budget==-8)?-9:
                    (s_vu1Budget==-9)?-4:
                    (s_vu1Budget==-4)?-5:
                    (s_vu1Budget==-5)?-1:
                    (s_vu1Budget==-1)?1:
                    (s_vu1Budget==1)?8:
                    (s_vu1Budget==8)?B3_VU1_MAX_BOXES:0;
        if(s_vu1Budget==-7)s_vu1InitAttempted=0;
        if(s_vu1Budget==-9)s_vu1VerifyAttempted=0;
        if(s_vu1Budget==-4)s_vu1VifAttempted=0;
        if(s_vu1Budget==-5)s_vu1BootAttempted=0;
        ++s_trianglePressCount;
        printf("BOX3D_PAD_TRIANGLE,press=%u,stage=%d,ready=%d,status=%s,scene=%d\n",
               s_trianglePressCount,s_vu1Budget,s_vu1Ready,
               b3Vu1BoxesStatus(),s_scene.kind);
    }
    if(pressed&PAD_SQUARE)
    {
        s_shadowVisualMode=(s_shadowVisualMode+1)%5;
        // Geometry may have changed while shadow drawing was disabled.
        s_shadowBenchCacheValid=0;
        b3GsSetShadowFilterMode(s_shadowVisualMode==3?0:
                                s_shadowVisualMode==4?3:s_shadowVisualMode);
        printf("BOX3D_SHADOW_MODE,mode=%d,label=%s\n",
               s_shadowVisualMode,
               (const char*[]){"PCSS","PCF","HARD","OFF","LEGACY"}[s_shadowVisualMode]);
    }
#endif
    if(pressed&PAD_LEFT)return -1;
    if(pressed&PAD_RIGHT)return +1;
    return 0;
}
#endif
#if defined(BOX3D_GS_PROGRESSIVE)
static unsigned s_culledFaces, s_emittedFaces;
#if defined(BOX3D_PS2_UNIFIED)
static uint32_t s_beforeGsSubmitTicks;
static float s_lastPhysicsMs,s_lastBuildMs,s_lastGpuMs,s_lastVsyncMs;
static float s_lastWorkPct;
static uint64_t s_sumPhysicsTicks,s_sumBuildTicks,s_sumGsTicks;
static uint64_t s_sumShadowTicks,s_sumBoxTicks;
static uint32_t s_frameShadowTicks,s_frameBoxTicks;
static double s_profilePairsMs,s_profileCollideMs,s_profileSolveMs;
static double s_profileConstraintsMs;
static uint32_t s_maxWorkTicks;
static int s_perfSamples;
#endif
#endif

static b3GsPoint3 pt(float x,float y,float z)
{
    b3GsPoint3 result={x,y,z};
    return result;
}
#if defined(BOX3D_PS2_UNIFIED)
static void cacheSceneBodyPoses(void)
{
    for(int i=0;i<s_scene.count;++i)
    {
        SceneBody* body=&s_scene.objects[i];
        body->cachedPosition=b3Body_GetPosition(body->id);
        body->cachedRotation=b3Body_GetRotation(body->id);
        // Three quaternion rotations replace eight vertex rotations
        // plus six face-normal rotations. The same basis serves both.
        const b3Vec3 basis[3]={
            {1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}
        };
        for(int a=0;a<3;++a)
            body->cachedAxes[a]=b3RotateVector(body->cachedRotation,basis[a]);
        const b3Vec3* ax=body->cachedAxes;
        const float centerX=(float)body->cachedPosition.x+
             body->localCenter.x*ax[0].x+
             body->localCenter.y*ax[1].x+
             body->localCenter.z*ax[2].x;
        const float centerY=(float)body->cachedPosition.y+
             body->localCenter.x*ax[0].y+
             body->localCenter.y*ax[1].y+
             body->localCenter.z*ax[2].y;
        const float centerZ=(float)body->cachedPosition.z+
             body->localCenter.x*ax[0].z+
             body->localCenter.y*ax[1].z+
             body->localCenter.z*ax[2].z;
        const b3GsPoint3 dx=pt(body->half.x*ax[0].x,
                               body->half.x*ax[0].y,
                               body->half.x*ax[0].z);
        const b3GsPoint3 dy=pt(body->half.y*ax[1].x,
                               body->half.y*ax[1].y,
                               body->half.y*ax[1].z);
        const b3GsPoint3 dz=pt(body->half.z*ax[2].x,
                               body->half.z*ax[2].y,
                               body->half.z*ax[2].z);
        for(int v=0;v<8;++v)
        {
            float x=(v&1)?dx.x:-dx.x;
            float y=(v&1)?dx.y:-dx.y;
            float z=(v&1)?dx.z:-dx.z;
            x+=(v&2)?dy.x:-dy.x;
            y+=(v&2)?dy.y:-dy.y;
            z+=(v&2)?dy.z:-dy.z;
            x+=(v&4)?dz.x:-dz.x;
            y+=(v&4)?dz.y:-dz.y;
            z+=(v&4)?dz.z:-dz.z;
            body->cachedVertices[v]=pt(centerX+x,centerY+y,centerZ+z);
        }
    }
}
#endif
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
#if defined(BOX3D_PS2_UNIFIED)
    b3GsInvalidateGroundCache();
    s_shadowBenchCacheValid=0;
#endif
    // Reuse the backing array across scenes without stale local offsets.
    for(int i=0;i<MAX_BOXES;++i)s_scene.objects[i].localCenter=pt(0,0,0);
#if defined(BOX3D_GS_PROGRESSIVE)
    s_ragdoll=(Human){0};
#endif
    b3WorldDef wd=b3DefaultWorldDef();
    wd.workerCount=1;
#if defined(BOX3D_PS2_UNIFIED)
    // Sustained physics stress should not disappear after the boxes
    // settle and all island sleepers go idle. Force awake for tiers
    // 64/144/256/400 only; regular demos keep natural sleeping.
    if(kind>=8)wd.enableSleep=s_stressSleeping!=0;
    s_sumPhysicsTicks=0;
    s_sumBuildTicks=0;
    s_sumGsTicks=0;
    s_sumShadowTicks=0;
    s_sumBoxTicks=0;
    s_profilePairsMs=0.0;
    s_profileCollideMs=0.0;
    s_profileSolveMs=0.0;
    s_profileConstraintsMs=0.0;
    s_maxWorkTicks=0;
    s_perfSamples=0;
#endif
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
#if defined(BOX3D_PS2_UNIFIED)
    else if(kind>=8 && kind<=11)
    {
        // Benchmark/Many Bodies stress presets, inspired by upstream
        // Benchmark/Large Pyramid and Wide Pyramid workloads. Four
        // progressively denser dynamic-box grids, all using the real
        // Box3D solver and the SAME native GS/PCSS render path.
        int side=b3Ps2DemoStressSide(kind-8);
        int count=side*side; // 64, 144, 256, 400
        add_box(0,pt(0,-1,0),pt(40,1,40),identity,noSpin,
                0.65f,173,183,188);
        float spacing=1.04f;
        for(int z=0;z<side;++z)
        for(int x=0;x<side;++x)
        {
            float px=((float)x-0.5f*(float)(side-1))*spacing;
            float pz=((float)z-0.5f*(float)(side-1))*spacing;
            // Stagger the drop heights to maintain contacts and
            // broadphase movement instead of a perfectly static grid.
            float py=1.0f+0.65f*(float)((x*7+z*11)%8);
            add_box(1,pt(px,py,pz),pt(0.48f,0.48f,0.48f),
                    identity,noSpin,0.65f,
                    (uint8_t)(148+(x*13)%64),
                    (uint8_t)(155+(z*11)%54),
                    (uint8_t)(184+(x+z)%34));
        }
        s_scene.camera=b3GsLookAt(pt(29,31,38),pt(0,1.9f,0),300);
        printf("BOX3D_STRESS_SCENE,boxes=%d,side=%d,real_dynamic_bodies=%d\n",
               count,side,count);
    }
#endif
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
#if defined(BOX3D_PS2_UNIFIED)
    b3Pos origin=body->cachedPosition;
    b3Quat q=body->cachedRotation;
    const b3GsPoint3* verts=body->cachedVertices;
#else
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
#endif
#if defined(BOX3D_GS_SOLID)
    // Draw solid box sides for each dynamic body and the inclined plane.
    // The large static ground remains a wireframe to preserve the grid.
    if(body != &s_scene.objects[0])
    {
#if defined(BOX3D_PS2_UNIFIED)
        b3GsProjectedVertex screen[8];
        int allProjected=b3GsProjectHull(s_scene.camera,verts,screen);
#endif
        static const int faces[6][4]={
            {0,2,3,1}, {4,5,7,6}, {0,4,6,2},
            {1,3,7,5}, {0,1,5,4}, {2,6,7,3}
        };
        static const b3Vec3 faceNormals[6]={
            {0,0,-1},{0,0,1},{-1,0,0},{1,0,0},{0,-1,0},{0,1,0}
        };
        const b3Vec3 light={0.44f,0.82f,0.36f};
#if defined(BOX3D_GS_PROGRESSIVE)
        const b3Vec3 toEye={
            s_scene.camera.eye.x-(float)origin.x,
            s_scene.camera.eye.y-(float)origin.y,
            s_scene.camera.eye.z-(float)origin.z
        };
#endif
        for(int face=0;face<6;++face)
        {
#if defined(BOX3D_PS2_UNIFIED)
            const int axis=(face<2)?2:(face<4?0:1);
            const float sign=(face&1)?1.0f:-1.0f;
            const b3Vec3* nAxis=&body->cachedAxes[axis];
            b3Vec3 n={sign*nAxis->x,sign*nAxis->y,sign*nAxis->z};
#else
            b3Vec3 n=b3RotateVector(q,faceNormals[face]);
#endif
#if defined(BOX3D_GS_PROGRESSIVE)
            // EE back-face culling before 2-triangle emission.
            // About half of convex box faces are not visible from a
            // camera outside the hull. Retains GS Z for occlusion.
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
#if defined(BOX3D_PS2_UNIFIED)
            if(allProjected)
            {
                b3GsTriangleProjected(&screen[quad[0]],&screen[quad[1]],
                                      &screen[quad[2]],rr,gg,bb);
                b3GsTriangleProjected(&screen[quad[0]],&screen[quad[2]],
                                      &screen[quad[3]],rr,gg,bb);
                continue;
            }
#endif
            // Robust near-plane clipped fallback for an exceptional box.
            b3GsTriangle3(s_scene.camera,verts[quad[0]],verts[quad[1]],
                          verts[quad[2]],rr,gg,bb);
            b3GsTriangle3(s_scene.camera,verts[quad[0]],verts[quad[2]],
                          verts[quad[3]],rr,gg,bb);
        }
    }
#endif
    // The stress scenes intentionally omit tiny edge strokes to
    // preserve GS GIF packet capacity for the actual solid geometry
    // and expensive Box3D contact solver at 400 bodies. Silhouette
    // AA1 remains enabled for all regular scenes.
#if defined(BOX3D_PS2_UNIFIED)
    if(s_scene.kind>=8)return;
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
#if defined(BOX3D_PS2_UNIFIED)
    // Exact same transformed vertices as the solid box triangles.
    for(int v=0;v<8;++v)
        out[v]=(b3ShadowVec3){body->cachedVertices[v].x,
                              body->cachedVertices[v].y,
                              body->cachedVertices[v].z};
#else
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
#endif
}
#if defined(BOX3D_PS2_UNIFIED)
// Real clipped silhouettes from the first PCSS radius-classification
// pass, retained until all GS shadow-mask groups have been drawn.
// Avoid repeating Box3D body transforms + convex hull/clip work for
// every accepted box. Static EE BSS prevents large stack frames.
static b3ShadowPolygon s_shadowProjectedCache[MAX_BOXES];
static b3GsProjectedVertex s_shadowScreenCache[MAX_BOXES][B3_SHADOW_MAX_VERTS];
static unsigned char s_shadowScreenValid[MAX_BOXES];
static b3Ps2ShadowBounds s_shadowBounds[MAX_BOXES];
static b3Ps2ShadowBounds s_shadowUnions[MAX_BOXES];
static int s_shadowParents[MAX_BOXES];
static int s_shadowFirst[MAX_BOXES],s_shadowNext[MAX_BOXES];
static unsigned s_shadowCachedProjected;
static int s_shadowCachedGroups,s_shadowCachedMinRadius,s_shadowCachedMaxRadius;
#endif
#if defined(BOX3D_PS2_UNIFIED)
// One GS subtractive composite per connected component of overlapping
// PCSS penumbrae. Previously each radius bucket was subtracted in
// isolation, so one silhouette produced TWO (or more) dark copies when
// near and far radius groups touched. Union in screen space first.
static void render_receiver_shadows(int receiver)
{
    const b3ShadowVec3 direction={-0.44f,-0.82f,-0.36f};
    b3ShadowReceiver plane=makeReceiver(receiver);
    unsigned projected=0,triangles=0;
    int minRadius=5,maxRadius=0;
    int groups=0;
    // Only when all BENCH bodies sleep can the frame's shadow
    // polygons and union groups be copied from the previous frame.
    // Camera and light are fixed for these stress-scene pages.
    const int cacheHit=receiver==0 && s_scene.kind>=8 &&
        s_stressSleeping && s_shadowBenchCacheValid &&
        b3World_GetAwakeBodyCount(s_scene.world)==0;
    float cameraZ=
        (plane.origin.x-s_scene.camera.eye.x)*s_scene.camera.forward.x+
        (plane.origin.y-s_scene.camera.eye.y)*s_scene.camera.forward.y+
        (plane.origin.z-s_scene.camera.eye.z)*s_scene.camera.forward.z;
    if(cameraZ<2.0f)cameraZ=2.0f;
    float pxPerMeter=s_scene.camera.focal/cameraZ;

    // Align the blur's long axis with the light projected onto the
    // receiving plane. This is an approximation, not depth-map PCSS.
    float dn=direction.x*plane.normal.x+
             direction.y*plane.normal.y+
             direction.z*plane.normal.z;
    b3ShadowVec3 tangent={
        direction.x-dn*plane.normal.x,
        direction.y-dn*plane.normal.y,
        direction.z-dn*plane.normal.z
    };
    b3ShadowVec3 tangentTip={
        plane.origin.x+tangent.x,
        plane.origin.y+tangent.y,
        plane.origin.z+tangent.z
    };
    int majorX=1,ax,ay,bx,by;
    if(b3GsProjectScreen(s_scene.camera,toPoint3(plane.origin),&ax,&ay) &&
       b3GsProjectScreen(s_scene.camera,toPoint3(tangentTip),&bx,&by))
    {
        int dx=bx-ax,dy=by-ay;
        if(dx<0)dx=-dx;
        if(dy<0)dy=-dy;
        majorX=(dx>=dy);
    }

    if(cacheHit)
    {
        projected=s_shadowCachedProjected;
        minRadius=s_shadowCachedMinRadius;
        maxRadius=s_shadowCachedMaxRadius;
        groups=s_shadowCachedGroups;
    }
    else
    {
        // Shadows off and scene changes invalidate this cache.
        // While physics bodies remain awake, rebuild every frame.
        for(int i=0;i<s_scene.count;++i)s_shadowBounds[i].valid=0;
    for(int caster=1;caster<s_scene.count;++caster)
    {
        if(caster==receiver)continue;
        b3ShadowVec3 corners[8];
        b3ShadowPolygon poly;
        cornersForBody(&s_scene.objects[caster],corners);
        if(b3ShadowProjectBox(corners,direction,&plane,&poly)<3)
            continue;
        s_shadowProjectedCache[caster]=poly;
        s_shadowScreenValid[caster]=0;
        b3Ps2ShadowBounds* b=&s_shadowBounds[caster];
        b->valid=1;
        ++projected;
        float gap=1000000.0f;
        for(int v=0;v<8;++v)
        {
            b3ShadowVec3 p=corners[v];
            float d=(p.x-plane.origin.x)*plane.normal.x+
                    (p.y-plane.origin.y)*plane.normal.y+
                    (p.z-plane.origin.z)*plane.normal.z;
            if(d<gap)gap=d;
        }
        if(gap<0.0f)gap=0.0f;
        b->radius=(unsigned char)b3Ps2ShadowRadius(gap,pxPerMeter);
        if(b->radius<minRadius)minRadius=b->radius;
        if(b->radius>maxRadius)maxRadius=b->radius;
        b->x0=B3_PS2_GS_W;b->y0=B3_PS2_GS_H;
        b->x1=-1;b->y1=-1;
        int allProjected=1,allRasterSafe=1;
        for(int v=0;v<poly.count;++v)
        {
            b3ShadowVec3 point=
                b3ShadowOnReceiver(poly.vertices[v],&plane,0.045f);
            b3GsProjectedVertex* vtx=&s_shadowScreenCache[caster][v];
            if(!b3GsProjectPoint(s_scene.camera,toPoint3(point),vtx))
            {
                allProjected=0;
                break;
            }
            if(vtx->x16 < -4096 || vtx->y16 < -4096 ||
               vtx->x16 > (B3_PS2_GS_W+256)*16 ||
               vtx->y16 > (B3_PS2_GS_H+256)*16)
                allRasterSafe=0;
            if(vtx->x<b->x0)b->x0=vtx->x;
            if(vtx->y<b->y0)b->y0=vtx->y;
            if(vtx->x>b->x1)b->x1=vtx->x;
            if(vtx->y>b->y1)b->y1=vtx->y;
        }
        s_shadowScreenValid[caster]=(unsigned char)(allProjected&&allRasterSafe);
        if(!allProjected)
        {
            // A clipped near-plane polygon is conservatively fullscreen
            // so its filter cannot cut off at an invalid bounding box.
            b->x0=0;b->y0=0;
            b->x1=B3_PS2_GS_W-1;
            b->y1=B3_PS2_GS_H-1;
        }
    }
    groups=b3Ps2ShadowGroup(s_shadowBounds,s_scene.count,
                                s_shadowParents,s_shadowUnions);
    // Build the member list once instead of rescanning all N casters
    // for each of the potentially N disconnected components.
    for(int i=0;i<s_scene.count;++i)s_shadowFirst[i]=-1;
    for(int caster=s_scene.count-1;caster>=1;--caster)
    {
        if(!s_shadowBounds[caster].valid)continue;
        int root=s_shadowParents[caster];
        s_shadowNext[caster]=s_shadowFirst[root];
        s_shadowFirst[root]=caster;
    }
        if(receiver==0 && s_scene.kind>=8)
        {
            s_shadowCachedProjected=projected;
            s_shadowCachedMinRadius=minRadius;
            s_shadowCachedMaxRadius=maxRadius;
            s_shadowCachedGroups=groups;
            s_shadowBenchCacheValid=1;
        }
    }
    for(int root=1;root<s_scene.count;++root)
    {
        const b3Ps2ShadowBounds* group=&s_shadowUnions[root];
        if(!group->valid)continue;
        // A component keeps one chosen radius (maximum of its members),
        // so overlapping casters cannot darken the same pixel twice.
        // Separated components still keep their individual radii.
        b3GsShadowSetPenumbra(group->radius,
                             group->x0,group->y0,group->x1,group->y1,
                             majorX);
        b3GsShadowBegin();
        for(int caster=s_shadowFirst[root];caster>=0;
            caster=s_shadowNext[caster])
        {
            const b3ShadowPolygon* poly=&s_shadowProjectedCache[caster];
            if(poly->count<3)continue;
            if(s_shadowScreenValid[caster])
            {
                // No repeated screen transforms of each fan vertex.
                const b3GsProjectedVertex* screen=s_shadowScreenCache[caster];
                for(int t=1;t+1<poly->count;++t)
                {
                    b3GsTriangleProjected(&screen[0],&screen[t],
                                          &screen[t+1],39,48,56);
                    ++triangles;
                }
            }
            else
            {
                // Exceptional near-plane polygons retain robust clipping.
                b3GsPoint3 hull[B3_SHADOW_MAX_VERTS];
                for(int v=0;v<poly->count;++v)
                    hull[v]=toPoint3(
                        b3ShadowOnReceiver(poly->vertices[v],&plane,0.045f));
                for(int t=1;t+1<poly->count;++t)
                {
                    b3GsTriangle3(s_scene.camera,hull[0],hull[t],
                                  hull[t+1],39,48,56);
                    ++triangles;
                }
            }
        }
        b3GsShadowEnd();
    }
    if(s_frame%240==0)
    {
        printf("BOX3D_PCSS_UNION,scene=%d,receiver=%d,casters=%u,components=%d,composites=%d,radius_min=%d,radius_max=%d,major_axis=%c,shadow_cache=%d\n",
               s_scene.kind,receiver,projected,groups,groups,
               projected?minRadius:0,maxRadius,majorX?'X':'Y',cacheHit);
        printf("BOX3D_SHADOW_RECEIVERS,scene=%d,receiver=%d,hulls=%u,triangles=%u\n",
               s_scene.kind,receiver,projected,triangles);
    }
}
#else
static void render_receiver_shadows(int receiver)
{
    unsigned emittedTriangles=0;
    unsigned projectedHulls=0;
    const b3ShadowVec3 direction={-0.44f,-0.82f,-0.36f};
    b3ShadowReceiver plane=makeReceiver(receiver);

#if defined(BOX3D_GS_PCSS_SHADOWS)
    // A separate subtractive composite for EVERY caster used to stack
    // up to 40 dark shadows at the same floor location. The result was
    // an opaque black orb plus the hard shadow. Form the UNION of caster
    // silhouettes at each penumbra radius, then filter/composite ONCE
    // for the whole radius bucket. Thus there are at most 5 composites
    // per receiver rather than one for every box.
    uint8_t casterRadius[MAX_BOXES]={0};
    int bx0[6],by0[6],bx1[6],by1[6];
    unsigned bucketCasters[6]={0};
    int minRadius=5,maxRadius=0;
    for(int radius=1;radius<=5;++radius)
    {
        bx0[radius]=B3_PS2_GS_W;
        by0[radius]=B3_PS2_GS_H;
        bx1[radius]=-1;
        by1[radius]=-1;
    }
    float cameraZ=
        (plane.origin.x-s_scene.camera.eye.x)*s_scene.camera.forward.x+
        (plane.origin.y-s_scene.camera.eye.y)*s_scene.camera.forward.y+
        (plane.origin.z-s_scene.camera.eye.z)*s_scene.camera.forward.z;
    if(cameraZ<2.0f)cameraZ=2.0f;
    float pxPerMeter=s_scene.camera.focal/cameraZ;

    // Screen-space major axis of the light's projection along the
    // receiving plane. The penumbra's minor axis is deliberately
    // shorter to avoid a circular dark halo around narrow casters.
    float dn=direction.x*plane.normal.x+
             direction.y*plane.normal.y+
             direction.z*plane.normal.z;
    b3ShadowVec3 tangent={
        direction.x-dn*plane.normal.x,
        direction.y-dn*plane.normal.y,
        direction.z-dn*plane.normal.z
    };
    b3ShadowVec3 tangentTip={
        plane.origin.x+tangent.x,
        plane.origin.y+tangent.y,
        plane.origin.z+tangent.z
    };
    int lightMajorX=1;
    int ax,ay,bx,by;
    if(b3GsProjectScreen(s_scene.camera,toPoint3(plane.origin),&ax,&ay) &&
       b3GsProjectScreen(s_scene.camera,toPoint3(tangentTip),&bx,&by))
    {
        int dx=bx-ax,dy=by-ay;
        if(dx<0)dx=-dx;
        if(dy<0)dy=-dy;
        lightMajorX=(dx>=dy);
    }

    for(int caster=1;caster<s_scene.count;++caster)
    {
        if(caster==receiver)continue;
        b3ShadowVec3 corners[8];
        b3ShadowPolygon poly;
        cornersForBody(&s_scene.objects[caster],corners);
        if(b3ShadowProjectBox(corners,direction,&plane,&poly)<3)
            continue;
#if defined(BOX3D_PS2_UNIFIED)
        s_shadowProjectedCache[caster]=poly;
#endif

        // The actual nearest corner-plane gap, not the distance of
        // the body COM. Contact stays narrow, floating bodies soften.
        float gap=1000000.0f;
        for(int v=0;v<8;++v)
        {
            const b3ShadowVec3 p=corners[v];
            float d=(p.x-plane.origin.x)*plane.normal.x+
                    (p.y-plane.origin.y)*plane.normal.y+
                    (p.z-plane.origin.z)*plane.normal.z;
            if(d<gap)gap=d;
        }
        if(gap<0.0f)gap=0.0f;
        int radius=b3Ps2ShadowRadius(gap,pxPerMeter);
        casterRadius[caster]=(uint8_t)radius;
        ++bucketCasters[radius];
        if(radius<minRadius)minRadius=radius;
        if(radius>maxRadius)maxRadius=radius;

        // Conservative screen bounds for the UNION of all shadows in
        // this radius bucket. One enlarged scissor per group prevents
        // rectangular cutoff artifacts and limits GS fill.
        int x0=B3_PS2_GS_W,y0=B3_PS2_GS_H;
        int x1=-1,y1=-1,allProjected=1;
        for(int v=0;v<poly.count;++v)
        {
            b3ShadowVec3 vertex=
                b3ShadowOnReceiver(poly.vertices[v],&plane,0.045f);
            int x,y;
            if(!b3GsProjectScreen(s_scene.camera,toPoint3(vertex),&x,&y))
            {
                allProjected=0;
                break;
            }
            if(x<x0)x0=x;
            if(x>x1)x1=x;
            if(y<y0)y0=y;
            if(y>y1)y1=y;
        }
        if(!allProjected)
        {
            x0=0;y0=0;
            x1=B3_PS2_GS_W-1;
            y1=B3_PS2_GS_H-1;
        }
        if(x0<bx0[radius])bx0[radius]=x0;
        if(y0<by0[radius])by0[radius]=y0;
        if(x1>bx1[radius])bx1[radius]=x1;
        if(y1>by1[radius])by1[radius]=y1;
    }
    int passes=5;
#else
    int passes=1;
#endif

    for(int pass=1;pass<=passes;++pass)
    {
#if defined(BOX3D_GS_PCSS_SHADOWS)
        if(bucketCasters[pass]==0)continue;
        b3GsShadowSetPenumbra(pass,bx0[pass],by0[pass],
                               bx1[pass],by1[pass],lightMajorX);
#endif
        // Inside this pass overlapping silhouettes REPLACE the same
        // 128-valued coverage; they do not repeatedly darken the scene.
        b3GsShadowBegin();
        for(int caster=1;caster<s_scene.count;++caster)
        {
            if(caster==receiver)continue;
#if defined(BOX3D_GS_PCSS_SHADOWS)
            if(casterRadius[caster]!=pass)continue;
#endif
#if defined(BOX3D_PS2_UNIFIED)
            const b3ShadowPolygon* poly=&s_shadowProjectedCache[caster];
            if(poly->count<3)continue;
#else
            b3ShadowVec3 corners[8];
            b3ShadowPolygon localPoly;
            cornersForBody(&s_scene.objects[caster],corners);
            if(b3ShadowProjectBox(corners,direction,&plane,&localPoly)<3)
                continue;
            const b3ShadowPolygon* poly=&localPoly;
#endif
            ++projectedHulls;
            // Convert each projected convex hull point only ONCE,
            // rather than twice inside every fan triangle.
            b3GsPoint3 hull[B3_SHADOW_MAX_VERTS];
            for(int v=0;v<poly->count;++v)
                hull[v]=toPoint3(b3ShadowOnReceiver(poly->vertices[v],
                                                     &plane,0.045f));
            for(int t=1;t+1<poly->count;++t)
            {
                b3GsTriangle3(s_scene.camera,hull[0],hull[t],
                              hull[t+1],39,48,56);
                ++emittedTriangles;
            }
        }
        b3GsShadowEnd();
    }

#if defined(BOX3D_GS_PCSS_SHADOWS)
    if(s_frame%240==0)
    {
        int nonempty=b3Ps2ShadowGroupCount(bucketCasters);
        printf("BOX3D_PCSS_RECEIVER,scene=%d,receiver=%d,casters=%u,radius_min=%d,radius_max=%d,buckets=%d,composites=%d,major_axis=%c\n",
               s_scene.kind,receiver,projectedHulls,
               projectedHulls?minRadius:0,maxRadius,nonempty,nonempty,
               lightMajorX?'X':'Y');
    }
#endif
    if(s_frame%240==0)
        printf("BOX3D_SHADOW_RECEIVERS,scene=%d,receiver=%d,hulls=%u,triangles=%u\n",
               s_scene.kind,receiver,projectedHulls,emittedTriangles);
}
#endif
#endif

// Each stress-scene solid box emits 8 positions + 6 shaded faces to
// VU1 via VIF1 DMA. The VU1 microprogram projects 8 corners and
// XGKICKs the 12 unculled GS triangles. Occlusion remains GS Z-tested.
// Fallback boxes preserve the original clipped EE renderer.
#if defined(BOX3D_PS2_UNIFIED)
static int queue_vu1_box(const SceneBody* body)
{
    const b3Vec3* axes=body->cachedAxes;
    const b3Vec3 light={0.44f,0.82f,0.36f};
    uint8_t colors[6][3];
    for(int face=0;face<6;++face)
    {
        int axis=(face<2)?2:(face<4?0:1);
        float sign=(face&1)?1.0f:-1.0f;
        const b3Vec3* n=&axes[axis];
        float diffuse=sign*(n->x*light.x+n->y*light.y+n->z*light.z);
        if(diffuse<0.0f)diffuse=0.0f;
        float shade=0.44f+0.54f*diffuse;
        colors[face][0]=(uint8_t)(body->r*shade);
        colors[face][1]=(uint8_t)(body->g*shade);
        colors[face][2]=(uint8_t)(body->b*shade);
    }
    return b3Vu1BoxesAdd(body->cachedVertices,colors);
}
#endif

static void render_official_hud(void)
{
#if defined(BOX3D_GS_PROGRESSIVE)
    const char* sceneNames[]={"BOX STACK","INCLINED PLANE",
                             "SPINNING BOOKS","RESTITUTION",
                             "SLIDE TWIST","CONVEYOR BELT","FAR STACK",
                             "RAGDOLL BOX"
#if defined(BOX3D_PS2_UNIFIED)
                             ,"BENCH 64 BOX","BENCH 144 BOX",
                             "BENCH 256 BOX","BENCH 400 BOX"
#endif
                             };
#else
    const char* sceneNames[]={"BOX STACK","INCLINED PLANE","SPINNING BOOKS"};
#endif
    // A restrained top-left scene panel approximating ImGui sample
    // information, but all glyph pixels are native GS sprites.
    b3GsOverlayBegin();
#if defined(BOX3D_GS_PROGRESSIVE)
#if defined(BOX3D_PS2_UNIFIED)
    b3GsOverlayPanel(14,14,348,142);
    b3GsOverlayText(28,24,"BOX3D PS2 480P",239,245,247);
    b3GsOverlayText(28,53,sceneNames[s_scene.kind],189,214,223);
    char page[48];
    // Always show the selected diagnostic stage, even when VU1 failed
    // during initialization. Previously 'ready == 0' masked ALL
    // controller interactions as "VU1 EE SAFE".
    const int vuMode=s_vu1Budget;
    static const char* const vuStageLabel[]={
        "VU1 ACK","VU1 PATH3","VU1 VIF","VU1 INIT ARM",
        "VU1 BOOT","VU1 VUONLY","VU1 INIT","VU1 VERIFY ARM",
        "VU1 VERIFY"
    };
    if(s_scene.kind>=8)
    {
        // Sleep state is shown independently of S2/S4 and VU1 tier.
        if(vuMode<0)
            snprintf(page,sizeof(page),"PAGE %d/%d S%d TEST SL%d",
                     s_scene.kind+1,SCENE_COUNT,s_simSubsteps,s_stressSleeping);
        else
            snprintf(page,sizeof(page),"PAGE %d/%d S%d D%d SL%d",
                     s_scene.kind+1,SCENE_COUNT,s_simSubsteps,vuMode,s_stressSleeping);
    }
    else if(vuMode<0)
        snprintf(page,sizeof(page),"PAGE %d/%d PCSS S%d TEST",
                 s_scene.kind+1,SCENE_COUNT,s_simSubsteps);
    else
        snprintf(page,sizeof(page),"PAGE %d/%d PCSS S%d D%d",
                 s_scene.kind+1,SCENE_COUNT,s_simSubsteps,vuMode);

    b3GsOverlayText(28,82,page,221,233,244);
#else
    b3GsOverlayPanel(14,14,310,104);
    b3GsOverlayText(28,24,"BOX3D   PS2",239,245,247);
    b3GsOverlayText(28,53,sceneNames[s_scene.kind],189,214,223);
#endif
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
#if defined(BOX3D_PS2_UNIFIED)
    static const char* shadowLabels[]={
        "SHADOW PCSS","SHADOW PCF","SHADOW HARD","SHADOW OFF",
        "SHADOW LEGACY PCSS"
    };
    b3GsOverlayText(28,112,shadowLabels[s_shadowVisualMode],188,226,240);
    // Unambiguous controller acknowledgement. This panel is still
    // visible in the EE-only first diagnostic stage, even if a later
    // VIF1 stage deadlocks before the next Present().
    b3GsOverlayPanel(374,14,252,132);
    b3GsOverlayText(390,26,"TRIANGLE VU1R8",255,233,116);
    char padInfo[40];
    snprintf(padInfo,sizeof(padInfo),"PRESS %u",
             s_trianglePressCount);
    b3GsOverlayText(390,54,padInfo,255,255,245);
    if(vuMode==-2)b3GsOverlayText(390,82,vuStageLabel[0],255,238,132);
    else if(vuMode==-3)b3GsOverlayText(390,82,vuStageLabel[1],255,238,132);
    else if(vuMode==-4)b3GsOverlayText(390,82,vuStageLabel[2],255,238,132);
    else if(vuMode==-6)b3GsOverlayText(390,82,vuStageLabel[3],255,238,132);
    else if(vuMode==-7)b3GsOverlayText(390,82,vuStageLabel[6],255,238,132);
    else if(vuMode==-8)b3GsOverlayText(390,82,vuStageLabel[7],255,238,132);
    else if(vuMode==-9)b3GsOverlayText(390,82,vuStageLabel[8],255,238,132);
    else if(vuMode==-5)b3GsOverlayText(390,82,vuStageLabel[4],255,238,132);
    else if(vuMode==-1)b3GsOverlayText(390,82,vuStageLabel[5],255,238,132);
    else if(vuMode>0)
    {
        char modeInfo[24];
        snprintf(modeInfo,sizeof(modeInfo),"VU1 BOX %d",vuMode);
        b3GsOverlayText(390,82,modeInfo,255,238,132);
    }
    else
        b3GsOverlayText(390,82,"VU1 EE SAFE",180,244,180);
    // Never disguise an unavailable VU1 as the "EE SAFE" stage.
    // This text is visible without any button press.
    const char* status=b3Vu1BoxesStatus();
    if(!s_vu1Ready)
        b3GsOverlayText(390,110,status,255,139,130);
    else if(s_scene.kind<8)
        b3GsOverlayText(390,110,"BENCH SCENE ONLY",209,219,241);
    else
        b3GsOverlayText(390,110,status,155,243,176);
    // Display previous frame's measured utilization, NOT PCSX2 FPS
    // capped to 60 by VSync. Includes physics, GS packet build and
    // DMA/GS FINISH, excludes deliberate VSync idle time.
    char meter[64];
    // Independent VU0 micro-mode diagnostic; no hardware is accessed
    // by drawing this panel, and VU1 BOX 400 stays untouched.
    if(s_scene.kind>=8)
    {
        char vu0Line[32];
        static const char* const stages[]={
            "NOT INIT","ARM","UPLOAD","VERIFY","DIFF"
        };
        if(b3Ps2GetVU0Enabled()==6)
            snprintf(vu0Line,sizeof(vu0Line),"VU0 LIVE MICRO");
        else
            snprintf(vu0Line,sizeof(vu0Line),"VU0 %s %s",
                     stages[s_vu0MicroStage],
                     s_vu0MicroStage>1?
                        (s_vu0MicroResult?"OK":"FAIL"):"");
        b3GsOverlayPanel(460,164,166,70);
        b3GsOverlayText(468,174,"SELECT MICRO",227,215,143);
        b3GsOverlayText(468,205,vu0Line,
                        s_vu0MicroResult?155:255,
                        s_vu0MicroResult?243:159,176);
    }
    b3GsOverlayPanel(14,164,440,70);
    snprintf(meter,sizeof(meter),"PHYS %.1f  DRAW %.1f",
             (double)s_lastPhysicsMs,(double)s_lastBuildMs);
    b3GsOverlayText(26,172,meter,245,239,185);
    snprintf(meter,sizeof(meter),"GSWAIT %.1f  USE %.0f%%",
             (double)s_lastGpuMs,(double)s_lastWorkPct);
    b3GsOverlayText(26,202,meter,245,239,185);
#else
    b3GsOverlayText(14,37,"GS PCSS HEIGHT",188,226,240);
    b3GsOverlayPanel(206,184,106,30);
    b3GsOverlayText(214,195,"PCSS 3D",250,236,130);
#endif
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
#if defined(BOX3D_GS_KAWASE_WIDE)
#if defined(BOX3D_GS_PROGRESSIVE)
    b3GsOverlayText(28,82,"WIDE DUAL KAWASE",188,226,240);
#else
    b3GsOverlayText(14,37,"WIDE DUAL KAWASE",188,226,240);
#endif
#else
    b3GsOverlayText(14,37,"SOFT GS KAWASE",188,226,240);
#endif
#endif
#if defined(BOX3D_PS2_PAD_NAV)
#if defined(BOX3D_GS_PROGRESSIVE)
    b3GsOverlayPanel(14,B3_PS2_GS_H-64,288,48);
    b3GsOverlayText(26,B3_PS2_GS_H-50,"LEFT  PREV  RIGHT  NEXT",213,226,236);
#else
    b3GsOverlayPanel(6,B3_PS2_GS_H-24,182,19);
    b3GsOverlayText(10,B3_PS2_GS_H-19,"LEFT PREV RIGHT NEXT",213,226,236);
#endif
#endif
}
#endif
static void render(void)
{
#if defined(BOX3D_PS2_UNIFIED)
    s_frameShadowTicks=0;
    s_frameBoxTicks=0;
    s_lastVu1Boxes=0;
    cacheSceneBodyPoses();
#endif
#if defined(BOX3D_GS_PROGRESSIVE)
    s_culledFaces=0;
    s_emittedFaces=0;
#endif
    b3GsBegin();
#if defined(BOX3D_GS_OFFICIAL)
    // Frame layers: atmosphere -> textured-look floor -> contact shadows
    // -> lit rigid bodies -> scene panel. Native GS GIF A+D throughout.
    b3GsSkyGradient();
#if defined(BOX3D_PS2_UNIFIED)
    if(!b3GsEmitCachedGround())
    {
        b3GsBeginGroundRecord();
        render_ground();
        b3GsEndGroundRecord();
    }
#else
    render_ground();
#endif
#if defined(BOX3D_GS_RECEIVER_SHADOWS)
    // Layered receiver rendering prevents Z-tested shadow holes:
    // 1. floor shadows are composited onto the floor
    // 2. opaque ramp covers any floor shadows behind it
    // 3. ramp-local shadows land on the now-visible ramp
    // 4. dynamics are drawn last and occlude their own floor shadows.
#if defined(BOX3D_PS2_UNIFIED)
    uint32_t phaseStart=b3Ps2ReadCpuCycles();
#endif
#if defined(BOX3D_PS2_UNIFIED)
    if(s_shadowVisualMode!=3)
#endif
        render_receiver_shadows(0);
#if defined(BOX3D_PS2_UNIFIED)
    s_frameShadowTicks+=b3Ps2DeltaCpuCycles(phaseStart,b3Ps2ReadCpuCycles());
#endif
    int firstDynamic=1;
    if(s_scene.kind==1
#if defined(BOX3D_GS_PROGRESSIVE)
       ||s_scene.kind==4||s_scene.kind==5
#endif
      )
    {
#if defined(BOX3D_PS2_UNIFIED)
        phaseStart=b3Ps2ReadCpuCycles();
#endif
        draw_box(&s_scene.objects[1]); // static inclined plane / platform
#if defined(BOX3D_PS2_UNIFIED)
        s_frameBoxTicks+=b3Ps2DeltaCpuCycles(phaseStart,b3Ps2ReadCpuCycles());
        phaseStart=b3Ps2ReadCpuCycles();
#endif
#if defined(BOX3D_PS2_UNIFIED)
        if(s_shadowVisualMode!=3)
#endif
            render_receiver_shadows(1);
#if defined(BOX3D_PS2_UNIFIED)
        s_frameShadowTicks+=b3Ps2DeltaCpuCycles(phaseStart,b3Ps2ReadCpuCycles());
#endif
        firstDynamic=2;
    }
#if defined(BOX3D_PS2_UNIFIED)
    phaseStart=b3Ps2ReadCpuCycles();
#endif
#if defined(BOX3D_PS2_UNIFIED)
    if(s_scene.kind>=8 && s_vu1Ready &&
       (s_vu1Budget==-1 || s_vu1Budget>0))
    {
        const int diagnostic=s_vu1Budget<0;
        const int maxQueued=diagnostic?1:s_vu1Budget;
        b3Vu1BoxesBegin(s_scene.camera,!diagnostic);
        int queued=0;
        for(int i=firstDynamic;i<s_scene.count;++i)
        {
            const SceneBody* body=&s_scene.objects[i];
            if(queued<maxQueued && queue_vu1_box(body))
            {
                ++queued;
                // Diagnostic VU transform does not XGKICK; retain an
                // EE-drawn copy so the picture stays fully visible.
                if(diagnostic)draw_box(body);
            }
            else
                draw_box(body);
        }
        if(queued!=0)
        {
            // PATH3: floor + PCSS + any exceptional EE solids.
            // PATH1: VU1-generated GS triangles, followed by HUD PATH3.
            if(s_frame<2)
                printf("BOX3D_VU1_PHASE,before_path3_flush,scene=%d,queued=%d,emit=%d\n",
                       s_scene.kind,queued,!diagnostic);
            if(b3GsFlushForVu1())
            {
                if(s_frame<2)
                    puts("BOX3D_VU1_PHASE,after_path3_flush");
                s_lastVu1Boxes=b3Vu1BoxesSubmit();
                if(s_frame<2)
                    printf("BOX3D_VU1_PHASE,after_submit,result=%d\n",
                           s_lastVu1Boxes);
                if(s_lastVu1Boxes<0)
                {
                    puts("BOX3D_VU1_DISABLED,vif1_timeout");
                    s_vu1Budget=0;
                }
            }
            else
                for(int i=firstDynamic;i<s_scene.count;++i)
                    draw_box(&s_scene.objects[i]);
        }
    }
    else
#endif
    {
        for(int i=firstDynamic;i<s_scene.count;++i)
            draw_box(&s_scene.objects[i]);
    }
#if defined(BOX3D_PS2_UNIFIED)
    // The first four steps separate ordinary EE drawing, PATH3,
    // explicit intent to initialize VU1, and the initialization itself.
    // Critical: VU1 MPG/MMIO reads are NEVER executed on program boot.
    if(s_scene.kind>=8 && s_vu1Budget==-7 && !s_vu1InitAttempted)
    {
        s_vu1InitAttempted=1;
        puts("BOX3D_VU1_PHASE,lazy_init_upload_begin");
        const int uploaded=b3Vu1BoxesInit();
        printf("BOX3D_VU1_PHASE,lazy_init_upload_end,uploaded=%d,status=%s\n",
               uploaded,b3Vu1BoxesStatus());
    }
    if(s_scene.kind>=8 && s_vu1Budget==-9 && !s_vu1VerifyAttempted)
    {
        s_vu1VerifyAttempted=1;
        puts("BOX3D_VU1_PHASE,lazy_verify_begin");
        s_vu1Ready=b3Vu1BoxesVerify();
        printf("BOX3D_VU1_PHASE,lazy_verify_end,ready=%d,status=%s\n",
               s_vu1Ready,b3Vu1BoxesStatus());
    }
    // PATH3 is testable even if VU1 has never initialized.
    if(s_scene.kind>=8 && s_vu1Budget==-3)
        b3GsFlushForVu1();
    // After successful init, test the VIF upload and MSCAL separately.
    if(s_scene.kind>=8 && s_vu1Ready &&
       ((s_vu1Budget==-4 && !s_vu1VifAttempted) ||
        (s_vu1Budget==-5 && !s_vu1BootAttempted)))
    {
        if(b3GsFlushForVu1())
        {
            if(s_vu1Budget==-4)
            {
                s_vu1VifAttempted=1;
                if(!b3Vu1BoxesUploadTest(s_scene.camera))
                {
                    puts("BOX3D_VU1_DISABLED,vif_only_failed");
                    s_vu1Ready=0;
                }
            }
            else
            {
                s_vu1BootAttempted=1;
                puts("BOX3D_VU1_BOOT,one_shot=1,phase=before_test");
                if(!b3Vu1BoxesBootTest())
                {
                    puts("BOX3D_VU1_DISABLED,boot_mscal_failed");
                    s_vu1Ready=0;
                }
            }
        }
    }
#endif
#if defined(BOX3D_PS2_UNIFIED)
    s_frameBoxTicks+=b3Ps2DeltaCpuCycles(phaseStart,b3Ps2ReadCpuCycles());
    if(s_frame%240==0 && s_scene.kind>=8)
        printf("BOX3D_VU1_FRAME,scene=%d,ready=%d,budget=%d,boxes=%d\n",
               s_scene.kind,s_vu1Ready,s_vu1Budget,s_lastVu1Boxes);
#endif
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
#if defined(BOX3D_PS2_UNIFIED)
    s_beforeGsSubmitTicks=b3Ps2ReadCpuCycles();
#endif
    b3GsPresent();
}

int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    puts("BOX3D_SAMPLE_START,gs=native_gif,physics=Box3D");
#if defined(BOX3D_PS2_UNIFIED)
    puts("BOX3D_PAD_EXTRA,UP=substeps,DOWN=vu0_0_5_6_after_diff,SQUARE=shadow,CROSS=sleep,SELECT=vu0_micro_arm_upload_verify_run,TRIANGLE=vu1_diagnostics");
#endif
#if defined(BOX3D_GS_PCSS_SHADOWS)
#if defined(BOX3D_PS2_UNIFIED)
    puts("BOX3D_UNIFIED,video=640x480p,shadow=PCSS,pcf_filter=4tap_contact_9tap_prefiltered_far,pagination=dpad,stress=64_144_256_400");
    puts("BOX3D_SHADOW_FILTER,PCSS_BUCKETS,source=640x480,radius=1..5,pcf=bilinear");
#else
    puts("BOX3D_SHADOW_FILTER,PCSS_INSPIRED_PER_CASTER,source=320x224,radius=1..5,taps=9_or_21");
#endif
#elif defined(BOX3D_GS_PCF_SHADOWS)
#if defined(BOX3D_GS_PCF_4TAP)
    printf("BOX3D_SHADOW_FILTER,PCF_4_BILINEAR,source=%dx%d,radius=1px,weight_sum=96\n",
           B3_PS2_GS_W,B3_PS2_GS_H);
    puts("BOX3D_PCF_ACTIVE,mode=4tap_fullres_mask,HUD=top_left+bottom_right");
#else
    puts("BOX3D_SHADOW_FILTER,PCF_MASK_3X3,source=320x224,radius=1px,weight_sum=96");
    puts("BOX3D_PCF_ACTIVE,mode=3x3_fullres_mask,HUD=top_left+bottom_right");
#endif
#endif
    b3SetAssertFcn(fail_assert);
    b3SetLogFcn(log_message);
    if(!b3GsInit())
    {
        puts("BOX3D_SAMPLE_FAIL,gs_initialization");
        for(;;) SleepThread();
    }
#if defined(BOX3D_PS2_UNIFIED)
    // Do not initialize VIF1, upload MPG, or touch VU1 microcode
    // until the user enters the explicit INIT diagnostic stage.
    s_vu1Ready=0;
    puts("BOX3D_VU1_STARTUP,lazy=1,ready=0,ee_safe=1");
    puts("BOX3D_VU1_BUILD,revision=vu1r8_sleep_shadow_cache,vu0=micro_live2");
    puts("BOX3D_VU1_SAFE_DEFAULT,ee_geometry=1,triangle=ack_path3_init_arm_init_verify_arm_verify_vif_boot");
#endif
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
#if defined(BOX3D_PS2_UNIFIED)
            int newKind=b3Ps2DemoNextPage(s_scene.kind,direction);
#else
            int newKind=(s_scene.kind+SCENE_COUNT+direction)%SCENE_COUNT;
#endif
            b3DestroyWorld(s_scene.world);
            scene_create(newKind);
            s_frame=0; // full duration after manually changing scenes
            printf("BOX3D_PAD_SCENE,dir=%d,scene=%d\n",direction,newKind);
        }
        else if(
#if defined(BOX3D_PS2_UNIFIED)
                // Manual pagination should KEEP the chosen stress tier
                // for as long as the D-pad controller is active.
                !s_padReady &&
#endif
                s_frame>0 && s_frame%SCENE_FRAMES==0)
        {
            b3DestroyWorld(s_scene.world);
            scene_create((s_scene.kind+1)%SCENE_COUNT);
            s_frame=0;
        }
#if defined(BOX3D_GS_KAWASE_SHADOWS)
        uint32_t frameStart=b3Ps2ReadCpuCycles();
#endif
#if defined(BOX3D_PS2_UNIFIED)
        b3World_Step(s_scene.world,1.0f/60.0f,s_simSubsteps);
        if(s_vu0Mode!=b3Ps2GetVU0Enabled())
        {
            s_vu0Mode=b3Ps2GetVU0Enabled();
            printf("BOX3D_VU0_LIVE_FALLBACK,mode=%d,status=%s\n",
                   s_vu0Mode,b3Vu0MicroStatus());
        }
#else
        b3World_Step(s_scene.world,1.0f/60.0f,4);
#endif
#if defined(BOX3D_GS_KAWASE_SHADOWS)
        uint32_t physicsEnd=b3Ps2ReadCpuCycles();
#endif
        render();
#if defined(BOX3D_PS2_UNIFIED)
        {
            // Read the already-captured per-stage timings; no VSync
            // ambiguity. A frame that reads "FPS 60" can still be
            // using 90% of its CPU+GS budget.
            uint32_t physicsTicks=
                b3Ps2DeltaCpuCycles(frameStart,physicsEnd);
            uint32_t buildTicks=
                b3Ps2DeltaCpuCycles(physicsEnd,s_beforeGsSubmitTicks);
            uint32_t gpuTicks=b3GsGetLastGsWaitTicks();
            uint32_t vsyncTicks=b3GsGetLastVsyncWaitTicks();
            const float ticksPerMs=294912.0f; // nominal EE 294.912MHz
            s_lastPhysicsMs=(float)physicsTicks/ticksPerMs;
            s_lastBuildMs=(float)buildTicks/ticksPerMs;
            s_lastGpuMs=(float)gpuTicks/ticksPerMs;
            s_lastVsyncMs=(float)vsyncTicks/ticksPerMs;
            uint32_t workTicks=physicsTicks+buildTicks+gpuTicks;
            s_lastWorkPct=(float)workTicks/(4915200.0f/100.0f);
            s_sumPhysicsTicks+=physicsTicks;
            s_sumBuildTicks+=buildTicks;
            s_sumGsTicks+=gpuTicks;
            s_sumShadowTicks+=s_frameShadowTicks;
            s_sumBoxTicks+=s_frameBoxTicks;
            b3Profile profile=b3World_GetProfile(s_scene.world);
            s_profilePairsMs+=profile.pairs;
            s_profileCollideMs+=profile.collide;
            s_profileSolveMs+=profile.solve;
            s_profileConstraintsMs+=profile.constraints;
            if(workTicks>s_maxWorkTicks)s_maxWorkTicks=workTicks;
            ++s_perfSamples;
            if(s_perfSamples>=60)
            {
                double divisor=294912.0*(double)s_perfSamples;
                printf("BOX3D_PERF,scene=%d,frame=%lu,bodies=%d,avg_physics_ms=%.3f,avg_build_ms=%.3f,avg_shadow_ms=%.3f,avg_box_ms=%.3f,avg_gpu_wait_ms=%.3f,peak_work_ms=%.3f,avg_work_pct=%.1f,vsync_idle_ms=%.3f,gif_qwords=%u,substeps=%d,vu0=%d,shadow_mode=%d,vu1_enabled=%d,vu1_boxes=%d,sleep=%d,awake=%d,vu0_micro_calls=%u\n",
                       s_scene.kind,(unsigned long)s_frame,s_scene.count-1,
                       (double)s_sumPhysicsTicks/divisor,
                       (double)s_sumBuildTicks/divisor,
                       (double)s_sumShadowTicks/divisor,
                       (double)s_sumBoxTicks/divisor,
                       (double)s_sumGsTicks/divisor,
                       (double)s_maxWorkTicks/294912.0,
                       (double)(s_sumPhysicsTicks+s_sumBuildTicks+
                                s_sumGsTicks)/
                         (4915200.0*(double)s_perfSamples/100.0),
                       (double)s_lastVsyncMs,b3GsGetLastGifQwords(),
                       s_simSubsteps,b3Ps2GetVU0Enabled(),s_shadowVisualMode,
                        s_vu1Ready&&s_vu1Budget>0,s_lastVu1Boxes,
                       b3World_IsSleepingEnabled(s_scene.world),
                       b3World_GetAwakeBodyCount(s_scene.world),
                       b3Vu0MicroGetLiveCalls());
                b3Counters counters=b3World_GetCounters(s_scene.world);
                printf("BOX3D_PHYS_PROFILE,scene=%d,frame=%lu,substeps=%d,vu0=%d,pairs_ms=%.3f,collide_ms=%.3f,solve_ms=%.3f,constraints_ms=%.3f,contacts=%d,awake=%d\n",
                       s_scene.kind,(unsigned long)s_frame,
                       s_simSubsteps,s_vu0Mode,
                       s_profilePairsMs/(double)s_perfSamples,
                       s_profileCollideMs/(double)s_perfSamples,
                       s_profileSolveMs/(double)s_perfSamples,
                       s_profileConstraintsMs/(double)s_perfSamples,
                       counters.contactCount,
                       b3World_GetAwakeBodyCount(s_scene.world));
                s_profilePairsMs=0.0;
                s_profileCollideMs=0.0;
                s_profileSolveMs=0.0;
                s_profileConstraintsMs=0.0;
                s_perfSamples=0;
                s_sumPhysicsTicks=0;
                s_sumBuildTicks=0;
                s_sumGsTicks=0;
                s_sumShadowTicks=0;
                s_sumBoxTicks=0;
                s_maxWorkTicks=0;
            }
        }
#endif
#if defined(BOX3D_GS_PROGRESSIVE)
        if(s_frame%120==0)
            printf("BOX3D_GS_GEOMETRY,frame=%lu,culled_faces=%u,emitted_faces=%u,gif_qwords=%u,dma_submits=1\n",
                   (unsigned long)s_frame,s_culledFaces,s_emittedFaces,
                   b3GsGetLastGifQwords());
#endif
#if defined(BOX3D_GS_KAWASE_SHADOWS) && !defined(BOX3D_PS2_UNIFIED)
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
