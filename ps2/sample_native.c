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

#include <kernel.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define SCENE_COUNT 3
#define MAX_BOXES 20
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
        add_box(0,pt(0,-1,0),pt(20,1,20),identity,noSpin,0.7f,65,95,112);
        for(int i=0;i<12;++i)
        {
            float y=0.75f+1.25f*(float)i;
            add_box(1,pt(0,y,0),pt(0.5f,0.5f,0.5f),identity,noSpin,0.7f,
                    (uint8_t)(90+i*10),(uint8_t)(190-i*6),230);
        }
        s_scene.camera=b3GsLookAt(pt(18,13,22),pt(0,6.5f,0),200);
        puts("BOX3D_SAMPLE_SCENE,Stacking/BoxStack,boxes=12");
    }
    else if(kind==1)
    {
        // Shapes / Inclined Plane: static sloping platform with five
        // independently friction-tuned dynamic cubes.
        add_box(0,pt(0,-1,0),pt(25,1,25),identity,noSpin,0.7f,65,95,112);
        b3Quat incline=b3MakeQuatFromAxisAngle((b3Vec3){1,0,0},0.60f);
        add_box(0,pt(0,5,-1),pt(8,0.4f,5),incline,noSpin,1.0f,116,156,190);
        for(int i=0;i<5;++i)
        {
            add_box(1,pt(-5.0f+2.5f*i,11.3f,-4.0f),pt(0.5f,0.5f,0.5f),
                    identity,noSpin,0.04f*(float)((i+1)*(i+1)),
                    240,(uint8_t)(210-24*i),(uint8_t)(70+24*i));
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
            out->r=240;out->g=(uint8_t)(130+40*i);out->b=90;
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
            float shade=0.30f+0.67f*diffuse;
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
        b3GsLine3(s_scene.camera,verts[endpoints[j][0]],verts[endpoints[j][1]],
                  body->r,body->g,body->b);
    }
}

static void render(void)
{
    b3GsBegin();
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
