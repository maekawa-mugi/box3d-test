// SPDX-License-Identifier: MIT
// Host-side ABI and vertex math checks for ps2/vu1_boxes.vsm.
// The hardware microprogram / XGKICK must also be tested on PCSX2.
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct Upload {
    _Alignas(16) float vertices[8][4];
    uint32_t colors[6][4];
} Upload;
_Static_assert(sizeof(Upload)==224, "VU1 input upload is 14 qwords");
_Static_assert(offsetof(Upload,colors)==128, "VU1 RGBAQ offset");

static void check_projection(float x,float y,float z,
                             float focal,int width,int height)
{
    // Camera aligned along +Z; this is the VMUL/VMADD / Q sequence
    // mirrored from the VU1 camera constants, including XYOFFSET.
    float mat[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,0}};
    float in[4]={x,y,z,1};
    float camera[3]={0};
    for(int a=0;a<3;++a)
        camera[a]=mat[0][a]*in[0]+mat[1][a]*in[1]+
                  mat[2][a]*in[2]+mat[3][a]*in[3];
    float q=1/camera[2];
    int screenX=(int)((2048+width*0.5f)*16+
                       camera[0]*q*focal*16);
    int screenY=(int)((2048+height*0.5f)*16-
                       camera[1]*q*focal*16);
    int depth=(int)((q-1/220.0f)*30000);
    int xref=(int)((2048+width*0.5f+focal*x/z)*16);
    int yref=(int)((2048+height*0.5f-focal*y/z)*16);
    int zref=(int)((1/z-1/220.0f)*30000);
    // Float32 instruction ordering may differ by +/-1 fixed-point LSB.
    assert(abs(screenX-xref)<=1 && abs(screenY-yref)<=1);
    assert(abs(depth-zref)<=1);
}
int main(void)
{
    assert(sizeof(float)==4);
    // ILW loads the low 16 bits of the V4_32 control word. Sending
    // 1.0f used to read as zero, silently disabling XGKICK.
    float control=0.0f;
    uint32_t enabled=1u, bits=0u;
    memcpy(&control,&enabled,sizeof(enabled));
    memcpy(&bits,&control,sizeof(bits));
    assert((bits&0xffffu)==1u);
    float wrong=1.0f;
    memcpy(&bits,&wrong,sizeof(bits));
    assert((bits&0xffffu)==0u);
    check_projection(0,0,10,600,640,480);
    check_projection(-4.1f,2.9f,24.4f,600,640,480);
    check_projection(8.4f,-4.3f,85.1f,600,640,480);
    static const int quads[6][4]={
        {0,2,3,1},{4,5,7,6},{0,4,6,2},
        {1,3,7,5},{0,1,5,4},{2,6,7,3}
    };
    int indices[36],faces[36],k=0;
    for(int f=0;f<6;++f)
    {
        const int* q=quads[f];
        const int tri[6]={q[0],q[1],q[2],q[0],q[2],q[3]};
        for(int j=0;j<6;++j){indices[k]=tri[j];faces[k++]=f;}
    }
    assert(k==36);
    for(int i=0;i<36;++i)
    {
        assert(indices[i]>=0 && indices[i]<8);
        assert(faces[i]>=0 && faces[i]<6);
    }
    puts("PASS: VU1 14-QW upload layout, 36 vertices, XYZ 12.4 and reversed Z");
    return 0;
}
