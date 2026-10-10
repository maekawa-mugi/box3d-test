// SPDX-License-Identifier: MIT
// Real VU1 micro-mode transform and XGKICK, not a COP2 macro-mode stub.
// VIF1 uploads one shared camera, then 8 corners + 6 per-face colors per
// box. Microprogram transforms 8 vertices once and emits 12 triangles
// directly to GS through GIF PATH1. PCSS always executes earlier on PATH3.
#include "vu1_boxes.h"

#include <dma.h>
#include <gif_tags.h>
#include <gs_gp.h>
#include <kernel.h>
#include <packet2.h>
#include <packet2_utils.h>
#include <packet2_vif.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

// PS2SDK packet2_vif_add_micro_program expects u32* exactly.
// The R5900 toolchain typedefs uint32_t as unsigned long and u32 as
// unsigned int, so use the SDK type for DVP link symbols.
extern u32 B3Vu1Boxes_CodeStart __attribute__((section(".vudata")));
extern u32 B3Vu1Boxes_CodeEnd __attribute__((section(".vudata")));
extern u32 B3Vu1Boxes_RunStart __attribute__((section(".vudata")));

enum { B3_VU1_VIF_QWORDS=8192, B3_VU1_CODE_QWORDS=256 };
static packet2_t* s_vifPacket;
static _Alignas(64) b3Vu1BoxInput s_upload[B3_VU1_MAX_BOXES];
static _Alignas(64) float s_camera[7][4];
static _Alignas(64) uint32_t s_indices[36][4];
static _Alignas(64) uint64_t s_gifTag[2];
static b3GsCamera s_currentCamera;
static int s_count;
static int s_ready;
static int s_emitToGs;
static unsigned s_mainProgramPc;
static unsigned s_submissionCount;
// PS2SDK dma_channel_wait(timeout=0) never terminates on a stalled VIF.
// The finite timeout is an emergency diagnostic, not a hardware recovery.
enum { B3_VU1_DMA_WAIT_LIMIT=50000000 };

// The VIF UNPACK_V4_32 source count is measured in 16-byte quadwords.
// Check the R5900 ABI, not only the host C17 layout regression test.
_Static_assert(sizeof(u32)==4, "PS2SDK VIF u32 must be 32-bit");
_Static_assert(sizeof(uint32_t)==4, "VIF source uint32_t must be 32-bit");
_Static_assert(sizeof(b3Vu1BoxInput)==224, "VU1 box is exactly 14 QW");
_Static_assert(sizeof(s_indices[0])==16, "VU1 indices need V4_32");
_Static_assert(sizeof(s_gifTag)==16, "GIF tag needs one quadword");

static float b3Vu1Dot(b3GsPoint3 a,b3GsPoint3 b)
{
    return a.x*b.x+a.y*b.y+a.z*b.z;
}

int b3Vu1BoxesInit(void)
{
    static const unsigned char faces[6][4]={
        {0,2,3,1},{4,5,7,6},{0,4,6,2},
        {1,3,7,5},{0,1,5,4},{2,6,7,3}
    };
    s_ready=0;
    s_count=0;
    // VU1 microprogram has a two-instruction E-bit boot stub at PC=0,
    // followed by the full transform code. Compute the latter's PC
    // from DVP linker symbols, rather than assuming assembly layout.
    uintptr_t start=(uintptr_t)&B3Vu1Boxes_CodeStart;
    uintptr_t main=(uintptr_t)&B3Vu1Boxes_RunStart;
    if(main<start || (main-start)!=16u)
    {
        printf("BOX3D_VU1_FAIL,entry_alignment,delta=%lu\n",
               (unsigned long)(main-start));
        return 0;
    }
    s_mainProgramPc=(unsigned)((main-start)/8u);
    // The PS2SDK VU1 example uses the same VIF1+packet2 APIs.
    if(dma_channel_initialize(DMA_CHANNEL_VIF1,NULL,0)<0)
    {
        puts("BOX3D_VU1_FAIL,init_vif1");
        return 0;
    }
    s_vifPacket=packet2_create(B3_VU1_VIF_QWORDS,P2_TYPE_NORMAL,
                               P2_MODE_CHAIN,1);
    packet2_t* startup=packet2_create(B3_VU1_CODE_QWORDS,
                                      P2_TYPE_NORMAL,P2_MODE_CHAIN,1);
    if(!s_vifPacket || !startup)
    {
        if(startup)packet2_free(startup);
        puts("BOX3D_VU1_FAIL,packet_allocation");
        return 0;
    }
    packet2_vif_add_micro_program(startup,0,
                                 &B3Vu1Boxes_CodeStart,
                                 &B3Vu1Boxes_CodeEnd);
    packet2_utils_vu_add_end_tag(startup);
    dma_channel_send_packet2(startup,DMA_CHANNEL_VIF1,1);
    int startupWait=dma_channel_wait(DMA_CHANNEL_VIF1,
                                     B3_VU1_DMA_WAIT_LIMIT);
    packet2_free(startup);
    if(startupWait<0)
    {
        puts("BOX3D_VU1_FAIL,microcode_upload_timeout");
        return 0;
    }

    int k=0;
    for(int face=0;face<6;++face)
    {
        const unsigned char* q=faces[face];
        const int v[6]={q[0],q[1],q[2],q[0],q[2],q[3]};
        for(int j=0;j<6;++j)
        {
            s_indices[k][0]=(uint32_t)v[j];
            s_indices[k][1]=(uint32_t)face;
            s_indices[k][2]=0;
            s_indices[k][3]=0;
            ++k;
        }
    }
    // GIF PACKED RGBAQ + XYZ2. Packed XYZ2 accepts a float4 storing
    // 32-bit integer X,Y,Z in x,y,z. Low 16 bits of XY are GS 12.4,
    // Z is a 32-bit integer (the scene framebuffer uses PSMZ16).
    s_gifTag[0]=GIF_SET_TAG(36,1,1,3,GIF_FLG_PACKED,2);
    s_gifTag[1]=(uint64_t)GIF_REG_RGBAQ |
                ((uint64_t)GIF_REG_XYZ2<<4);
    packet2_reset(s_vifPacket,0);
    packet2_utils_vu_add_unpack_data(s_vifPacket,128,s_indices,36,0);
    packet2_utils_vu_add_unpack_data(s_vifPacket,256,s_gifTag,1,0);
    packet2_utils_vu_add_end_tag(s_vifPacket);
    dma_channel_send_packet2(s_vifPacket,DMA_CHANNEL_VIF1,1);
    if(dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
    {
        puts("BOX3D_VU1_FAIL,static_data_upload_timeout");
        return 0;
    }
    s_ready=1;
    puts("BOX3D_VU1_READY,vif1_dma=1,vu1_micro=1,xgkick=1,triangles_per_box=12");
    return 1;
}

void b3Vu1BoxesBegin(b3GsCamera c,int emitToGs)
{
    s_count=0;
    s_emitToGs=(emitToGs!=0);
    s_currentCamera=c;
    // VU output formula matches projectDepthPrecise(): 12.4 XY and
    // reversed Z16 with 1/z linear in screen space.
    s_camera[0][0]=c.right.x;
    s_camera[0][1]=c.up.x;
    s_camera[0][2]=c.forward.x;
    s_camera[0][3]=0.0f;
    s_camera[1][0]=c.right.y;
    s_camera[1][1]=c.up.y;
    s_camera[1][2]=c.forward.y;
    s_camera[1][3]=0.0f;
    s_camera[2][0]=c.right.z;
    s_camera[2][1]=c.up.z;
    s_camera[2][2]=c.forward.z;
    s_camera[2][3]=0.0f;
    s_camera[3][0]=-b3Vu1Dot(c.eye,c.right);
    s_camera[3][1]=-b3Vu1Dot(c.eye,c.up);
    s_camera[3][2]=-b3Vu1Dot(c.eye,c.forward);
    s_camera[3][3]=0.0f;
    s_camera[4][0]=c.focal*16.0f;
    s_camera[4][1]=-c.focal*16.0f;
    s_camera[4][2]=30000.0f;
    s_camera[4][3]=0.0f;
    s_camera[5][0]=(2048.0f+B3_PS2_GS_W*0.5f)*16.0f;
    s_camera[5][1]=(2048.0f+B3_PS2_GS_H*0.5f)*16.0f;
    s_camera[5][2]=-30000.0f/220.0f;
    s_camera[5][3]=0.0f;
    // IMPORTANT: ILW reads the low 16 integer bits, NOT a C float.
    // 1.0f is 0x3f800000, whose low 16 bits are zero! Previous code
    // accidentally skipped XGKICK even in D1. Send exact integer 1.
    const u32 kick=emitToGs?1u:0u;
    memcpy(&s_camera[6][0],&kick,sizeof(kick));
    s_camera[6][1]=0.0f;
    s_camera[6][2]=0.0f;
    s_camera[6][3]=0.0f;
}

int b3Vu1BoxesAdd(const b3GsPoint3 corners[8],
                  const uint8_t colors[6][3])
{
    if(!s_ready || s_count>=B3_VU1_MAX_BOXES)return 0;
    // Conservative camera-space bounding sphere. Near-clipped or
    // out-of-GS-XYZ-range boxes remain on the safe EE triangle path.
    b3GsPoint3 mid={
        (corners[0].x+corners[7].x)*0.5f,
        (corners[0].y+corners[7].y)*0.5f,
        (corners[0].z+corners[7].z)*0.5f
    };
    b3GsPoint3 radiusVector={
        (corners[7].x-corners[0].x)*0.5f,
        (corners[7].y-corners[0].y)*0.5f,
        (corners[7].z-corners[0].z)*0.5f
    };
    float radius=sqrtf(b3Vu1Dot(radiusVector,radiusVector));
    b3GsPoint3 eyeVector={mid.x-s_currentCamera.eye.x,
                          mid.y-s_currentCamera.eye.y,
                          mid.z-s_currentCamera.eye.z};
    float z=b3Vu1Dot(eyeVector,s_currentCamera.forward);
    if(z-radius<=0.625f || z+radius>=220.0f)return 0;
    float x=b3Vu1Dot(eyeVector,s_currentCamera.right);
    float y=b3Vu1Dot(eyeVector,s_currentCamera.up);
    float inv=1.0f/(z-radius);
    if((fabsf(x)+radius)*s_currentCamera.focal*inv >
       (B3_PS2_GS_W*0.5f+256.0f))return 0;
    if((fabsf(y)+radius)*s_currentCamera.focal*inv >
       (B3_PS2_GS_H*0.5f+256.0f))return 0;

    b3Vu1BoxInput* p=&s_upload[s_count++];
    for(int i=0;i<8;++i)
    {
        p->vertices[i][0]=corners[i].x;
        p->vertices[i][1]=corners[i].y;
        p->vertices[i][2]=corners[i].z;
        p->vertices[i][3]=1.0f;
    }
    for(int i=0;i<6;++i)
    {
        p->colors[i][0]=colors[i][0];
        p->colors[i][1]=colors[i][1];
        p->colors[i][2]=colors[i][2];
        p->colors[i][3]=128u;
    }
    return 1;
}

int b3Vu1BoxesUploadTest(b3GsCamera camera)
{
    if(!s_ready)return 0;
    b3Vu1BoxesBegin(camera,0);
    packet2_reset(s_vifPacket,0);
    packet2_utils_vu_add_unpack_data(s_vifPacket,0,s_camera,7,0);
    packet2_utils_vu_add_end_tag(s_vifPacket);
    puts("BOX3D_VU1_PHASE,vif_only_dma_start");
    dma_channel_send_packet2(s_vifPacket,DMA_CHANNEL_VIF1,1);
    if(dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
    {
        puts("BOX3D_VU1_FAIL,vif_only_timeout");
        s_ready=0;
        return 0;
    }
    puts("BOX3D_VU1_PHASE,vif_only_dma_done");
    return 1;
}

int b3Vu1BoxesBootTest(void)
{
    if(!s_ready)return 0;
    packet2_reset(s_vifPacket,0);
    // PC=0 is an E-bit stub. Deliberately no camera/corner upload
    // and no GIF PATH1 traffic. This tests VIF1 MSCAL/FLUSHE alone.
    packet2_utils_vu_add_start_program(s_vifPacket,0);
    packet2_chain_open_cnt(s_vifPacket,0,0,0);
    packet2_vif_flushe(s_vifPacket,0);
    packet2_vif_nop(s_vifPacket,0);
    packet2_vif_nop(s_vifPacket,0);
    packet2_vif_nop(s_vifPacket,0);
    packet2_chain_close_tag(s_vifPacket);
    packet2_utils_vu_add_end_tag(s_vifPacket);
    puts("BOX3D_VU1_PHASE,boot_mscal_start");
    dma_channel_send_packet2(s_vifPacket,DMA_CHANNEL_VIF1,1);
    if(dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
    {
        puts("BOX3D_VU1_FAIL,boot_mscal_timeout");
        s_ready=0;
        return 0;
    }
    puts("BOX3D_VU1_PHASE,boot_mscal_done");
    return 1;
}

int b3Vu1BoxesSubmit(void)
{
    if(!s_ready || s_count==0)return 0;
    packet2_reset(s_vifPacket,0);
    packet2_utils_vu_add_unpack_data(s_vifPacket,0,s_camera,7,0);
    for(int i=0;i<s_count;++i)
    {
        // DMA REF sources stay live and aligned throughout the transfer.
        packet2_utils_vu_add_unpack_data(s_vifPacket,16,
                                         s_upload[i].vertices,8,0);
        packet2_utils_vu_add_unpack_data(s_vifPacket,24,
                                         s_upload[i].colors,6,0);
        // FLUSH+MSCAL ensures input for the next box cannot overwrite
        // VU1 DMEM while the previous microprogram is still running.
        packet2_utils_vu_add_start_program(s_vifPacket,s_mainProgramPc);
        // Single output GIF buffer in VU1 DMEM. FLUSHA MUST wait
        // for XGKICK/PATH1 to finish before a following MSCAL writes
        // the same 256..328 output qwords. This barrier lives on VIF1,
        // so the EE does not poll every box.
        packet2_chain_open_cnt(s_vifPacket,0,0,0);
        if(s_emitToGs)
            packet2_vif_flusha(s_vifPacket,0);
        else
            packet2_vif_flushe(s_vifPacket,0);
        packet2_vif_nop(s_vifPacket,0);
        packet2_vif_nop(s_vifPacket,0);
        packet2_vif_nop(s_vifPacket,0);
        packet2_chain_close_tag(s_vifPacket);
    }
    packet2_chain_open_cnt(s_vifPacket,0,0,0);
    // FLUSHA is a GIF PATH1 completion barrier. The following EE GIF
    // HUD packet cannot overtake VU1 XGKICK data.
    if(s_emitToGs)
        packet2_vif_flusha(s_vifPacket,0);
    else
        packet2_vif_flushe(s_vifPacket,0);
    packet2_vif_nop(s_vifPacket,0);
    packet2_vif_nop(s_vifPacket,0);
    packet2_vif_nop(s_vifPacket,0);
    packet2_chain_close_tag(s_vifPacket);
    packet2_utils_vu_add_end_tag(s_vifPacket);
    if(s_submissionCount<3)
        printf("BOX3D_VU1_PHASE,before_vif1_dma,boxes=%d,qwords=%u\n",
               s_count,(unsigned)packet2_get_qw_count(s_vifPacket));
    dma_channel_send_packet2(s_vifPacket,DMA_CHANNEL_VIF1,1);
    if(dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
    {
        puts("BOX3D_VU1_FAIL,vif1_dma_timeout");
        s_ready=0;
        return -1;
    }
    if(s_submissionCount<3)
        puts("BOX3D_VU1_PHASE,after_vif1_dma");
    ++s_submissionCount;
    return s_count;
}
