// SPDX-License-Identifier: MIT
// Real VU1 micro-mode transform and XGKICK, not a COP2 macro-mode stub.
// VIF1 uploads one shared camera, then 8 corners + 6 per-face colors per
// box. Microprogram transforms 8 vertices once and emits 12 triangles
// directly to GS through GIF PATH1. PCSS always executes earlier on PATH3.
#include "vu1_boxes.h"
#include "vu1_memory_map.h"
#include "vu1_vif_stream.h"

#include <dma.h>
#include <gif_tags.h>
#include <gs_gp.h>
#include <kernel.h>
#include <packet2.h>
#include <packet2_utils.h>
#include <packet2_vif.h>
#include <vif_registers.h>
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

enum { B3_VU1_VIF_QWORDS=8192, B3_VU1_CODE_WORDS=4608 };
static _Alignas(64) uint32_t s_mpgStream[B3_VU1_CODE_WORDS];
static _Alignas(64) uint32_t s_staticStream[160];
static _Alignas(64) uint32_t s_cameraTestStream[36];
static _Alignas(64) uint32_t s_bootTestStream[4];
static _Alignas(64) uint32_t s_vuOnlyStream[104];
static packet2_t* s_vifPacket;
static _Alignas(64) b3Vu1BoxInput s_upload[B3_VU1_MAX_BOXES];
static _Alignas(64) float s_camera[7][4];
static _Alignas(64) uint32_t s_indices[36][4];
static _Alignas(64) uint64_t s_gifTag[2];
static b3GsCamera s_currentCamera;
static int s_count;
static int s_ready;
static int s_microUploaded;
static const char* s_status="VU1 NOT INIT";
const char* b3Vu1BoxesStatus(void) {return s_status;}
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
    s_microUploaded=0;
    s_status="VU1 INIT";
    s_count=0;
    const uintptr_t start=(uintptr_t)&B3Vu1Boxes_CodeStart;
    const uintptr_t run=(uintptr_t)&B3Vu1Boxes_RunStart;
    const uintptr_t end=(uintptr_t)&B3Vu1Boxes_CodeEnd;
    if(run<start || run-start!=16u || end<=start ||
       end-start>16384u || ((end-start)&15u)!=0u)
    {
        printf("BOX3D_VU1_FAIL,microcode_layout,entry_delta=%lu,code_bytes=%lu\n",
               (unsigned long)(run-start),(unsigned long)(end-start));
        s_status="VU1 FAIL LAYOUT";
        return 0;
    }
    s_mainProgramPc=(unsigned)((run-start)/8u);

    // Avoid packet2 chain REF/TTE and its assertion/exit paths for MPG.
    // A contiguous, aligned VIF stream is sent by normal DMA. The entire
    // stream is fully constructed and bounds-checked before touching VIF1.
    const size_t mpgWords=b3Vu1BuildMpgStream(
        s_mpgStream,B3_VU1_CODE_WORDS,
        &B3Vu1Boxes_CodeStart,(end-start)/8u);
    if(!mpgWords || (mpgWords&3u))
    {
        puts("BOX3D_VU1_FAIL,mpg_stream_layout");
        s_status="VU1 FAIL MPG PACK";
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
    s_gifTag[0]=GIF_SET_TAG(36,1,1,3,GIF_FLG_PACKED,2);
    s_gifTag[1]=(uint64_t)GIF_REG_RGBAQ |
                ((uint64_t)GIF_REG_XYZ2<<4);
    const size_t staticWords=b3Vu1BuildStaticStream(
        s_staticStream,160,s_indices,s_gifTag);
    if(staticWords!=160u)
    {
        puts("BOX3D_VU1_FAIL,static_stream_layout");
        s_status="VU1 FAIL STATIC PACK";
        return 0;
    }
    if(dma_channel_initialize(DMA_CHANNEL_VIF1,NULL,0)<0)
    {
        puts("BOX3D_VU1_FAIL,init_vif1");
        s_status="VU1 FAIL VIF INIT";
        return 0;
    }
    printf("BOX3D_VU1_PHASE,mpg_normal_dma_begin,qwords=%u\n",
           (unsigned)(mpgWords/4u));
    if(dma_channel_send_normal(DMA_CHANNEL_VIF1,s_mpgStream,
                               (int)(mpgWords/4u),0,0)<0 ||
       dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
    {
        puts("BOX3D_VU1_FAIL,mpg_normal_dma_timeout");
        s_status="VU1 FAIL MPG DMA";
        return 0;
    }
    puts("BOX3D_VU1_PHASE,mpg_normal_dma_done");
    if(dma_channel_send_normal(DMA_CHANNEL_VIF1,s_staticStream,
                               (int)(staticWords/4u),0,0)<0 ||
       dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
    {
        puts("BOX3D_VU1_FAIL,static_normal_dma_timeout");
        s_status="VU1 FAIL STATIC";
        return 0;
    }
    s_microUploaded=1;
    s_status="VU1 MPG UPLOADED";
    puts("BOX3D_VU1_UPLOAD,normal_dma=1,mpg_done=1,static_done=1,readback_pending=1");
    return 1;
}

int b3Vu1BoxesVerify(void)
{
    if(!s_microUploaded)
    {
        s_status="VU1 FAIL NO UPLOAD";
        puts("BOX3D_VU1_FAIL,verify_before_upload");
        return 0;
    }
    s_ready=0;
    s_status="VU1 VERIFYING";
    puts("BOX3D_VU1_PHASE,verify_vif_idle");
    // Wait for VIF1's own command processor as well as the DMA
    // channel. Bounded polling keeps broken MPG uploads from
    // permanently hanging the EE before any VU1 BOOT test.
    unsigned waitVif=B3_VU1_DMA_WAIT_LIMIT;
    while(waitVif && ((VIF1_STAT&3u)!=0u ||
                       ((VIF1_STAT>>24)&31u)!=0u))
        --waitVif;
    if(!waitVif)
    {
        printf("BOX3D_VU1_FAIL,mpg_vif_busy,vif_stat=%08x,vif_err=%08x,vif_code=%08x\n",
               (unsigned)VIF1_STAT,(unsigned)VIF1_ERR,
               (unsigned)VIF1_CODE);
        s_status="VU1 FAIL VIF IDLE";
        return 0;
    }
    // EE can inspect VU1 instruction memory while the VU is idle.
    // Never launch MSCAL unless the complete VIF MPG readback matches.
    const u32* image=&B3Vu1Boxes_CodeStart;
    const uintptr_t imageBytes=
        (uintptr_t)&B3Vu1Boxes_CodeEnd-(uintptr_t)image;
    if(imageBytes<16 || imageBytes>16384 || (imageBytes&7u))
    {
        printf("BOX3D_VU1_FAIL,microcode_size,bytes=%lu\n",
               (unsigned long)imageBytes);
        s_status="VU1 FAIL MPG SIZE";
        return 0;
    }
    // PS2SDK's VU1_MICROMEM1_START is 0x1100C000, actually VU1
    // DATA memory on EE/PCSX2. VU1 microcode resides at 0x11008000.
    const volatile u32* vuCode=(const volatile u32*)B3_VU1_EE_MICRO_BASE;
    const unsigned instructionWords=(unsigned)(imageBytes/sizeof(u32));
    printf("BOX3D_VU1_IMAGE,bytes=%lu,words=%u,boot_pc=0,transform_pc=%u,micro_addr=%08x,data_addr=%08x,first=%08x:%08x,vif_stat=%08x,vif_err=%08x\n",
           (unsigned long)imageBytes,instructionWords,
           s_mainProgramPc,(unsigned)B3_VU1_EE_MICRO_BASE,
           (unsigned)B3_VU1_EE_DATA_BASE,
           (unsigned)image[0],(unsigned)image[1],
           (unsigned)VIF1_STAT,(unsigned)VIF1_ERR);
    unsigned mismatch=instructionWords;
    for(unsigned i=0;i<instructionWords;++i)
    {
        if(vuCode[i]!=image[i]) {mismatch=i;break;}
    }
    if(mismatch!=instructionWords)
    {
        printf("BOX3D_VU1_FAIL,mpg_readback,word=%u,want=%08x,got=%08x,vif_code=%08x,vif_stat=%08x,vif_err=%08x\n",
               mismatch,(unsigned)image[mismatch],(unsigned)vuCode[mismatch],
               (unsigned)VIF1_CODE,(unsigned)VIF1_STAT,
               (unsigned)VIF1_ERR);
        // Safety: never write the EE-mapped VU1 microcode window while
        // diagnosing a startup reset. The VIF1 MPG upload must verify
        // byte-for-byte before any MSCAL instruction can run.
        s_status="VU1 FAIL READBACK";
        return 0;
    }
    puts("BOX3D_VU1_IMAGE,mpg_dma_readback=ok");
    puts("BOX3D_VU1_IMAGE,readback=ok");

    // The large DMA-chain packet for per-box geometry is allocated only
    // after MPG readback succeeds. It is not needed by the INIT stage.
    if(!s_vifPacket)
        s_vifPacket=packet2_create(B3_VU1_VIF_QWORDS,P2_TYPE_NORMAL,
                                   P2_MODE_CHAIN,1);
    if(!s_vifPacket)
    {
        puts("BOX3D_VU1_FAIL,geometry_packet_allocation");
        s_status="VU1 FAIL ALLOC";
        return 0;
    }
    s_ready=1;
    s_status="VU1 READY DMA";
    puts("BOX3D_VU1_READY,vif1_dma=1,vu1_micro=1,xgkick=1,triangles_per_box=12,micro_cpu_fallback=disabled");
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
    const size_t words=b3Vu1BuildCameraStream(
        s_cameraTestStream,36,s_camera);
    if(words!=36u)
    {
        puts("BOX3D_VU1_FAIL,vif_camera_stream_layout");
        s_status="VU1 FAIL VIF PACK";
        return 0;
    }
    // Same verified normal-DMA transport as the successful MPG upload.
    puts("BOX3D_VU1_PHASE,vif_only_normal_dma_start");
    if(dma_channel_send_normal(DMA_CHANNEL_VIF1,s_cameraTestStream,
                               (int)(words/4u),0,0)<0 ||
       dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
    {
        printf("BOX3D_VU1_FAIL,vif_only_dma_timeout,vif_stat=%08x,vif_err=%08x\n",
               (unsigned)VIF1_STAT,(unsigned)VIF1_ERR);
        s_status="VU1 FAIL VIF DMA";
        s_ready=0;
        return 0;
    }
    printf("BOX3D_VU1_PHASE,vif_only_normal_dma_done,vif_stat=%08x,vif_err=%08x\n",
           (unsigned)VIF1_STAT,(unsigned)VIF1_ERR);
    s_status="VU1 VIF OK";
    return 1;
}

int b3Vu1BoxesBootTest(void)
{
    if(!s_ready)return 0;
    // The PC=0 microprogram is the two-instruction E-bit stub.
    // No packet2 REF/TTE DMA chain: issue one aligned normal-DMA QW.
    const size_t words=b3Vu1BuildBootStream(s_bootTestStream,4);
    if(words!=4u)
    {
        puts("BOX3D_VU1_FAIL,boot_stream_layout");
        s_status="VU1 FAIL BOOT PACK";
        return 0;
    }
    const u32* image=&B3Vu1Boxes_CodeStart;
    // DVP VU upper instruction E bit: bit 30.
    if((image[1]&0x40000000u)==0)
    {
        printf("BOX3D_VU1_FAIL,boot_stub_e_bit_missing,upper=%08x\n",
               (unsigned)image[1]);
        s_status="VU1 FAIL BOOT STUB";
        return 0;
    }
    printf("BOX3D_VU1_BOOT,normal_dma_pre_mscal,vif_stat=%08x,vif_err=%08x,vif_code=%08x,upper=%08x\n",
           (unsigned)VIF1_STAT,(unsigned)VIF1_ERR,
           (unsigned)VIF1_CODE,(unsigned)image[1]);
    puts("BOX3D_VU1_PHASE,boot_normal_dma_start");
    if(dma_channel_send_normal(DMA_CHANNEL_VIF1,s_bootTestStream,
                               (int)(words/4u),0,0)<0 ||
       dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
    {
        printf("BOX3D_VU1_FAIL,boot_normal_dma_timeout,vif_stat=%08x,vif_err=%08x,vif_code=%08x\n",
               (unsigned)VIF1_STAT,(unsigned)VIF1_ERR,
               (unsigned)VIF1_CODE);
        s_status="VU1 FAIL BOOT";
        s_ready=0;
        return 0;
    }
    printf("BOX3D_VU1_PHASE,boot_normal_dma_done,vif_stat=%08x,vif_err=%08x\n",
           (unsigned)VIF1_STAT,(unsigned)VIF1_ERR);
    s_status="VU1 BOOT OK";
    return 1;
}

int b3Vu1BoxesSubmit(void)
{
    if(!s_ready || s_count==0)return 0;
    // Isolate the first full VU transform from legacy packet2 chain
    // assertions. In VUONLY there is exactly one box and XGKICK=0.
    // Use the already verified normal-DMA VIF1 path instead.
    if(!s_emitToGs && s_count==1)
    {
        const size_t words=b3Vu1BuildVuOnlyStream(
            s_vuOnlyStream,104,s_camera,
            s_upload[0].vertices,s_upload[0].colors,s_mainProgramPc);
        if(words!=104u)
        {
            puts("BOX3D_VU1_FAIL,vuonly_stream_layout");
            s_ready=0;
            s_status="VU1 FAIL VU PACK";
            return -1;
        }
        puts("BOX3D_VU1_PHASE,vuonly_normal_dma_begin");
        if(dma_channel_send_normal(DMA_CHANNEL_VIF1,s_vuOnlyStream,
                                   (int)(words/4u),0,0)<0 ||
           dma_channel_wait(DMA_CHANNEL_VIF1,B3_VU1_DMA_WAIT_LIMIT)<0)
        {
            printf("BOX3D_VU1_FAIL,vuonly_normal_dma_timeout,vif_stat=%08x,vif_err=%08x\n",
                   (unsigned)VIF1_STAT,(unsigned)VIF1_ERR);
            s_ready=0;
            s_status="VU1 FAIL VU DMA";
            return -1;
        }
        printf("BOX3D_VU1_PHASE,vuonly_normal_dma_done,vif_stat=%08x,vif_err=%08x\n",
               (unsigned)VIF1_STAT,(unsigned)VIF1_ERR);
        s_status="VU1 VUONLY OK";
        ++s_submissionCount;
        return 1;
    }
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
        s_status="VU1 FAIL SUBMIT";
        s_ready=0;
        return -1;
    }
    if(s_submissionCount<3)
        puts("BOX3D_VU1_PHASE,after_vif1_dma");
    ++s_submissionCount;
    return s_count;
}
