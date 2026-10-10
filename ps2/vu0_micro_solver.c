// SPDX-License-Identifier: MIT
// VU0 micro-mode contact probe. The existing Box3D solver stays unchanged
// until PCSX2 verifies this actual VCALLMS path and its numeric outputs.
#include "vu0_micro_solver.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#if defined(BOX3D_PS2) && defined(__mips__)

// PCSX2 EE MMIO layout, different from some misleading SDK aliases.
// The VU0 micro and data windows are each 4KB, mirrored at 16KB spans.
#define B3_VU0_MICRO_BASE ((volatile uint32_t*)0x11000000u)
#define B3_VU0_DATA_BASE  ((volatile uint32_t*)0x11004000u)
enum { B3_VU0_MICRO_WORD_CAP=1024, B3_VU0_WAIT_LIMIT=8000000 };

extern uint32_t B3Vu0Contacts_CodeStart __attribute__((section(".vudata")));
extern uint32_t B3Vu0Contacts_CodeEnd __attribute__((section(".vudata")));

static int s_uploaded,s_verified;
static const char* s_status="VU0 MICRO NOT INIT";
static _Alignas(64) b3Vu0MicroBlock s_microData;

static int b3Vu0WaitIdle(void)
{
    for(unsigned spins=0;spins<B3_VU0_WAIT_LIMIT;++spins)
    {
        uint32_t status;
        __asm__ __volatile__("cfc2 %0, $29" : "=r"(status) :: "memory");
        if((status&1u)==0)return 1;
    }
    return 0;
}

const char* b3Vu0MicroStatus(void){return s_status;}

int b3Vu0MicroUpload(void)
{
    s_uploaded=0;
    s_verified=0;
    s_status="VU0 MICRO UPLOAD";
    // The EE frame loop is stopped here, before any VU0/COP2 work.
    if(!b3Vu0WaitIdle())
    {
        s_status="VU0 FAIL BUSY";
        return 0;
    }
    uintptr_t start=(uintptr_t)&B3Vu0Contacts_CodeStart;
    uintptr_t end=(uintptr_t)&B3Vu0Contacts_CodeEnd;
    if(end<=start || end-start>4096u || ((end-start)&15u)!=0u)
    {
        printf("BOX3D_VU0_FAIL,code_layout,start=%08lx,end=%08lx\n",
               (unsigned long)start,(unsigned long)end);
        s_status="VU0 FAIL LAYOUT";
        return 0;
    }
    const uint32_t* source=&B3Vu0Contacts_CodeStart;
    size_t words=(end-start)/4u;
    printf("BOX3D_VU0_PHASE,micro_upload_begin,words=%u\n",(unsigned)words);
    volatile uint32_t* dst=B3_VU0_MICRO_BASE;
    for(size_t i=0;i<words;++i)dst[i]=source[i];
    __asm__ __volatile__("sync.l" ::: "memory");
    s_uploaded=1;
    s_status="VU0 MICRO UPLOADED";
    puts("BOX3D_VU0_PHASE,micro_upload_done");
    return 1;
}

int b3Vu0MicroVerify(void)
{
    if(!s_uploaded){s_status="VU0 FAIL NOT UPLOADED";return 0;}
    if(!b3Vu0WaitIdle()){s_status="VU0 FAIL BUSY";return 0;}
    s_verified=0;
    const uint32_t* source=&B3Vu0Contacts_CodeStart;
    size_t words=((uintptr_t)&B3Vu0Contacts_CodeEnd-
                  (uintptr_t)&B3Vu0Contacts_CodeStart)/4u;
    volatile const uint32_t* micro=B3_VU0_MICRO_BASE;
    puts("BOX3D_VU0_PHASE,micro_verify_begin");
    for(size_t i=0;i<words;++i)
    {
        uint32_t got=micro[i];
        if(got!=source[i])
        {
            printf("BOX3D_VU0_FAIL,micro_readback,index=%u,expected=%08x,got=%08x\n",
                   (unsigned)i,(unsigned)source[i],(unsigned)got);
            s_status="VU0 FAIL READBACK";
            return 0;
        }
    }
    s_verified=1;
    s_status="VU0 MICRO VERIFIED";
    puts("BOX3D_VU0_PHASE,micro_verify_done");
    return 1;
}

static int b3Vu0MicroRunBlock(b3Vu0MicroBlock* block)
{
    if(!s_verified || !b3Vu0WaitIdle())
        return 0;
    // Use volatile 32-bit stores into VU0 4KB DMEM. The 16 records
    // are 209 QWs, so the entire block fits with 752 bytes to spare.
    const uint32_t* in=(const uint32_t*)block;
    volatile uint32_t* vu=B3_VU0_DATA_BASE;
    for(size_t i=0;i<sizeof(*block)/4u;++i)vu[i]=in[i];
    __asm__ __volatile__("sync.l" ::: "memory");
    __asm__ __volatile__("vcallms 0" ::: "memory");
    if(!b3Vu0WaitIdle())
    {
        s_verified=0;
        s_status="VU0 FAIL EXEC TIMEOUT";
        return 0;
    }
    uint32_t* out=(uint32_t*)block;
    for(size_t i=0;i<sizeof(*block)/4u;++i)out[i]=vu[i];
    return 1;
}

int b3Vu0MicroDifferentialTest(void)
{
    if(!s_verified){s_status="VU0 FAIL NOT VERIFIED";return 0;}
    static _Alignas(64) b3Vu0MicroBlock expected;
    memset(&s_microData,0,sizeof(s_microData));
    for(unsigned g=0;g<B3_VU0_MICRO_GROUPS;++g)
    for(unsigned lane=0;lane<4;++lane)
    {
        b3Vu0MicroRecord* r=&s_microData.record[g];
        // Binary fractions ensure exact expected values without reliance
        // on EE/COP1 vs VU0 implementation-specific rounding.
        r->oldImpulse[lane]=1.0f+(float)(g%4u)*0.25f;
        r->normalMass[lane]=0.25f+(float)(lane%3u)*0.125f;
        r->normalSpeed[lane]=((int)lane-2)*0.25f+
                              (float)(g%5u)*0.125f;
        r->velocityBias[lane]=0.125f;
        r->oldPointTotal[lane]=(float)g*0.125f;
        r->oldManifoldTotal[lane]=(float)lane*0.25f;
        r->oldTwistLimit[lane]=0.5f;
        r->leverArm[lane]=0.5f;
    }
    memcpy(&expected,&s_microData,sizeof(expected));
    b3Vu0MicroReference(&expected);
    puts("BOX3D_VU0_PHASE,micro_execute_begin,groups=16,lanes=64");
    if(!b3Vu0MicroRunBlock(&s_microData))
    {
        puts("BOX3D_VU0_FAIL,micro_execution");
        return 0;
    }
    float worst=0.0f;
    unsigned mismatches=0u;
    for(unsigned g=0;g<B3_VU0_MICRO_GROUPS;++g)
    {
        const float* got=(const float*)&s_microData.record[g].newImpulse;
        const float* ref=(const float*)&expected.record[g].newImpulse;
        for(unsigned k=0;k<20u;++k)
        {
            float delta=fabsf(got[k]-ref[k]);
            if(delta>worst)worst=delta;
            if(!isfinite(got[k]) || delta>0.0001f)
            {
                if(mismatches<4)
                    printf("BOX3D_VU0_DIFF,group=%u,output=%u,expected=%.6f,got=%.6f\n",
                           g,k,(double)ref[k],(double)got[k]);
                ++mismatches;
            }
        }
    }
    if(mismatches)
    {
        printf("BOX3D_VU0_FAIL,micro_differential,mismatches=%u,worst=%f\n",
               mismatches,(double)worst);
        s_status="VU0 FAIL DIFF";
        return 0;
    }
    printf("BOX3D_VU0_PASS,backend=micro,contacts=64,outputs=320,worst=%f\n",
           (double)worst);
    s_status="VU0 MICRO DIFF OK";
    return 1;
}
#else
static const char* s_status="VU0 MICRO PS2 ONLY";
const char* b3Vu0MicroStatus(void){return s_status;}
int b3Vu0MicroUpload(void){return 0;}
int b3Vu0MicroVerify(void){return 0;}
int b3Vu0MicroDifferentialTest(void){return 0;}
#endif
