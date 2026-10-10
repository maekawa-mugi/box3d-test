// SPDX-License-Identifier: MIT
// VU0 micro-mode contact probe. The existing Box3D solver stays unchanged
// until PCSX2 verifies this actual VCALLMS path and its numeric outputs.
#include "vu0_micro_solver.h"
#include "../src/ps2_vu0.h"

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
static unsigned s_liveCalls;
static unsigned s_stepCalls;
static unsigned s_batchCalls;
static unsigned s_liveGroups;

void b3Vu0MicroBeginPhysicsStep(void)
{
    s_stepCalls=0;
    s_batchCalls=0;
}
_Static_assert(sizeof(b3Ps2VU0NormalSolveData)==sizeof(b3Vu0MicroRecord),
               "Box3D SIMD4 normal impulse ABI must match VU0 micro record");
_Static_assert(B3_PS2_VU0_BATCH_GROUPS==B3_VU0_MICRO_GROUPS,
               "Solver batch must fit the VU0 256-QW data memory");

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

static int b3Vu0MicroRunBlock(b3Vu0MicroBlock* block,unsigned groups)
{
    if(!s_verified || groups==0 || groups>B3_VU0_MICRO_GROUPS)
        return 0;
    if(!b3Vu0WaitIdle())
    {
        s_verified=0;
        s_status="VU0 FAIL BUSY";
        return 0;
    }
    // DMA is not required for VU0: the mapped 4KB VU0 data window
    // contains a bounded control QW followed by N 13-QW records.
    block->control[0]=groups;
    const uint32_t* in=(const uint32_t*)block;
    volatile uint32_t* vu=B3_VU0_DATA_BASE;
    const size_t inputWords=4u+groups*B3_VU0_MICRO_RECORD_QWS*4u;
    for(size_t i=0;i<inputWords;++i)vu[i]=in[i];
    __asm__ __volatile__("sync.l" ::: "memory");
    __asm__ __volatile__("vcallms 0" ::: "memory");
    if(!b3Vu0WaitIdle())
    {
        s_verified=0;
        s_status="VU0 FAIL EXEC TIMEOUT";
        return 0;
    }
    // No need to read input back: only five output QWs per group.
    uint32_t* out=(uint32_t*)block;
    for(unsigned g=0;g<groups;++g)
    {
        const size_t index=4u+g*B3_VU0_MICRO_RECORD_QWS*4u+8u*4u;
        for(unsigned i=0;i<20u;++i)out[index+i]=vu[index+i];
    }
    return 1;
}

int b3Vu0MicroIsReady(void){return s_verified;}
unsigned b3Vu0MicroGetLiveCalls(void){return s_liveCalls;}
unsigned b3Vu0MicroGetLiveGroups(void){return s_liveGroups;}

// Each unbatched VCALLMS is a synchronous EE/VU0 round-trip. In large
// awake scenes bound the number per physics step; use scalar thereafter
// without changing the user's selected mode or disabling valid microcode.
// Return 1 on success, 0 for a safe budget fallback, -1 on failure.
int b3Vu0MicroNormalSolve4(b3Ps2VU0NormalSolveData* record)
{
    if(!record || !s_verified)return -1;
    if(s_stepCalls>=B3_VU0_MICRO_MAX_LIVE_PER_STEP)return 0;
    memcpy(&s_microData.record[0],record,sizeof(*record));
    if(!b3Vu0MicroRunBlock(&s_microData,1u))return -1;
    // Validate real moving-contact inputs on startup, rather than
    // trusting a differential probe made only from binary fractions.
    // The comparison is bounded: first 64 calls and one every 1024.
    if(s_liveCalls<64u || (s_liveCalls&1023u)==0u)
    {
        const b3Vu0MicroRecord* got=&s_microData.record[0];
        for(unsigned lane=0;lane<4u;++lane)
        {
            float old=record->oldImpulse[lane];
            float next=old-record->normalMass[lane]*
                (record->normalSpeed[lane]+record->velocityBias[lane]);
            if(next<0.0f)next=0.0f;
            const float ref[5]={
                next,
                next-old,
                record->oldPointTotal[lane]+next,
                record->oldManifoldTotal[lane]+next,
                record->oldTwistLimit[lane]+record->leverArm[lane]*next
            };
            const float actual[5]={
                got->newImpulse[lane],got->deltaImpulse[lane],
                got->newPointTotal[lane],got->newManifoldTotal[lane],
                got->newTwistLimit[lane]
            };
            for(unsigned k=0;k<5u;++k)
            {
                // EE COP1 and VU0 can round differently on real values;
                // allow a relative error, but never allow NaN or INF.
                float tol=0.002f*(1.0f+fabsf(ref[k]));
                if(!isfinite(actual[k]) || !isfinite(ref[k]) ||
                   fabsf(actual[k]-ref[k])>tol)
                {
                    printf("BOX3D_VU0_FAIL,live_diff,call=%u,lane=%u,field=%u,ref=%f,got=%f\n",
                           s_liveCalls,lane,k,(double)ref[k],
                           (double)actual[k]);
                    s_verified=0;
                    s_status="VU0 FAIL LIVE DIFF";
                    return -1;
                }
            }
        }
    }
    memcpy(record,&s_microData.record[0],sizeof(*record));
    ++s_liveCalls;
    ++s_liveGroups;
    ++s_stepCalls;
    return 1;
}


/* One VCALLMS processes 1..16 independent SIMD4 constraints in a
 * verified 209-QW VU0 DMEM block. Both inputs and outputs are copied
 * only for active groups. The original caller data is never modified
 * until the entire output batch has passed numeric validation. */
int b3Vu0MicroNormalSolveBatch(b3Ps2VU0NormalSolveData* records,
                               unsigned groups)
{
    if(!records || !s_verified || groups==0u ||
       groups>B3_VU0_MICRO_GROUPS)return -1;
    if(s_batchCalls>=B3_VU0_MICRO_MAX_BATCHES_PER_STEP)return 0;
    for(unsigned g=0;g<groups;++g)
        memcpy(&s_microData.record[g],&records[g],sizeof(records[g]));
    if(!b3Vu0MicroRunBlock(&s_microData,groups))return -1;

    // Validate real non-binary contact values on the first 16 batches,
    // then sample every 256th batch; always reject nonfinite results.
    int compare=(s_liveCalls<16u || (s_liveCalls&255u)==0u);
    for(unsigned g=0;g<groups;++g)
    {
        const b3Vu0MicroRecord* got=&s_microData.record[g];
        const b3Ps2VU0NormalSolveData* input=&records[g];
        for(unsigned lane=0;lane<4u;++lane)
        {
            const float old=input->oldImpulse[lane];
            float next=old-input->normalMass[lane]*
                (input->normalSpeed[lane]+input->velocityBias[lane]);
            if(next<0.0f)next=0.0f;
            const float expected[5]={
                next,next-old,input->oldPointTotal[lane]+next,
                input->oldManifoldTotal[lane]+next,
                input->oldTwistLimit[lane]+input->leverArm[lane]*next
            };
            const float actual[5]={
                got->newImpulse[lane],got->deltaImpulse[lane],
                got->newPointTotal[lane],got->newManifoldTotal[lane],
                got->newTwistLimit[lane]
            };
            for(unsigned k=0;k<5u;++k)
            {
                const float tolerance=0.002f*(1.0f+fabsf(expected[k]));
                if(!isfinite(actual[k]) || !isfinite(expected[k]) ||
                   (compare && fabsf(actual[k]-expected[k])>tolerance))
                {
                    printf("BOX3D_VU0_FAIL,batch_diff,group=%u,lane=%u,field=%u,expected=%f,got=%f\n",
                           g,lane,k,(double)expected[k],(double)actual[k]);
                    s_verified=0;
                    s_status="VU0 FAIL BATCH DIFF";
                    return -1;
                }
            }
        }
    }

    for(unsigned g=0;g<groups;++g)
        memcpy(&records[g],&s_microData.record[g],sizeof(records[g]));
    ++s_liveCalls;
    ++s_batchCalls;
    s_liveGroups+=groups;
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
    if(!b3Vu0MicroRunBlock(&s_microData,B3_VU0_MICRO_GROUPS))
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
    // The verified 16-group path does not establish correctness of the
    // 1-group live VCALLMS. In particular, an immediate IADDIU -> IBNE
    // counter dependency can accidentally execute a second record.
    // Verify the actual live group count without adding a pad stage.
    for(unsigned lane=0;lane<4u;++lane)
    {
        b3Vu0MicroRecord* r=&s_microData.record[0];
        r->oldImpulse[lane]=0.5f+0.25f*(float)lane;
        r->normalMass[lane]=0.125f+0.125f*(float)lane;
        r->normalSpeed[lane]=-1.0f+0.25f*(float)lane;
        r->velocityBias[lane]=0.125f;
        r->oldPointTotal[lane]=0.25f;
        r->oldManifoldTotal[lane]=0.5f;
        r->oldTwistLimit[lane]=0.75f;
        r->leverArm[lane]=0.5f;
        // Stale 16-group results must not be mistaken for a new run.
        r->newImpulse[lane]=0.0f;
        r->deltaImpulse[lane]=0.0f;
        r->newPointTotal[lane]=0.0f;
        r->newManifoldTotal[lane]=0.0f;
        r->newTwistLimit[lane]=0.0f;
    }
    memcpy(&expected,&s_microData,sizeof(expected));
    b3Vu0MicroReference(&expected);

    // QW[1+13+8] is the first output of the *next* record.
    // b3Vu0MicroRunBlock(groups=1) uploads only QW[0..13].
    // An off-by-one VU loop must not overwrite this sentinel.
    volatile uint32_t* vu=B3_VU0_DATA_BASE;
    const size_t guardWord=4u+B3_VU0_MICRO_RECORD_QWS*4u+8u*4u;
    const uint32_t sentinel=0x48234567u;
    vu[guardWord]=sentinel;
    __asm__ __volatile__("sync.l" ::: "memory");
    if(!b3Vu0MicroRunBlock(&s_microData,1u))
    {
        s_verified=0;
        s_status="VU0 FAIL LIVE GROUP";
        puts("BOX3D_VU0_FAIL,one_group_execute");
        return 0;
    }
    for(unsigned k=0;k<20u;++k)
    {
        float got=((const float*)&s_microData.record[0].newImpulse)[k];
        float want=((const float*)&expected.record[0].newImpulse)[k];
        if(!isfinite(got) || fabsf(got-want)>0.0001f)
        {
            printf("BOX3D_VU0_FAIL,one_group_diff,field=%u,expected=%f,got=%f\n",
                   k,(double)want,(double)got);
            s_verified=0;
            s_status="VU0 FAIL LIVE GROUP";
            return 0;
        }
    }
    if(vu[guardWord]!=sentinel)
    {
        printf("BOX3D_VU0_FAIL,one_group_overrun,guard=%08x\n",
               (unsigned)vu[guardWord]);
        s_verified=0;
        s_status="VU0 FAIL GROUP OVERRUN";
        return 0;
    }
    printf("BOX3D_VU0_PASS,backend=micro,contacts=64,outputs=320,one_group=ok,worst=%f\n",
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
void b3Vu0MicroBeginPhysicsStep(void){}
int b3Vu0MicroIsReady(void){return 0;}
unsigned b3Vu0MicroGetLiveCalls(void){return 0;}
unsigned b3Vu0MicroGetLiveGroups(void){return 0;}
int b3Vu0MicroNormalSolve4(b3Ps2VU0NormalSolveData* data)
{(void)data;return 0;}
int b3Vu0MicroNormalSolveBatch(b3Ps2VU0NormalSolveData* records,unsigned groups)
{(void)records;(void)groups;return -1;}
#endif
