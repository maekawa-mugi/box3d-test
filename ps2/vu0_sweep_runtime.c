// SPDX-License-Identifier: MIT
// Real VU0 4-point sequential contact normal sweep. Promoted only after
// on-hardware 1- and 7-group differential validation; old micro restored
// automatically when startup promotion fails.
#include "vu0_sweep.h"
#include "vu0_micro_solver.h"
#include "vu0_mmio_transfer.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#if defined(BOX3D_PS2) && defined(__mips__)
#define SWEEP_MICRO ((volatile uint32_t*)0x11000000u)
#define SWEEP_DATA ((volatile uint32_t*)0x11004000u)
extern uint32_t B3Vu0Sweep_CodeStart __attribute__((section(".vudata")));
extern uint32_t B3Vu0Sweep_CodeEnd __attribute__((section(".vudata")));
static int s_resident,s_ready;
static const char* s_status="VU0 SWEEP NOT INIT";
static unsigned s_epoch,s_stepCalls,s_liveCalls,s_liveGroups;
static int s_idleKnown;
static _Alignas(64) b3Vu0SweepBlock s_result,s_expected;
static int waitIdle(void)
{
    for(unsigned i=0;i<8000000u;++i){
        uint32_t stat;
        __asm__ __volatile__("cfc2 %0, $29":"=r"(stat)::"memory");
        if(!(stat&1u))return 1;
    }
    return 0;
}
int b3Vu0SweepResident(void){return s_resident;}
int b3Vu0SweepIsReady(void){return s_resident&&s_ready;}
const char* b3Vu0SweepStatus(void){return s_status;}
unsigned b3Vu0SweepGetLiveCalls(void){return s_liveCalls;}
unsigned b3Vu0SweepGetLiveGroups(void){return s_liveGroups;}
unsigned b3Vu0SweepStepEpoch(void){return s_epoch;}
void b3Vu0SweepBeginPhysicsStep(void){++s_epoch;s_stepCalls=0;s_idleKnown=0;}
static int runSweep(const b3Vu0SweepRecord* input,b3Vu0SweepRecord* output,unsigned count)
{
    if(!count||count>B3_VU0_SWEEP_GROUPS)return 0;
    // A completed VCALLMS already verified VU0 idle. Only the first
    // launch of each physics step requires a new preflight poll.
    if(!s_idleKnown && !waitIdle())return 0;
    s_idleKnown=0;
    SWEEP_DATA[0]=count;
    const unsigned char* src=(const unsigned char*)input;
    volatile unsigned char* dst=(volatile unsigned char*)SWEEP_DATA+16u;
    for(unsigned g=0;g<count;++g){
        size_t offset=g*B3_VU0_SWEEP_RECORD_QWS*16u;
        for(unsigned q=0;q<20u;q+=4u)
            b3Vu0Push64(dst+offset+q*16u,src+offset+q*16u);
        // The two remaining QWs have no output-region overlap.
        const uint32_t* words=(const uint32_t*)(const void*)(src+offset+320u);
        volatile uint32_t* mmio=(volatile uint32_t*)(void*)(dst+offset+320u);
        for(unsigned i=0;i<8u;++i)mmio[i]=words[i];
    }
    __asm__ __volatile__("sync.l":::"memory");
    __asm__ __volatile__("vcallms 0":::"memory");
    if(!waitIdle())return 0;
    s_idleKnown=1;
    unsigned char* out=(unsigned char*)output;
    for(unsigned g=0;g<count;++g){
        size_t offset=(g*B3_VU0_SWEEP_RECORD_QWS+
                      B3_VU0_SWEEP_INPUT_QWS)*16u;
        b3Vu0Pull64(out+offset,dst+offset);
        b3Vu0Pull64(out+offset+64u,dst+offset+64u);
    }
    return 1;
}
static int check(const b3Vu0SweepRecord* input,const b3Vu0SweepRecord* result,
                 unsigned count,int compare)
{
    for(unsigned g=0;g<count;++g){
        if(compare){
            memcpy(&s_expected.record[g],&input[g],sizeof(input[g]));
            b3Vu0SweepReference(&s_expected.record[g]);
        }
        const float* actual=(const float*)result[g].next;
        const float* expected=(const float*)s_expected.record[g].next;
        for(unsigned k=0;k<32u;++k){
            if(!isfinite(actual[k])||
               (compare&&(!isfinite(expected[k])||
                fabsf(actual[k]-expected[k])>0.002f*(1.0f+fabsf(expected[k]))))){
                printf("BOX3D_VU0_SWEEP_FAIL,g=%u,k=%u,got=%f,ref=%f\n",
                       g,k,(double)actual[k],compare?(double)expected[k]:0.0);
                s_status="VU0 FAIL SWEEP DIFF";s_ready=0;return 0;
            }
        }
    }
    return 1;
}
int b3Vu0SweepPromote(void)
{
    if(!b3Vu0MicroIsReady())return 0;
    s_resident=0;s_ready=0;s_idleKnown=0;s_status="VU0 SWEEP VERIFY";
    uintptr_t start=(uintptr_t)&B3Vu0Sweep_CodeStart;
    uintptr_t end=(uintptr_t)&B3Vu0Sweep_CodeEnd;
    if(end<=start||end-start>4096u||((end-start)&15u)!=0u||!waitIdle())
        goto restore;
    const uint32_t* code=&B3Vu0Sweep_CodeStart;
    unsigned words=(unsigned)((end-start)/4u);
    for(unsigned i=0;i<words;++i)SWEEP_MICRO[i]=code[i];
    __asm__ __volatile__("sync.l":::"memory");
    s_resident=1;
    for(unsigned i=0;i<words;++i)if(SWEEP_MICRO[i]!=code[i])goto restore;
    for(unsigned test=0;test<2u;++test){
        unsigned count=test?B3_VU0_SWEEP_GROUPS:1u;
        memset(&s_result,0,sizeof(s_result));
        for(unsigned g=0;g<count;++g){
            b3Vu0SweepRecord* r=&s_result.record[g];
            for(unsigned lane=0;lane<4u;++lane){
                for(unsigned p=0;p<4u;++p){
                    r->old[p][lane]=0.25f*(float)(p+1u+g%3u);
                    r->mass[p][lane]=0.25f;
                    r->speed[p][lane]=0.25f*(float)((int)p-(int)lane);
                    r->bias[p][lane]=0.125f*(float)(p%2u);
                }
                for(unsigned k=0;k<6u;++k)r->coupling[k][lane]=0.125f*(float)(1u+k%3u);
            }
        }
        volatile uint32_t* guard=SWEEP_DATA+
            4u+B3_VU0_SWEEP_RECORD_QWS*4u+B3_VU0_SWEEP_INPUT_QWS*4u;
        if(count==1u)*guard=0x5a325a32u;
        if(!runSweep(s_result.record,s_result.record,count)||
           !check(s_result.record,s_result.record,count,1)||
           (count==1u&&*guard!=0x5a325a32u))goto restore;
    }
    s_ready=1;s_status="VU0 SWEEP READY";
    puts("BOX3D_VU0_SWEEP_PASS,groups=7,points=4,lanes=4");
    return 1;
restore:
    s_resident=0;s_ready=0;s_idleKnown=0;
    puts("BOX3D_VU0_SWEEP_FALLBACK,restore_64_lane_kernel");
    if(!b3Vu0MicroUpload()||!b3Vu0MicroVerify()||!b3Vu0MicroDifferentialTest())
        puts("BOX3D_VU0_SWEEP_FALLBACK,legacy_restore_failed");
    return 0;
}
int b3Vu0SweepSolve(b3Vu0SweepRecord* records,unsigned count)
{
    if(!s_resident||!s_ready||!records||!count||count>B3_VU0_SWEEP_GROUPS)
        return -1;
    if(s_stepCalls>=512u)return 0;
    if(!runSweep(records,s_result.record,count)){
        s_ready=0;s_status="VU0 FAIL SWEEP BUSY";return -1;
    }
    int compare=s_liveCalls<16u||(s_liveCalls&255u)==0u;
    if(!check(records,s_result.record,count,compare))return -1;
    for(unsigned g=0;g<count;++g){
        memcpy(records[g].next,s_result.record[g].next,sizeof(records[g].next));
        memcpy(records[g].delta,s_result.record[g].delta,sizeof(records[g].delta));
    }
    ++s_stepCalls;++s_liveCalls;s_liveGroups+=count;
    return 1;
}
#else
int b3Vu0SweepResident(void){return 0;}
int b3Vu0SweepIsReady(void){return 0;}
const char* b3Vu0SweepStatus(void){return "VU0 SWEEP PS2 ONLY";}
unsigned b3Vu0SweepGetLiveCalls(void){return 0;}
unsigned b3Vu0SweepGetLiveGroups(void){return 0;}
unsigned b3Vu0SweepStepEpoch(void){return 0;}
void b3Vu0SweepBeginPhysicsStep(void){}
int b3Vu0SweepPromote(void){return 0;}
int b3Vu0SweepSolve(b3Vu0SweepRecord* records,unsigned count)
{(void)records;(void)count;return -1;}
#endif
