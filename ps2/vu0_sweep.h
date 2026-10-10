// SPDX-License-Identifier: MIT
// Four sequential manifold normal points per independent SIMD4 group.
// Each point's accumulated impulse affects the later points via J M^-1 J^T.
// This is a genuinely coupled Gauss-Seidel sweep in VU0 micro mode,
// rather than four EE/VU0 round trips and repeated EE velocity updates.
#pragma once
#include <stddef.h>
#include <stdint.h>

enum { B3_VU0_SWEEP_GROUPS=8, B3_VU0_SWEEP_POINTS=4,
       B3_VU0_SWEEP_INPUT_QWS=22, B3_VU0_SWEEP_OUTPUT_QWS=8,
       B3_VU0_SWEEP_RECORD_QWS=30,
       B3_VU0_SWEEP_TOTAL_QWS=1+B3_VU0_SWEEP_GROUPS*B3_VU0_SWEEP_RECORD_QWS };
typedef struct b3Vu0SweepRecord
{
    _Alignas(16) float old[4][4];
    float mass[4][4];
    float speed[4][4];
    float bias[4][4];
    // Influence of delta impulse at point q on speed of point p>q:
    // 10,20,30,21,31,32. Precomputed once per world step per constraint.
    float coupling[6][4];
    float next[4][4];
    float delta[4][4];
} b3Vu0SweepRecord;
typedef struct b3Vu0SweepBlock
{
    _Alignas(16) uint32_t control[4];
    b3Vu0SweepRecord record[B3_VU0_SWEEP_GROUPS];
} b3Vu0SweepBlock;

_Static_assert(sizeof(b3Vu0SweepRecord)==30u*16u,"sweep ABI stride");
_Static_assert(offsetof(b3Vu0SweepRecord,coupling)==16u*16u,"sweep K offset");
_Static_assert(offsetof(b3Vu0SweepRecord,next)==22u*16u,"sweep output offset");
_Static_assert(offsetof(b3Vu0SweepRecord,delta)==26u*16u,"sweep delta offset");
_Static_assert(sizeof(b3Vu0SweepBlock)==B3_VU0_SWEEP_TOTAL_QWS*16u,"sweep DMEM size");
_Static_assert(B3_VU0_SWEEP_TOTAL_QWS<=256u,"sweep VU0 DMEM capacity");

// Scalar oracle and safe fallback. This reference runs the same four
// ordered point updates with each lane independent.
static inline void b3Vu0SweepReference(b3Vu0SweepRecord* r)
{
    for(unsigned lane=0;lane<4u;++lane)
    {
        float vn[4];
        for(unsigned p=0;p<4u;++p)vn[p]=r->speed[p][lane];
        for(unsigned p=0;p<4u;++p)
        {
            float old=r->old[p][lane];
            float neg=r->mass[p][lane]*vn[p]+r->bias[p][lane];
            float next=old-neg;
            if(next<0.0f)next=0.0f;
            float delta=next-old;
            r->next[p][lane]=next;
            r->delta[p][lane]=delta;
            for(unsigned q=p+1u;q<4u;++q)
            {
                unsigned k=(q==1?0u:q==2?(p==0?1u:3u):
                            p==0?2u:p==1?4u:5u);
                vn[q]+=r->coupling[k][lane]*delta;
            }
        }
    }
}
int b3Vu0SweepIsReady(void);
int b3Vu0SweepPromote(void);
int b3Vu0SweepSolve(b3Vu0SweepRecord* records,unsigned groups);
unsigned b3Vu0SweepStepEpoch(void);

// The legacy driver checks which microprogram currently owns VU0 IMEM.
int b3Vu0SweepResident(void);
const char* b3Vu0SweepStatus(void);
unsigned b3Vu0SweepGetLiveCalls(void);
unsigned b3Vu0SweepGetLiveGroups(void);
void b3Vu0SweepBeginPhysicsStep(void);
