// SPDX-License-Identifier: MIT
// First real VU0 micro-mode contact arithmetic batch, 64 lanes per VCALLMS.
// This has no Box3D solver side effects until differential validation passes.
#pragma once
#include <stddef.h>
#include <stdint.h>

// 16 independent groups of 4 scalar contact normals. Inputs are eight
// 16-byte vectors, followed by five output vectors: 13 QWs/record.
// 16 records + one control QW = 209 QWs = 3344 / 4096 VU0 DMEM bytes.
enum { B3_VU0_MICRO_GROUPS=16, B3_VU0_MICRO_LANES=4,
       B3_VU0_MICRO_INPUT_QWS=8, B3_VU0_MICRO_OUTPUT_QWS=5,
       B3_VU0_MICRO_RECORD_QWS=13, B3_VU0_MICRO_TOTAL_QWS=209 };

typedef struct b3Vu0MicroRecord
{
    _Alignas(16) float oldImpulse[4];       // offset QW+0
    float normalMass[4];                    // +1
    float normalSpeed[4];                   // +2
    float velocityBias[4];                  // +3
    float oldPointTotal[4];                 // +4
    float oldManifoldTotal[4];              // +5
    float oldTwistLimit[4];                 // +6
    float leverArm[4];                      // +7
    float newImpulse[4];                    // +8
    float deltaImpulse[4];                  // +9
    float newPointTotal[4];                 // +10
    float newManifoldTotal[4];              // +11
    float newTwistLimit[4];                 // +12
} b3Vu0MicroRecord;

typedef struct b3Vu0MicroBlock
{
    _Alignas(16) uint32_t control[4];       // QW 0, reserved
    b3Vu0MicroRecord record[B3_VU0_MICRO_GROUPS];
} b3Vu0MicroBlock;

_Static_assert(_Alignof(b3Vu0MicroRecord)>=16,"VU0 record alignment");
_Static_assert(sizeof(b3Vu0MicroRecord)==13u*16u,"VU0 record stride");
_Static_assert(offsetof(b3Vu0MicroRecord,newImpulse)==8u*16u,
               "VU0 outputs begin at QW+8");
_Static_assert(_Alignof(b3Vu0MicroBlock)>=16,"VU0 block alignment");
_Static_assert(sizeof(b3Vu0MicroBlock)==B3_VU0_MICRO_TOTAL_QWS*16u,
               "VU0 block size");

// Box3D's mode-5 normal impulse step, same arithmetic order as scalar.
// Kept in a standalone header for portable host differential tests.
// Input, output layout and source association are the microprogram ABI.
static inline void b3Vu0MicroReference(b3Vu0MicroBlock* block)
{
    for(size_t g=0;g<B3_VU0_MICRO_GROUPS;++g)
    for(size_t lane=0;lane<B3_VU0_MICRO_LANES;++lane)
    {
        b3Vu0MicroRecord* r=&block->record[g];
        const float impulse=r->oldImpulse[lane];
        const float negative=r->normalMass[lane]*
            (r->normalSpeed[lane]+r->velocityBias[lane]);
        const float unclamped=impulse-negative;
        const float next=unclamped>=0.0f?unclamped:0.0f;
        r->newImpulse[lane]=next;
        r->deltaImpulse[lane]=next-impulse;
        r->newPointTotal[lane]=r->oldPointTotal[lane]+next;
        r->newManifoldTotal[lane]=r->oldManifoldTotal[lane]+next;
        r->newTwistLimit[lane]=r->oldTwistLimit[lane]+r->leverArm[lane]*next;
    }
}

// Only the 4 explicit, opt-in diagnostic stages touch VU0 hardware.
// Never call during an active VU0 microprogram or a Box3D world step.
const char* b3Vu0MicroStatus(void);
int b3Vu0MicroUpload(void);
int b3Vu0MicroVerify(void);
int b3Vu0MicroDifferentialTest(void);
