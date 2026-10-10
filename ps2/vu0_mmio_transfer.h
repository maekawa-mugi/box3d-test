// SPDX-License-Identifier: MIT
// Dedicated aligned 128-bit transfer between EE RAM and VU0 data memory.
// The hot path is adapted from the LQ/SQ *technique* investigated in
// maekawa-mugi/ps2-libc-arena (ee_hand_memcpy.S / ee_fast_memcpy.S).
// Unlike general memcpy: the VU0 layout is fixed, non-overlapping and
// always 16-byte aligned. Do not use generic newlib or cached memcpy
// on this volatile EE MMIO region. No early-EE branch delay loops.
#pragma once
#include "vu0_micro_solver.h"
#include <stdint.h>
#include <stddef.h>

// Define B3_VU0_FORCE_WORD_MMIO=1 to restore scalar MMIO transfers
// without changing the microcode or the TRIANGLE controller workflow.
#if defined(BOX3D_PS2) && defined(__mips__) && !defined(B3_VU0_FORCE_WORD_MMIO)
#define B3_VU0_QUAD_MMIO 1
#else
#define B3_VU0_QUAD_MMIO 0
#endif

static inline void b3Vu0Push64(volatile unsigned char* dst,
                               const unsigned char* src)
{
#if B3_VU0_QUAD_MMIO
    __asm__ __volatile__(
        "lq $8,0(%0)\n\t"
        "lq $9,16(%0)\n\t"
        "lq $10,32(%0)\n\t"
        "lq $11,48(%0)\n\t"
        "sq $8,0(%1)\n\t"
        "sq $9,16(%1)\n\t"
        "sq $10,32(%1)\n\t"
        "sq $11,48(%1)\n\t"
        : : "r"(src),"r"(dst) : "$8","$9","$10","$11","memory");
#else
    volatile uint32_t* d=(volatile uint32_t*)(void*)dst;
    const uint32_t* s=(const uint32_t*)(const void*)src;
    for(unsigned i=0;i<16u;++i)d[i]=s[i];
#endif
}

static inline void b3Vu0Pull64(unsigned char* dst,
                               const volatile unsigned char* src)
{
#if B3_VU0_QUAD_MMIO
    __asm__ __volatile__(
        "lq $8,0(%1)\n\t"
        "lq $9,16(%1)\n\t"
        "lq $10,32(%1)\n\t"
        "lq $11,48(%1)\n\t"
        "sq $8,0(%0)\n\t"
        "sq $9,16(%0)\n\t"
        "sq $10,32(%0)\n\t"
        "sq $11,48(%0)\n\t"
        : : "r"(dst),"r"(src) : "$8","$9","$10","$11","memory");
#else
    uint32_t* d=(uint32_t*)(void*)dst;
    const volatile uint32_t* s=(const volatile uint32_t*)(const volatile void*)src;
    for(unsigned i=0;i<16u;++i)d[i]=s[i];
#endif
}

static inline void b3Vu0Pull16(unsigned char* dst,
                               const volatile unsigned char* src)
{
#if B3_VU0_QUAD_MMIO
    __asm__ __volatile__(
        "lq $8,0(%1)\n\t"
        "sq $8,0(%0)\n\t"
        : : "r"(dst),"r"(src) : "$8","memory");
#else
    uint32_t* d=(uint32_t*)(void*)dst;
    const volatile uint32_t* s=(const volatile uint32_t*)(const volatile void*)src;
    for(unsigned i=0;i<4u;++i)d[i]=s[i];
#endif
}

// VU0 QW 0.x holds the group count. Every active record has exactly
// eight input QWs and five output QWs. Never write the output region,
// and never touch DMEM slots belonging to an inactive group.
static inline void b3Vu0PushRecordInputs(volatile uint32_t* vu,
                                          const void* records,
                                          unsigned groups)
{
    vu[0]=groups;
    const unsigned char* input=(const unsigned char*)records;
    volatile unsigned char* memory=(volatile unsigned char*)vu+16u;
    for(unsigned g=0;g<groups;++g)
    {
        size_t offset=(size_t)g*B3_VU0_MICRO_RECORD_QWS*16u;
        b3Vu0Push64(memory+offset,input+offset);
        b3Vu0Push64(memory+offset+64u,input+offset+64u);
    }
}

static inline void b3Vu0PullRecordOutputs(void* records,
                                           const volatile uint32_t* vu,
                                           unsigned groups)
{
    unsigned char* output=(unsigned char*)records;
    const volatile unsigned char* memory=(const volatile unsigned char*)vu+16u;
    for(unsigned g=0;g<groups;++g)
    {
        size_t offset=(size_t)g*B3_VU0_MICRO_RECORD_QWS*16u+
                      B3_VU0_MICRO_INPUT_QWS*16u;
        b3Vu0Pull64(output+offset,memory+offset);
        b3Vu0Pull16(output+offset+64u,memory+offset+64u);
    }
}
