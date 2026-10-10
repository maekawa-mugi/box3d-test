// SPDX-License-Identifier: MIT
// Portable VIF1 normal-DMA stream builder. No DMAC REF/TTE or packet2 asserts.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// A normal DMA VIF packet is a sequence of 32-bit VIF command words.
// NOP+MPG is followed immediately by 64-bit instructions, then NOPs
// for 128-bit DMA alignment. A NUM field of zero means 256 instructions.
static inline size_t b3Vu1BuildMpgStream(uint32_t* out,size_t capacityWords,
                                         const void* image,size_t instructions)
{
    if(!out || !image || instructions<2u || instructions>2048u ||
       (instructions&1u))return 0;
    size_t used=0,pos=0;
    while(pos<instructions)
    {
        size_t n=instructions-pos;
        if(n>256u)n=256u;
        if(n&1u)--n;
        if(!n || used+2u+n*2u+2u+4u>capacityWords)return 0;
        out[used++]=0; // VIF NOP
        out[used++]=0x4a000000u | ((uint32_t)(n&255u)<<16) |
                    (uint32_t)pos; // VIF MPG, micro-PC in instructions
        memcpy(out+used,(const unsigned char*)image+pos*8u,n*8u);
        used+=n*2u;
        out[used++]=0; // VIF NOP padding (MPG count already consumed)
        out[used++]=0;
        pos+=n;
    }
    out[used++]=0x10000000u; // VIF FLUSHE
    out[used++]=0;
    out[used++]=0;
    out[used++]=0;
    return used;
}

// Upload 36 index/face QWs to VU1 DMEM[128] and a GIF tag QW to [256].
// VIF1 UNPACK V4_32 is unsigned, non-double-buffered, unmasked.
static inline size_t b3Vu1BuildStaticStream(uint32_t* out,size_t capacityWords,
                                            const void* indices,
                                            const void* gifTag)
{
    if(!out || !indices || !gifTag || capacityWords<160u)return 0;
    size_t n=0;
    out[n++]=0x01000101u; // STCYCL CL=1 WL=1
    out[n++]=0x6c000000u | (36u<<16) | 0x4000u | 128u;
    memcpy(out+n,indices,36u*16u);
    n+=36u*4u;
    out[n++]=0;
    out[n++]=0;
    out[n++]=0x01000101u;
    out[n++]=0x6c000000u | (1u<<16) | 0x4000u | 256u;
    memcpy(out+n,gifTag,16u);
    n+=4u;
    out[n++]=0;
    out[n++]=0;
    out[n++]=0x10000000u;
    out[n++]=0;
    out[n++]=0;
    out[n++]=0;
    return n;
}

/* 7 camera QWs UNPACK V4_32 at VU1 DMEM[0], with FLUSHE. */
static inline size_t b3Vu1BuildCameraStream(uint32_t* out,
                                           size_t capacityWords,
                                           const void* camera)
{
    if(!out || !camera || capacityWords<36u)return 0;
    out[0]=0x01000101u; // STCYCL CL=WL=1
    out[1]=0x6c074000u; // UNSIGNED UNPACK V4_32, NUM=7, addr=0
    memcpy(out+2,camera,7u*16u);
    out[30]=0;out[31]=0;
    out[32]=0x10000000u; // FLUSHE
    out[33]=0;out[34]=0;out[35]=0;
    return 36u; // 9 quadwords
}

/* One QW: FLUSH, MSCAL PC=0, FLUSHE, NOP. */
static inline size_t b3Vu1BuildBootStream(uint32_t* out,
                                         size_t capacityWords)
{
    if(!out || capacityWords<4u)return 0;
    out[0]=0x11000000u;
    out[1]=0x14000000u;
    out[2]=0x10000000u;
    out[3]=0;
    return 4u;
}

/*
 * Single-box VUONLY test: one safe VIF1 normal-DMA stream.
 * No REF/TTE tags and NO XGKICK. Camera is followed by 8 vertices
 * and 6 colors, then FLUSH, MSCAL(entry), FLUSHE, NOP.
 * Camera includes FLUSHE and is harmless before the later UNPACKs.
 */
static inline size_t b3Vu1BuildVuOnlyStream(uint32_t* out,
                                            size_t capacityWords,
                                            const void* camera,
                                            const void* vertices,
                                            const void* colors,
                                            uint32_t entryPc)
{
    if(!out || !camera || !vertices || !colors ||
       capacityWords<104u || entryPc>0xffffu)return 0;
    size_t at=b3Vu1BuildCameraStream(out,capacityWords,camera);
    if(at!=36u)return 0;
    out[at++]=0x01000101u; // STCYCL
    out[at++]=0x6c084010u; // UNPACK V4_32 NUM=8 USN=1 DMEM[16]
    memcpy(out+at,vertices,8u*16u);
    at+=32u;
    out[at++]=0;
    out[at++]=0;
    out[at++]=0x01000101u;
    out[at++]=0x6c064018u; // UNPACK V4_32 NUM=6 USN=1 DMEM[24]
    memcpy(out+at,colors,6u*16u);
    at+=24u;
    out[at++]=0;
    out[at++]=0;
    out[at++]=0x11000000u; // FLUSH, wait until VU is idle
    out[at++]=0x14000000u | entryPc; // MSCAL
    out[at++]=0x10000000u; // FLUSHE, wait for E-bit termination
    out[at++]=0;
    return at; // 104 words, 26 quadwords
}

/* Whole-frame VIF1 normal DMA: shared camera, one MSCAL per box.
 * FLUSHA (0x13) waits for XGKICK GIF PATH1 before the next box
 * overwrites the output buffer in VU1 DMEM[256..328]. */
#define B3_VU1_MAX_STREAM_BOXES 400u
#define B3_VU1_SUBMIT_MAX_WORDS (36u+68u*B3_VU1_MAX_STREAM_BOXES)
static inline size_t b3Vu1BuildBoxBatchStream(
    uint32_t* out,size_t capacityWords,const void* camera,
    const void* boxInputs,size_t stride,size_t count,
    uint32_t entryPc,int emitToGs)
{
    if(!out || !camera || !boxInputs || count==0 ||
       count>B3_VU1_MAX_STREAM_BOXES || stride<224u ||
       entryPc>0xffffu || capacityWords<36u+68u*count)return 0;
    size_t n=b3Vu1BuildCameraStream(out,capacityWords,camera);
    if(n!=36u)return 0;
    const unsigned char* box=(const unsigned char*)boxInputs;
    for(size_t i=0;i<count;++i,box+=stride)
    {
        out[n++]=0x01000101u; // STCYCL
        out[n++]=0x6c084010u; // V4_32, NUM=8, VU DMEM[16]
        memcpy(out+n,box,128u);n+=32u;
        out[n++]=0;out[n++]=0;
        out[n++]=0x01000101u;
        out[n++]=0x6c064018u; // V4_32, NUM=6, VU DMEM[24]
        memcpy(out+n,box+128u,96u);n+=24u;
        out[n++]=0;out[n++]=0;
        out[n++]=0x11000000u; // FLUSH
        out[n++]=0x14000000u|entryPc; // MSCAL
        out[n++]=emitToGs?0x13000000u:0x10000000u;
        out[n++]=0; // NOP
    }
    return n;
}
