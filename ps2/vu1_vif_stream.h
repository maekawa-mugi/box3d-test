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
