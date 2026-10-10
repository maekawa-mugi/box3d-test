// SPDX-License-Identifier: MIT
// Portable VIF1 MPG/UNPACK packet-format regression; run with host CC.
#include "vu1_vif_stream.h"
#include <assert.h>
#include <stdio.h>
int main(void){
    uint32_t image[4096],out[4608],indices[36*4],tag[4];
    for(unsigned i=0;i<4096;i++)image[i]=0xa0000000u+i;
    for(unsigned i=0;i<144;i++)indices[i]=i;
    for(unsigned i=0;i<4;i++)tag[i]=0xf0000000u+i;
    for(size_t count=2;count<=2048;count+=2){
        size_t words=b3Vu1BuildMpgStream(out,4608,image,count);
        assert(words && words%4==0);
        size_t pc=0,at=0;
        while(pc<count){size_t n=count-pc;if(n>256)n=256;
            assert(out[at++]==0);
            assert(out[at++]==(0x4a000000u|((n&255u)<<16)|pc));
            for(size_t i=0;i<n*2;i++)assert(out[at+i]==image[pc*2+i]);
            at+=n*2;
            assert(out[at++]==0 && out[at++]==0);
            pc+=n; }
        assert(at+4==words);
        assert(out[at++]==0x10000000u);
        for(int j=0;j<3;j++)assert(out[at++]==0);
    }
    assert(b3Vu1BuildMpgStream(out,3,image,2)==0);
    assert(b3Vu1BuildMpgStream(out,4608,image,3)==0);
    assert(b3Vu1BuildStaticStream(out,160,indices,tag)==160);
    assert(out[0]==0x01000101u && out[1]==0x6c244080u);
    for(int i=0;i<144;i++)assert(out[2+i]==indices[i]);
    assert(out[148]==0x01000101u && out[149]==0x6c014100u);
    for(int i=0;i<4;i++)assert(out[150+i]==tag[i]);
    assert(out[156]==0x10000000u);
    puts("PASS: VIF1 normal DMA MPG (2..2048 instructions) and static UNPACK stream");
    return 0;
}
