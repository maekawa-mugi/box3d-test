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
    uint32_t camera[28];
    for(unsigned i=0;i<28;i++)camera[i]=0xc0000000u+i;
    assert(b3Vu1BuildCameraStream(out,35,camera)==0);
    assert(b3Vu1BuildCameraStream(out,36,camera)==36);
    assert(out[0]==0x01000101u && out[1]==0x6c074000u);
    for(unsigned i=0;i<28;i++)assert(out[2+i]==camera[i]);
    assert(out[30]==0 && out[31]==0 && out[32]==0x10000000u);
    assert(out[33]==0 && out[34]==0 && out[35]==0);
    assert(b3Vu1BuildBootStream(out,3)==0);
    assert(b3Vu1BuildBootStream(out,4)==4);
    assert(out[0]==0x11000000u && out[1]==0x14000000u);
    assert(out[2]==0x10000000u && out[3]==0);
    uint32_t vertices[32],colors[24];
    for(unsigned i=0;i<32;i++)vertices[i]=0x70000000u+i;
    for(unsigned i=0;i<24;i++)colors[i]=0x80000000u+i;
    assert(b3Vu1BuildVuOnlyStream(out,103,camera,vertices,colors,2)==0);
    assert(b3Vu1BuildVuOnlyStream(out,104,camera,vertices,colors,2)==104);
    assert(out[36]==0x01000101u && out[37]==0x6c084010u);
    for(unsigned i=0;i<32;i++)assert(out[38+i]==vertices[i]);
    assert(out[70]==0 && out[71]==0);
    assert(out[72]==0x01000101u && out[73]==0x6c064018u);
    for(unsigned i=0;i<24;i++)assert(out[74+i]==colors[i]);
    assert(out[98]==0 && out[99]==0);
    assert(out[100]==0x11000000u && out[101]==0x14000002u);
    assert(out[102]==0x10000000u && out[103]==0);
    assert(b3Vu1BuildVuOnlyStream(out,104,camera,vertices,colors,65536u)==0);
    puts("PASS: VIF1 MPG, UNPACK, BOOT, VUONLY single-box normal DMA");
    return 0;
}
