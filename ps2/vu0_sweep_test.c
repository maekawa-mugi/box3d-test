// SPDX-License-Identifier: MIT
#include "vu0_sweep.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int main(void)
{
    assert(B3_VU0_SWEEP_TOTAL_QWS==241u);
    assert(sizeof(b3Vu0SweepRecord)==480u);
    assert(offsetof(b3Vu0SweepRecord,next)==352u);
    assert(offsetof(b3Vu0SweepRecord,delta)==416u);
    b3Vu0SweepBlock x={0};
    for(unsigned seed=0;seed<64u;++seed)
    for(unsigned g=0;g<B3_VU0_SWEEP_GROUPS;++g){
        b3Vu0SweepRecord* r=&x.record[g];
        memset(r,0,sizeof(*r));
        for(unsigned lane=0;lane<4u;++lane){
            for(unsigned p=0;p<4u;++p){
                r->old[p][lane]=0.25f*(float)(p+1u);
                r->mass[p][lane]=0.125f*(float)(lane+1u);
                r->speed[p][lane]=0.25f*(float)((int)p-(int)lane);
                r->bias[p][lane]=0.125f*(float)(p%2u);
            }
            for(unsigned k=0;k<6u;++k)r->coupling[k][lane]=0.0625f*(float)(k+1u);
        }
        b3Vu0SweepReference(r);
        for(unsigned lane=0;lane<4u;++lane){
            float speed[4];
            for(unsigned p=0;p<4u;++p)speed[p]=r->speed[p][lane];
            for(unsigned p=0;p<4u;++p){
                float old=r->old[p][lane];
                float imp=old-(r->mass[p][lane]*speed[p]+r->bias[p][lane]);
                if(imp<0.0f)imp=0.0f;
                assert(fabsf(r->next[p][lane]-imp)<0.0001f);
                float delta=imp-old;
                assert(fabsf(r->delta[p][lane]-delta)<0.0001f);
                for(unsigned q=p+1u;q<4u;++q){
                    unsigned k=q==1u?0u:q==2u?(p==0u?1u:3u):p==0u?2u:p==1u?4u:5u;
                    speed[q]+=r->coupling[k][lane]*delta;
                }
            }
        }
    }

    // Differential against sequential *body velocity* updates, not
    // just a duplicate sweep reference. Use a Y-axis normal with
    // different angular Jacobians for four contacts.
    for(unsigned trial=0;trial<32u;++trial){
        b3Vu0SweepRecord r={0};
        float ja[4],jb[4];
        for(unsigned p=0;p<4u;++p){
            ja[p]=0.125f*(float)(p+1u);
            jb[p]=0.25f*(float)((int)p-2);
        }
        for(unsigned lane=0;lane<4u;++lane){
            float va=0.125f*(float)trial,vb=0.25f*(float)lane;
            float wa=0.25f,wb=-0.125f;
            for(unsigned p=0;p<4u;++p){
                r.old[p][lane]=0.25f;
                r.mass[p][lane]=0.5f;
                r.bias[p][lane]=0.125f;
                r.speed[p][lane]=(vb-va)+wb*jb[p]-wa*ja[p];
            }
            unsigned pIdx[6]={0,0,0,1,1,2};
            unsigned qIdx[6]={1,2,3,2,3,3};
            for(unsigned k=0;k<6u;++k)
                r.coupling[k][lane]=0.75f+
                    0.5f*ja[pIdx[k]]*ja[qIdx[k]]+
                    0.25f*jb[pIdx[k]]*jb[qIdx[k]];
        }
        b3Vu0SweepReference(&r);
        for(unsigned lane=0;lane<4u;++lane){
            float va=0.125f*(float)trial,vb=0.25f*(float)lane;
            float wa=0.25f,wb=-0.125f;
            for(unsigned p=0;p<4u;++p){
                float old=r.old[p][lane];
                float vn=(vb-va)+wb*jb[p]-wa*ja[p];
                float next=old-r.mass[p][lane]*vn-r.bias[p][lane];
                if(next<0.0f)next=0.0f;
                float delta=next-old;
                assert(fabsf(next-r.next[p][lane])<0.001f);
                assert(fabsf(delta-r.delta[p][lane])<0.001f);
                va-=0.5f*delta;vb+=0.25f*delta;
                wa-=0.5f*ja[p]*delta;wb+=0.25f*jb[p]*delta;
            }
        }
    }
    puts("PASS: 4-point VU0 sweep, 8-group ABI and rigid-body sequential differential");
    return 0;
}
