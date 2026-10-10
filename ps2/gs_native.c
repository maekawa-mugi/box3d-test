// SPDX-License-Identifier: MIT
// PS2 native GS wireframe backend using packed GIF A+D and channel-2 DMA.
// PS2SDK graph manages the CRT/read circuit only. No draw, no gsKit.
#include "gs_native.h"
#if defined(BOX3D_PS2_UNIFIED)
#include "pcss_filter_tiers.h"
#include "ee_clock.h"
#endif
#if defined(BOX3D_GS_PCSS_SHADOWS)
#include "shadow_penumbra.h"
#endif
#include <dma.h>
#include <gif_tags.h>
#include <graph.h>
#include <gs_gp.h>
#include <gs_psm.h>
#if defined(BOX3D_GS_PROGRESSIVE)
// GS 4 MiB VRAM: two 640x480 PSMCT16 colors (1.17 MiB),
// Z16 (0.59 MiB), and 640x480 PSMCT32 RAW shadow mask (1.17 MiB).
// Reserve roughly 1 MiB for future textures/VRAM page padding.
#define B3_GS_COLOR_PSM GS_PSM_16
#define B3_GS_DEPTH_PSM GS_ZBUF_16
#else
#define B3_GS_COLOR_PSM GS_PSM_32
#define B3_GS_DEPTH_PSM GS_ZBUF_32
#endif
#include <gs_privileged.h>
#include <kernel.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef struct B3GsQword { uint64_t data, reg; } B3GsQword;
#if defined(BOX3D_PS2_UNIFIED)
// Max GIF PACKED NLOOP is 32767: reserve one TAG + at most 32758 A+D
// entries. Stress presets emit many real Box3D hull triangles.
#define B3_GS_QWORDS 32760
#elif defined(BOX3D_GS_RECEIVER_SHADOWS)
// Legacy packet limit (no longer built).
#define B3_GS_QWORDS 16384
#elif defined(BOX3D_GS_OFFICIAL)
#define B3_GS_QWORDS 8192
#else
#define B3_GS_QWORDS 2048
#endif
static _Alignas(64) B3GsQword s_packet[B3_GS_QWORDS];
static unsigned s_used;
static unsigned s_lastGifQwords;
#if defined(BOX3D_PS2_UNIFIED)
static uint32_t s_lastGsWaitTicks,s_lastVsyncWaitTicks;
#endif
static int s_framebuffer[2];
static int s_back = 1;
#if defined(BOX3D_GS_SOLID)
static int s_zbuffer;
#endif
static int s_errors;
#if defined(BOX3D_GS_OFFICIAL)
static int s_shadowPass;
#endif
#if defined(BOX3D_GS_KAWASE_SHADOWS)
// Scale each stage with internal resolution. 320x224 -> 160x112
// -> 80x56 -> 40x28. The progressive edition uses 640x480 ->
// 320x240 -> 160x120 -> 80x60.
#define B3_KAWASE_HALF_W (B3_PS2_GS_W/2)
#define B3_KAWASE_HALF_H (B3_PS2_GS_H/2)
#define B3_KAWASE_QUARTER_W (B3_PS2_GS_W/4)
#define B3_KAWASE_QUARTER_H (B3_PS2_GS_H/4)
#if defined(BOX3D_GS_KAWASE_WIDE)
#define B3_KAWASE_EIGHTH_W (B3_PS2_GS_W/8)
#define B3_KAWASE_EIGHTH_H (B3_PS2_GS_H/8)
static int s_eighthMask[2];
#endif
static int s_shadowMask;
#if defined(BOX3D_PS2_UNIFIED)
// One disjoint 320x240 PSMCT32 target (physically 320x256), allocated
// within the existing 4 MiB GS VRAM. Pre-filter distant PCSS silhouettes
// here before the separable 3x3 final convolution.
static int s_pcssPrefilter;
static int s_pcssQuarterFilter;
// Disjoint half-size PSMCT32 filtered coverage. Build all taps here,
// then darken the PSMCT16 scene only ONCE to avoid repeated 5-bit rounding.
static int s_pcssFiltered;
#endif
static int s_halfMask[2];
static int s_quarterMask[2];
#if defined(BOX3D_GS_PCSS_SHADOWS)
static int s_pcssRadius=1;
static int s_pcssMajorX=1;
// Scissor bounds enclose the UNION of silhouettes for an entire
// radius bucket, including kernel support; never one box per pass.
static int s_pcssX0=0,s_pcssY0=0;
static int s_pcssX1=B3_PS2_GS_W-1,s_pcssY1=B3_PS2_GS_H-1;
#endif
#endif

// Coordinates are signed screen pixels. We use XYOFFSET=2048.0 pixels
// and add that same offset to the XY2 12.4 fixed-point vertices.
static uint64_t b3GsXYZDepth(int x, int y, uint32_t depth)
{
    return GS_SET_XYZ((uint16_t)((2048 + x) << 4),
                      (uint16_t)((2048 + y) << 4), depth);
}
static uint64_t b3GsXYZ(int x, int y)
{
    return b3GsXYZDepth(x,y,0);
}
#if defined(BOX3D_GS_PROGRESSIVE)
// 12.4 GS XY inputs retain the entire 1/16-pixel raster coordinate.
// Converting to integer pixels before GS_SET_XYZ defeated subpixel
// coverage, particularly obvious on sloped edges at 640x480.
static uint64_t b3GsXYZDepthFixed(int x16,int y16,uint32_t depth)
{
    return GS_SET_XYZ((uint16_t)((2048<<4)+x16),
                      (uint16_t)((2048<<4)+y16),depth);
}
static int b3GsSubpixel(float v)
{
    float x=v*16.0f;
    return (int)(x+(x>=0.0f?0.5f:-0.5f));
}
#endif

static void b3GsAD(uint64_t data, uint64_t reg)
{
    if (s_used < B3_GS_QWORDS - 1)
    {
        s_packet[s_used].data = data;
        s_packet[s_used].reg = reg;
        ++s_used;
    }
    else
    {
        ++s_errors;
    }
}


// GS framebuffer pages: PSMCT16/PSMZ16 = 64x64 pixels,
// PSMCT32/PSMZ32 = 64x32 pixels. PS2SDK graph_vram_size()
// pads width and word alignment but NOT height to whole GS pages.
// 640x480 PSM16 must reserve 640x512 or it will overlap the
// following surface. 320x240 PSM32 must reserve 320x256.
static int b3GsPaddedSurfaceHeight(int height,int psm)
{
    int pageH=(psm==GS_PSM_16 || psm==GS_PSM_16S ||
               psm==GS_PSMZ_16 || psm==GS_PSMZ_16S)?64:32;
    return (height+pageH-1)&~(pageH-1);
}
static int b3GsAllocatePageSurface(int width,int height,int psm)
{
    return graph_vram_allocate(width,b3GsPaddedSurfaceHeight(height,psm),
                               psm,GRAPH_ALIGN_PAGE);
}

int b3GsInit(void)
{
    graph_vram_clear();
    s_framebuffer[0] = b3GsAllocatePageSurface(B3_PS2_GS_W, B3_PS2_GS_H,
                                           B3_GS_COLOR_PSM);
    s_framebuffer[1] = b3GsAllocatePageSurface(B3_PS2_GS_W, B3_PS2_GS_H,
                                           B3_GS_COLOR_PSM);
#if defined(BOX3D_GS_SOLID)
    // PS2 GS ZBUF_32 is a distinct 8192-byte-page-aligned VRAM buffer.
    // The renderer still uses the two previously proven color buffers.
    s_zbuffer = b3GsAllocatePageSurface(B3_PS2_GS_W, B3_PS2_GS_H,
                                    B3_GS_DEPTH_PSM);
    if(s_zbuffer < 0)
    {
        puts("BOX3D_GS_FAIL,zbuffer_allocation");
        return 0;
    }
#endif
#if defined(BOX3D_GS_KAWASE_SHADOWS)
    // RGB mask/blur pages: no CPU round trips and no texture upload.
    // Scratch is disjoint from both color buffers and the depth buffer.
    s_shadowMask=b3GsAllocatePageSurface(B3_PS2_GS_W,B3_PS2_GS_H,
                                     GS_PSM_32);
#if defined(BOX3D_GS_PCF_SHADOWS)
    // PCF samples only RAW full-resolution coverage. No extra half
    // or quarter pages are needed: reclaim ~175 KiB raw texel space
    // for future megademo assets / render targets in 4 MiB GS VRAM.
    if(s_shadowMask<0)
    {
        puts("BOX3D_GS_FAIL,pcf_mask_vram_allocation");
        return 0;
    }
#if defined(BOX3D_PS2_UNIFIED)
    s_pcssPrefilter=b3GsAllocatePageSurface(B3_KAWASE_HALF_W,
                                             B3_KAWASE_HALF_H,GS_PSM_32);
    if(s_pcssPrefilter<0)
    {
        puts("BOX3D_GS_FAIL,pcss_prefilter_vram");
        return 0;
    }
    s_pcssQuarterFilter=b3GsAllocatePageSurface(B3_KAWASE_QUARTER_W,
                                                  B3_KAWASE_QUARTER_H,
                                                  GS_PSM_32);
    if(s_pcssQuarterFilter<0)
    {
        puts("BOX3D_GS_FAIL,pcss_quarter_prefilter_vram");
        return 0;
    }
    s_pcssFiltered=b3GsAllocatePageSurface(B3_KAWASE_HALF_W,
                                           B3_KAWASE_HALF_H,GS_PSM_32);
    if(s_pcssFiltered<0)
    {
        puts("BOX3D_GS_FAIL,pcss_filtered_vram");
        return 0;
    }
    puts("BOX3D_GS_PCSS_PREFILTER,half=320x240,quarter=160x120,filtered=320x240,disjoint=1");
#endif
#else
    for(int i=0;i<2;++i)
    {
        s_halfMask[i]=b3GsAllocatePageSurface(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,
                                         GS_PSM_32);
        s_quarterMask[i]=b3GsAllocatePageSurface(B3_KAWASE_QUARTER_W,B3_KAWASE_QUARTER_H,
                                            GS_PSM_32);
#if defined(BOX3D_GS_KAWASE_WIDE)
        s_eighthMask[i]=b3GsAllocatePageSurface(B3_KAWASE_EIGHTH_W,B3_KAWASE_EIGHTH_H,
                                            GS_PSM_32);
#endif
    }
    if(s_shadowMask<0 || s_halfMask[0]<0 || s_halfMask[1]<0 ||
       s_quarterMask[0]<0 || s_quarterMask[1]<0
#if defined(BOX3D_GS_KAWASE_WIDE)
       || s_eighthMask[0]<0 || s_eighthMask[1]<0
#endif
       )
    {
        puts("BOX3D_GS_FAIL,kawase_vram_allocation");
        return 0;
    }
#endif
#endif
#if defined(BOX3D_GS_PROGRESSIVE) && defined(BOX3D_GS_KAWASE_WIDE)
    // Report *actual* page-rounded end address, not a logical-pixel
    // count. A valid 640x480 wide layout ends at 4,112,384 bytes.
    if(s_eighthMask[1]>=0)
    {
        int endWords=s_eighthMask[1]+
            graph_vram_size(B3_KAWASE_EIGHTH_W,
                            b3GsPaddedSurfaceHeight(B3_KAWASE_EIGHTH_H,
                                                    GS_PSM_32),
                            GS_PSM_32,GRAPH_ALIGN_PAGE);
        printf("BOX3D_GS_VRAM,mode=480p_wide,total_bytes=%d,limit_bytes=4194304,free_bytes=%d\n",
               endWords*4,4194304-endWords*4);
    }
#endif
    if (s_framebuffer[0] < 0 || s_framebuffer[1] < 0)
    {
        puts("BOX3D_GS_FAIL,framebuffer_allocation");
        return 0;
    }

    // graph library is the only video-mode helper. All drawing uses GS
    // registers directly via the packed GIF A+D stream below.
    graph_initialize(s_framebuffer[0], B3_PS2_GS_W, B3_PS2_GS_H,
                     B3_GS_COLOR_PSM, 0, 0);
#if defined(BOX3D_GS_PROGRESSIVE)
    // HDTV 480p uses a 720x480 active display in 1440 GS clock units.
    // Crop/center our *real* 640 source pixels to prevent DISPFB from
    // reading 80 bogus pixels per line outside the framebuffer.
    int region=GRAPH_MODE_HDTV_480P;
    if (graph_set_mode(GRAPH_MODE_NONINTERLACED,region,GRAPH_MODE_FRAME,
                       GRAPH_DISABLE)<0)
#else
    int region=graph_get_region();
    if (graph_set_mode(GRAPH_MODE_NONINTERLACED,region,GRAPH_MODE_FIELD,
                       GRAPH_DISABLE)<0)
#endif
    {
        puts("BOX3D_GS_FAIL,crt_mode");
        return 0;
    }
    graph_set_screen(0, 0, B3_PS2_GS_W, B3_PS2_GS_H);
#if defined(BOX3D_GS_PROGRESSIVE)
    // PS2SDK 480P graph_mode: x=232,y=35,width=1440,height=480.
    // One input pixel spans two GS clock units (MAGH=1). Center
    // 1280 clocks within 1440: x=232+(1440-1280)/2 = 312.
    *GS_REG_DISPLAY1=GS_SET_DISPLAY(312,35,1,0,1279,479);
    *GS_REG_DISPLAY2=GS_SET_DISPLAY(312,35,1,0,1279,479);
#endif
    graph_set_framebuffer(0,s_framebuffer[0],B3_PS2_GS_W,B3_GS_COLOR_PSM,0,0);
    graph_set_framebuffer(1,s_framebuffer[0],B3_PS2_GS_W,B3_GS_COLOR_PSM,0,0);
    graph_enable_output();
    dma_channel_initialize(DMA_CHANNEL_GIF, NULL, 0);
#if defined(BOX3D_GS_SOLID)
    printf("BOX3D_GS_READY,native_gif=1,solid=1,zbuf=%d,width=%d,height=%d,fb=2,psm=%d\n",
           s_zbuffer,B3_PS2_GS_W,B3_PS2_GS_H,B3_GS_COLOR_PSM);
#if defined(BOX3D_GS_PROGRESSIVE)
    puts("BOX3D_GS_VIDEO,480P,640x480,noninterlaced=1,GS_PSM16=1,GS_ZBUF16=1");
#endif
#else
    puts("BOX3D_GS_READY,native_gif=1,width=320,height=224,fb=2");
#endif
    return 1;
}

void b3GsBegin(void)
{
    s_used = 1; // GIF packed A+D tag filled in during Present
    s_errors = 0;
#if defined(BOX3D_GS_OFFICIAL)
    s_shadowPass = 0;
#endif
    b3GsAD(GS_SET_FRAME(s_framebuffer[s_back] >> 11, B3_PS2_GS_W >> 6, B3_GS_COLOR_PSM, 0),
           GS_REG_FRAME_1);
    b3GsAD(GS_SET_XYOFFSET(2048 << 4, 2048 << 4), GS_REG_XYOFFSET_1);
    b3GsAD(GS_SET_SCISSOR(0, B3_PS2_GS_W - 1, 0, B3_PS2_GS_H - 1), GS_REG_SCISSOR_1);
#if defined(BOX3D_GS_SOLID)
    b3GsAD(GS_SET_ZBUF(s_zbuffer >> 11, B3_GS_DEPTH_PSM, 0), GS_REG_ZBUF_1);
    // ZTE=1 + ZTST=ALLPASS enables depth writes during the sprite clear.
    // The same sprite clears both color and Z to zero at the start of
    // every frame. Geometry subsequently uses GS ZTST=GREATER_EQUAL.
    b3GsAD(GS_SET_TEST(0, 0, 0, 0, 0, 0, 1, 1), GS_REG_TEST_1);
#else
    b3GsAD(GS_SET_TEST(0, 0, 0, 0, 0, 0, 0, 0), GS_REG_TEST_1);
#endif
    b3GsAD(GS_SET_PRMODECONT(1), GS_REG_PRMODECONT);
    b3GsAD(GS_SET_COLCLAMP(1), GS_REG_COLCLAMP);

    // Opaque background sprite. GS_PRIM=6, ABE=0, no texture/depth.
    b3GsAD(GS_SET_PRIM(6, 0, 0, 0, 0, 0, 0, 0, 0), GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(10, 14, 27, 0x80, 0x3f800000), GS_REG_RGBAQ);
    b3GsAD(b3GsXYZ(0, 0), GS_REG_XYZ2);
    b3GsAD(b3GsXYZ(B3_PS2_GS_W, B3_PS2_GS_H), GS_REG_XYZ2);
#if defined(BOX3D_GS_SOLID)
    b3GsAD(GS_SET_TEST(0, 0, 0, 0, 0, 0, 1, 2), GS_REG_TEST_1);
#endif
}

static void b3GsLineWithDepth(int x0,int y0,uint32_t z0,
                              int x1,int y1,uint32_t z1,
                              uint8_t r,uint8_t g,uint8_t b,
                              int fx0,int fy0,int fx1,int fy1)
{
    // Reject wholly offscreen segments; clip very large projected
    // coordinates into the GS's representable 12.4 XY range.
    if ((x0 < -64 && x1 < -64) || (x0 > B3_PS2_GS_W + 64 && x1 > B3_PS2_GS_W + 64) ||
        (y0 < -64 && y1 < -64) || (y0 > B3_PS2_GS_H + 64 && y1 > B3_PS2_GS_H + 64))
        return;
    if (s_used + 6 >= B3_GS_QWORDS) { ++s_errors; return; }
    if (x0 < -256) x0 = -256;
    if (x1 < -256) x1 = -256;
    if (x0 > B3_PS2_GS_W+256) x0 = B3_PS2_GS_W+256;
    if (x1 > B3_PS2_GS_W+256) x1 = B3_PS2_GS_W+256;
    if (y0 < -256) y0 = -256;
    if (y1 < -256) y1 = -256;
    if (y0 > B3_PS2_GS_H+256) y0 = B3_PS2_GS_H+256;
    if (y1 > B3_PS2_GS_H+256) y1 = B3_PS2_GS_H+256;
#if defined(BOX3D_GS_PROGRESSIVE)
    // GS PRIM.AA1 anti-aliases LINES (not filled triangle edges).
    // GS PRIM.ABE and source-over ALPHA use the line coverage alpha.
    b3GsAD(GS_SET_ALPHA(0,1,0,1,0),GS_REG_ALPHA_1);
    b3GsAD(GS_SET_PRIM(1,0,0,0,1,1,0,0,0),GS_REG_PRIM);
#else
    b3GsAD(GS_SET_PRIM(1,0,0,0,0,0,0,0,0),GS_REG_PRIM);
#endif
    b3GsAD(GS_SET_RGBAQ(r, g, b, 0x80, 0x3f800000), GS_REG_RGBAQ);
#if defined(BOX3D_GS_PROGRESSIVE)
    // Clamp the subpixel coordinates whenever the coarse pixel coords
    // were clamped to a representable GS range.
    if((fx0>>4)<-256)fx0=-256*16;
    if((fx1>>4)<-256)fx1=-256*16;
    if((fy0>>4)<-256)fy0=-256*16;
    if((fy1>>4)<-256)fy1=-256*16;
    if((fx0>>4)>B3_PS2_GS_W+256)fx0=(B3_PS2_GS_W+256)*16;
    if((fx1>>4)>B3_PS2_GS_W+256)fx1=(B3_PS2_GS_W+256)*16;
    if((fy0>>4)>B3_PS2_GS_H+256)fy0=(B3_PS2_GS_H+256)*16;
    if((fy1>>4)>B3_PS2_GS_H+256)fy1=(B3_PS2_GS_H+256)*16;
    b3GsAD(b3GsXYZDepthFixed(fx0,fy0,z0),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepthFixed(fx1,fy1,z1),GS_REG_XYZ2);
#else
    (void)fx0;(void)fy0;(void)fx1;(void)fy1;
    b3GsAD(b3GsXYZDepth(x0,y0,z0),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepth(x1,y1,z1),GS_REG_XYZ2);
#endif
}
void b3GsLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
    b3GsLineWithDepth(x0,y0,0,x1,y1,0,r,g,b,
                      x0<<4,y0<<4,x1<<4,y1<<4);
}

static b3GsPoint3 sub(b3GsPoint3 a, b3GsPoint3 b)
{
    b3GsPoint3 c = {a.x-b.x, a.y-b.y, a.z-b.z};
    return c;
}
static float dot(b3GsPoint3 a, b3GsPoint3 b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
static b3GsPoint3 cross(b3GsPoint3 a, b3GsPoint3 b)
{
    b3GsPoint3 c = {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
    return c;
}
static b3GsPoint3 norm(b3GsPoint3 a)
{
    float len = sqrtf(dot(a,a));
    if (len < 0.00001f) { b3GsPoint3 zero = {0,0,0}; return zero; }
    b3GsPoint3 b = {a.x/len, a.y/len, a.z/len};
    return b;
}

b3GsCamera b3GsLookAt(b3GsPoint3 eye, b3GsPoint3 target, float focal)
{
    const b3GsPoint3 up = {0,1,0};
    b3GsCamera c;
    c.eye=eye;
    c.forward=norm(sub(target,eye));
    c.right=norm(cross(c.forward,up));
    c.up=cross(c.right,c.forward);
    c.focal=focal;
    return c;
}

static int projectDepthPrecise(b3GsCamera c,b3GsPoint3 v,int* ox,int* oy,
                               uint32_t* oz,int* ox16,int* oy16)
{
    b3GsPoint3 d=sub(v,c.eye);
    float z=dot(d,c.forward);
    if (!isfinite(z) || z<=0.5f || z >= 220.0f) return 0;
    float px=B3_PS2_GS_W*0.5f+c.focal*dot(d,c.right)/z;
    float py=B3_PS2_GS_H*0.5f-c.focal*dot(d,c.up)/z;
    if (!isfinite(px)||!isfinite(py)||fabsf(px)>10000.0f||fabsf(py)>10000.0f) return 0;
    *ox=(int)px; *oy=(int)py;
#if defined(BOX3D_GS_PROGRESSIVE)
    if(ox16)*ox16=b3GsSubpixel(px);
    if(oy16)*oy16=b3GsSubpixel(py);
#else
    (void)ox16;(void)oy16;
#endif
#if defined(BOX3D_GS_RECEIVER_SHADOWS)
    // Critical for native GS perspective rasterization: GS interpolates
    // vertex Z *linearly in screen space*. Camera-space distance 'z' is
    // not linear after perspective division. Different triangulations
    // of the SAME floor plane (checker tiles vs projected shadow hull)
    // therefore failed Z tests intermittently with the old (220-z)
    // mapping. Reciprocal 1/z IS linear across projected plane triangles.
    // Inverted depth keeps ZTST=GEQUAL; clear remains Z=0. At near=0.5,
    // the largest encoded depth is < 2^32. A 0.045m receiver-normal
    // bias gives a small positive foreground offset.
#if defined(BOX3D_GS_PROGRESSIVE)
    // Z16 stores only 16 bits: map near 0.5 to < 65535.
    *oz=(uint32_t)((1.0f/z - 1.0f/220.0f)*30000.0f);
#else
    *oz=(uint32_t)((1.0f/z - 1.0f/220.0f)*100000000.0f);
#endif
#else
    // Legacy verified wireframe / solid / official variants retain the
    // previous depth mapping, for independent regression comparisons.
    *oz=(uint32_t)((220.0f-z)*1048576.0f);
#endif
    return 1;
}
static int projectDepth(b3GsCamera c,b3GsPoint3 v,
                        int* ox,int* oy,uint32_t* oz)
{
    return projectDepthPrecise(c,v,ox,oy,oz,NULL,NULL);
}
#if defined(BOX3D_PS2_UNIFIED)
// This fast path is only valid when every box vertex is safely in front
// of the near plane and inside the GS 12.4 extended raster bounds.
// Near-clipped or otherwise exceptional boxes use b3GsTriangle3().
int b3GsProjectPoint(b3GsCamera camera,b3GsPoint3 vertex,
                     b3GsProjectedVertex* p)
{
    if(!projectDepthPrecise(camera,vertex,&p->x,&p->y,
                            &p->depth,&p->x16,&p->y16))
        return 0;
    // Out-of-range vertices require the original clipping path.
    return p->x16>=-4096 && p->y16>=-4096 &&
           p->x16<=(B3_PS2_GS_W+256)*16 &&
           p->y16<=(B3_PS2_GS_H+256)*16;
}
int b3GsProjectHull(b3GsCamera camera,const b3GsPoint3 vertices[8],
                    b3GsProjectedVertex out[8])
{
    for(int i=0;i<8;++i)
        if(!b3GsProjectPoint(camera,vertices[i],&out[i]))return 0;
    return 1;
}
void b3GsTriangleProjected(const b3GsProjectedVertex* a,
                           const b3GsProjectedVertex* b,
                           const b3GsProjectedVertex* c,
                           uint8_t r,uint8_t g,uint8_t blue)
{
    // Equivalent conservative screen rejection to Triangle3Raw.
    if((a->x<0 && b->x<0 && c->x<0) ||
       (a->x>=B3_PS2_GS_W && b->x>=B3_PS2_GS_W && c->x>=B3_PS2_GS_W) ||
       (a->y<0 && b->y<0 && c->y<0) ||
       (a->y>=B3_PS2_GS_H && b->y>=B3_PS2_GS_H && c->y>=B3_PS2_GS_H))
        return;
    if(s_used+6>=B3_GS_QWORDS) {++s_errors;return;}
    b3GsAD(GS_SET_PRIM(3,0,0,0,0,0,0,0,0),GS_REG_PRIM);
    // The same projected-triangle fast path serves solids and coverage
    // masks. Shadows must write constant RGB=128 into the 32-bit mask.
    b3GsAD(GS_SET_RGBAQ(s_shadowPass?128:r,s_shadowPass?128:g,
                        s_shadowPass?128:blue,0x80,0x3f800000),
           GS_REG_RGBAQ);
    b3GsAD(b3GsXYZDepthFixed(a->x16,a->y16,a->depth),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepthFixed(b->x16,b->y16,b->depth),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepthFixed(c->x16,c->y16,c->depth),GS_REG_XYZ2);
}
#endif
#if defined(BOX3D_GS_PCSS_SHADOWS)
int b3GsProjectScreen(b3GsCamera c,b3GsPoint3 p,int* x,int* y)
{
    uint32_t unusedDepth;
    return projectDepth(c,p,x,y,&unusedDepth);
}
#endif
#if !defined(BOX3D_GS_SOLID)
static int project(b3GsCamera c,b3GsPoint3 v,int* ox,int* oy)
{
    uint32_t z;
    return projectDepth(c,v,ox,oy,&z);
}
#endif

void b3GsLine3(b3GsCamera c,b3GsPoint3 a,b3GsPoint3 b,
               uint8_t r,uint8_t g,uint8_t blue)
{
    int x0,y0,x1,y1;
#if defined(BOX3D_GS_SOLID)
    uint32_t z0,z1;
    int fx0=0,fy0=0,fx1=0,fy1=0;
    if (projectDepthPrecise(c,a,&x0,&y0,&z0,&fx0,&fy0) &&
        projectDepthPrecise(c,b,&x1,&y1,&z1,&fx1,&fy1))
    {
#if defined(BOX3D_GS_PROGRESSIVE)
        // The previous +4096 bias was written for Z32. On 16-bit Z
        // it pushed *every floor grid line* in front of the platform,
        // making the opaque slab look transparent. Use just one depth
        // quantization unit for a restrained polygon-offset.
        const uint32_t bias=1u;
        z0=z0<65535u-bias?z0+bias:65535u;
        z1=z1<65535u-bias?z1+bias:65535u;
#else
        z0+=4096u;z1+=4096u;
#endif
        b3GsLineWithDepth(x0,y0,z0,x1,y1,z1,r,g,blue,
                          fx0,fy0,fx1,fy1);
    }
#else
    if (project(c,a,&x0,&y0) && project(c,b,&x1,&y1))
        b3GsLine(x0,y0,x1,y1,r,g,blue);
#endif
}

static void b3GsTriangle3Raw(b3GsCamera c,b3GsPoint3 a,b3GsPoint3 b,
                              b3GsPoint3 d,uint8_t r,uint8_t g,uint8_t blue)
{
#if defined(BOX3D_GS_SOLID)
    int x0,y0,x1,y1,x2,y2;
    int fx0=0,fy0=0,fx1=0,fy1=0,fx2=0,fy2=0;
    uint32_t z0,z1,z2;
    if (!projectDepthPrecise(c,a,&x0,&y0,&z0,&fx0,&fy0) ||
        !projectDepthPrecise(c,b,&x1,&y1,&z1,&fx1,&fy1) ||
        !projectDepthPrecise(c,d,&x2,&y2,&z2,&fx2,&fy2)) return;

    // Avoid huge triangles crossing the near plane; full clipping will
    // be added after this independent GS bring-up succeeds.
    if ((x0 < 0 && x1 < 0 && x2 < 0) ||
        (x0 >= B3_PS2_GS_W && x1 >= B3_PS2_GS_W && x2 >= B3_PS2_GS_W) ||
        (y0 < 0 && y1 < 0 && y2 < 0) ||
        (y0 >= B3_PS2_GS_H && y1 >= B3_PS2_GS_H && y2 >= B3_PS2_GS_H))
        return;
    if(s_used + 6 >= B3_GS_QWORDS) { ++s_errors; return; }
    // GS coordinates are 12.4 fixed-point. Screen visible range is
    // [0,320]x[0,224]; clamp raster input to an extended safe range.
    if(x0 < -256) x0 = -256; if(x1 < -256) x1 = -256; if(x2 < -256) x2 = -256;
    if(y0 < -256) y0 = -256; if(y1 < -256) y1 = -256; if(y2 < -256) y2 = -256;
    if(x0 > B3_PS2_GS_W+256) x0 = B3_PS2_GS_W+256;
    if(x1 > B3_PS2_GS_W+256) x1 = B3_PS2_GS_W+256;
    if(x2 > B3_PS2_GS_W+256) x2 = B3_PS2_GS_W+256;
    if(y0 > B3_PS2_GS_H+256) y0 = B3_PS2_GS_H+256;
    if(y1 > B3_PS2_GS_H+256) y1 = B3_PS2_GS_H+256;
    if(y2 > B3_PS2_GS_H+256) y2 = B3_PS2_GS_H+256;
#if defined(BOX3D_GS_OFFICIAL)
#if defined(BOX3D_GS_KAWASE_SHADOWS)
    // Build an RGB coverage mask, *not* a directly alpha-blended patch.
    // GS does not blend destination alpha alongside RGB, so the blur
    // stores shadow strength in the color channels for later subtraction.
    b3GsAD(GS_SET_PRIM(3,0,0,0,0,0,0,0,0),GS_REG_PRIM);
    if(s_shadowPass)
        b3GsAD(GS_SET_RGBAQ(128,128,128,0x80,0x3f800000),GS_REG_RGBAQ);
    else
        b3GsAD(GS_SET_RGBAQ(r,g,blue,0x80,0x3f800000),GS_REG_RGBAQ);
#else
    b3GsAD(GS_SET_PRIM(3, 0, 0, 0, s_shadowPass ? 1 : 0, 0, 0, 0, 0), GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(r,g,blue,s_shadowPass ? 0x50 : 0x80,0x3f800000),GS_REG_RGBAQ);
#endif
#else
    b3GsAD(GS_SET_PRIM(3, 0, 0, 0, 0, 0, 0, 0, 0), GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(r,g,blue,0x80,0x3f800000),GS_REG_RGBAQ);
#endif
#if defined(BOX3D_GS_PROGRESSIVE)
    // GS 12.4 precision is independent of the coarse-screen bounds
    // above; keep all fractional screen-space bits for every vertex.
    if(fx0< -4096)fx0=-4096; if(fx1< -4096)fx1=-4096; if(fx2< -4096)fx2=-4096;
    if(fy0< -4096)fy0=-4096; if(fy1< -4096)fy1=-4096; if(fy2< -4096)fy2=-4096;
    if(fx0>(B3_PS2_GS_W+256)*16)fx0=(B3_PS2_GS_W+256)*16;
    if(fx1>(B3_PS2_GS_W+256)*16)fx1=(B3_PS2_GS_W+256)*16;
    if(fx2>(B3_PS2_GS_W+256)*16)fx2=(B3_PS2_GS_W+256)*16;
    if(fy0>(B3_PS2_GS_H+256)*16)fy0=(B3_PS2_GS_H+256)*16;
    if(fy1>(B3_PS2_GS_H+256)*16)fy1=(B3_PS2_GS_H+256)*16;
    if(fy2>(B3_PS2_GS_H+256)*16)fy2=(B3_PS2_GS_H+256)*16;
    b3GsAD(b3GsXYZDepthFixed(fx0,fy0,z0),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepthFixed(fx1,fy1,z1),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepthFixed(fx2,fy2,z2),GS_REG_XYZ2);
#else
    b3GsAD(b3GsXYZDepth(x0,y0,z0),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepth(x1,y1,z1),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepth(x2,y2,z2),GS_REG_XYZ2);
#endif
#else
    (void)c;(void)a;(void)b;(void)d;(void)r;(void)g;(void)blue;
#endif
}


// Clip 3D triangles against the near camera plane BEFORE projection.
// Previously one near-plane vertex caused an entire 3D triangle to
// disappear. This is especially visible with wide projected shadows.
// The legacy solid/official ELFs retain their original renderer.
void b3GsTriangle3(b3GsCamera c,b3GsPoint3 a,b3GsPoint3 b,
                   b3GsPoint3 d,uint8_t r,uint8_t g,uint8_t blue)
{
#if defined(BOX3D_GS_RECEIVER_SHADOWS)
    const float nearPlane=0.625f;
    b3GsPoint3 in[3]={a,b,d},out[4];
    int n=0;
    b3GsPoint3 prev=in[2];
    float dp=dot(sub(prev,c.eye),c.forward)-nearPlane;
    int insidePrev=dp>=0.0f;
    for(int i=0;i<3;++i)
    {
        b3GsPoint3 current=in[i];
        float dc=dot(sub(current,c.eye),c.forward)-nearPlane;
        int inside=dc>=0.0f;
        if(inside!=insidePrev)
        {
            float t=dp/(dp-dc);
            b3GsPoint3 p={
                prev.x+t*(current.x-prev.x),
                prev.y+t*(current.y-prev.y),
                prev.z+t*(current.z-prev.z)
            };
            if(n<4)out[n++]=p;
        }
        if(inside && n<4)out[n++]=current;
        prev=current;dp=dc;insidePrev=inside;
    }
    if(n<3)return;
    for(int i=1;i+1<n;++i)
        b3GsTriangle3Raw(c,out[0],out[i],out[i+1],r,g,blue);
#else
    b3GsTriangle3Raw(c,a,b,d,r,g,blue);
#endif
}

// Native PS2 GS aesthetic layer, approximating the official demo's
// sky/sun/ground/GUI presentation without GPU shaders. The upstream
// renderer uses Preetham sky, IBL, PBR, GTAO and cascade shadows; this
// is deliberately a lightweight approximation with GS primitives.
#if defined(BOX3D_GS_OFFICIAL)

// A Gouraud-shaded screen-space triangle. The GS interpolates sky colors
// per pixel (IIP=1) while Z=0 keeps the background behind 3D shapes.
static void b3GsSkyTriangle(int x0,int y0,uint8_t r0,uint8_t g0,uint8_t b0,
                            int x1,int y1,uint8_t r1,uint8_t g1,uint8_t b1,
                            int x2,int y2,uint8_t r2,uint8_t g2,uint8_t b2)
{
    if(s_used+9>=B3_GS_QWORDS) { ++s_errors; return; }
    b3GsAD(GS_SET_PRIM(3,1,0,0,0,0,0,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(r0,g0,b0,0x80,0x3f800000),GS_REG_RGBAQ);
    b3GsAD(b3GsXYZ(x0,y0),GS_REG_XYZ2);
    b3GsAD(GS_SET_RGBAQ(r1,g1,b1,0x80,0x3f800000),GS_REG_RGBAQ);
    b3GsAD(b3GsXYZ(x1,y1),GS_REG_XYZ2);
    b3GsAD(GS_SET_RGBAQ(r2,g2,b2,0x80,0x3f800000),GS_REG_RGBAQ);
    b3GsAD(b3GsXYZ(x2,y2),GS_REG_XYZ2);
}
static void b3GsSkyBand(int y0,int y1,
                         uint8_t r0,uint8_t g0,uint8_t b0,
                         uint8_t r1,uint8_t g1,uint8_t b1)
{
    b3GsSkyTriangle(0,y0,r0,g0,b0,B3_PS2_GS_W,y0,r0,g0,b0,
                    0,y1,r1,g1,b1);
    b3GsSkyTriangle(B3_PS2_GS_W,y0,r0,g0,b0,
                    B3_PS2_GS_W,y1,r1,g1,b1,0,y1,r1,g1,b1);
}
void b3GsSkyGradient(void)
{
    // Gradient from blue zenith to hazy daylight horizon, akin to
    // upstream Preetham + AgX, but produced entirely by GS IIP.
    b3GsSkyBand(0,104,80,137,190,155,188,206);
    b3GsSkyBand(104,B3_PS2_GS_H,155,188,206,227,222,207);
}

#if defined(BOX3D_GS_KAWASE_SHADOWS)

// Each offscreen pass changes GS FRAME_1 and SCISSOR_1, never the CRT.
// Full-size shadow mask uses the *existing* ZBUF and scene depth;
// the two smaller pyramid levels use no depth testing.
static void b3GsKawaseTarget(int vram,int width,int height)
{
    int colorPsm=(vram==s_framebuffer[0]||vram==s_framebuffer[1])
                 ? B3_GS_COLOR_PSM : GS_PSM_32;
    b3GsAD(GS_SET_FRAME(vram>>11,(width+63)>>6,colorPsm,0),
           GS_REG_FRAME_1);
    b3GsAD(GS_SET_SCISSOR(0,width-1,0,height-1),GS_REG_SCISSOR_1);
    b3GsAD(GS_SET_TEST(0,0,0,0,0,0,0,0),GS_REG_TEST_1);
}
static void b3GsKawaseClear(int width,int height)
{
    b3GsAD(GS_SET_PRIM(6,0,0,0,0,0,0,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(0,0,0,0,0x3f800000),GS_REG_RGBAQ);
    b3GsAD(b3GsXYZ(0,0),GS_REG_XYZ2);
    b3GsAD(b3GsXYZ(width,height),GS_REG_XYZ2);
}

// GS bilinear texture source; never read and render into the same VRAM
// buffer in one pass. TEXFLUSH invalidates GS texture cache on each pass.
// Region clamp avoids sampling the undefined padded TBW/TW/TH texels.
// TEX0 TW/TH are exponents of the texture allocation dimensions.
// 640x480 requires 2^10 x 2^9 (not legacy 2^9 x 2^8);
// 40x28 requires 2^6 x 2^5. Derive for EVERY texture stage.
static int b3GsTexLog2(int size)
{
    int exp=0,pow=1;
    while(pow<size && exp<10){pow<<=1;++exp;}
    return exp;
}
static void b3GsKawaseSource(int vram,int width,int height,
                             int tw,int th)
{
    (void)tw;(void)th; // legacy call hints intentionally not authoritative
    b3GsAD(GS_SET_TEXFLUSH(0),GS_REG_TEXFLUSH);
    b3GsAD(GS_SET_TEX0_SMALL(vram>>6,(width+63)>>6,GS_PSM_32,
                              b3GsTexLog2(width),b3GsTexLog2(height),1,1),GS_REG_TEX0_1);
    b3GsAD(GS_SET_TEX1(1,0,1,1,0,0,0),GS_REG_TEX1_1);
    b3GsAD(GS_SET_CLAMP(2,2,0,width-1,0,height-1),GS_REG_CLAMP_1);
    b3GsAD(GS_SET_RGBAQ(128,128,128,128,0x3f800000),GS_REG_RGBAQ);
}
// Source UVs are nonnegative (14-bit unsigned), while dest XY may shift
// in either direction. Shifting the destination achieves the four
// symmetrically offset Kawase taps with GS scissor handling boundaries.
static void b3GsKawaseBlit(int targetW,int targetH,int srcW,int srcH,
                           int shiftX,int shiftY)
{
    b3GsAD(GS_SET_PRIM(6,0,1,0,1,0,1,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_UV(8,8),GS_REG_UV);
    b3GsAD(b3GsXYZ(shiftX,shiftY),GS_REG_XYZ2);
    b3GsAD(GS_SET_UV((srcW<<4)+8,(srcH<<4)+8),GS_REG_UV);
    b3GsAD(b3GsXYZ(targetW+shiftX,targetH+shiftY),GS_REG_XYZ2);
}


// Uniform GS shadow composite contract for PCF, PCSS and Dual Kawase:
// source texture stores RGB coverage [0..128] on a disjoint VRAM page;
// the scene remains in the back framebuffer, and a weighted sampled
// coverage is SUBTRACTED from scene RGB with the GS fixed-function ALPHA
// unit. Only a single filtered mask from a given caster group is used:
// never blindly overdraw an additional opaque RAW silhouette.
static void b3GsShadowCompositeSource(int mask,int width,int height)
{
    b3GsKawaseTarget(s_framebuffer[s_back],B3_PS2_GS_W,B3_PS2_GS_H);
    b3GsKawaseSource(mask,width,height,0,0);
}
static void b3GsShadowCompositeTap(int width,int height,
                                    int x,int y,int fix)
{
    b3GsAD(GS_SET_ALPHA(2,0,2,1,fix),GS_REG_ALPHA_1);
    b3GsKawaseBlit(B3_PS2_GS_W,B3_PS2_GS_H,width,height,x,y);
}
#if defined(BOX3D_PS2_UNIFIED)
// A sub-texel bilinear tap: XY is shifted by whole pixels, while UV is
// shifted by +/-0.5 texel TOWARD the caster for every off-center tap.
// This samples between two pixel rows/columns even at 1:1 scale,
// instead of repeatedly reading individual jagged binary-mask pixels.
// UV stays nonnegative; scissor clips destination beyond the receiver.
static void b3GsShadowCompositeSmoothTap(int dx,int dy,
                                         int ix,int iy,int fix)
{
    const int w=B3_PS2_GS_W,h=B3_PS2_GS_H;
    int ux=8+(ix<0?8:ix>0?-8:0);
    int uy=8+(iy<0?8:iy>0?-8:0);
    b3GsAD(GS_SET_ALPHA(2,0,2,1,fix),GS_REG_ALPHA_1);
    b3GsAD(GS_SET_PRIM(6,0,1,0,1,0,1,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_UV(ux,uy),GS_REG_UV);
    b3GsAD(b3GsXYZ(dx,dy),GS_REG_XYZ2);
    b3GsAD(GS_SET_UV((w<<4)+ux,(h<<4)+uy),GS_REG_UV);
    b3GsAD(b3GsXYZ(w+dx,h+dy),GS_REG_XYZ2);
}
#endif

static void b3GsShadowCompositeFour(int width,int height,int shift,int fix)
{
    b3GsAD(GS_SET_ALPHA(2,0,2,1,fix),GS_REG_ALPHA_1);
    b3GsKawaseBlit(B3_PS2_GS_W,B3_PS2_GS_H,width,height,-shift,-shift);
    b3GsKawaseBlit(B3_PS2_GS_W,B3_PS2_GS_H,width,height, shift,-shift);
    b3GsKawaseBlit(B3_PS2_GS_W,B3_PS2_GS_H,width,height,-shift, shift);
    b3GsKawaseBlit(B3_PS2_GS_W,B3_PS2_GS_H,width,height, shift, shift);
}

#if defined(BOX3D_GS_PCF_4TAP)
// Bilinear texel half-offsets realize the separable 3x3 tent kernel:
// [1 2 1] x [1 2 1] / 16, using four equal-weight GS samples.
// Source UV uses unsigned 4.4 pixel positions: 8 +/- 8 is 0 or 16.
static void b3GsPcf4Tap(int width,int height)
{
    b3GsAD(GS_SET_PRIM(6,0,1,0,1,0,1,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_ALPHA(2,0,2,1,24),GS_REG_ALPHA_1);
    for(int dy=-1;dy<=1;dy+=2)
    for(int dx=-1;dx<=1;dx+=2)
    {
        b3GsAD(GS_SET_UV(8+8*dx,8+8*dy),GS_REG_UV);
        b3GsAD(b3GsXYZ(0,0),GS_REG_XYZ2);
        b3GsAD(GS_SET_UV((width<<4)+8+8*dx,
                         (height<<4)+8+8*dy),GS_REG_UV);
        b3GsAD(b3GsXYZ(width,height),GS_REG_XYZ2);
    }
}
#endif

static void b3GsKawasePass(int target,int targetW,int targetH,
                           int source,int sourceW,int sourceH,
                           int sourceTw,int sourceTh,int radius)
{
    b3GsKawaseTarget(target,targetW,targetH);
    b3GsKawaseClear(targetW,targetH);
    b3GsKawaseSource(source,sourceW,sourceH,sourceTw,sourceTh);
    // Add source RGB / 4 for each of four offset sprites. The color
    // coverage is averaged correctly even though GS alpha is not.
    b3GsAD(GS_SET_ALPHA(0,2,2,1,32),GS_REG_ALPHA_1);
    b3GsKawaseBlit(targetW,targetH,sourceW,sourceH,-radius,-radius);
    b3GsKawaseBlit(targetW,targetH,sourceW,sourceH, radius,-radius);
    b3GsKawaseBlit(targetW,targetH,sourceW,sourceH,-radius, radius);
    b3GsKawaseBlit(targetW,targetH,sourceW,sourceH, radius, radius);
}
#if defined(BOX3D_GS_PCSS_SHADOWS)
void b3GsShadowSetPenumbra(int radius,int x0,int y0,
                           int x1,int y1,int majorX)
{
    if(radius<1)radius=1;
    if(radius>5)radius=5;
    s_pcssRadius=radius;
    s_pcssMajorX=(majorX!=0);
    // Pad the union bounds by more than the largest projected filter
    // shift. The RAW mask clear always covers its FULL texture;
    // this scissor applies ONLY to the final filtered composite.
    int pad=radius*3+6;
    x0-=pad;y0-=pad;x1+=pad;y1+=pad;
    if(x0<0)x0=0;
    if(y0<0)y0=0;
    if(x1>=B3_PS2_GS_W)x1=B3_PS2_GS_W-1;
    if(y1>=B3_PS2_GS_H)y1=B3_PS2_GS_H-1;
    // Silhouette fully out of view: scissor to one pixel and let
    // projection/clipping skip triangles naturally.
    if(x0>x1 || y0>y1){x0=x1=0;y0=y1=0;}
    s_pcssX0=x0;s_pcssY0=y0;
    s_pcssX1=x1;s_pcssY1=y1;
}
static void b3GsPcssScissor(void)
{
    b3GsAD(GS_SET_SCISSOR(s_pcssX0,s_pcssX1,
                          s_pcssY0,s_pcssY1),GS_REG_SCISSOR_1);
}
#endif
void b3GsShadowBegin(void)
{
    s_shadowPass=1;
    // The physical receiver bounds clip shadow geometry, while later
    // opaque solids occlude it in screen space. Z writing stays off
    // during the RGB coverage-mask pass.
    b3GsAD(GS_SET_ZBUF(s_zbuffer>>11,B3_GS_DEPTH_PSM,1),GS_REG_ZBUF_1);
    // Clear all RAW texels for every caster: a scissored partial clear
    // left stale pixels for neighboring bilinear taps, creating boxes.
    b3GsKawaseTarget(s_shadowMask,B3_PS2_GS_W,B3_PS2_GS_H);
    b3GsKawaseClear(B3_PS2_GS_W,B3_PS2_GS_H);
    // The receiver polygon is geometrically clipped. Do not Z-discard
    // it against checker-floor triangles with different tessellation.
    // The renderer composites shadows before subsequent opaque objects.
}
void b3GsShadowEnd(void)
{
    s_shadowPass=0;
#if defined(BOX3D_GS_PCSS_SHADOWS)
    // PCSS groups casters by receiver gap. Each group has one clipped
    // mask and one subtractive composite, not one pass per body.
    // Nearby geometry uses the 4-bilinear-tap PCF3x3/96 FIX kernel;
    // distant groups prefilter to half resolution, then use 9 taps.
    // GS bilinear PCF at contact. At larger gaps the raw binary
    // silhouettes are first filtered by native GS 2:1 downsampling.
    // This removes isolated jagged mask texels before radius sampling.
#if defined(BOX3D_PS2_UNIFIED)
    int pcssTier=b3Ps2PcssDownsampleLevel(s_pcssRadius);
    if(pcssTier>=1)
    {
        // Connected filter footprint: full 640x480 -> half 320x240.
        // The 4-bit GS bilinear interpolation, not widely separated
        // subtractive shadow copies, produces the near penumbra.
        b3GsKawaseTarget(s_pcssPrefilter,B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
        b3GsKawaseClear(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
        b3GsKawaseSource(s_shadowMask,B3_PS2_GS_W,B3_PS2_GS_H,0,0);
        b3GsAD(GS_SET_ALPHA(0,2,2,1,128),GS_REG_ALPHA_1);
        b3GsKawaseBlit(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,
                       B3_PS2_GS_W,B3_PS2_GS_H,0,0);
        if(pcssTier>=2)
        {
            // Far casters: another real 2:1 GS bilinear downsample.
            // Disjoint PSMCT32 buffers, no extra rendering mode/ELF.
            b3GsKawaseTarget(s_pcssQuarterFilter,
                             B3_KAWASE_QUARTER_W,B3_KAWASE_QUARTER_H);
            b3GsKawaseClear(B3_KAWASE_QUARTER_W,B3_KAWASE_QUARTER_H);
            b3GsKawaseSource(s_pcssPrefilter,
                             B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,0,0);
            b3GsAD(GS_SET_ALPHA(0,2,2,1,128),GS_REG_ALPHA_1);
            b3GsKawaseBlit(B3_KAWASE_QUARTER_W,B3_KAWASE_QUARTER_H,
                           B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,0,0);
        }
    }
    else
#endif
        b3GsShadowCompositeSource(s_shadowMask,B3_PS2_GS_W,B3_PS2_GS_H);
    b3GsPcssScissor();
    if(s_pcssRadius==1)
    {
#if defined(BOX3D_GS_PCF_4TAP)
        // Near-contact PCSS IS PCF: four half-texel GS bilinear samples
        // realize the 3x3 separable tent (FIX=24 per tap, total 96).
        b3GsPcf4Tap(B3_PS2_GS_W,B3_PS2_GS_H);
#else
        for(int iy=-1;iy<=1;++iy)
        for(int ix=-1;ix<=1;++ix)
        {
            int fix=(ix==0&&iy==0)?24:
                    ((ix==0||iy==0)?12:6);
            b3GsShadowCompositeTap(B3_PS2_GS_W,B3_PS2_GS_H,
                                   ix,iy,fix);
        }
#endif
    }
    else
    {
#if defined(BOX3D_PS2_UNIFIED)
        // Filter on an 8-bit/channel scratch target before touching the
        // 5-bit/channel display framebuffer. The nine 2D tent taps use
        // weights totaling 96 and have connected support in both axes.
        // Integer minor-axis compression previously mapped +/-1 to zero,
        // accidentally collapsing the entire 3x3 kernel into one row.
        const int src=(pcssTier==2)?s_pcssQuarterFilter:s_pcssPrefilter;
        const int srcW=(pcssTier==2)?
                       B3_KAWASE_QUARTER_W:B3_KAWASE_HALF_W;
        const int srcH=(pcssTier==2)?
                       B3_KAWASE_QUARTER_H:B3_KAWASE_HALF_H;
        b3GsKawaseTarget(s_pcssFiltered,B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
        b3GsKawaseClear(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
        b3GsKawaseSource(src,srcW,srcH,0,0);
        for(int iy=-1;iy<=1;++iy)
        for(int ix=-1;ix<=1;++ix)
        {
            int fix=(ix==0&&iy==0)?24:
                    ((ix==0||iy==0)?12:6);
            // ADD sampled coverage into the disjoint 32-bit mask.
            b3GsAD(GS_SET_ALPHA(0,2,2,1,fix),GS_REG_ALPHA_1);
            b3GsKawaseBlit(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,
                           srcW,srcH,ix,iy);
        }
        // A single final RGB subtraction onto the 16-bit framebuffer.
        b3GsShadowCompositeSource(s_pcssFiltered,
                                  B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
        b3GsPcssScissor();
        b3GsShadowCompositeTap(B3_KAWASE_HALF_W,
                               B3_KAWASE_HALF_H,0,0,128);
#else
        for(int iy=-2;iy<=2;++iy)
        for(int ix=-2;ix<=2;++ix)
        {
            int fix=b3Ps2ShadowWeight5(ix,iy);
            if(fix==0)continue;
            int dx=b3Ps2ShadowOffset5(ix,s_pcssRadius);
            int dy=b3Ps2ShadowOffset5(iy,s_pcssRadius);
            if(s_pcssMajorX)
                dy=b3Ps2ShadowCompressMinor(dy);
            else
                dx=b3Ps2ShadowCompressMinor(dx);
            b3GsShadowCompositeTap(B3_PS2_GS_W,B3_PS2_GS_H,
                                   dx,dy,fix);
        }
#endif
    }
    // Next receiver, another caster, and the HUD all need full scissor.
    b3GsAD(GS_SET_SCISSOR(0,B3_PS2_GS_W-1,0,B3_PS2_GS_H-1),
           GS_REG_SCISSOR_1);
#elif defined(BOX3D_GS_PCF_SHADOWS)
    // PCF-like visibility filtering on a pre-rasterized shadow mask.
    // This is NOT depth-map PCF: the geometry/receiver has already
    // resolved occlusion into RGB coverage. Sample that coverage using
    // GS bilinear TEX1 and nine neighboring taps at FULL resolution.
    //
    // One sample at (x,y) with a 3x3 separable Gaussian-ish kernel:
    //      6 12  6
    //     12 24 12   (sum = 96)
    //      6 12  6
    // The GS fixed-function subtract blend uses per-tap FIX = weight.
    // At mask RGB=128 this produces at most 96 gray levels of shadow,
    // preserves the original silhouette, and softens ~1-2 pixels rather
    // than blurring through repeated 80x56 down/up Kawase stages.
    //
    // No render-to-texture feedback: RAW and destination framebuffer
    // occupy disjoint PS2 GS VRAM allocations. TEXFLUSH handles cache.
    b3GsShadowCompositeSource(s_shadowMask,B3_PS2_GS_W,B3_PS2_GS_H);
#if defined(BOX3D_GS_PCF_4TAP)
    b3GsPcf4Tap(B3_PS2_GS_W,B3_PS2_GS_H);
#else
    const int radius=1;
    for(int iy=-1;iy<=1;++iy)
    {
        for(int ix=-1;ix<=1;++ix)
        {
            const int fix=(ix==0 && iy==0)?24:
                          ((ix==0 || iy==0)?12:6);
            // GS ALPHA = (ZERO - TEXEL_RGB) * FIX/128 + DEST_RGB.
            // For a 128-valued mask, sum 96 => darkening 96/255.
            b3GsShadowCompositeTap(B3_PS2_GS_W,B3_PS2_GS_H,
                                   ix*radius,iy*radius,fix);
        }
    }
#endif
#else
    // Full mask -> half via one GS bilinear draw.
    b3GsKawaseTarget(s_halfMask[0],B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
    b3GsKawaseClear(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
    b3GsKawaseSource(s_shadowMask,B3_PS2_GS_W,B3_PS2_GS_H,9,8);
    b3GsAD(GS_SET_ALPHA(0,2,2,1,128),GS_REG_ALPHA_1);
    b3GsKawaseBlit(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,
                   B3_PS2_GS_W,B3_PS2_GS_H,0,0);
#if defined(BOX3D_GS_KAWASE_WIDE)
    // Near penumbra FIRST, before the half-resolution target is reused
    // later by the pyramid. This is already bilinearly filtered at
    // half-res, unlike the regressed opaque RAW-direct component.
    // 4 * FIX=20 => 80 units for a connected, fairly sharp core.
    b3GsShadowCompositeSource(s_halfMask[0],
                              B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
    b3GsShadowCompositeFour(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,1,20);
#endif
    // Downsample/Kawase blur: half -> quarter.
    b3GsKawasePass(s_quarterMask[0],B3_KAWASE_QUARTER_W,
                   B3_KAWASE_QUARTER_H,s_halfMask[0],
                   B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,8,7,1);
#if defined(BOX3D_GS_KAWASE_WIDE)
    // Broad but continuous blur: progressive downsamples with a single
    // texel-step at each level, instead of spreading just four distant
    // samples of the original full-resolution mask.
    // All reads/writes are disjoint VRAM buffers.
    b3GsKawasePass(s_eighthMask[0],B3_KAWASE_EIGHTH_W,
                   B3_KAWASE_EIGHTH_H,s_quarterMask[0],
                   B3_KAWASE_QUARTER_W,B3_KAWASE_QUARTER_H,0,0,1);
    b3GsKawasePass(s_eighthMask[1],B3_KAWASE_EIGHTH_W,
                   B3_KAWASE_EIGHTH_H,s_eighthMask[0],
                   B3_KAWASE_EIGHTH_W,B3_KAWASE_EIGHTH_H,0,0,1);
    b3GsKawasePass(s_quarterMask[1],B3_KAWASE_QUARTER_W,
                   B3_KAWASE_QUARTER_H,s_eighthMask[1],
                   B3_KAWASE_EIGHTH_W,B3_KAWASE_EIGHTH_H,0,0,1);
#else
    // Original quarter-res version remains available as comparison.
    b3GsKawasePass(s_quarterMask[1],B3_KAWASE_QUARTER_W,
                   B3_KAWASE_QUARTER_H,s_quarterMask[0],
                   B3_KAWASE_QUARTER_W,B3_KAWASE_QUARTER_H,7,6,1);
#endif
    // Upsample/Kawase blur: quarter -> half, separate VRAM page.
    b3GsKawasePass(s_halfMask[1],B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,
                   s_quarterMask[1],B3_KAWASE_QUARTER_W,
                   B3_KAWASE_QUARTER_H,7,6,
#if defined(BOX3D_GS_KAWASE_WIDE)
                   1
#else
                   2
#endif
                   );
    // Optional refinement ping-pong at half resolution.
    b3GsKawasePass(s_halfMask[0],B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,
                   s_halfMask[1],B3_KAWASE_HALF_W,
                   B3_KAWASE_HALF_H,8,7,1);

    // Back to main scene. GS RGB subtract blend:
    // (ZERO - SRC_RGB)*(FIX/128) + DEST_RGB.
    // Four shifted textured sprites with FIX=28 yield up to 112
    // gray levels of shadow at 128-level coverage. Previous FIX=16
    // yielded only 64, so a sparse RAW mask disappeared after filtering.
    // RGB coverage rather than alpha controls blend opacity.
#if defined(BOX3D_GS_KAWASE_WIDE)
    // The far penumbra contributes only 4 * FIX=4 = 16 units.
    // Total near (80) + far (16) = 96, entirely from FILTERED masks.
    // No RAW shadow is directly subtracted from the final image.
    b3GsShadowCompositeSource(s_halfMask[0],
                              B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
    b3GsShadowCompositeFour(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,1,4);
#else
    b3GsShadowCompositeSource(s_halfMask[0],
                              B3_KAWASE_HALF_W,B3_KAWASE_HALF_H);
    b3GsShadowCompositeFour(B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,2,28);
#endif
#endif
    // Restore the depth test before drawing another receiver or the
    // dynamic boxes. The shadow mask itself must not alter scene depth.
    b3GsAD(GS_SET_ZBUF(s_zbuffer>>11,B3_GS_DEPTH_PSM,0),GS_REG_ZBUF_1);
    b3GsAD(GS_SET_TEST(0,0,0,0,0,0,1,2),GS_REG_TEST_1);
}

// Diagnostic overlays deliberately bypass GS alpha/depth tests.
// Show three actual VRAM textures side-by-side after the soft composite:
// first full-resolution shadow mask, quarter-res ping-pong result,
// and final half-res result. Black thumbnails mean an earlier stage
// failed; visible thumbnails with no shadow imply final composite failure.
#if defined(BOX3D_GS_KAWASE_DEBUG)
static void b3GsKawasePreview(int x,int y,int targetWidth,int targetHeight,
                              int source,int sourceWidth,int sourceHeight,
                              int tw,int th)
{
    b3GsKawaseTarget(s_framebuffer[s_back],B3_PS2_GS_W,B3_PS2_GS_H);
    b3GsAD(GS_SET_PRIM(6,0,0,0,0,0,0,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(22,30,42,128,0x3f800000),GS_REG_RGBAQ);
    b3GsAD(b3GsXYZ(x-2,y-2),GS_REG_XYZ2);
    b3GsAD(b3GsXYZ(x+targetWidth+2,y+targetHeight+2),GS_REG_XYZ2);

    b3GsKawaseSource(source,sourceWidth,sourceHeight,tw,th);
    b3GsAD(GS_SET_PRIM(6,0,1,0,0,0,1,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_UV(8,8),GS_REG_UV);
    b3GsAD(b3GsXYZ(x,y),GS_REG_XYZ2);
    b3GsAD(GS_SET_UV((sourceWidth<<4)+8,(sourceHeight<<4)+8),GS_REG_UV);
    b3GsAD(b3GsXYZ(x+targetWidth,y+targetHeight),GS_REG_XYZ2);
}
void b3GsKawaseDebugOverlay(void)
{
    b3GsKawasePreview(8,171,92,40,s_shadowMask,
                      B3_PS2_GS_W,B3_PS2_GS_H,9,8);
    b3GsKawasePreview(114,171,92,40,s_quarterMask[1],
                      B3_KAWASE_QUARTER_W,B3_KAWASE_QUARTER_H,7,6);
    b3GsKawasePreview(220,171,92,40,s_halfMask[0],
                      B3_KAWASE_HALF_W,B3_KAWASE_HALF_H,8,7);
    b3GsOverlayBegin();
    b3GsOverlayText(10,161,"RAW",252,245,223);
    b3GsOverlayText(116,161,"QUARTER",252,245,223);
    b3GsOverlayText(222,161,"FINAL",252,245,223);
}
#endif
#else
void b3GsShadowBegin(void)
{
    // Blend semi-transparent hard-edged shadow geometry onto the ground.
    // Stop writing depth, so overlapped shadows don't block other bodies.
    s_shadowPass=1;
    b3GsAD(GS_SET_ZBUF(s_zbuffer>>11,B3_GS_DEPTH_PSM,1),GS_REG_ZBUF_1);
    // These convex silhouettes are clipped to the receiver. Foreground
    // occluders will be drawn AFTER the overlay, avoiding shadow holes.
    b3GsAD(GS_SET_TEST(0,0,0,0,0,0,0,0),GS_REG_TEST_1);
    b3GsAD(GS_SET_ALPHA(0,1,0,1,0),GS_REG_ALPHA_1);
}
void b3GsShadowEnd(void)
{
    s_shadowPass=0;
    b3GsAD(GS_SET_ZBUF(s_zbuffer>>11,B3_GS_DEPTH_PSM,0),GS_REG_ZBUF_1);
    b3GsAD(GS_SET_TEST(0,0,0,0,0,0,1,2),GS_REG_TEST_1);
}
#endif

// HUD is 2D GS sprite pixel glyphs, not BIOS scr_printf or a texture.
// Z test is disabled only after rendering all world geometry.
void b3GsOverlayBegin(void)
{
    b3GsAD(GS_SET_TEST(0,0,0,0,0,0,0,0),GS_REG_TEST_1);
    b3GsAD(GS_SET_ALPHA(0,1,0,1,0),GS_REG_ALPHA_1);
}
void b3GsOverlayPanel(int x,int y,int width,int height)
{
    if(s_used+5>=B3_GS_QWORDS) { ++s_errors; return; }
    b3GsAD(GS_SET_PRIM(6,0,0,0,1,0,0,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(15,24,36,93,0x3f800000),GS_REG_RGBAQ);
    b3GsAD(b3GsXYZ(x,y),GS_REG_XYZ2);
    b3GsAD(b3GsXYZ(x+width,y+height),GS_REG_XYZ2);
}

// Minimal 5x7 uppercase HUD glyphs. Packed left-to-right, bit4 first.
// Spaces and unsupported letters are left blank.
static const uint8_t s_font[36][7]={
    {14,17,17,31,17,17,17}, // A
    {30,17,17,30,17,17,30}, // B
    {14,17,16,16,16,17,14}, // C
    {30,17,17,17,17,17,30}, // D
    {31,16,16,30,16,16,31}, // E
    {31,16,16,30,16,16,16}, // F
    {14,17,16,23,17,17,14}, // G
    {17,17,17,31,17,17,17}, // H
    {31,4,4,4,4,4,31}, // I
    {7,2,2,2,18,18,12}, // J
    {17,18,20,24,20,18,17}, // K
    {16,16,16,16,16,16,31}, // L
    {17,27,21,21,17,17,17}, // M
    {17,25,21,19,17,17,17}, // N
    {14,17,17,17,17,17,14}, // O
    {30,17,17,30,16,16,16}, // P
    {14,17,17,17,21,18,13}, // Q
    {30,17,17,30,20,18,17}, // R
    {15,16,16,14,1,1,30}, // S
    {31,4,4,4,4,4,4}, // T
    {17,17,17,17,17,17,14}, // U
    {17,17,17,17,17,10,4}, // V
    {17,17,17,21,21,21,10}, // W
    {17,17,10,4,10,17,17}, // X
    {17,17,10,4,4,4,4}, // Y
    {31,1,2,4,8,16,31}, // Z
    {14,17,19,21,25,17,14}, // 0
    {4,12,4,4,4,4,14}, // 1
    {14,17,1,2,4,8,31}, // 2
    {30,1,1,14,1,1,30}, // 3
    {2,6,10,18,31,2,2}, // 4
    {31,16,30,1,1,17,14}, // 5
    {7,8,16,30,17,17,14}, // 6
    {31,1,2,4,8,8,8}, // 7
    {14,17,17,14,17,17,14}, // 8
    {14,17,17,15,1,2,28} // 9
};
void b3GsOverlayText(int x,int y,const char* text,uint8_t r,uint8_t g,uint8_t b)
{
    if(text==NULL) return;
#if defined(BOX3D_GS_PROGRESSIVE)
    const int scale=2;
#else
    const int scale=1;
#endif
    b3GsAD(GS_SET_PRIM(6,0,0,0,0,0,0,0,0),GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(r,g,b,0x80,0x3f800000),GS_REG_RGBAQ);
    for(const unsigned char* cp=(const unsigned char*)text;*cp;++cp)
    {
        unsigned ch=*cp;
        int glyph=-1;
        if(ch>='a' && ch<='z') ch -= 32;
        if(ch>='A' && ch<='Z') glyph=(int)(ch-'A');
        if(ch>='0' && ch<='9') glyph=(int)(ch-'0'+26);
        if(glyph>=0)
        {
            for(int row=0;row<7;++row)
            {
                unsigned mask=s_font[glyph][row];
                for(int col=0;col<5;++col)
                {
                    if(mask & (1u << (4-col)))
                    {
                        if(s_used+3>=B3_GS_QWORDS){++s_errors;return;}
                        b3GsAD(b3GsXYZ(x+col*scale,y+row*scale),GS_REG_XYZ2);
                        b3GsAD(b3GsXYZ(x+(col+1)*scale,
                                        y+(row+1)*scale),GS_REG_XYZ2);
                    }
                }
            }
        }
        x+=6*scale;
        if(x>B3_PS2_GS_W-6*scale)break;
    }
}
#else
void b3GsSkyGradient(void){}
void b3GsShadowBegin(void){}
void b3GsShadowEnd(void){}
void b3GsOverlayBegin(void){}
void b3GsOverlayPanel(int x,int y,int w,int h)
{(void)x;(void)y;(void)w;(void)h;}
void b3GsOverlayText(int x,int y,const char* t,uint8_t r,uint8_t g,uint8_t b)
{(void)x;(void)y;(void)t;(void)r;(void)g;(void)b;}
#endif

unsigned b3GsGetLastGifQwords(void)
{
    return s_lastGifQwords;
}
#if defined(BOX3D_PS2_UNIFIED)
uint32_t b3GsGetLastGsWaitTicks(void) {return s_lastGsWaitTicks;}
uint32_t b3GsGetLastVsyncWaitTicks(void) {return s_lastVsyncWaitTicks;}
#endif
void b3GsPresent(void)
{
    // One GIF PACKED tag (NREG=1, A+D), followed by raw 128b A+D pairs.
    // Finish event is emitted by GS after all prior drawing.
    // PS2SDK draw_finish convention: write 1 to GS FINISH register.
    b3GsAD(1, GS_REG_FINISH);
    if (s_errors) printf("BOX3D_GS_WARNING,packet_overflow=%d\n",s_errors);
    s_packet[0].data = GIF_SET_TAG(s_used - 1, 1, 0, 0, GIF_FLG_PACKED, 1);
    s_packet[0].reg = GIF_REG_AD;
    s_lastGifQwords = s_used;

    // Clear GS FINISH (W1C), DMA sends cache-flushed packet and wait for
    // GS completion, not just DMA drain. Never write displayed framebuffer.
#if defined(BOX3D_PS2_UNIFIED)
    uint32_t gsBegin=b3Ps2ReadCpuCycles();
#endif
    *GS_REG_CSR = (uint64_t)2;
    dma_channel_send_normal(DMA_CHANNEL_GIF, s_packet, s_used, 0, 0);
    dma_channel_wait(DMA_CHANNEL_GIF, 0);
    while (((*GS_REG_CSR) & (uint64_t)2) == 0) {}
#if defined(BOX3D_PS2_UNIFIED)
    uint32_t gsComplete=b3Ps2ReadCpuCycles();
    s_lastGsWaitTicks=b3Ps2DeltaCpuCycles(gsBegin,gsComplete);
#endif
    graph_wait_vsync();
#if defined(BOX3D_PS2_UNIFIED)
    s_lastVsyncWaitTicks=
        b3Ps2DeltaCpuCycles(gsComplete,b3Ps2ReadCpuCycles());
#endif
    graph_set_framebuffer(0,s_framebuffer[s_back],B3_PS2_GS_W,B3_GS_COLOR_PSM,0,0);
    graph_set_framebuffer(1,s_framebuffer[s_back],B3_PS2_GS_W,B3_GS_COLOR_PSM,0,0);
    s_back ^= 1;
}
