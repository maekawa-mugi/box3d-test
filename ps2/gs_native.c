// SPDX-License-Identifier: MIT
// PS2 native GS wireframe backend using packed GIF A+D and channel-2 DMA.
// PS2SDK graph manages the CRT/read circuit only. No draw, no gsKit.
#include "gs_native.h"
#include <dma.h>
#include <gif_tags.h>
#include <graph.h>
#include <gs_gp.h>
#include <gs_psm.h>
#include <gs_privileged.h>
#include <kernel.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef struct B3GsQword { uint64_t data, reg; } B3GsQword;
#if defined(BOX3D_GS_OFFICIAL)
#define B3_GS_QWORDS 8192
#else
#define B3_GS_QWORDS 2048
#endif
static _Alignas(64) B3GsQword s_packet[B3_GS_QWORDS];
static unsigned s_used;
static int s_framebuffer[2];
static int s_back = 1;
#if defined(BOX3D_GS_SOLID)
static int s_zbuffer;
#endif
static int s_errors;
#if defined(BOX3D_GS_OFFICIAL)
static int s_shadowPass;
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

int b3GsInit(void)
{
    graph_vram_clear();
    s_framebuffer[0] = graph_vram_allocate(B3_PS2_GS_W, B3_PS2_GS_H,
                                           GS_PSM_32, GRAPH_ALIGN_PAGE);
    s_framebuffer[1] = graph_vram_allocate(B3_PS2_GS_W, B3_PS2_GS_H,
                                           GS_PSM_32, GRAPH_ALIGN_PAGE);
#if defined(BOX3D_GS_SOLID)
    // PS2 GS ZBUF_32 is a distinct 8192-byte-page-aligned VRAM buffer.
    // The renderer still uses the two previously proven color buffers.
    s_zbuffer = graph_vram_allocate(B3_PS2_GS_W, B3_PS2_GS_H,
                                    GS_ZBUF_32, GRAPH_ALIGN_PAGE);
    if(s_zbuffer < 0)
    {
        puts("BOX3D_GS_FAIL,zbuffer_allocation");
        return 0;
    }
#endif
    if (s_framebuffer[0] < 0 || s_framebuffer[1] < 0)
    {
        puts("BOX3D_GS_FAIL,framebuffer_allocation");
        return 0;
    }

    // graph library is the only video-mode helper. All drawing uses GS
    // registers directly via the packed GIF A+D stream below.
    graph_initialize(s_framebuffer[0], B3_PS2_GS_W, B3_PS2_GS_H, GS_PSM_32, 0, 0);
    int region = graph_get_region();
    if (graph_set_mode(GRAPH_MODE_NONINTERLACED, region, GRAPH_MODE_FIELD,
                       GRAPH_DISABLE) < 0)
    {
        puts("BOX3D_GS_FAIL,crt_mode");
        return 0;
    }
    graph_set_screen(0, 0, B3_PS2_GS_W, B3_PS2_GS_H);
    graph_set_framebuffer(0, s_framebuffer[0], B3_PS2_GS_W, GS_PSM_32, 0, 0);
    graph_set_framebuffer(1, s_framebuffer[0], B3_PS2_GS_W, GS_PSM_32, 0, 0);
    graph_enable_output();
    dma_channel_initialize(DMA_CHANNEL_GIF, NULL, 0);
#if defined(BOX3D_GS_SOLID)
    printf("BOX3D_GS_READY,native_gif=1,solid=1,zbuf=%d,width=320,height=224,fb=2\n",s_zbuffer);
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
    b3GsAD(GS_SET_FRAME(s_framebuffer[s_back] >> 11, B3_PS2_GS_W >> 6, GS_PSM_32, 0),
           GS_REG_FRAME_1);
    b3GsAD(GS_SET_XYOFFSET(2048 << 4, 2048 << 4), GS_REG_XYOFFSET_1);
    b3GsAD(GS_SET_SCISSOR(0, B3_PS2_GS_W - 1, 0, B3_PS2_GS_H - 1), GS_REG_SCISSOR_1);
#if defined(BOX3D_GS_SOLID)
    b3GsAD(GS_SET_ZBUF(s_zbuffer >> 11, GS_ZBUF_32, 0), GS_REG_ZBUF_1);
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
                              uint8_t r,uint8_t g,uint8_t b)
{
    // Reject wholly offscreen segments; clip very large projected
    // coordinates into the GS's representable 12.4 XY range.
    if ((x0 < -64 && x1 < -64) || (x0 > B3_PS2_GS_W + 64 && x1 > B3_PS2_GS_W + 64) ||
        (y0 < -64 && y1 < -64) || (y0 > B3_PS2_GS_H + 64 && y1 > B3_PS2_GS_H + 64))
        return;
    if (s_used + 5 >= B3_GS_QWORDS) { ++s_errors; return; }
    if (x0 < -256) x0 = -256;
    if (x1 < -256) x1 = -256;
    if (x0 > 576) x0 = 576;
    if (x1 > 576) x1 = 576;
    if (y0 < -256) y0 = -256;
    if (y1 < -256) y1 = -256;
    if (y0 > 480) y0 = 480;
    if (y1 > 480) y1 = 480;
    b3GsAD(GS_SET_PRIM(1, 0, 0, 0, 0, 0, 0, 0, 0), GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(r, g, b, 0x80, 0x3f800000), GS_REG_RGBAQ);
    b3GsAD(b3GsXYZDepth(x0, y0, z0), GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepth(x1, y1, z1), GS_REG_XYZ2);
}
void b3GsLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
    b3GsLineWithDepth(x0,y0,0,x1,y1,0,r,g,b);
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

static int projectDepth(b3GsCamera c,b3GsPoint3 v,int* ox,int* oy,
                        uint32_t* oz)
{
    b3GsPoint3 d=sub(v,c.eye);
    float z=dot(d,c.forward);
    if (!isfinite(z) || z<=0.5f || z >= 220.0f) return 0;
    float px=B3_PS2_GS_W*0.5f+c.focal*dot(d,c.right)/z;
    float py=B3_PS2_GS_H*0.5f-c.focal*dot(d,c.up)/z;
    if (!isfinite(px)||!isfinite(py)||fabsf(px)>10000.0f||fabsf(py)>10000.0f) return 0;
    *ox=(int)px; *oy=(int)py;
    // GS ZTEST=GREATER_EQUAL: near must be greater. 2^20 units/m.
    // All valid depth values are positive; clear sprite uses Z=0.
    *oz=(uint32_t)((220.0f-z)*1048576.0f);
    return 1;
}
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
    if (projectDepth(c,a,&x0,&y0,&z0) &&
        projectDepth(c,b,&x1,&y1,&z1))
    {
        // Small positive bias: wire outlines remain visible over filled
        // triangles instead of flickering at near-equal Z.
        b3GsLineWithDepth(x0,y0,z0+4096,x1,y1,z1+4096,r,g,blue);
    }
#else
    if (project(c,a,&x0,&y0) && project(c,b,&x1,&y1))
        b3GsLine(x0,y0,x1,y1,r,g,blue);
#endif
}

void b3GsTriangle3(b3GsCamera c,b3GsPoint3 a,b3GsPoint3 b,
                   b3GsPoint3 d,uint8_t r,uint8_t g,uint8_t blue)
{
#if defined(BOX3D_GS_SOLID)
    int x0,y0,x1,y1,x2,y2;
    uint32_t z0,z1,z2;
    if (!projectDepth(c,a,&x0,&y0,&z0) ||
        !projectDepth(c,b,&x1,&y1,&z1) ||
        !projectDepth(c,d,&x2,&y2,&z2)) return;

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
    if(x0 > 576) x0 = 576; if(x1 > 576) x1 = 576; if(x2 > 576) x2 = 576;
    if(y0 > 480) y0 = 480; if(y1 > 480) y1 = 480; if(y2 > 480) y2 = 480;
#if defined(BOX3D_GS_OFFICIAL)
    b3GsAD(GS_SET_PRIM(3, 0, 0, 0, s_shadowPass ? 1 : 0, 0, 0, 0, 0), GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(r,g,blue,s_shadowPass ? 0x50 : 0x80,0x3f800000),GS_REG_RGBAQ);
#else
    b3GsAD(GS_SET_PRIM(3, 0, 0, 0, 0, 0, 0, 0, 0), GS_REG_PRIM);
    b3GsAD(GS_SET_RGBAQ(r,g,blue,0x80,0x3f800000),GS_REG_RGBAQ);
#endif
    b3GsAD(b3GsXYZDepth(x0,y0,z0),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepth(x1,y1,z1),GS_REG_XYZ2);
    b3GsAD(b3GsXYZDepth(x2,y2,z2),GS_REG_XYZ2);
#else
    (void)c;(void)a;(void)b;(void)d;(void)r;(void)g;(void)blue;
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

void b3GsShadowBegin(void)
{
    // Blend semi-transparent hard-edged shadow geometry onto the ground.
    // Stop writing depth, so overlapped shadows don't block other bodies.
    s_shadowPass=1;
    b3GsAD(GS_SET_ZBUF(s_zbuffer>>11,GS_ZBUF_32,1),GS_REG_ZBUF_1);
    // (Cs-Cd)*As + Cd, source alpha 80/128 ~= 63% shadow darkening.
    b3GsAD(GS_SET_ALPHA(0,1,0,1,0),GS_REG_ALPHA_1);
}
void b3GsShadowEnd(void)
{
    s_shadowPass=0;
    b3GsAD(GS_SET_ZBUF(s_zbuffer>>11,GS_ZBUF_32,0),GS_REG_ZBUF_1);
}

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
                        b3GsAD(b3GsXYZ(x+col,y+row),GS_REG_XYZ2);
                        b3GsAD(b3GsXYZ(x+col+1,y+row+1),GS_REG_XYZ2);
                    }
                }
            }
        }
        x+=6;
        if(x>B3_PS2_GS_W-6)break;
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

void b3GsPresent(void)
{
    // One GIF PACKED tag (NREG=1, A+D), followed by raw 128b A+D pairs.
    // Finish event is emitted by GS after all prior drawing.
    // PS2SDK draw_finish convention: write 1 to GS FINISH register.
    b3GsAD(1, GS_REG_FINISH);
    if (s_errors) printf("BOX3D_GS_WARNING,packet_overflow=%d\n",s_errors);
    s_packet[0].data = GIF_SET_TAG(s_used - 1, 1, 0, 0, GIF_FLG_PACKED, 1);
    s_packet[0].reg = GIF_REG_AD;

    // Clear GS FINISH (W1C), DMA sends cache-flushed packet and wait for
    // GS completion, not just DMA drain. Never write displayed framebuffer.
    *GS_REG_CSR = (uint64_t)2;
    dma_channel_send_normal(DMA_CHANNEL_GIF, s_packet, s_used, 0, 0);
    dma_channel_wait(DMA_CHANNEL_GIF, 0);
    while (((*GS_REG_CSR) & (uint64_t)2) == 0) {}
    graph_wait_vsync();
    graph_set_framebuffer(0,s_framebuffer[s_back],B3_PS2_GS_W,GS_PSM_32,0,0);
    graph_set_framebuffer(1,s_framebuffer[s_back],B3_PS2_GS_W,GS_PSM_32,0,0);
    s_back ^= 1;
}
