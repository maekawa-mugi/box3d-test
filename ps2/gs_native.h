// SPDX-License-Identifier: MIT
// Minimal native GS + GIF renderer. No gsKit and no PS2SDK draw library.
// graph is used only for CRT mode / framebuffer display and dma for transfer.
#ifndef B3_PS2_NATIVE_GS_H
#define B3_PS2_NATIVE_GS_H

#include <stdint.h>

/*
 * One canonical retail/demo configuration. PCF is not a competing ELF:
 * its 4-tap bilinear tent is the near-field filter INSIDE the PCSS
 * contact-hardening algorithm. All physics scenes share this build.
 */
#if defined(BOX3D_PS2_UNIFIED)
#define BOX3D_GS_SOLID 1
#define BOX3D_GS_OFFICIAL 1
#define BOX3D_GS_RECEIVER_SHADOWS 1
#define BOX3D_GS_KAWASE_SHADOWS 1
#define BOX3D_GS_PCF_SHADOWS 1
#define BOX3D_GS_PCF_4TAP 1
#define BOX3D_GS_PCSS_SHADOWS 1
#define BOX3D_GS_PROGRESSIVE 1
#define BOX3D_PS2_PAD_NAV 1
#endif
#if defined(BOX3D_GS_PROGRESSIVE)
#define B3_PS2_GS_W 640
#define B3_PS2_GS_H 480
#else
#define B3_PS2_GS_W 320
#define B3_PS2_GS_H 224
#endif

typedef struct b3GsPoint3 { float x, y, z; } b3GsPoint3;
typedef struct b3GsCamera {
    b3GsPoint3 eye, right, up, forward;
    float focal;
} b3GsCamera;

int b3GsInit(void);
void b3GsBegin(void);
void b3GsLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b);
void b3GsLine3(b3GsCamera camera, b3GsPoint3 a, b3GsPoint3 b,
               uint8_t r, uint8_t g, uint8_t blue);
// Draw one filled 3D triangle with GS native PRIM=TRIANGLE, using a
// monotonic reversed-Z value so nearer triangles pass GS depth tests.
// Built only into the separate BOX3D_GS_SOLID demonstration ELF.
void b3GsTriangle3(b3GsCamera camera, b3GsPoint3 a, b3GsPoint3 b,
                   b3GsPoint3 c, uint8_t r, uint8_t g, uint8_t blue);
// Official-demo-inspired ambient renderer, built only with
// BOX3D_GS_OFFICIAL. Uses native GIF, no texture/gsKit/libdraw.
void b3GsSkyGradient(void);
void b3GsShadowBegin(void);
void b3GsShadowEnd(void);
#if defined(BOX3D_GS_PCSS_SHADOWS)
// Configure ONE grouped PCSS pass (all casters with the same radius).
// The union bbox is padded for sampling, then scissored for composite.
// majorX=1 when projected light direction is X-dominant; 0 for Y.
void b3GsShadowSetPenumbra(int radius,int x0,int y0,
                           int x1,int y1,int majorX);
int b3GsProjectScreen(b3GsCamera camera,b3GsPoint3 point,
                      int* x,int* y);
#endif
#if defined(BOX3D_GS_KAWASE_DEBUG)
void b3GsKawaseDebugOverlay(void);
#endif
// Offscreen stages are active only in BOX3D_GS_KAWASE_SHADOWS.
// The normal direct-shadow build retains its existing API/behavior.
void b3GsOverlayBegin(void);
void b3GsOverlayPanel(int x,int y,int w,int h);
void b3GsOverlayText(int x,int y,const char* text,uint8_t r,uint8_t g,uint8_t b);
void b3GsPresent(void);
// Actual GIF DMA packet length from the most recent completed frame.
unsigned b3GsGetLastGifQwords(void);
#if defined(BOX3D_PS2_UNIFIED)
// Separately instrument time spent waiting for the GIF/GS draw to
// finish and time waiting for the 60Hz VSync. 60fps alone hides load.
uint32_t b3GsGetLastGsWaitTicks(void);
uint32_t b3GsGetLastVsyncWaitTicks(void);
#endif
b3GsCamera b3GsLookAt(b3GsPoint3 eye, b3GsPoint3 target, float focal);

#endif
