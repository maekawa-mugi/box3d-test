// SPDX-License-Identifier: MIT
// Minimal native GS + GIF renderer. No gsKit and no PS2SDK draw library.
// graph is used only for CRT mode / framebuffer display and dma for transfer.
#ifndef B3_PS2_NATIVE_GS_H
#define B3_PS2_NATIVE_GS_H

#include <stdint.h>
#define B3_PS2_GS_W 320
#define B3_PS2_GS_H 224

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
void b3GsPresent(void);
b3GsCamera b3GsLookAt(b3GsPoint3 eye, b3GsPoint3 target, float focal);

#endif
