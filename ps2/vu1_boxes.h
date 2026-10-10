// SPDX-License-Identifier: MIT
// GS box geometry emitted by VU1 through GIF PATH1. PCSS stays on PATH3.
// All input arrays are copied into persistent aligned EE memory before DMA.
#pragma once
#include <stdint.h>
#include "gs_native.h"

enum { B3_VU1_MAX_BOXES=400, B3_VU1_CORNERS=8, B3_VU1_FACES=6 };
typedef struct b3Vu1BoxInput
{
    _Alignas(16) float vertices[8][4];
    uint32_t colors[6][4];
} b3Vu1BoxInput;

// Initialize once, after the GS has initialized its GIF channel.
// Returns zero if packet2/VIF1 could not initialize.
int b3Vu1BoxesInit(void);
// emitToGs=0 runs full VU1 transform/assembly but skips XGKICK.
// Diagnostic output must still be drawn through the EE fallback.
void b3Vu1BoxesBegin(b3GsCamera camera,int emitToGs);
int b3Vu1BoxesAdd(const b3GsPoint3 corners[8],
                  const uint8_t colors[6][3]);
// Called only after b3GsFlushForVu1 has completed PATH3.
// Waits for VIF1 and GIF PATH1 before returning, so HUD on PATH3
// cannot overtake the triangle stream. Returns submitted box count.
int b3Vu1BoxesSubmit(void);
