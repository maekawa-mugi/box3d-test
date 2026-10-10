// SPDX-License-Identifier: MIT
// Portable convex shadow geometry for the GS-native Box3D demo.
// World-space directional projection onto an arbitrary finite receiver.
// Not a shadow map, stencil volume, or general-purpose self-shadow shader.
#ifndef B3_PS2_SHADOW_PROJECT_H
#define B3_PS2_SHADOW_PROJECT_H

#define B3_SHADOW_MAX_VERTS 32

typedef struct b3ShadowVec3 { float x,y,z; } b3ShadowVec3;
typedef struct b3ShadowVec2 { float u,v; } b3ShadowVec2;

typedef struct b3ShadowReceiver
{
    b3ShadowVec3 origin;
    b3ShadowVec3 axisU,axisV,normal; // Orthonormal basis.
    float halfU,halfV;              // Finite receiver area.
} b3ShadowReceiver;

typedef struct b3ShadowPolygon
{
    int count;
    b3ShadowVec2 vertices[B3_SHADOW_MAX_VERTS];
} b3ShadowPolygon;

// A convex box's eight corners project to a convex polygon. Use all eight,
// *not* only one bottom-face quad (which causes missing/shredded shadows
// when the box is elevated or rotated). Clip the hull to the receiver.
// Returns polygon size (0 on empty, invalid or backfacing receiver).
int b3ShadowProjectBox(const b3ShadowVec3 corners[8],
                       b3ShadowVec3 lightRay,
                       const b3ShadowReceiver* surface,
                       b3ShadowPolygon* result);

b3ShadowVec3 b3ShadowOnReceiver(b3ShadowVec2 p,
                                const b3ShadowReceiver* surface,
                                float normalBias);
#endif
