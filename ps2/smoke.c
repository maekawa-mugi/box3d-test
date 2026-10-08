// SPDX-License-Identifier: MIT
// Headless sanity test: compile on desktop first, then on the PS2 EE.
#include "box3d/box3d.h"
#include <math.h>
#include <stdio.h>

int main(void)
{
    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.workerCount = 1;
    b3WorldId worldId = b3CreateWorld(&worldDef);

    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = (b3Vec3){0.0f, -1.0f, 0.0f};
    b3BodyId ground = b3CreateBody(worldId, &groundDef);
    b3BoxHull groundBox = b3MakeBoxHull(20.0f, 1.0f, 20.0f);
    b3ShapeDef groundShape = b3DefaultShapeDef();
    b3CreateHullShape(ground, &groundShape, &groundBox.base);

    b3BodyDef fallingDef = b3DefaultBodyDef();
    fallingDef.type = b3_dynamicBody;
    fallingDef.position = (b3Vec3){0.0f, 4.0f, 0.0f};
    b3BodyId falling = b3CreateBody(worldId, &fallingDef);
    b3BoxHull box = b3MakeCubeHull(0.5f);
    b3ShapeDef fallingShape = b3DefaultShapeDef();
    fallingShape.density = 1.0f;
    b3CreateHullShape(falling, &fallingShape, &box.base);

    for (int i = 0; i < 180; ++i)
    {
        b3World_Step(worldId, 1.0f / 60.0f, 4);
    }

    b3Vec3 pos = b3Body_GetPosition(falling);
    printf("PS2 scalar baseline: cube y=%.4f\n", pos.y);
    int ok = isfinite(pos.y) && pos.y > 0.30f && pos.y < 0.80f;
    b3DestroyWorld(worldId);
    puts(ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
