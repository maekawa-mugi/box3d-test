// SPDX-License-Identifier: MIT
// Headless scalar validation on a host or PS2 EE.
#include "box3d/box3d.h"
#include <math.h>
#include <stdio.h>

int main( void )
{
    int64_t initialBytes = b3GetByteCount();
    int initialWorlds = b3GetWorldCount();
    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.workerCount = 1;
    b3WorldId worldId = b3CreateWorld( &worldDef );

    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = (b3Vec3){ 0.0f, -1.0f, 0.0f };
    b3BodyId ground = b3CreateBody( worldId, &groundDef );
    b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
    b3ShapeDef groundShape = b3DefaultShapeDef();
    b3CreateHullShape( ground, &groundShape, &groundBox.base );

    b3BodyDef fallingDef = b3DefaultBodyDef();
    fallingDef.type = b3_dynamicBody;
    fallingDef.position = (b3Vec3){ 0.0f, 4.0f, 0.0f };
    b3BodyId falling = b3CreateBody( worldId, &fallingDef );
    b3BoxHull box = b3MakeCubeHull( 0.5f );
    b3ShapeDef fallingShape = b3DefaultShapeDef();
    fallingShape.density = 1.0f;
    b3CreateHullShape( falling, &fallingShape, &box.base );

    for ( int i = 0; i < 15; ++i )
        b3World_Step( worldId, 1.0f / 60.0f, 4 );
    float earlyY = b3Body_GetPosition( falling ).y;

    for ( int i = 15; i < 180; ++i )
        b3World_Step( worldId, 1.0f / 60.0f, 4 );
    float landedY = b3Body_GetPosition( falling ).y;

    for ( int i = 180; i < 300; ++i )
        b3World_Step( worldId, 1.0f / 60.0f, 4 );
    float settledY = b3Body_GetPosition( falling ).y;

    int gravityOk = isfinite( earlyY ) && earlyY > 2.0f && earlyY < 4.0f;
    int contactOk = isfinite( landedY ) && landedY > 0.30f && landedY < 0.80f;
    int stabilityOk = isfinite( settledY ) && settledY > 0.30f && settledY < 0.80f &&
                      fabsf( settledY - landedY ) < 0.10f;
    printf( "PS2 scalar: earlyY=%.4f landedY=%.4f settledY=%.4f\n", earlyY, landedY, settledY );

    b3DestroyWorld( worldId );
    int cleanupOk = b3GetWorldCount() == initialWorlds && b3GetByteCount() == initialBytes;
    if ( !gravityOk ) puts( "FAIL: gravity" );
    if ( !contactOk ) puts( "FAIL: ground contact" );
    if ( !stabilityOk ) puts( "FAIL: contact stability" );
    if ( !cleanupOk ) puts( "FAIL: world/memory cleanup" );
    int ok = gravityOk && contactOk && stabilityOk && cleanupOk;
    puts( ok ? "PASS" : "FAIL" );
    return ok ? 0 : 1;
}
