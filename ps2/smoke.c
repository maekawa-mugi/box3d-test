// SPDX-License-Identifier: MIT
// Same falling-cube physics test on host and on the PS2 EE.
// The PS2 executable mirrors ps2-libc-arena: GS stages + stdout and hold.
#include "box3d/box3d.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#if defined( BOX3D_PS2 )
#include <debug.h>
#include <kernel.h>
#endif

static void report_stage( int stage, const char* description )
{
    printf( "BOX3D_STAGE,%d,%s\n", stage, description );
#if defined( BOX3D_PS2 )
    scr_setfontcolor( 0xebe5e2u );
    scr_printf( "%d. %s\n", stage, description );
#endif
}

#if defined( BOX3D_PS2 )
static void ps2_box3d_log( const char* message )
{
    printf( "BOX3D_LOG,%s\n", message );
    scr_setfontcolor( 0x98928fu );
    scr_printf( "%s\n", message );
}

static int ps2_box3d_assert( const char* condition, const char* filename, int line )
{
    printf( "BOX3D_ASSERT,%s,%s,%d\n", condition, filename, line );
    scr_setfontcolor( 0xff7777u );
    scr_printf( "ASSERT: %s\n%s:%d\n", condition, filename, line );
    scr_printf( "STOP: reset PS2 to exit.\n" );
    // An ordinary breakpoint/trap would hide this diagnostic on some loaders.
    for ( ;; )
    {
        SleepThread();
    }
    return 0;
}
#endif

int main( void )
{
#if defined( BOX3D_PS2 )
    // PS2SDK stdout may be unconnected. GS is the primary diagnostics path.
    setvbuf( stdout, NULL, _IONBF, 0 );
    init_scr();
    scr_clear();
    report_stage( 0, "EE Box3D scalar ELF started" );
    b3SetAssertFcn( ps2_box3d_assert );
    b3SetLogFcn( ps2_box3d_log );
#else
    report_stage( 0, "Host Box3D scalar smoke started" );
#endif

    int64_t initialBytes = b3GetByteCount();
    int initialWorlds = b3GetWorldCount();

    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.workerCount = 1;
    b3WorldId worldId = b3CreateWorld( &worldDef );
    report_stage( 1, "Physics world created" );

    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = (b3Vec3){ 0.0f, -1.0f, 0.0f };
    b3BodyId ground = b3CreateBody( worldId, &groundDef );
    b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
    b3ShapeDef groundShape = b3DefaultShapeDef();
    b3CreateHullShape( ground, &groundShape, &groundBox.base );
    report_stage( 2, "Static ground hull created" );

    b3BodyDef fallingDef = b3DefaultBodyDef();
    fallingDef.type = b3_dynamicBody;
    fallingDef.position = (b3Vec3){ 0.0f, 4.0f, 0.0f };
    b3BodyId falling = b3CreateBody( worldId, &fallingDef );
    b3BoxHull box = b3MakeCubeHull( 0.5f );
    b3ShapeDef fallingShape = b3DefaultShapeDef();
    fallingShape.density = 1.0f;
    b3CreateHullShape( falling, &fallingShape, &box.base );
    report_stage( 3, "Dynamic cube hull created" );

    for ( int i = 0; i < 15; ++i )
    {
        b3World_Step( worldId, 1.0f / 60.0f, 4 );
    }
    float earlyY = b3Body_GetPosition( falling ).y;
    report_stage( 4, "Completed 15 gravity steps" );

    for ( int i = 15; i < 180; ++i )
    {
        b3World_Step( worldId, 1.0f / 60.0f, 4 );
    }
    float landedY = b3Body_GetPosition( falling ).y;
    report_stage( 5, "Completed 180 contact steps" );

    for ( int i = 180; i < 300; ++i )
    {
        b3World_Step( worldId, 1.0f / 60.0f, 4 );
    }
    float settledY = b3Body_GetPosition( falling ).y;
    report_stage( 6, "Completed 300 stability steps" );

    int gravityOk = isfinite( earlyY ) && earlyY > 2.0f && earlyY < 4.0f;
    int contactOk = isfinite( landedY ) && landedY > 0.30f && landedY < 0.80f;
    int stabilityOk = isfinite( settledY ) && settledY > 0.30f && settledY < 0.80f &&
                      fabsf( settledY - landedY ) < 0.10f;

    printf( "BOX3D_RESULT,early_y=%.4f,landed_y=%.4f,settled_y=%.4f\n",
            earlyY, landedY, settledY );
#if defined( BOX3D_PS2 )
    // PS2SDK's scr_printf may not implement float formatting; preformat with
    // newlib and pass only a literal "%s" to the GS debug renderer.
    char resultLine[160];
    snprintf( resultLine, sizeof( resultLine ),
              "Y: early %.4f, landed %.4f, settled %.4f", earlyY, landedY, settledY );
    scr_printf( "%s\n", resultLine );
#endif

    b3DestroyWorld( worldId );
    int cleanupOk = b3GetWorldCount() == initialWorlds && b3GetByteCount() == initialBytes;
    report_stage( 7, "World destroyed, allocations checked" );

    if ( !gravityOk ) puts( "BOX3D_FAIL,gravity" );
    if ( !contactOk ) puts( "BOX3D_FAIL,contact" );
    if ( !stabilityOk ) puts( "BOX3D_FAIL,stability" );
    if ( !cleanupOk ) puts( "BOX3D_FAIL,cleanup" );
    int ok = gravityOk && contactOk && stabilityOk && cleanupOk;
    puts( ok ? "BOX3D_PASS" : "BOX3D_FAIL" );
#if defined( BOX3D_PS2 )
    scr_setfontcolor( ok ? 0x77ff77u : 0xff7777u );
    scr_printf( "%s: gravity=%d contact=%d stability=%d cleanup=%d\n",
                ok ? "PASS" : "FAIL", gravityOk, contactOk, stabilityOk, cleanupOk );
    scr_setfontcolor( 0x98928fu );
    scr_printf( "Result held. Reset PS2 to exit.\n" );
    // Keep final error/success visible even if the loader erases the screen.
    for ( ;; )
    {
        SleepThread();
    }
#endif
    return ok ? 0 : 1;
}
