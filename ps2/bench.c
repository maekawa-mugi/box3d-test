// SPDX-License-Identifier: MIT
// Real EE timing and numerical-equivalence harness, scalar vs R5900 MMI.
// Reports raw Count ticks; does not interpret PCSX2 speed as console silicon.
#include "box3d/box3d.h"
#include "../src/ps2_mmi.h"
#include "ee_clock.h"

#include <debug.h>
#include <kernel.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BENCH_MAX_BODIES 32
#define BENCH_STEPS 120
#define BENCH_REPEATS 4
#define BENCH_MICRO_ITERATIONS 128

typedef struct BenchResult
{
    uint64_t ticks;
    float positionSum;
    float velocitySum;
    float firstY;
    uint32_t slowSamples;
    int cleanupOk;
} BenchResult;

static void screen_line( const char* line )
{
    puts( line );
    scr_printf( "%s\n", line );
}

static void screen_stop( const char* why )
{
    scr_setfontcolor( 0xff7777u );
    scr_printf( "FAIL: %s\n", why );
    printf( "BOX3D_BENCH_FAIL,%s\n", why );
    screen_line( "Result held. Reset PS2." );
    for ( ;; ) SleepThread();
}

static int bench_assert( const char* expr, const char* file, int line )
{
    char text[160];
    snprintf( text, sizeof( text ), "ASSERT %s @ %s:%d", expr, file, line );
    screen_stop( text );
    return 0;
}

static void bench_log( const char* message )
{
    printf( "BOX3D_LOG,%s\n", message );
}

// Verify byte-for-byte for 16-byte aligned and 8-byte misaligned regions,
// zero-sized operations, short tails and blocks around the MMI threshold.
static void test_mmi_kernels( void )
{
    static uint64_t storageA[96] __attribute__((aligned(16)));
    static uint64_t storageB[96] __attribute__((aligned(16)));
    static uint64_t reference[96] __attribute__((aligned(16)));
    static const unsigned sizes[] = { 0, 1, 2, 3, 4, 7, 8, 16, 17, 31, 32, 48, 64 };
    for ( unsigned mode = 0; mode < 2; ++mode )
    {
        b3Ps2SetMMIEnabled( (int)mode );
        for ( unsigned offset = 0; offset < 2; ++offset )
        {
            for ( unsigned n = 0; n < sizeof(sizes)/sizeof(sizes[0]); ++n )
            {
                const size_t words = sizes[n];
                for ( unsigned j = 0; j < 96; ++j )
                {
                    storageA[j] = UINT64_C(0x55aabbccdd001122) ^ (uint64_t)j;
                    storageB[j] = UINT64_C(0xf0f0f0f00f0f0f0f) ^ ((uint64_t)j << 10);
                    reference[j] = storageA[j];
                }
                uint64_t* a = storageA + offset;
                const uint64_t* b = storageB + offset;
                uint64_t* r = reference + offset;
                for ( size_t j = 0; j < words; ++j ) r[j] |= b[j];
                b3Ps2OrWords( a, b, words );
                if ( memcmp( storageA, reference, sizeof( storageA ) ) != 0 )
                {
                    screen_stop( "MMI OR differs from scalar" );
                }

                for ( unsigned j = 0; j < 96; ++j ) storageA[j] = reference[j] = UINT64_MAX;
                memset( r, 0, words * sizeof( uint64_t ) );
                b3Ps2ClearBytes( a, words * sizeof( uint64_t ) );
                if ( memcmp( storageA, reference, sizeof( storageA ) ) != 0 )
                {
                    screen_stop( "MMI CLEAR differs from scalar" );
                }
            }
        }
    }
    b3Ps2SetMMIEnabled( 0 );
    screen_line( "MMI kernels: scalar == MMI (aligned + tails)" );
    puts( "BOX3D_MMI_CHECK,PASS" );
}

static uint64_t micro_sample( int mode, int useOr )
{
    static uint64_t dst[256] __attribute__((aligned(16)));
    static uint64_t src[256] __attribute__((aligned(16)));
    uint64_t elapsed = 0;
    b3Ps2SetMMIEnabled( mode );
    for ( unsigned i = 0; i < 256; ++i )
    {
        dst[i] = UINT64_C(0x3355771199aabbdd) + i;
        src[i] = UINT64_C(0xff00ff00ff00ff00) ^ i;
    }
    for ( unsigned i = 0; i < BENCH_MICRO_ITERATIONS; ++i )
    {
        uint32_t start = b3Ps2ReadCpuCycles();
        if ( useOr ) b3Ps2OrWords( dst, src, 256 );
        else b3Ps2ClearBytes( dst, sizeof(dst) );
        uint32_t delta = b3Ps2DeltaCpuCycles( start, b3Ps2ReadCpuCycles() );
        if ( delta >= UINT32_C(0x80000000) ) screen_stop( "micro timer span >= 2^31" );
        elapsed += delta;
    }
    // Keep side effects visible in the output and independent of the clock.
    printf( "BOX3D_MICRO,%s,mode=%d,ticks=%llu,check=%llx\n",
            useOr ? "or" : "clear", mode,
            (unsigned long long)elapsed, (unsigned long long)dst[0] );
    return elapsed;
}

static BenchResult run_physics( int mode, int side, int layers )
{
    BenchResult result = { 0 };
    const int bodies = side * side * layers;
    b3BodyId ids[BENCH_MAX_BODIES];
    if ( bodies > BENCH_MAX_BODIES ) screen_stop( "too many bodies" );

    b3Ps2SetMMIEnabled( mode );
    const int64_t beforeBytes = b3GetByteCount();
    const int beforeWorlds = b3GetWorldCount();

    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.workerCount = 1;
    worldDef.enableSleep = false;
    b3WorldId world = b3CreateWorld( &worldDef );

    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = (b3Vec3){ 0.0f, -1.0f, 0.0f };
    b3BodyId ground = b3CreateBody( world, &groundDef );
    b3BoxHull groundHull = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3CreateHullShape( ground, &shapeDef, &groundHull.base );

    b3BoxHull cube = b3MakeCubeHull( 0.5f );
    int index = 0;
    for ( int layer = 0; layer < layers; ++layer )
    {
        for ( int z = 0; z < side; ++z )
        {
            for ( int x = 0; x < side; ++x )
            {
                b3BodyDef def = b3DefaultBodyDef();
                def.type = b3_dynamicBody;
                def.position = (b3Vec3){
                    1.06f * (float)x - 0.53f * (float)(side-1),
                    1.1f + 1.06f * (float)layer,
                    1.06f * (float)z - 0.53f * (float)(side-1)
                };
                ids[index] = b3CreateBody( world, &def );
                b3ShapeDef cubeShape = b3DefaultShapeDef();
                cubeShape.density = 1.0f;
                cubeShape.friction = 0.6f;
                b3CreateHullShape( ids[index], &cubeShape, &cube.base );
                ++index;
            }
        }
    }

    for ( int step = 0; step < BENCH_STEPS; ++step )
    {
        uint32_t start = b3Ps2ReadCpuCycles();
        b3World_Step( world, 1.0f / 60.0f, 4 );
        uint32_t delta = b3Ps2DeltaCpuCycles( start, b3Ps2ReadCpuCycles() );
        if ( delta >= UINT32_C(0x80000000) )
        {
            result.slowSamples += 1;
        }
        else
        {
            result.ticks += delta;
        }
    }

    result.firstY = b3Body_GetPosition( ids[0] ).y;
    for ( int i = 0; i < bodies; ++i )
    {
        b3Vec3 p = b3Body_GetPosition( ids[i] );
        b3Vec3 v = b3Body_GetLinearVelocity( ids[i] );
        if ( !isfinite( p.x ) || !isfinite( p.y ) || !isfinite( p.z ) ||
             !isfinite( v.x ) || !isfinite( v.y ) || !isfinite( v.z ) )
        {
            screen_stop( "non-finite body state" );
        }
        result.positionSum += p.x + p.y + p.z;
        result.velocitySum += v.x + v.y + v.z;
    }

    b3DestroyWorld( world );
    result.cleanupOk = beforeBytes == b3GetByteCount() && beforeWorlds == b3GetWorldCount();
    if ( !result.cleanupOk ) screen_stop( "world memory leak" );
    if ( result.slowSamples ) screen_stop( "step time >= 2^31 Count ticks" );
    return result;
}

static void scene_benchmark( const char* label, int side, int layers )
{
    // ABBA order cancels some first-run / time drift; report all raw samples.
    const int modes[BENCH_REPEATS] = { 0, 1, 1, 0 };
    BenchResult runs[BENCH_REPEATS];
    uint64_t scalarTicks = 0, mmiTicks = 0;
    for ( int repeat = 0; repeat < BENCH_REPEATS; ++repeat )
    {
        const int mode = modes[repeat];
        runs[repeat] = run_physics( mode, side, layers );
        uint64_t mean = runs[repeat].ticks / BENCH_STEPS;
        printf( "BOX3D_SCENE,%s,repeat=%d,mmi=%d,total_ticks=%llu,ticks_per_step=%llu,y=%.6f,pos_sum=%.6f,vel_sum=%.6f\n",
                label, repeat, mode, (unsigned long long)runs[repeat].ticks,
                (unsigned long long)mean, runs[repeat].firstY,
                runs[repeat].positionSum, runs[repeat].velocitySum );
        if ( mode ) mmiTicks += runs[repeat].ticks;
        else scalarTicks += runs[repeat].ticks;
    }

    for ( int repeat = 0; repeat < BENCH_REPEATS; ++repeat )
    {
        if ( fabsf( runs[0].firstY - runs[repeat].firstY ) > 0.001f ||
             fabsf( runs[0].positionSum - runs[repeat].positionSum ) > 0.001f ||
             fabsf( runs[0].velocitySum - runs[repeat].velocitySum ) > 0.001f )
        {
            screen_stop( "MMI changes physics result" );
        }
    }

    char display[160];
    double ratio = mmiTicks ? (double)scalarTicks / (double)mmiTicks : 0.0;
    snprintf( display, sizeof(display), "%s (%d bodies): scalar %llu, MMI %llu cyc/step (x%.3f)",
              label, side*side*layers,
              (unsigned long long)(scalarTicks / (2*BENCH_STEPS)),
              (unsigned long long)(mmiTicks / (2*BENCH_STEPS)), ratio );
    screen_line( display );
    printf( "BOX3D_SCENE_SUMMARY,%s,scalar_ticks=%llu,mmi_ticks=%llu,speedup=%.5f,correct=1\n",
            label,(unsigned long long)scalarTicks,(unsigned long long)mmiTicks,ratio );
}

int main( void )
{
    setvbuf( stdout, NULL, _IONBF, 0 );
    init_scr();
    scr_clear();
    b3SetAssertFcn( bench_assert );
    b3SetLogFcn( bench_log );

    screen_line( "Box3D PS2 EE MMI A/B correctness + cycles" );
    screen_line( "CP0 Count 294.912MHz nominal; PCSX2 != silicon" );
    test_mmi_kernels();

    uint64_t clearS = micro_sample( 0, 0 );
    uint64_t clearM = micro_sample( 1, 0 );
    uint64_t orS = micro_sample( 0, 1 );
    uint64_t orM = micro_sample( 1, 1 );
    char display[160];
    snprintf( display, sizeof(display), "2KB ops: clear %llu / %llu, OR %llu / %llu ticks",
              (unsigned long long)(clearS / BENCH_MICRO_ITERATIONS),
              (unsigned long long)(clearM / BENCH_MICRO_ITERATIONS),
              (unsigned long long)(orS / BENCH_MICRO_ITERATIONS),
              (unsigned long long)(orM / BENCH_MICRO_ITERATIONS) );
    screen_line( display );

    scene_benchmark( "eight-box", 2, 2 );
    scene_benchmark( "32-box", 4, 2 );
    b3Ps2SetMMIEnabled( 0 );
    scr_setfontcolor( 0x77ff77u );
    screen_line( "PASS: MMI bitsets match, physics snapshots match" );
    puts( "BOX3D_BENCH_PASS" );
    screen_line( "Result held; reset PS2 to exit." );
    for ( ;; ) SleepThread();
    return 0;
}
