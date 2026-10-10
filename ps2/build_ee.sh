#!/bin/sh
# SPDX-License-Identifier: MIT
# Build the real R5900 PS2SDK EE ELF and verify that physics symbols linked.
# PS2 libc arena is the reference for compiler flags, GS and linker libraries.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
PS2DEV=${PS2DEV:-/usr/local/ps2dev}
PS2SDK=${PS2SDK:-$PS2DEV/ps2sdk}
# Make the default PS2DEV layout work even without a preconfigured PATH.
PATH="$PS2DEV/ee/bin:$PATH"
export PATH PS2DEV PS2SDK
CC=${CC:-mips64r5900el-ps2-elf-gcc}
NM=${NM:-mips64r5900el-ps2-elf-nm}
READELF=${READELF:-mips64r5900el-ps2-elf-readelf}
CMAKE=${CMAKE:-cmake}
OUT=${OUT:-$ROOT/ps2/box3d_ps2_smoke.elf}
BUILD=${BUILD:-$ROOT/build/ps2-ee}

die() { printf 'PS2 BUILD ERROR: %s\n' "$*" >&2; exit 1; }

for tool in "$CC" "$NM" "$READELF" "$CMAKE"; do
    command -v "$tool" >/dev/null 2>&1 || die "Missing executable: $tool"
done

# Prefer Ninja if present, otherwise use GNU Make. Allow overrides through
# CMAKE_GENERATOR, and pass the exact build-program path to CMake.
if [ -n "${CMAKE_GENERATOR:-}" ]; then
    GENERATOR=$CMAKE_GENERATOR
elif command -v ninja >/dev/null 2>&1; then
    GENERATOR=Ninja
else
    GENERATOR='Unix Makefiles'
fi
case "$GENERATOR" in
    Ninja) BUILD_PROGRAM=$(command -v ninja || true) ;;
    'Unix Makefiles') BUILD_PROGRAM=$(command -v make || true) ;;
    *) die "Unsupported CMAKE_GENERATOR '$GENERATOR' (choose Ninja or Unix Makefiles)" ;;
esac
[ -n "$BUILD_PROGRAM" ] || die "No build program for $GENERATOR; install ninja or make"
echo "PS2 build generator: $GENERATOR ($BUILD_PROGRAM)"
case "$("$CC" -dumpmachine)" in
    *r5900*) ;;
    *) die "Refusing non-R5900 compiler: $("$CC" -dumpmachine)" ;;
esac
[ -f "$PS2SDK/ee/include/debug.h" ] || die "Missing PS2SDK debug.h: $PS2SDK"
[ -f "$PS2SDK/ee/lib/libdebug.a" ] || die "Missing PS2SDK libdebug.a: $PS2SDK"
[ -f "$PS2SDK/ee/lib/libkernel.a" ] || die "Missing PS2SDK libkernel.a: $PS2SDK"

# The native PS2SDK executable, not a PS2 Linux userspace program, can read
# privileged CP0 Count. Avoid reusing any host build directory or archive.
"$CMAKE" -S "$ROOT" -B "$BUILD" -G "$GENERATOR" \
    -DCMAKE_MAKE_PROGRAM="$BUILD_PROGRAM" \
    -DCMAKE_TOOLCHAIN_FILE="$ROOT/cmake/toolchains/ps2-ee.cmake" \
    -DBOX3D_PS2=ON -DCMAKE_BUILD_TYPE=Release
"$CMAKE" --build "$BUILD" --target box3d

LIB="$BUILD/src/libbox3d.a"
[ -s "$LIB" ] || die "Box3D EE archive is absent: $LIB"

# Known working standalone EE pattern from ps2-libc-arena; link after objects.
# CMake supplies the same -march/-G0/-fno-delayed-branch baseline to Box3D.
"$CC" -O2 -std=gnu17 -march=r5900 -G0 -fno-lto -fno-delayed-branch \
    -D_EE -DBOX3D_PS2=1 -DB3_ENABLE_ASSERT=1 \
    -I"$ROOT/include" -I"$PS2SDK/ee/include" -I"$PS2SDK/common/include" \
    "$ROOT/ps2/smoke.c" "$LIB" \
    -L"$PS2SDK/ee/lib" -lm -ldebug -lkernel -o "$OUT"

# Prevent a host executable, bare C probe, or missing physics step symbol from
# being mistaken for a PS2 physics ELF. These checks are necessary, not proof
# of an emulator or physical-PS2 run.
"$READELF" -h "$OUT" | grep -q 'Machine:.*MIPS' ||
    die "Output is not a MIPS ELF"
"$READELF" -h "$OUT" | grep -q 'Type:.*EXEC' ||
    die "Output is not an executable ELF"
"$NM" --defined-only "$OUT" | grep -Eq ' [Tt] b3World_Step$' ||
    die "Physics step implementation missing from linked ELF"
"$NM" --defined-only "$OUT" | grep -Eq ' [Tt] main$' ||
    die "PS2 smoke main() missing from linked ELF"

# Build separate physics + MMI correctness/timing ELF. Smoke stays standalone.
BENCH_OUT=${BENCH_OUT:-$ROOT/ps2/box3d_ps2_bench.elf}
"$CC" -O2 -std=gnu17 -march=r5900 -G0 -fno-lto -fno-delayed-branch \
    -D_EE -DBOX3D_PS2=1 -DB3_ENABLE_ASSERT=1 \
    -I"$ROOT/include" -I"$PS2SDK/ee/include" -I"$PS2SDK/common/include" \
    "$ROOT/ps2/bench.c" "$LIB" \
    -L"$PS2SDK/ee/lib" -lm -ldebug -lkernel -o "$BENCH_OUT"
"$READELF" -h "$BENCH_OUT" | grep -q 'Machine:.*MIPS' ||
    die "Benchmark is not a MIPS ELF"
"$READELF" -h "$BENCH_OUT" | grep -q 'Type:.*EXEC' ||
    die "Benchmark is not an executable ELF"
for symbol in main b3World_Step b3Ps2ClearBytes b3Ps2OrWords b3Ps2SetMMIEnabled b3Ps2VU0Dot4 b3Ps2SetVU0Enabled; do
    "$NM" --defined-only "$BENCH_OUT" | grep -Eq " [Tt] ${symbol}$" ||
        die "Benchmark missing linked symbol: $symbol"
done

# Native GS 3D wireframe sample runner: draw directly through GIF A+D + DMA.
# graph is used only for CRT/display framebuffer, NOT for drawing; no gsKit.
SAMPLE_OUT=${SAMPLE_OUT:-$ROOT/ps2/box3d_ps2_samples.elf}
"$CC" -O2 -std=gnu17 -march=r5900 -G0 -fno-lto -fno-delayed-branch \
    -D_EE -DBOX3D_PS2=1 -DB3_ENABLE_ASSERT=1 \
    -I"$ROOT/include" -I"$PS2SDK/ee/include" -I"$PS2SDK/common/include" \
    -I"$PS2SDK/ee/graph/include" -I"$PS2SDK/ee/dma/include" \
    "$ROOT/ps2/sample_native.c" "$ROOT/ps2/gs_native.c" "$LIB" \
    -L"$PS2SDK/ee/lib" -lgraph -ldma -lm -lkernel -o "$SAMPLE_OUT"
"$READELF" -h "$SAMPLE_OUT" | grep -q 'Machine:.*MIPS' ||
    die "Native GS sample is not a MIPS ELF"
"$READELF" -h "$SAMPLE_OUT" | grep -q 'Type:.*EXEC' ||
    die "Native GS sample is not an executable ELF"
for symbol in main b3World_Step b3Body_GetPosition b3Body_GetRotation b3GsBegin b3GsLine3 b3GsPresent; do
    "$NM" --defined-only "$SAMPLE_OUT" | grep -Eq " [Tt] ${symbol}$" ||
        die "Native GS sample missing linked symbol: $symbol"
done

# Alternate renderer with native GS triangles and PSMZ32 hardware Z-test.
# The confirmed working wireframe ELF remains untouched as a build target.
SOLID_SAMPLE_OUT=${SOLID_SAMPLE_OUT:-$ROOT/ps2/box3d_ps2_samples_solid.elf}
"$CC" -O2 -std=gnu17 -march=r5900 -G0 -fno-lto -fno-delayed-branch \
    -D_EE -DBOX3D_PS2=1 -DB3_ENABLE_ASSERT=1 -DBOX3D_GS_SOLID=1 \
    -I"$ROOT/include" -I"$PS2SDK/ee/include" -I"$PS2SDK/common/include" \
    -I"$PS2SDK/ee/graph/include" -I"$PS2SDK/ee/dma/include" \
    "$ROOT/ps2/sample_native.c" "$ROOT/ps2/gs_native.c" "$LIB" \
    -L"$PS2SDK/ee/lib" -lgraph -ldma -lm -lkernel -o "$SOLID_SAMPLE_OUT"
"$READELF" -h "$SOLID_SAMPLE_OUT" | grep -q 'Machine:.*MIPS' ||
    die "Solid sample is not a MIPS ELF"
"$READELF" -h "$SOLID_SAMPLE_OUT" | grep -q 'Type:.*EXEC' ||
    die "Solid sample is not executable ELF"
for symbol in main b3World_Step b3Body_GetRotation b3GsTriangle3 b3GsLine3 b3GsPresent; do
    "$NM" --defined-only "$SOLID_SAMPLE_OUT" | grep -Eq " [Tt] ${symbol}$" ||
        die "Solid native GS sample missing symbol: $symbol"
done

# Additional visual comparison ELF: official-demo-inspired lighting and UI,
# still raw GS GIF A+D (no gsKit, no PS2SDK libdraw).
OFFICIAL_SAMPLE_OUT=${OFFICIAL_SAMPLE_OUT:-$ROOT/ps2/box3d_ps2_samples_official.elf}
"$CC" -O2 -std=gnu17 -march=r5900 -G0 -fno-lto -fno-delayed-branch \
    -D_EE -DBOX3D_PS2=1 -DB3_ENABLE_ASSERT=1 \
    -DBOX3D_GS_SOLID=1 -DBOX3D_GS_OFFICIAL=1 \
    -I"$ROOT/include" -I"$PS2SDK/ee/include" -I"$PS2SDK/common/include" \
    -I"$PS2SDK/ee/graph/include" -I"$PS2SDK/ee/dma/include" \
    "$ROOT/ps2/sample_native.c" "$ROOT/ps2/gs_native.c" "$LIB" \
    -L"$PS2SDK/ee/lib" -lgraph -ldma -lm -lkernel -o "$OFFICIAL_SAMPLE_OUT"
"$READELF" -h "$OFFICIAL_SAMPLE_OUT" | grep -q 'Machine:.*MIPS' ||
    die "Official-style sample is not a MIPS ELF"
"$READELF" -h "$OFFICIAL_SAMPLE_OUT" | grep -q 'Type:.*EXEC' ||
    die "Official-style sample is not an executable ELF"
for symbol in main b3World_Step b3GsSkyGradient b3GsTriangle3 b3GsShadowBegin b3GsOverlayText b3GsPresent; do
    "$NM" --defined-only "$OFFICIAL_SAMPLE_OUT" | grep -Eq " [Tt] ${symbol}$" ||
        die "Official-style GS sample missing symbol: $symbol"
done

# Explicit GS Z-tested clipped box silhouette / receiver shadow test ELF.
# The prior official-looking executable remains intact for direct A/B.
SHADOW_SAMPLE_OUT=${SHADOW_SAMPLE_OUT:-$ROOT/ps2/box3d_ps2_samples_shadows.elf}
"$CC" -O2 -std=gnu17 -march=r5900 -G0 -fno-lto -fno-delayed-branch \
    -D_EE -DBOX3D_PS2=1 -DB3_ENABLE_ASSERT=1 \
    -DBOX3D_GS_SOLID=1 -DBOX3D_GS_OFFICIAL=1 \
    -DBOX3D_GS_RECEIVER_SHADOWS=1 \
    -I"$ROOT/include" -I"$PS2SDK/ee/include" -I"$PS2SDK/common/include" \
    -I"$PS2SDK/ee/graph/include" -I"$PS2SDK/ee/dma/include" \
    "$ROOT/ps2/sample_native.c" "$ROOT/ps2/gs_native.c" \
    "$ROOT/ps2/shadow_project.c" "$LIB" \
    -L"$PS2SDK/ee/lib" -lgraph -ldma -lm -lkernel -o "$SHADOW_SAMPLE_OUT"
"$READELF" -h "$SHADOW_SAMPLE_OUT" | grep -q 'Machine:.*MIPS' ||
    die "Projected shadow sample is not a MIPS ELF"
"$READELF" -h "$SHADOW_SAMPLE_OUT" | grep -q 'Type:.*EXEC' ||
    die "Projected shadow sample is not an executable ELF"
for symbol in main b3World_Step b3GsTriangle3 b3GsShadowBegin b3ShadowProjectBox b3GsPresent; do
    "$NM" --defined-only "$SHADOW_SAMPLE_OUT" | grep -Eq " [Tt] ${symbol}$" ||
        die "Projected shadow sample missing: $symbol"
done

printf 'PS2 BUILD OK: %s\n' "$OUT"
printf 'PS2 BENCH BUILD OK: %s\n' "$BENCH_OUT"
printf 'PS2 NATIVE GS SAMPLES OK: %s\n' "$SAMPLE_OUT"
printf 'PS2 NATIVE GS SOLID SAMPLES OK: %s\n' "$SOLID_SAMPLE_OUT"
printf 'PS2 OFFICIAL-STYLE GS SAMPLES OK: %s\n' "$OFFICIAL_SAMPLE_OUT"
printf 'PS2 CLIPPED PROJECTED SHADOW SAMPLES OK: %s\n' "$SHADOW_SAMPLE_OUT"
printf 'Target: %s\n' "$("$CC" -dumpmachine)"
printf 'Run this ELF in PCSX2 or on PS2 and record BOX3D_STAGE / PASS / FAIL.\n'
