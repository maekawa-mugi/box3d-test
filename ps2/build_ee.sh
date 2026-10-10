#!/bin/sh
# SPDX-License-Identifier: MIT
# Unified native GS Box3D PS2 demo: 640x480p, PCSS, D-pad pages, stress.
# Intentionally creates ONE ELF. The old *.elf variants are archived
# source experiments and are no longer built.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
PS2DEV=${PS2DEV:-/usr/local/ps2dev}
PS2SDK=${PS2SDK:-$PS2DEV/ps2sdk}
PATH="$PS2DEV/ee/bin:$PATH"
export PATH PS2DEV PS2SDK
CC=${CC:-mips64r5900el-ps2-elf-gcc}
NM=${NM:-mips64r5900el-ps2-elf-nm}
READELF=${READELF:-mips64r5900el-ps2-elf-readelf}
DVP=${DVP:-dvp-as}
CMAKE=${CMAKE:-cmake}
OUT=${OUT:-$ROOT/ps2/box3d_ps2.elf}
BUILD=${BUILD:-$ROOT/build/ps2-ee}
die() { printf 'PS2 BUILD ERROR: %s\n' "$*" >&2; exit 1; }
for tool in "$CC" "$NM" "$READELF" "$DVP" "$CMAKE"; do
    command -v "$tool" >/dev/null 2>&1 || die "Missing executable: $tool"
done
case "$("$CC" -dumpmachine)" in
    *r5900*) ;;
    *) die "Compiler is not targeting PS2 R5900" ;;
esac
[ -f "$PS2SDK/ee/lib/libpad.a" ] || die "Missing PS2SDK libpad.a"
[ -f "$PS2SDK/ee/lib/libkernel.a" ] || die "Missing PS2SDK libkernel.a"
[ -f "$PS2SDK/ee/lib/libpacket2.a" ] || die "Missing PS2SDK libpacket2.a"
sh "$ROOT/ps2/check_gs_preprocessor.sh"
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
    *) die "Unsupported CMAKE_GENERATOR: $GENERATOR" ;;
esac
[ -n "$BUILD_PROGRAM" ] || die "Missing $GENERATOR build program"
"$CMAKE" -S "$ROOT" -B "$BUILD" -G "$GENERATOR" \
    -DCMAKE_MAKE_PROGRAM="$BUILD_PROGRAM" \
    -DCMAKE_TOOLCHAIN_FILE="$ROOT/cmake/toolchains/ps2-ee.cmake" \
    -DBOX3D_PS2=ON -DCMAKE_BUILD_TYPE=Release
"$CMAKE" --build "$BUILD" --target box3d
LIB="$BUILD/src/libbox3d.a"
[ -s "$LIB" ] || die "Missing R5900 Box3D archive: $LIB"

# Native VU1 microcode is assembled by DVP, not EE GCC.
"$DVP" "$ROOT/ps2/vu1_boxes.vsm" -o "$BUILD/vu1_boxes_micro.o"
[ -s "$BUILD/vu1_boxes_micro.o" ] || die "VU1 microcode object was not produced"
# VU0 micro mode is a separate 4KB resident program with one 64-lane
# contact-normal batch per VCALLMS. It is NOT executed automatically.
"$DVP" "$ROOT/ps2/vu0_micro_solver.vsm" -o "$BUILD/vu0_contact_micro.o"
[ -s "$BUILD/vu0_contact_micro.o" ] || die "VU0 microcode object was not produced"
"$DVP" "$ROOT/ps2/vu0_sweep.vsm" -o "$BUILD/vu0_sweep_micro.o"
[ -s "$BUILD/vu0_sweep_micro.o" ] || die "VU0 sweep microcode object was not produced"

"$CC" -O2 -std=gnu17 -march=r5900 -G0 -fno-lto -fno-delayed-branch \
    -D_EE -DBOX3D_PS2=1 -DB3_ENABLE_ASSERT=1 -DBOX3D_PS2_UNIFIED=1 \
    -I"$ROOT/include" -I"$PS2SDK/ee/include" -I"$PS2SDK/common/include" \
    -I"$PS2SDK/ee/graph/include" -I"$PS2SDK/ee/dma/include" \
    -I"$PS2SDK/ee/rpc/pad/include" \
    "$ROOT/ps2/sample_native.c" "$ROOT/ps2/gs_native.c" \
    "$ROOT/ps2/vu1_boxes.c" "$BUILD/vu1_boxes_micro.o" \
    "$ROOT/ps2/vu0_micro_solver.c" "$ROOT/ps2/vu0_sweep_runtime.c" "$BUILD/vu0_sweep_micro.o" "$BUILD/vu0_contact_micro.o" \
    "$ROOT/ps2/shadow_project.c" "$ROOT/shared/human.c" \
    "$ROOT/shared/utils.c" "$LIB" \
    -L"$PS2SDK/ee/lib" -lpacket2 -lgraph -ldma -lpad -lm -lkernel \
    -o "$OUT"

"$READELF" -h "$OUT" | grep -q 'Machine:.*MIPS' ||
    die "Output is not a MIPS executable"
"$READELF" -h "$OUT" | grep -q 'Type:.*EXEC' ||
    die "Output is not an executable ELF"
for symbol in main b3World_Step b3GsPresent b3GsShadowSetPenumbra b3ShadowProjectBox CreateHuman padRead b3Vu1BoxesInit b3Vu1BoxesSubmit; do
    "$NM" --defined-only "$OUT" | grep -Eq " [Tt] ${symbol}$" ||
        die "Unified ELF missing symbol: $symbol"
done
for micro_symbol in B3Vu1Boxes_CodeStart B3Vu1Boxes_RunStart B3Vu0Contacts_CodeStart B3Vu0Contacts_CodeEnd B3Vu0Sweep_CodeStart B3Vu0Sweep_CodeEnd; do
    "$NM" --defined-only "$OUT" | grep -Eq " [TtDd] ${micro_symbol}$" ||
        die "Unified ELF missing VU1 microprogram symbol: $micro_symbol"
done
printf 'BOX3D PS2 UNIFIED OK: %s\n' "$OUT"
printf 'VIDEO: 640x480p | SHADOW: PCSS | GEOMETRY: VU1 microcode + VIF1 DMA (stress, toggle TRIANGLE)\n'
printf 'Stale box3d_ps2_samples_*.elf files may exist from older builds; do not load them.\n'
