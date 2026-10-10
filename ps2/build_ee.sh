#!/bin/sh
# SPDX-License-Identifier: MIT
# Build the real R5900 PS2SDK EE ELF and verify that physics symbols linked.
# PS2 libc arena is the reference for compiler flags, GS and linker libraries.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
PS2DEV=${PS2DEV:-/usr/local/ps2dev}
PS2SDK=${PS2SDK:-$PS2DEV/ps2sdk}
CC=${CC:-mips64r5900el-ps2-elf-gcc}
NM=${NM:-mips64r5900el-ps2-elf-nm}
READELF=${READELF:-mips64r5900el-ps2-elf-readelf}
CMAKE=${CMAKE:-cmake}
OUT=${OUT:-$ROOT/ps2/box3d_ps2_smoke.elf}
BUILD=${BUILD:-$ROOT/build/ps2-ee}

die() { printf 'PS2 BUILD ERROR: %s\n' "$*" >&2; exit 1; }

for tool in "$CC" "$NM" "$READELF" "$CMAKE" ninja; do
    command -v "$tool" >/dev/null 2>&1 || die "Missing executable: $tool"
done
case "$("$CC" -dumpmachine)" in
    *r5900*) ;;
    *) die "Refusing non-R5900 compiler: $("$CC" -dumpmachine)" ;;
esac
[ -f "$PS2SDK/ee/include/debug.h" ] || die "Missing PS2SDK debug.h: $PS2SDK"
[ -f "$PS2SDK/ee/lib/libdebug.a" ] || die "Missing PS2SDK libdebug.a: $PS2SDK"
[ -f "$PS2SDK/ee/lib/libkernel.a" ] || die "Missing PS2SDK libkernel.a: $PS2SDK"

# The native PS2SDK executable, not a PS2 Linux userspace program, can read
# privileged CP0 Count. Avoid reusing any host build directory or archive.
"$CMAKE" -S "$ROOT" -B "$BUILD" -G Ninja \
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

printf 'PS2 BUILD OK: %s\n' "$OUT"
printf 'Target: %s\n' "$("$CC" -dumpmachine)"
printf 'Run this ELF in PCSX2 or on PS2 and record BOX3D_STAGE / PASS / FAIL.\n'
