#!/bin/sh
# SPDX-License-Identifier: MIT
# Cheap local preprocessor gate for all GS visual ELF variants.
# This runs on the HOST; no PCSX2, PS2SDK or CI is required.
# Includes are removed from a temporary stdin stream ONLY because the
# actual PS2SDK headers are target-specific. All #if/#else/#endif
# directives from the real ps2/gs_native.c are preserved verbatim.
set -eu
ROOT=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
HOST_CPP=${HOST_CPP:-cpp}

if ! command -v "$HOST_CPP" >/dev/null 2>&1; then
    printf 'PS2 GS preprocessor: skipped (host cpp unavailable)\n'
    exit 0
fi

check()
{
    label=$1
    shift
    # cpp validates branch nesting even with all target includes stripped.
    sed '/^[[:space:]]*#[[:space:]]*include[[:space:]]/d' \
        "$ROOT/ps2/gs_native.c" |
        "$HOST_CPP" -P -x c "$@" - >/dev/null ||
        {
            printf 'PS2 GS preprocessor FAILED: %s\n' "$label" >&2
            exit 1
        }
    printf 'PS2 GS preprocessor PASS: %s\n' "$label"
}

check wireframe
check solid -DBOX3D_GS_SOLID=1
check official -DBOX3D_GS_SOLID=1 -DBOX3D_GS_OFFICIAL=1
check receiver -DBOX3D_GS_SOLID=1 -DBOX3D_GS_OFFICIAL=1 \
    -DBOX3D_GS_RECEIVER_SHADOWS=1
check kawase -DBOX3D_GS_SOLID=1 -DBOX3D_GS_OFFICIAL=1 \
    -DBOX3D_GS_RECEIVER_SHADOWS=1 -DBOX3D_GS_KAWASE_SHADOWS=1
check kawase-debug -DBOX3D_GS_SOLID=1 -DBOX3D_GS_OFFICIAL=1 \
    -DBOX3D_GS_RECEIVER_SHADOWS=1 -DBOX3D_GS_KAWASE_SHADOWS=1 \
    -DBOX3D_GS_KAWASE_DEBUG=1
