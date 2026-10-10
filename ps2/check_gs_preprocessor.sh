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

# Only the single supported GS 480p PCSS configuration.
# Include gs_native.h explicitly because the local stripped-include
# preprocessor gate must still see BOX3D_PS2_UNIFIED macro expansion.
# Do not regress to enumerating incompatible legacy combinations.
check unified-480p-pcss -DBOX3D_PS2_UNIFIED=1 \
    -include "$ROOT/ps2/gs_native.h"
