// SPDX-License-Identifier: MIT
// EE physical map, verified against PCSX2 pcsx2/Memory.cpp and
// glampert/quake2-ps2 src/ps2/debug/pipeline_dump.cpp.
// PS2SDK vif_registers.h has VU1_MEM1_START and
// VU1_MICROMEM1_START swapped relative to these actual mappings.
// Do not substitute those SDK macro names without checking the map.
#pragma once
#define B3_VU1_EE_MICRO_BASE 0x11008000u
#define B3_VU1_EE_DATA_BASE  0x1100C000u
