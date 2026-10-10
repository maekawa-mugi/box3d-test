# SPDX-License-Identifier: MIT
# Alternative PS2SDK EE build once the CMake static archive exists.
# Preferred one-command path: sh ps2/build_ee.sh
PS2DEV ?= /usr/local/ps2dev
PS2SDK ?= $(PS2DEV)/ps2sdk
EE_BIN = box3d_ps2_smoke.elf
EE_OBJS = smoke.o
BOX3D_PS2_LIB ?= ../build/ps2-ee/src/libbox3d.a
EE_INCS = -I../include
EE_CFLAGS = -O2 -std=gnu17 -march=r5900 -G0 -fno-lto -fno-delayed-branch -D_EE -DBOX3D_PS2=1
EE_LIBS = $(BOX3D_PS2_LIB) -lm -ldebug -lkernel

.PHONY: all
all: $(EE_BIN)

include $(PS2SDK)/Defs.make
include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.eeglobal

# Re-link when the archive changes, even when smoke.o remains unchanged.
$(EE_BIN): $(BOX3D_PS2_LIB)
