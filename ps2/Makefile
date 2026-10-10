# Build the PS2SDK EE smoke ELF after configuring and building libbox3d.a.
EE_BIN = box3d_ps2_smoke.elf
EE_OBJS = smoke.o
BOX3D_PS2_LIB ?= ../build/ps2-ee/src/libbox3d.a

EE_INCS += -I../include
EE_CFLAGS += -std=gnu17 -DBOX3D_PS2=1
# Static libraries are order-sensitive. Keep libbox3d before libm.
EE_LIBS += $(BOX3D_PS2_LIB) -lm
all: $(EE_BIN)

include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.eeglobal
