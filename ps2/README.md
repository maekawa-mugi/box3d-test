# Experimental PlayStation 2 EE port

This branch implements a **single-EE-worker scalar baseline**. MMI and VU0
acceleration are **not implemented**. It uses PS2SDK/newlib `memalign` for
32-byte allocations and simple operations in place of unsupported 64-bit atomics.

**Not thread-safe:** pseudo-atomics, mutexes, and semaphores on EE do not
synchronize with other EE threads or interrupts. World creation and
`b3World_SetWorkerCount()` force the serial path. Never call Box3D concurrently.

## 1. Build the EE static library (PS2DEV required)

Export `PS2DEV` or add its EE compiler to your `PATH`:

```sh
cmake -S . -B build/ps2-ee -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/ps2-ee.cmake \
  -DBOX3D_PS2=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/ps2-ee --target box3d
```

Expected result: `build/ps2-ee/src/libbox3d.a`. Link with `-lm`.

## 2. Link the PS2SDK EE smoke ELF

Export `PS2SDK` and run `make -C ps2` from the repository root.
This uses PS2SDK EE startup/link rules, `ps2/smoke.c`, the prebuilt static
library, and `-lm`. It should produce `ps2/box3d_ps2_smoke.elf`.
For a custom path, use `make -C ps2 BOX3D_PS2_LIB=/path/to/libbox3d.a`.

Run the ELF with a preferred emulator or loader. The smoke program checks
gravity after 15 steps, ground contact at 180, contact stability at 300 and
that world/memory accounting returns to the initial state on teardown.

**PS2 cross-compilation, ELF linking and emulator/hardware execution are
not yet verified.** The PS2SDK sample Makefile rules may require adjustment
for the installed PS2SDK release.

## 3. Run scalar regressions on the host

```sh
cmake -S . -B build/ps2-host-smoke -G Ninja \
  -DBOX3D_DISABLE_SIMD=ON -DBOX3D_AVX2=OFF \
  -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF \
  -DBOX3D_PS2_SMOKE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/ps2-host-smoke --target box3d_ps2_smoke box3d_ps2_count_wrap
./build/ps2-host-smoke/bin/box3d_ps2_smoke
./build/ps2-host-smoke/bin/box3d_ps2_count_wrap
```

Host CI runs these checks in Debug and Release, including on draft PRs.

## EE-specific limitations (see attached EE Core user manual)

- **FPU:** COP1 supports only single precision, does not support IEEE-style
  NaNs/infinities/subnormal values, and has different rounding behavior.
  Host scalar success is not proof of EE numerical equivalence or determinism.
  Avoid NaN/Inf sentinel values, and compare numeric results on real EE.
- **Count clock:** CP0 Count is 32-bit. `src/ps2_count.h` extends the
  counter through one rollover using unsigned differences. Extra full wraps
  between polls cannot be recovered. The provisional default
  `BOX3D_PS2_COUNT_HZ=147456000` gives ~29 seconds per wrap and needs hardware
  calibration.
- **Scheduler:** sleep/yield and EE worker creation are unsupported.
- **Memory:** profile peak allocations and frame times on the real EE.
- **Acceleration:** profile scalar narrowphase/contact solving before
  implementing VU0 macro mode. MMI targets appropriate integer operations.

The supplied hardware manual is a reference, not redistributed in the repo.
