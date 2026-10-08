# Experimental PlayStation 2 EE port

This branch is an initial **single-EE-worker scalar baseline**, not an MMI or
VU0-optimized implementation. It keeps the original Box3D code paths and
public API, makes aligned allocations work with ps2sdk/newlib, and avoids
unsupported 64-bit GCC atomic library calls by using ordinary operations.
**The replacement atomics are NOT thread-safe**. World creation and
`b3World_SetWorkerCount()` force one worker. Do not call the library concurrently
from other EE threads, interrupt handlers, or custom task systems.

## Configure / build (ps2dev installed)

With `$PS2DEV/ee/bin` on `PATH` (or `$PS2DEV` exported):

```sh
cmake -S . -B build/ps2-ee -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/ps2-ee.cmake \
  -DBOX3D_PS2=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/ps2-ee --target box3d
```

The first goal is a static `libbox3d.a`. Link it into an ordinary PS2SDK EE
application via your project's EE Makefile. Header-only usage must also define
`BOX3D_PS2=1`, or inherit that definition from the CMake target.

## Reproducible headless test (host)

```sh
cmake -S . -B build/ps2-host-smoke -G Ninja \
  -DBOX3D_DISABLE_SIMD=ON -DBOX3D_AVX2=OFF \
  -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF \
  -DBOX3D_PS2_SMOKE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/ps2-host-smoke --target box3d_ps2_smoke
./build/ps2-host-smoke/bin/box3d_ps2_smoke
```

The same `ps2/smoke.c` can be linked as an EE application after the static
library builds. CMake executable linking on PS2 may require PS2SDK startup
and linker script configuration and is *not* part of the baseline claim.

## Limitations and next measurements

- EE-specific code has **not** been cross-compiled or run on hardware yet.
- PS2 newlib C17 support, `memalign`, atomics, and linker setup need verification
  on the installed ps2dev toolchain.
- CP0 Count timing is approximate and assumes a nominal counter frequency of
  147456000 Hz (`BOX3D_PS2_COUNT_HZ` override). The 32-bit wrap is tracked
  only while polling at least once every ~29 seconds; calibrate on real hardware.
- `b3Sleep` and `b3Yield` are no-ops; worker thread creation is unsupported.
- Scalar contact solving provides correctness/performance baseline; next up is
  isolated VU0 macro-mode work on the four-wide narrowphase/solver, after
  profiling and comparing identical simulations. MMI is for integer/filter
  kernels, not floating-point arithmetic.
