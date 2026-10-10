# Experimental PlayStation 2 EE port

This branch implements a **single-EE-worker scalar baseline** and an
optional **R5900 integer MMI bitset backend** and **VU0 COP2 macro-mode
four-lane floating-point dot products** in the wide contact solver. Other
physics math remains scalar. It uses PS2SDK/newlib `memalign` for
32-byte allocations and simple operations in place of unsupported 64-bit atomics.

**Not thread-safe:** pseudo-atomics, mutexes, and semaphores on EE do not
synchronize with other EE threads or interrupts. World creation and
`b3World_SetWorkerCount()` force the serial path. Never call Box3D concurrently.

## 1. Build the PS2 EE executable (PS2DEV + PS2SDK required)

The recommended entrypoint builds the R5900 scalar library and links the
physics-test ELF in one command. It uses the same conservative compiler flags,
PS2SDK debug screen and terminal-output strategy as
[ps2-libc-arena](https://github.com/maekawa-mugi/ps2-libc-arena).

```sh
sh ps2/build_ee.sh
# Expected: ps2/box3d_ps2_smoke.elf, ps2/box3d_ps2_bench.elf
#           and ps2/box3d_ps2_samples.elf
```

The shell build validates that the result is a MIPS executable with both the
physics step implementation and EE main() linked into it. This is not a
substitute for running the ELF in PCSX2 or on the actual console.

## 2. Build the EE static library manually (PS2DEV required)

Export `PS2DEV` or add its EE compiler to your `PATH`:

```sh
cmake -S . -B build/ps2-ee -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/ps2-ee.cmake \
  -DBOX3D_PS2=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/ps2-ee --target box3d
```

Expected result: `build/ps2-ee/src/libbox3d.a`. Link with `-lm`.

## 3. Alternative: link through PS2SDK EE Makefile

Export `PS2SDK` and run `make -C ps2` from the repository root.
This uses PS2SDK EE startup/link rules, `ps2/smoke.c`, the prebuilt static
library, and `-lm`. It should produce `ps2/box3d_ps2_smoke.elf`. The test initializes
the GS screen before world creation, emits stages 0 through 7 to the screen
and stdout, traps Box3D assertions into a persistent red error screen,
and holds the final PASS/FAIL result until console reset.
For a custom path, use `make -C ps2 BOX3D_PS2_LIB=/path/to/libbox3d.a`.

Run the ELF with a preferred emulator or loader. The smoke program checks
gravity after 15 steps, ground contact at 180, contact stability at 300 and
that world/memory accounting returns to the initial state on teardown.

**Scalar smoke has been cross-compiled and executed successfully on PCSX2 by
the tester.** The new MMI backend and benchmark have not yet been cross-built
or run here because this development environment does not have PS2DEV/PS2SDK.
The PS2SDK sample Makefile rules may require adjustment for the installed
PS2SDK release.

## 4. Run scalar regressions on the host

```sh
cmake -S . -B build/ps2-host-smoke -G Ninja \
  -DBOX3D_DISABLE_SIMD=ON -DBOX3D_AVX2=OFF \
  -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF \
  -DBOX3D_PS2_SMOKE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/ps2-host-smoke --target box3d_ps2_smoke box3d_ps2_count_wrap
./build/ps2-host-smoke/bin/box3d_ps2_smoke
./build/ps2-host-smoke/bin/box3d_ps2_count_wrap
```

The established scalar correctness run passed in PCSX2 (2026-10-10).
Host regression commands are optional; CI is not a release gate.

## MMI verification and PS2 timing benchmark

The new `ps2/box3d_ps2_bench.elf` runs two modes **within one binary**:
scalar/newlib bitset clear/OR versus R5900 128-bit integer MMI `SQ` and
`LQ/POR/SQ`. All Box3D float math remains the established scalar path.

1. On startup, test every byte of the zero and OR buffers for sizes from
   0 through 64 64-bit words, with aligned and offset-by-8-byte inputs.
   Both modes must agree exactly, including tails and guard elements.
2. Measure raw 2 KiB clear and OR times using read-only CP0 Count; report
   `BOX3D_MICRO` CSV-like records to stdout.
3. Run 120 physics steps for 8 and 32 dynamic boxes, across **ABBA**
   scalar/MMI trials with one EE worker and sleep disabled. Report each
   `BOX3D_SCENE` and aggregate `BOX3D_SCENE_SUMMARY` cycles per step,
   plus numerical equality within 0.001 for position and velocity
   checksums and first-body Y; stop with a persistent red screen if not.
4. Success emits `BOX3D_BENCH_PASS`. The program keeps the GS result
   visible instead of exiting into the loader.

**Important:** MMI performs integer operations only here. It does not
vectorize the floating-point contact solver. `b3InPlaceUnion` is mostly
unused on the single-worker PS2 runtime; the relevant physics hook is
`b3SetBitCountAndClear` in contact processing. The 2 KiB microbenchmark
is the direct measure of OR speed, not a promise of whole-engine gain.
An `x1.0` or worse physics result is a valid outcome. PCSX2 cycle counts
and timings should not be treated as hardware EE performance results.
A span of 2^31 Count ticks (~7.28 s nominal) or longer per step is
rejected, because its wrap cannot be distinguished reliably. No claim
of actual-hardware validation is made.

## VU0 macro-mode numerical A/B benchmark

The same `ps2/box3d_ps2_bench.elf` now also runs VU0 single and paired tests after the
original MMI tests. VU0 is the EE's 4-wide floating-point COP2 vector unit
and is distinct from integer MMI and VU1 micro mode.

- The VU0 kernel implements `b3DotW4` with `LQC2`, separate `VMUL.xyzw`,
  `VADD.xyzw` and `SQC2`. Each input/output is copied to **guaranteed
  16-byte-aligned** 112-byte scratch to avoid the EE LQC2/SQC2 alignment
  exception. The scratch packing and function-call overhead are included in
  the runtime comparison.
- The test compares 384 finite vector lanes with the scalar dot operation,
  checks an absolute plus relative tolerance and logs maximum error.
  The PS2 VU FPU can differ from EE COP1 by last bits and does not support
  NaNs/infinities/subnormals; invalid input is out of scope.
- ABBA physics samples with 8 and 32 boxes compare VU0-on vs scalar
  at 120 steps, without MMI interference. It checks position and velocity
  aggregates, first-box height, and cleanup. Outputs include
  `BOX3D_VU0_KERNEL`, `BOX3D_VU0_MICRO`, `BOX3D_VU0_SCENE`,
  `BOX3D_VU0_SUMMARY`, `BOX3D_VU0_BENCH_PASS`.
- **VU0 is opt-in** and disabled by default. Runtime mode 1 is the
  original one-dot-per-call backend; mode 2 fuses two independent dot4
  operations, interleaving six VMUL and four VADD operations to hide
  arithmetic latency. A standalone dot remains COP1 in mode 2.
  Hot paired sites now include narrowphase Gauss-arc tests, contact
  tangent/friction and iterative normal constraints; this is still not
  complete solver vectorization. The 224-byte aligned pair scratch is
  statically verified. The scene log includes actual `pairs=` calls to
  confirm use of the new physics path. We time each mode separately.
  
  The old COP1 microbenchmark used constant vector inputs that could be
  folded by GCC, creating a misleading 20-tick scalar "speed".
  Both the single-dot and new pair benchmarks now construct varying
  volatile-seeded operands **outside** the timed region; the test accumulates
  and prints results. This is a more useful comparison, though it still
  includes CP0 measurement overhead and is not a hardware result.
  
   Do not switch within a world
  step, use multiple EE workers, or use another VU0 microprogram concurrently.
  This initial per-call transfer implementation may well be *slower*;
  measure before committing to deeper contact-solver VU0 work.
- VU0 correctness and timing remain **unverified on PCSX2** until the
  new ELF is built and run by the tester. PS2 hardware timing is separate.

## EE-specific limitations (see attached EE Core user manual)

- **FPU:** COP1 supports only single precision, does not support IEEE-style
  NaNs/infinities/subnormal values, and has different rounding behavior.
  Host scalar success is not proof of EE numerical equivalence or determinism.
  Avoid NaN/Inf sentinel values, and compare numeric results on real EE.
- **Count clock:** CP0 Count is 32-bit. `src/ps2_count.h` extends the
  counter through one rollover using unsigned differences. Extra full wraps
  between polls cannot be recovered. The provisional default
  `BOX3D_PS2_COUNT_HZ=294912000` gives ~14.56 seconds per wrap and needs hardware
  calibration.
- **Scheduler:** sleep/yield and EE worker creation are unsupported.
- **Memory:** profile peak allocations and frame times on the real EE.
- **Acceleration:** aligned integer bitset zero/OR uses MMI; wide4
  contact dot products can optionally run on VU0 macro mode. Whole-solver
  VU0 vectorization and VU1 micro programs are not implemented.

The supplied hardware manual is a reference, not redistributed in the repo.

## Reading the on-screen stage log

| Stage | Meaning |
|---|---|
| 0 | EE ELF started; GS initialized |
| 1 | Box3D world creation |
| 2 | Ground hull creation |
| 3 | Dynamic cube creation |
| 4 | 15 gravity steps complete |
| 5 | 180 steps and first resting contact |
| 6 | 300 steps and stable resting contact |
| 7 | Destroyed world; memory accounting checked |

If the last stage remains visible, the next operation is the likely failing
stage. Box3D assertions show their condition and source line, then halt
instead of falling back into the loader. Results are also emitted as
`BOX3D_STAGE`, `BOX3D_RESULT`, `BOX3D_PASS` and `BOX3D_FAIL`
for automated serial-log collection. On the PS2 this program intentionally
never exits so a launcher cannot erase the display.

## VU0 pipeline/transfer diagnostic

The first PCSX2 VU0 trial passed numerical correctness but had poor
throughput: COP1 20 vs VU0 126 ticks for one `b3DotW4` and roughly 0.99x
whole-physics throughput. Packing six independent float4 values into a
112-byte record plus a function call and seven COP2 transfers for *one* dot
is the critical structural problem, not evidence that VU0's ALU is slower.

`BOX3D_VU0_BATCH` now reports three timings for **four dot4 records**:
COP1 scalar, four separate VU0 calls, and the new VU0 batch4 kernel.
The batch kernel arranges independent VMUL operations before their VADD
consumers and avoids four separate external calls. Check the three cycle
numbers and correctness on PCSX2 before widening the solver. This is
a standalone throughput experiment; the live solver still uses the
single-dot VU0 path and remains OFF by default.

Further optimization needs contact-solver data-flow batching so that many
operations stay in VF registers instead of paying LQC2/SQC2 transfers
for each temporary. A batch microbenchmark alone does not speed up
`b3World_Step`. Raw PCSX2 counts are not real PS2 hardware timing.

## Profiling the actual physics pipeline

`BOX3D_VU0_PHASE` and `BOX3D_VU0_PHASE_SUMMARY` are emitted for the
single-dot (mode 1) and paired-dot (mode 2) A/B scenes. They sample Box3D's
existing `b3World_GetProfile` **after** the measured `b3World_Step`,
so accessor overhead is excluded from outer CP0 timing. Each phase's
`*_ms` field is an average per step in nominal EE milliseconds, while the
original `BOX3D_VU0_SUMMARY` remains raw 294.912MHz Count ticks.
The on-screen `Phase ms: P/C/S` line reports *scalar/VU0* pairs,
narrowphase collide and solver timings.

A paired mode that doesn't execute any actual pair kernels now fails
explicitly (`pairs=0`), avoiding misleading "speedups" from unused
microbenchmarks. Inspect the `BOX3D_VU0_SCENE` call counts before
changing more VF instructions. The earlier four-record microbenchmark is
a standalone throughput test that does **not** imply that these grouped
operations are used by the live physics solver.

## Experimental mode 3: pair contact-preparation dot products

PCSX2 measurements with the mode-2 paired solver showed a real, repeatable
~1.027x-1.028x end-to-end improvement versus scalar, while the original
single-dot VU0 mode 1 was ~0.991x. The 32-box profile reduced solver time
from ~4.57 ms to ~4.44 ms (PCSX2, not physical hardware).

Mode 3 extends mode 2, pairing more independent inner products in contact
preparation: normal effective mass and three tangent effective-mass terms.
Each individual dot and all outer additions keep their original association.
Mode 2 is unchanged and remains the known-good runtime to compare against.

The benchmark now runs one additional *ABBA mode-2 versus mode-3* 32-box
comparison under label `extended 32-box`. Inspect the screen's mode2/mode3
cycles and `BOX3D_VU0_SCENE` CSV's `extended=` count (must be positive
in mode 3). The `BOX3D_VU0_PHASE_SUMMARY` fields identify the baseline
and target mode explicitly to prevent treating mode-2 results as COP1-only.
If mode 3 is slower, keep mode 2; this is an opt-in experiment, not the
default path. VU0 and PS2DEV crosscompilation of this latest experiment
remain unverified pending the next PCSX2 tester run.

## ABBA mode2->mode3 correctness fix / Amdahl bound

The first mode2-vs-mode3 PCSX2 screenshot erroneously displayed
`mode2 0 / mode3 ~3199847 cyc/step (x0.000)`. This was not a measurement:
the reducer grouped modes using `if(mode)`, so both **nonzero** modes
were accumulated in the target. Fixed with a shared, host-testable
`b3Ps2SummarizeABBA` reducer, grouping trials [0,3] vs [1,2] regardless
of selected mode numbers. Host regression:

```sh
cc -std=c17 -Wall -Wextra -Werror -pedantic \
    ps2/abba_stats_test.c -o /tmp/box3d_abba_stats_test
/tmp/box3d_abba_stats_test
```

`BOX3D_AMDAHL` now uses Box3D's existing phase measurements to print
the solver's fraction of the total step and the **theoretical maximum**
end-to-end acceleration if the solver alone were infinitely fast or 16x
faster. These are upper bounds under the assumption that all other
work remains unchanged, not predictions of VU0 performance.

VU0 peak vector multiply-add throughput assumes fully occupied four
float lanes and sustained issue, with data resident in VF registers.
Transferring/packing little dot4 jobs repeatedly and the scalar control
and broadphase paths do not meet those assumptions. Keep the old
mode2 backend as the performance/correctness reference until a new
experiment improves whole-engine `BOX3D_VU0_SUMMARY` on PCSX2.

## Experimental mode 4: VU0 shared-vector friction dot pairs

The PCSX2 mode2 vs mode3 32-box test was valid after fixing ABBA:
mode2 **1597985** vs mode3 **1592702** Count cycles/step, only ~1.003x
faster, with all correctness checks passing. This small difference is not
enough to claim a stable gain without repeat measurements.

Mode 4 retains mode 3 but replaces three friction dot-pair sites in the
live contact solver with a **shared-input VU0 macro kernel**. It keeps the
same velocity/impulse float4 vector in VF registers for both tangent dots,
loading only 9 input float4s (instead of 12 in the generic paired-dot path)
and storing 2 output float4s. The two results are computed with six
independent VMUL followed by four dependent VADD instructions, with
16-byte aligned 176-byte scratch. Mode 3 remains untouched for A/B.

The smoke benchmark now checks 768 finite output lanes against COP1
(`BOX3D_VU0_SHARED_CHECK`), reports `shared=` call counts, and runs a
mode-3 vs mode-4 ABBA comparison labelled `shared 32-box`.
It refuses a mode-4 physics run if the new shared kernel is not called.
The on-screen VU0 scenes are cleared to a fresh GS page before reporting
speed so the new comparison remains visible. These changes are **not
crosscompiled or emulator-tested yet**. Mode 0 remains the default.

## Experimental mode 5: fused normal-contact impulse constraint

Modes 0 through 4 are retained. Mode 5 keeps mode4 but replaces the
scalar update of a *normal contact constraint* in the repeated contact
solver loop with one COP2 macro-mode kernel. The VF registers retain
intermediate values across normal-speed bias, effective-mass multiply,
nonnegative impulse clamp, impulse delta, per-point normal total,
per-manifold normal total, and twist-limit accumulation.

The scratch layout is 8 input and 5 output float4 vectors, 208 bytes,
aligned to 16 bytes. Normal contact impulse semantics assume finite
inputs (VU0 has no IEEE NaN/Inf/subnormal behavior). The bench checks
1920 finite scalar-versus-VU outputs at startup, refuses to pass a
mode5 physics trial with `normals=0`, and runs a final ABBA
`normal 32-box` **mode4 vs mode5** comparison. Only accept a gain if
both the differential math and the 120-step physical snapshots pass.

The last PCSX2 screenshot confirmed **mode3 1568444** versus
**mode4 1567005** cycles/step (1.001x, noise-sized). The fused solver
path remains **untested** with PS2DEV/PCSX2 until the next run.
The standalone scalar smoke starts with VU0 disabled.

Host C17 normal-kernel scalar fallback smoke:
```sh
cc -std=c17 -Wall -Wextra -Werror -ffp-contract=off \
  src/ps2_vu0.c ps2/vu0_normal_test.c -lm -o /tmp/b3_vu0_normal
/tmp/b3_vu0_normal
```

## VU0 compute roofline probe (not an end-to-end optimization)

The PCSX2 mode4-vs-mode5 screenshot (2026-10-10) shows that moving the
normal impulse and its accumulators into a single 208-byte VU0 macro
kernel was a **regression**: mode4 ~1,554,726 and mode5 ~1,561,309
CP0 cycles/step, **0.996x**. Correctness PASS. Do not select mode5
for throughput; mode4 remains the reference.

The diagnostic `BOX3D_VU0_ALU_PROBE` now runs 8 *independent* VF
accumulators in a 64-iteration VU0 macro-mode loop, with a single
aligned operand vector kept resident in VF registers: 512 four-lane
vector additions (2048 float32 additions) per sample. It measures
and logs the **sustained compute-only** effective vector-add issue
rate, verifying every result exactly with binary-fraction inputs.

`nominal_gflops` multiplies observed operations per CP0 tick by
a nominal 294.912 MHz and is a *PCSX2 diagnostic*, not PS2 silicon
throughput or a prediction of Box3D's speed. Neither the setup nor
the argument-reset work is part of the timed kernel; the internal
loop and initial/final COP2 transfers *are*. A vector-add-only
probe is not the same as peak VU0 fused multiply-add throughput.
The point is to isolate pipeline utilization from the costly
per-constraint `LQC2/SQC2` and control-path work in Box3D.

## First visual samples, with native GS/GIF and no gsKit

`sh ps2/build_ee.sh` now produces `ps2/box3d_ps2_samples.elf` in
addition to the existing scalar correctness and VU0 benchmark ELFs.

**GS implementation:** `ps2/gs_native.c` uses PS2SDK's `graph` solely
to configure the CRT and the two display framebuffers, and PS2SDK's
`dma` only to send a cache-flushed DMA channel-2 transfer. **Every
drawing operation is encoded directly as raw GS A+D register writes
inside a packed GIF tag.** It configures FRAME_1, XYOFFSET_1,
SCISSOR_1, TEST_1, PRIM, RGBAQ, XYZ2 and FINISH, then waits for the
GS FINISH bit and VBlank before swapping display buffers. The
320x224x32bpp scene displays pure untextured wireframes with no z-buffer,
Sokol/ImGui, gsKit, libdraw, libpad or libfont dependencies.

The ELF uses the actual positions/rotations from the Box3D world and
a lightweight EE-side perspective projection, not precomputed animations.
It switches scenes automatically every 600 frames (50/60Hz display
depending on the console's NTSC/PAL region, physics fixed at 60Hz):

1. **Stacking / Box Stack**, adapted from `samples/sample_stacking.cpp`.
   Twelve of the desktop sample's forty dynamic 1m cubes over ground.
2. **Shapes / Inclined Plane**, adapted from `samples/sample_shapes.cpp`.
   Five dynamic cubes with varying friction on a tilted static box.
3. **Bodies / Spinning Books**, adapted from `samples/sample_bodies.cpp`.
   Three non-gravity thin rotating bodies, each spinning about a
   different axis.

All share one EE worker and the scalar physics backend (VU0 is disabled
by default). The samples periodically write `BOX3D_SAMPLE_SCENE` and
`BOX3D_SAMPLE_FRAME` telemetry to PS2SDK stdout. No controller input
yet: the sample list cycles for bring-up without pad or IOP modules.

The next goal after confirming GIF+GS output in PCSX2 is filled faces,
Z buffering, controller-driven sample selection and then the rest of
the upstream sample suite. This is a **first visual port**, not an
attempt to compile the original host-side C++/Sokol/ImGui application.

**Verification status:** unlike the previous scalar/VU0 PCSX2 smoke
screenshots, the new GS ELF has not yet been crosslinked or executed
on PCSX2/hardware in this environment. Do not treat an ELF header
check as proof that GS initialization or frame submission works.

## PCSX2 confirmed native GS wireframe, and optional filled renderer

Tester screenshot (2026-10-10) confirms a ground grid and animated
stacking cuboids rendered at a reported **60fps** in PCSX2 by
`ps2/box3d_ps2_samples.elf`, with actual Box3D simulation.
A second `ps2/box3d_ps2_samples_solid.elf` is now built, using
the same scene data but opting into hardware GS triangles and ZBUF32.
No gsKit or libdraw: only direct GS GIF A+D DMA packets plus
PS2SDK `graph` for CRT and `dma` for submission. The known-good
wireframe renderer remains available separately. The new solid
executable is not yet crossbuilt or run on PCSX2.

## Official demo visual approximation

`ps2/box3d_ps2_samples_official.elf` is an optional **third** native GS
sample with daylight sky, diffuse+ambient-lit pastel cuboids,
checkerboard ground, Z-tested hard planar contact shadows, and a small
native GS overlay showing the selected scene. It uses the *same*
C17 Box3D physics world as the already PCSX2-validated filled version
but does not depend on gsKit, PS2SDK draw, shaders or an IOP UI service.
See `ps2/NATIVE_GS_SAMPLES.md` for differences from the actual
desktop renderer's Preetham/PBR/IBL/GTAO/shadow-map pipeline.

The new ELF has not been compiled or run in PCSX2 yet. The old
`ps2/box3d_ps2_samples_solid.elf` stays available as the known-good
Z-buffered build, and `ps2/box3d_ps2_samples.elf` stays as wireframe.

The official-style first Box Stack also restores the **full 40 dynamic
boxes**, original 1.25m vertical spacing, 40m half-extent ground,
and approximately the original 40° yaw / 15° pitch / 50m orbit camera
looking at Y=20. The previously validated wireframe/filled builds
still use 12 boxes and their smaller scene camera. This changes
physics/rendering load; 60fps must be re-evaluated in the new ELF.

## Directional box silhouette shadow repair

A fourth visual ELF, `ps2/box3d_ps2_samples_shadows.elf`, keeps the
official-style materials and physics but replaces the broken projected
bottom-face shadow quads with full convex silhouettes of all eight box
corners, clipped against both the floor and inclined platform receivers.
Opaque depth is drawn first, then direct-GS alpha-blended shadow geometry
uses ZBUF32 with Z writes disabled. Camera-near-plane clipping is added
to this variant. See `ps2/NATIVE_GS_SAMPLES.md` for the PS2-era
historical references, exact method, test instructions and known
limits. This is **not yet** a real stencil shadow volume or a depth map.
The existing wireframe, solid, and official-style versions are kept.
