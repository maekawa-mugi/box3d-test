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
# Expected: ps2/box3d_ps2_smoke.elf and ps2/box3d_ps2_bench.elf
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
