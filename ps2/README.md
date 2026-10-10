# Box3D PS2: unified native GS demo

**Single supported sample ELF:** `ps2/box3d_ps2.elf`

This is a C17/R5900 PS2SDK port using native GS/GIF DMA, a genuine
640x480 progressive CRT mode, PSMCT16 double buffering, PSMZ16,
and Box3D real single-worker rigid-body simulation. It does not
depend on gsKit, libdraw or desktop Sokol/ImGui.

## Build

```sh
cd ~/box3d-test
git switch experiment/ps2-ee-scalar
git pull --ff-only origin experiment/ps2-ee-scalar
sh ps2/build_ee.sh
```

Run **only** `ps2/box3d_ps2.elf` in PCSX2 or on PS2 with a suitable
480p display. The script generates just one ELF, verifies its ELF
machine/type and required Box3D, GS, pad and ragdoll symbols.
Stale `ps2/box3d_ps2_samples_*.elf` files from earlier experiments
may remain on disk; the script no longer rebuilds them. Use the
path above, not those old variants. The smoke/bench sources remain
in the repository for developer reference.

## PCF and PCSS are not competing modes

The single demo applies a **PCSS-inspired, height-dependent penumbra**
to each receiving plane. Real Box3D caster corners and the receiver's
normal determine the gap. Casters are bucketed by a bounded
1..5-pixel radius; each bucket gets ONE unioned coverage mask and
ONE composite, preventing the older 40-fold subtractive black orb.
The directional light's projected tangent is used to limit minor-axis
spread.

**PCF** is the *filtering operation inside PCSS*: at contact, four
GS-bilinear taps implement a 3x3 separable tent with a 96-unit GS
FIX total, while raised casters use a 21-nonzero-tap binomial kernel
with symmetric half-texel bilinear sampling to soften jagged binary
silhouette edges. This is native GS fixed-function post-filtering,
not real shadow-depth-map PCSS blocker search.

The prior top/bottom software-renderer corruption was fixed by
padding 640x480 PSMCT16/PSMZ16 allocations to actual 64-row GS
pages and 32bpp mask allocations to physical 32-row pages. GS AA1
is enabled for line edges; 3D polygons use full GS 12.4 XY precision.
This is not full-scene MSAA.

## Twelve pages, one controller

PS2 D-pad **LEFT** goes to the previous scene; **RIGHT** to the
next. Each newly pressed direction immediately recreates the
Box3D world. Held buttons do not auto-repeat. The 600-frame
automatic tour still runs when idle.

Page 1 is Box Stack; page 8 is Ragdoll/Box with the original
12-body `shared/human.c` jointed physics (simple GS box proxies).
Pages 9 through 12 are the scalable stress presets:

| Page | Scenario | Dynamic boxes |
| --- | --- | ---: |
| 9 | BENCH 64 BOX | 64 |
| 10 | BENCH 144 BOX | 144 |
| 11 | BENCH 256 BOX | 256 |
| 12 | BENCH 400 BOX | 400 |

These benchmarks are **adapted Box3D grids**, not exact clones of
upstream Large/Wide Pyramid. Earlier pages remain Box Stack,
Inclined Plane, Spinning Books, Restitution, Slide Twist,
Conveyor Belt, Far Stack, and Ragdoll Box. The 21 upstream C++
sample modules contain far more registrations; see
`ps2/SAMPLE_PORTING_STATUS.md` for honest port coverage.

## Measure load even at 60fps

The HUD shows `CPU`, `GPU`, and `USE`, estimated from EE CP0
Count at nominal 294.912MHz. The CSV console logs every 60 frames:

```
BOX3D_PERF,scene=9,frame=60,bodies=144,physics_ms=...,build_ms=...,gpu_wait_ms=...,vsync_idle_ms=...,work_pct=...,gif_qwords=...
```

- `physics_ms`: Box3D `b3World_Step`
- `build_ms`: EE draw-vertex and packed-GIF command generation
- `gpu_wait_ms`: GIF DMA plus waiting for GS FINISH
- `vsync_idle_ms`: deliberately waiting for display VSync
- `work_pct`: (physics + build + GPU completion) / 16.667ms * 100

This is a pipeline elapsed-time estimate, **not raw GPU occupancy**.
It exposes headroom when the emulator displays exactly 60fps and
makes stress-tier comparisons reproducible. The GS GIF command buffer
is bounded below the hardware GIF NLOOP maximum (32767). Check for
`BOX3D_GS_WARNING,packet_overflow` on heavy scenes.

## Current verification limits

Previous 480p page-alignment improvements were visually verified in
PCSX2 by the tester, but the **new unified ELF, smooth PCSS sampling,
and large-box stress tests have not yet been compiled or tested on
PS2DEV/PCSX2 in this assistant environment**. Compiler/PCSX2 test
results are needed before claiming 60fps or elimination of jaggies.

The historical multi-ELF experiments and internal design notes
remain archived in `ps2/NATIVE_GS_SAMPLES.md` and
`ps2/PROGRESSIVE_GS.md`, but are **not current build instructions**.
VU1 batch transforms and full upstream scene parity remain pending.
