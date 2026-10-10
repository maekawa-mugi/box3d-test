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
FIX total. Raised casters now downsample their 640x480 coverage mask
into an independently allocated 320x240 PSMCT32 GS target using
hardware bilinear, then reconstruct a distance-dependent shadow
with a connected nine-tap 3x3 tent kernel (96 FIX units). This smooths
silhouette stair-stepping and avoids four widely separated blur lobes. This is native GS fixed-function post-filtering,
not real shadow-depth-map PCSS blocker search.

The prior top/bottom software-renderer corruption was fixed by
padding 640x480 PSMCT16/PSMZ16 allocations to actual 64-row GS
pages and 32bpp mask allocations to physical 32-row pages. GS AA1
is enabled for line edges; 3D polygons use full GS 12.4 XY precision.
This is not full-scene MSAA.

## Twelve pages, one controller

PS2 D-pad **LEFT** goes to the previous scene; **RIGHT** to the
next. Each newly pressed direction immediately recreates the
Box3D world. Held buttons do not auto-repeat. With a controller initialized,
the current page remains selected until the next D-pad press, allowing
long benchmark runs. If libpad initialization fails, the fallback
600-frame automatic tour remains active.

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

The HUD shows `PHYS`, `DRAW`, `GSWAIT`, and `USE`, estimated from EE CP0
Count at nominal 294.912MHz. The CSV console logs every 60 frames:

```
BOX3D_PERF,scene=9,frame=60,bodies=144,avg_physics_ms=...,avg_build_ms=...,avg_gpu_wait_ms=...,peak_work_ms=...,avg_work_pct=...,vsync_idle_ms=...,gif_qwords=...
```

- `avg_physics_ms`: average Box3D `b3World_Step` over 60 frames
- `avg_build_ms`: average EE draw-vertex and packed-GIF generation
- `avg_gpu_wait_ms`: average GIF DMA + GS FINISH completion
- `peak_work_ms`: slowest combined work frame in the 60-frame window
- `vsync_idle_ms`: deliberately waiting for display VSync
- `avg_work_pct`: average (physics + build + GPU completion) / 16.667ms * 100

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

Portable C17 page/benchmark sizing regression:

```sh
cc -O2 -std=c17 -Wall -Wextra -Werror -pedantic \
  ps2/unified_demo_test.c -o /tmp/b3_ps2_demo_test
/tmp/b3_ps2_demo_test
```

To keep tier comparisons meaningful even after all boxes land, **body
sleeping is disabled on stress pages only**. Regular eight demos retain
Box3D's default sleeping. The CSV output reports rolling 60-frame
average work and peak duration; screen HUD uses the last frame.

## 2026-10-10 PCSX2 feedback and optimization

Tester screenshots with PCSX2 D3D12 **software renderer** confirmed
that the unified ELF ran: scene 11/12 had 256 bodies at about
**20 fps**, with HUD CPU time near **38.3 ms** and USE near
**231%**. Scene 12/12 had 400 bodies at about **12 fps**, CPU near
**67.8 ms** and USE near **409%**. A thin tall caster's shadow
showed noisy black streaks, but the prior top/bottom framebuffer
artifacts were gone. These are **baseline screenshots before this
optimization**, not new performance results. The HUD's prior
`GPU 0.1/0.2` number was GS FINISH wait time in milliseconds,
**not** GPU utilization; it has been renamed to `GSWAIT`.

Two major repeated EE operations were removed in the unified build:
- Cache each body position/rotation and eight world-space hull vertices
  exactly once after `b3World_Step()`. Both the GS solid box renderer
  and projected shadows reuse the same pose. Other macro combinations
  retain their original code for regression isolation.
- The PCSS classification pass now caches each caster's convex shadow
  polygon and uses that polygon in the second pass, eliminating a
  second hull projection/clip and converting each polygon fan vertex
  only once. The physics body count, solver iterations and dynamic
  contact behavior are unchanged.

To improve **jagged** raised shadows, one physical GS 320x240 PSMCT32
prefilter target is now allocated with page-height padding to 320x256
(327,680 bytes). For PCSS radius 2..5, hardware bilinear downsamples
the full-res silhouette before a nine-tap 3x3 tent (sum FIX=96) with
radius-dependent offsets. At radius 1 the proven four-bilinear-tap
full-res PCF remains unchanged. The unified scene's four GS surfaces
and prefilter occupy **3,620,864 of 4,194,304 bytes**, leaving
**573,440 bytes**. There is no new ELF and no new CPU per-pixel blur.

The updated on-screen panel shows `PHYS` and `DRAW` separately and
calls the DMA/FINISH metric `GSWAIT`, so an EE Box3D solver bottleneck
can be distinguished from geometry/GIF command generation.
**Neither the FPS gain nor the new shadow softness is yet validated
in PCSX2**. Compare the before/after same 256/400-body pages and
capture `BOX3D_PERF` for quantitative results.

## 2026-10-10: Fix PCSS double shadows by combining overlapping masks

The 480p tester's Spinning Books screenshot after the first
half-resolution prefilter still showed distinct dark edges, i.e.
**double exposure**, not merely jagged texture sampling. Root cause
was the old radius-bucket implementation: when two differently blurred
caster groups landed on the same floor pixels, the GS executed two
independent subtractive `FIX=96` shadows on those pixels. Any amount
of bilinear PCF filtering still leaves a doubled/darker silhouette.

The single `ps2/box3d_ps2.elf` now builds overlap-connected caster
groups on the EE before sending any shadow mask GS draws. Each
caster's projected silhouette is expanded by the whole PCSS kernel
support. If two such expanded rectangles overlap, their casters are
merged into a single connected component (including transitive chains
of overlaps). The component's silhouettes form **one binary mask**
and undergo **exactly one GS filtering and subtractive composite**.
Casters with disjoint screen-space penumbrae keep separate masks and
their own distance-derived blur radii. An overlapping component uses
the **maximum radius** of its members, a deliberate conservative
approximation that trades some per-object softness control for
unambiguous, single-exposure shadows. This is still PCSS-inspired,
not a depth-buffer blocker search with a spatially varying penumbra.

The serial log every 240 frames includes:

```
BOX3D_PCSS_UNION,scene=2,receiver=0,casters=3,components=1,composites=1,radius_min=...,radius_max=...,major_axis=X
```

The *numbers in this sample line are illustrative*, not a measured
PCS​​X2 result. For three touching silhouettes, the intended value is
`composites=1` rather than two or three. The deterministic C17
regression includes a 40-caster overlapping test and disjoint groups:

```sh
cc -std=c17 -O2 -Wall -Wextra -Werror -pedantic \
  ps2/shadow_group_test.c -o /tmp/box3d_shadow_group_test
/tmp/box3d_shadow_group_test
```

No new ELF, no new VRAM pages, no changes to physics or controller
pages. The EE grouping requires O(N²) bounding-box comparisons
in the worst case; 256/400-box performance should therefore be
**re-measured**, not assumed faster. The host grouping algorithm has
passed its regression test, but **the newly pushed GS output has not
yet been rebuilt or visually confirmed on PCSX2**.

## 2026-10-10: no duplicate silhouettes in single 480p PCSS ELF

The software-renderer screenshot of Spinning Books showed **two
distinct dark traces** at the edge of one softened shadow, even after
the previous prefilter pass. There were *two separate bugs*:

- **Multiple subtractive passes:** radius-bucket shadow masks were
  composited directly onto the scene one after another. When
  silhouettes from different radius buckets touched, the same
  screen pixel was darkened repeatedly. The new
  `ps2/shadow_group.h` groups all casters whose *kernel-expanded*
  projected silhouettes overlap (including transitive chains).
  Each resulting connected component is rasterized as ONE GS
  coverage mask, and the scene is subtracted from only once.
  Isolated shadows retain independent PCSS radii. For overlapping
  silhouettes, the group's maximum radius is chosen to preserve
  soft separation without exposing a duplicate harder silhouette.
- **Sparse final taps:** the previous far PCSS nine-tap blend placed
  distinguishable copies of the same mask at `+/-(radius-1)` screen
  pixels. The new `pcss_filter_tiers.h` uses REAL GS bilinear
  2:1 downsamples instead: radius 1 samples the full-resolution
  640x480 mask with four bilinear taps, radii 2-3 use the 320x240
  prefilter, and radii 4-5 use a new disjoint 160x120 prefilter.
  The final 3x3 9-tap kernel has at most +/-1 screen-pixel spacing
  for every distance, so distant shadows blur continuously rather
  than splitting into parallel stripes.

The additional physical 160x120 PSMCT32 GS target uses a 192x128
page-rounded allocation (**98,304 bytes**). Total VRAM allocation
for the unified double-buffered 480p scene, Z, full-resolution mask,
half-resolution and quarter-resolution prefilters is now
**3,620,864 / 4,194,304 bytes**, leaving **573,440 bytes**. No new
ELF, no new sample mode, no CPU pixel blur.

The new serial log `BOX3D_PCSS_UNION` reports `casters`,
`components` and `composites`. Overlapping silhouettes should
produce `components=1,composites=1`. The following portable C17
regressions test overlap merging and blur tier limits:

```sh
cc -std=c17 -O2 -Wall -Wextra -Werror -pedantic \
  ps2/shadow_group_test.c -o /tmp/shadow_group_test
/tmp/shadow_group_test
cc -std=c17 -O2 -Wall -Wextra -Werror -pedantic \
  ps2/pcss_filter_tiers_test.c -o /tmp/pcss_filter_tiers_test
/tmp/pcss_filter_tiers_test
cc -std=c17 -O2 -Wall -Wextra -Werror -pedantic \
  ps2/kawase_layout_test.c -o /tmp/kawase_layout_test
/tmp/kawase_layout_test
```

**The C17 overlap/tier helper tests pass. New R5900 ELF linking and
PCS​​X2 output are unverified here.** If duplicate shadows remain,
capture Spinning Books plus the corresponding
`BOX3D_PCSS_UNION` log: if only one component and one composite
are reported, investigate GS texture sampling/texel alignment
rather than repeated caster passes.

## 2026-10-10: 32-bit PCSS filter accumulation (experimental)

The 3x3 far-field tent is no longer collapsed to a single horizontal or
vertical axis by integer minor-axis compression. Its nine weighted taps
accumulate on a disjoint 320x240 PSMCT32 render target, then a **single**
GS subtraction composites the result into the 640x480 PSMCT16 scene.
Near-contact shadows retain the existing four-bilinear-tap full-resolution
PCF path. Full/half/quarter mask sources stay disjoint from all destinations;
every texture change issues TEXFLUSH.

The additional PSMCT32 target costs 327,680 bytes (GS page rounded).
Expected total surface allocation: **3,948,544 / 4,194,304 bytes**,
leaving **245,760 bytes**. This is a change in GS blend path and must be
visually checked on PCSX2/real GS, particularly mask intensity and edges.
No FPS or silhouette-quality improvement is claimed without a new run.

## 2026-10-10: screen-space shadow union broadphase

The 1..5-radius caster component builder now uses 32-pixel X buckets
and prunes completed intervals before exact pairwise 2D overlap tests.
Disjoint horizontal regions avoid a full N*N comparison. Overlapping
rectangles still use the same deterministic lowest-index union root and
one subtractive mask composite per connected component. Dense fully
overlapping casters remain worst-case O(N^2); this is a broadphase
optimization rather than a claim of strict linear complexity.

## 2026-10-10: reusable projected GS box vertices

A fully projectable box now perspective-transforms eight unique vertices
once and reuses their GS 12.4 XY and Z values for both triangles of each
visible face. This removes repeated camera-space dot products/divisions
from solid-body draw packets. Any box containing near-clipped or
out-of-extended-raster vertices falls back to the original clipped
triangle path. Shadow geometry and physics remain unchanged.
Before/after equivalence of real GS depth and frame times is unverified.

## 2026-10-10: physics A/B controls and build-time diagnostics

The single demo ELF keeps its previous default: four physics substeps
and VU0 scalar mode 0. With a connected controller, **UP** toggles
four/two substeps; **DOWN** toggles VU0 scalar/mode-5 kernels. This
allows fair side-by-side CPU measurements without silently changing
physics stability or the default 400-body stress definition. Toggle
only outside the world step, as required by the existing VU0 kernels.
Mode 5 may change numeric results, and must not be declared equivalent
without comparison against the scalar reference.

The 60-frame BOX3D_PERF log now separates CPU shadow-mask/group build
(`avg_shadow_ms`) and opaque box-geometry command construction
(`avg_box_ms`) from total build time. These two intervals are subsets
of `avg_build_ms` and must not be added to it as independent costs.
New `substeps` and `vu0` columns record the actual physics mode.
The GSWAIT metric is an elapsed DMA/FINISH wait, not GPU utilization.

The host scalar smoke workflow additionally compiles/runs PCSS geometry,
union, blur-tier and VRAM allocation regressions. Cross-compilation and
actual R5900/GS pixel output still require PS2DEV/PCSX2.

## 2026-10-10: reusable projected shadow silhouette vertices

The PCSS classification stage now caches projected GS XY12.4/Z for
every valid clipped shadow polygon vertex, then emits its triangle fan
directly from those cached screen vertices. This avoids repeatedly
dividing/projecting the same fan vertices in the second shadow pass.
Projection failures retain the previous per-triangle near-plane-clipped
path. The projected-triangle helper also preserves constant RGB=128
coverage when rendering into the shadow mask rather than a solid box.
This remains an EE-side optimization; a real VU1/VIF transform pipeline
is separate, and still unimplemented/unverified.

## 2026-10-10: internal Box3D solver profiling

Every 60 frames, `BOX3D_PHYS_PROFILE` reports averaged Box3D internal
pairs/collide/solve/constraint timings along with live contact and awake
body counts. `BOX3D_PERF` remains the EE CP0 wall-clock measure;
the profile sub-stages can overlap or include nested work, and must not
be summed as if they were independent. Use the profile to decide whether
contact generation, solver iteration count, or shadow/geometry CPU time
is the next major constraint. No automatic reduction of solver quality
was made. CI now also runs a deterministic 250-case randomized union
comparison against the original all-pairs reference.

## 2026-10-10: offscreen-shadow bounds regression guard

The polygon classifier now distinguishes a failed perspective projection
(near/far clip, which needs a conservative full-screen bounds fallback)
from a valid projection outside the GS extended XY12.4 raster range.
Offscreen points keep their actual screen-space bounds for overlap
grouping. Only the fast direct XYZ emission is disabled; the original
clipped triangle routine handles raster-unsafe polygons. This prevents
an out-of-range but valid silhouette from unnecessarily expanding a
PCSS group to the full screen.

## 2026-10-10: static floor GIF cache and box basis reuse

The demo's camera/ground remains fixed within a page. Rather than
reprojecting all 288 checker-floor triangles every frame, its GS GIF A+D
commands are now recorded after the first rendering and replayed on each
following frame. Switching pages invalidates the cache, which is rebuilt
for the next scene camera; the framebuffer address is not captured.

Each box's 8 world corners and 6 face normals now reuse three quaternion-
rotated basis vectors per frame rather than performing 14 separate vector
rotations. Shadow polygons use the exact same cached box corners. A
per-component linked member list also replaces N-component-by-N-caster
scans during receiver shadow-mask rasterization. These optimizations
preserve the default 4-substep and scalar VU0 physics settings.
Rebuild/PCS​​X2 visual comparison is required to confirm frame-time gains.

## 2026-10-10: runtime shadow quality isolation

Press **SQUARE** to cycle `PCSS -> PCF -> HARD -> OFF`
without rebuilding or resetting the physics scene. This distinguishes
projection/coverage artifacts (visible even with HARD) from problems
introduced by half/quarter prefiltering and 32-bit weighted mask
accumulation (visible only with PCSS). PCF applies the legacy four-tap
full-resolution contact filter to the *same* projected and unioned mask.
OFF skips CPU shadow projection and all its GS passes, making its
`avg_build_ms` directly comparable to the shadowed modes. The current
mode is displayed in the scene HUD and logged with `BOX3D_PERF`.
Default mode remains PCSS, unchanged from the previous screenshot.

## 2026-10-10: real VU1/VIF1/GIF PATH1 solid-box pipeline (experimental)

The **400-body stress path now defaults to VU1 for solid boxes**; the
other showcase scenes continue using the previous EE geometry renderer
so their edge strokes and special platform layering do not regress.
Triangle toggles the VU1 geometry path on/off for same-scene A/B
benchmarking. If initialization fails, solids fall back to the EE path.

The new `vu1_boxes.vsm` is actual VU1 *micro-mode* code assembled with
`dvp-as`. VIF1 DMA uploads a camera transform and, for each solid box,
eight 16-byte world-space corners plus six 16-byte flat RGBAQ colors.
The microprogram perspective-divides each corner, converts to GS 12.4
XY plus reversed Z, emits 36 indexed vertices into VU1 data memory,
and sends a GIF PACKED `RGBAQ,XYZ2` triangle stream with `XGKICK`.
No screen-space division or GIF XYZ packet construction occurs on the
EE for boxes sent via VU1. Exceptional near-clipped or out-of-range
boxes are still rendered through the existing EE safety path.

**GIF order is explicit:** the earlier EE GIF PATH3 packet renders the
sky, floor and all PCSS masks and composites and is completed before
VU1 draws solids through PATH1; the final EE GIF PATH3 packet draws the
HUD, emits FINISH, waits VSync, and swaps framebuffers. The VIF chain
uses FLUSHA after each box to guarantee the one VU1 output buffer is
not overwritten while XGKICK still reads it. This is a *correctness*
baseline, not yet optimal double buffering; a later dual-buffer VU1
output can remove those GPU-side synchronization bubbles.

Build now requires `dvp-as`, `libpacket2.a`, `vu1_boxes.vsm` and
`vu1_boxes.c`. The ELF contains the actual VU1 microcode symbol,
verified by `nm`. The HUD page line displays `D1` when stress-scene
DMA geometry is enabled; `BOX3D_VU1_FRAME` reports number of VU1
boxes and `BOX3D_PERF` includes `vu1_enabled` / `vu1_boxes`.
CPU `DRAW` includes VIF1 waiting, unlike a pure CPU-packet count,
and GSWAIT only covers the final GIF DMA finish, not all VU1 GPU time.

This needs **PS2DEV / dvp-as compilation and PCSX2 software + hardware
renderer screenshots** before any speedup or pixel-equivalence claim.
Compare 256 and 400 boxes with the same `shadow_mode` and substeps,
toggle TRIANGLE to compare EE and VU1 and confirm that the PCSS
penumbra remains unchanged. The `vu1_boxes_layout_test.c` host test
checks ABI and projection constants but cannot verify VU opcodes.
