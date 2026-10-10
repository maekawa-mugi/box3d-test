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

## 2026-10-10: VU1 blackout recovery and staged diagnosis

A PCSX2 user reported an immediate black screen upon entering the stress
scenes with the first automatically-enabled VU1 path. A VIF1/GIF hang
is one likely cause, but the cause is **not yet confirmed** without
the emulator's EE console and a one-box reproduction.

To keep all pages usable, the demo now **starts with the established
EE rendering path** in every scene. On stress pages (9-12), press
TRIANGLE to cycle the number of boxes using VU1:

- D0: EE-only (safe default).
- D1: one box via VIF1 + VU1 + XGKICK; all other boxes via EE.
- D8: eight boxes via VU1, the rest via EE.
- D400: as many boxes as can be queued, up to 400, via VU1.
- Next press: D0 again.

The HUD page shows D0/D1/D8/D400. The EE console reports
`BOX3D_VU1_PHASE` before and after the PATH3 flush and the VIF1
DMA submit. SDK DMA waits now use a finite guard count and log
`BOX3D_VU1_FAIL,vif1_dma_timeout` rather than spinning forever at the
EE dma_channel_wait call. **A hung VU1/GIF can still stall the hardware
even after the DMA guard expires**. Power-cycle/restart PCSX2 when
testing a failing D1 step. The VU1 microprogram now has conservative
load/use scheduling for VU float and integer loads.

Recommended validation: first verify PCSS and all four stress pages
run in D0. Then enter D1 on BENCH 64 and copy the last
`BOX3D_VU1_PHASE` line if the screen freezes. Only advance to D8 and
D400 after the earlier level visibly runs for several frames. Compare
DRAW/PHYS against the same scene with D0. A black screen is **not
treated as solved** until PCSX2 confirms VU1 output.

## 2026-10-10: one-box VU1 blackout: separate VU from XGKICK

A tester confirms that **D0 remains visible but D1 blacks out even with
one VU1 box**. This is not a large-box-count performance issue, and
the current data does not prove which synchronization or VU instruction
is responsible. To isolate it, the TRIANGLE selector is now:

- **D0**: pure EE, known visible;
- **DT**: submit **one box to the same VIF1/VU1 projection program,
  but skip XGKICK** via a VU1 DMEM flag; the EE still draws that box;
- **D1**: VU1 with XGKICK for one box;
- **D8**: VU1 with XGKICK for eight boxes;
- **D400**: VU1 with XGKICK for all eligible boxes.

This does not pretend to fix D1 yet. Test BENCH 64 at DT first.
If DT goes black, the failure is in VIF1 or VU1 program execution,
before any GIF PATH1 output. If DT remains visible but D1 blacks out,
then focus on XGKICK/PATH1 and the GIF output packet. The EE console
prints `BOX3D_VU1_PHASE,before_path3_flush` with `emit=0/1`,
`after_path3_flush`, `before_vif1_dma` and `after_vif1_dma`.
Copy the **last line printed before blackout** to identify the stalled
operation. A finite DMA wait alone cannot repair a locked VU1 or GS.
D0 is the default, and the PCSS filtering is untouched.

## 2026-10-10: visible TRIANGLE confirmation and black-screen isolation

D0 is still the safe EE-only default. The upper-right HUD always shows
`TRIANGLE INPUT`, the number of accepted presses (`PRESS 0`, `PRESS 1`,
etc.) and a stage label. This makes controller recognition visible
**before any VU DMA is attempted**, rather than immediately freezing
on the first TRIANGLE press.

On a stress scene, repeated TRIANGLE presses select:

| Display | Work performed | Expected picture |
| --- | --- | --- |
| VU1 EE SAFE | Normal EE-only rendering | Normal |
| VU1 ACK | Button/HUD confirmation only | Normal |
| VU1 PATH3 | Flush existing GS PATH3 packet early, no VIF1 work | Normal |
| VU1 VIF | Same PATH3 flush plus VIF1 UNPACK of camera (no MSCAL) | Normal |
| VU1 VUONLY | Execute one-box VU1 transform, no XGKICK; EE also draws box | Normal unless VU stalls |
| VU1 BOX 1 | VU1 emits one box by XGKICK | Must visually test |
| VU1 BOX 8 | VU1 emits eight boxes by XGKICK | Must visually test |
| VU1 BOX 400 | VU1 emits up to 400 boxes | Must visually test |

This separates controller handling, GS finish barrier, DMA source
transfer, microprogram execution, and GIF PATH1 into different stages.
The VIF-only mode requires `BOX3D_VU1_PHASE,vif_only_dma_done` in the
PCSX2 console. The VU-only mode selects `VIF_FLUSHE`, not `FLUSHA`,
because there is no GIF PATH1 output to drain. This is a diagnostic
change, not evidence that VU1 drawing now works. Stop at the first
stage that freezes and report the last `BOX3D_VU1_PHASE` message.

## 2026-10-10: VU1-only hang isolated beyond VIF1

The user's PCSX2 run passes the `VU1 VIF` stage (camera UNPACK),
then blacks out at the immediately following `VU1 VUONLY` stage.
Thus a VIF1 DMA-only upload is not the failing step. The failure is
triggered when VU1 microcode is started, or when VIF waits for it.

The VU1 binary now starts with a **two-instruction early-termination
stub** at micro PC=0; the real projection entry is micro PC=2. A new
`VU1 BOOT` mode tests VIF MSCAL to PC=0 followed by FLUSHE, with no
dynamic vertices and no XGKICK. This distinguishes a bad VIF MSCAL,
microcode upload, or VU1 termination from the actual vertex/triangle
math program. The original projection code uses the symbol-derived
PC=2 and an explicit WAITQ on perspective DIV.

One correctness bug was also fixed: the XGKICK enable word was sent
as the float `1.0f` (bit representation `0x3f800000`) and read with
VU `ILW`, which observes only the low 16 bits (zero). The code now
uploads the integer bits `0x00000001` via memcpy, so an eventual D1
run genuinely enables XGKICK. This bug alone does **not** explain
the earlier VUONLY hang, so it is not claimed as a black-screen fix.

Revised TRIANGLE stages on stress scenes:
`D0 -> ACK -> PATH3 -> VIF -> BOOT -> VUONLY -> D1 -> D8 -> D400`.
Stop at the first failure and check the last
`BOX3D_VU1_PHASE` line. The initial D0 EE renderer and PCSS are
unchanged. Hardware/PCSX2 validation is still pending.

## 2026-10-10: VU1 BOOT crash to BIOS/browser

In PCSX2, all stress scenes pass PATH3 and VIF, but the BOOT press
blacks out and returns to the browser. The C `main` has an infinite
loop, so an explicit `return 1` from `main` is not supported by
the code. This is treated as a possible VU1, EE exception, DMA or
emulator failure, not a proven ordinary program exit.

To prevent blindly executing corrupted VU1 code, after the MPG
upload finishes `b3Vu1BoxesInit` now **reads VU1 micro memory at
`VU1_MICROMEM1_START` and checks every uploaded u32 against the DVP
image**. It prints `BOX3D_VU1_IMAGE,readback=ok` or
`BOX3D_VU1_FAIL,mpg_readback`, including the first mismatched word,
and disables the VU1 path on mismatch. The boot attempt logs
`VIF1_STAT`, `VIF1_ERR` and `VIF1_CODE` before MSCAL and on
return/timeout.

For reliable observability, the selector inserts a **BOOT ARM** stage
after VIF. BOOT ARM only changes the HUD; it invokes no VU
microcode. Press TRIANGLE again to run **BOOT**. BOOT is now a
**one-shot** per selection, not an MSCAL every subsequent frame.

Stress scene sequence:
`D0 -> ACK -> PATH3 -> VIF -> BOOT ARM -> BOOT -> VUONLY
 -> D1 -> D8 -> D400`.

For reproduction, capture the EE/PCSX2 console starting with
`BOX3D_VU1_IMAGE` and ending at the last `BOX3D_VU1_BOOT`
or `BOX3D_VU1_PHASE` log. If the emulator returns to the BIOS
browser, check PCSX2 logs for EE exception details too. The root
cause is still open until microcode upload / MSCAL completes in PCSX2.
D0 and PCSS remain unchanged.

## 2026-10-10: TRIANGLE stuck at EE SAFE

The demo HUD incorrectly showed `VU1 EE SAFE` after every triangle
press whenever either `s_vu1Ready == 0` or `scene.kind < 8`,
even though the controller handler DID advance the stage counter.
The previous MPG readback preflight could have disabled VU1, but
`EE SAFE` alone cannot prove this; a non-benchmark scene has the
same symptom.

The HUD now **always displays the selected stage** and the number of
TRIANGLE presses independent of VU1 readiness, and prints a separate
diagnostic status. The upper-right title `TRIANGLE VU1R3` identifies
this build. Status is one of `VU1 READY`, `BENCH SCENE ONLY`,
`VU1 FAIL ENTRY`, `VU1 FAIL VIF INIT`, `VU1 FAIL ALLOC`,
`VU1 FAIL MPG DMA`, `VU1 FAIL MPG SIZE`,
`VU1 FAIL READBACK`, `VU1 FAIL STATIC`,
or a relevant runtime DMA/BOOT failure. The same value appears
in `BOX3D_VU1_STARTUP` and `BOX3D_PAD_TRIANGLE` console lines.
Controller stages remain visible when initialization fails, but
actual VU1 execution stays disabled: **do not bypass a failed
microcode readback to make the stage advance**.

For reproduction, load the newest ELF and confirm `TRIANGLE VU1R3`
in the upper-right panel. Note whether `PRESS` increases on each
fresh triangle press, and copy the status text below the stage
label. An absent PRESS increment means an input/new-ELF issue.
A `VU1 FAIL ...` status means initialization was rejected before
any BOOT attempt. `BENCH SCENE ONLY` means switch to a
BENCH 64/144/256/400 page before attempting VU1.

## 2026-10-10: confirmed VU1 FAIL READBACK, guarded microcode recovery

PCSX2 now reports `VU1 FAIL READBACK` in the upper-right HUD. This
means the game's **MPG microcode preflight** rejected a mismatch
between the DVP image in EE RAM and the VU1 microcode window. It does
**not** mean MSCAL, the vertex transformer, or XGKICK ran.

The upload path now adds an explicit VIF1 `FLUSHE` after the last
`MPG` and before the DMA END tag. After the DMA channel finishes,
the preflight also waits with a finite counter for VIF1 command idle
(VPS and FQC fields). Only then does it compare the instruction
words. If the official PS2SDK VIF1 MPG path still disagrees, log
`BOX3D_VU1_MPG_MISMATCH` with the first word, expected and actual
hex values, plus VIF status and errors.

VU1 microcode memory is also mapped into EE's hardware address
space. **Only after VIF has gone idle**, a mismatching instruction
image is rewritten by the EE through the VU1 microcode MMIO window,
then read back and rechecked in full. VU1 is enabled **only if
every instruction matches exactly**. This backup affects only the
one-time microprogram installation; box vertices still use VIF1
DMA, execute on VU1, and reach GS via XGKICK.

The HUD now distinguishes `VU1 READY DMA` (MPG succeeded),
`VU1 READY CPU` (MMIO recovery verified), `VU1 FAIL VIF IDLE`
(VIF never became idle), and `VU1 FAIL READBACK`
(both verified upload methods failed). Starting mode is still
`VU1 EE SAFE`; never skip the ACK/PATH3/VIF/BOOT ARM steps.
Test BOOT in PCSX2 only once the HUD says READY. A successful
readback by itself does not prove that MSCAL or XGKICK works.

## 2026-10-10: confirmed VU1 micro/data addresses reversed in PS2SDK

PCSX2 `pcsx2/Memory.cpp` maps **VU1 instruction memory**
at `0x11008000` and **VU1 data memory** at `0x1100C000`.
See `glampert/quake2-ps2/src/ps2/debug/pipeline_dump.cpp`
for independent documentation of the SDK naming inversion.

The PS2SDK `vif_registers.h` macros currently say:

```c
#define VU1_MEM1_START      0x11008000
#define VU1_MICROMEM1_START 0x1100C000
```

These names do **not** describe the physical memory map correctly.
Our former `VU1_MICROMEM1_START` readback was inspecting DATA RAM,
which directly explains `VU1 FAIL READBACK`. Worse, the CPU-assisted
MPG recovery in the previous revision also wrote the expected
instructions to that incorrect DATA RAM address.

Use project-owned unambiguous constants from `vu1_memory_map.h`:

```c
#define B3_VU1_EE_MICRO_BASE 0x11008000u
#define B3_VU1_EE_DATA_BASE  0x1100C000u
```

Both the *readback* and *CPU fallback write* now address
`B3_VU1_EE_MICRO_BASE`, and the host layout regression test
asserts both address constants. The diagnostic
`BOX3D_VU1_IMAGE` line includes `micro_addr=11008000`,
`data_addr=1100c000`, and reports
`BOX3D_VU1_IMAGE,readback=ok` on successful verification.
The MPG VIF1 upload and CPU fallback are each verified against
the full program image before VU1 MSCAL is permitted.

**Reboot PCSX2 with the newly built ELF.** The previous binary
may have written into VU1 DATA RAM during attempted recovery.
Successful microcode readback establishes only correct upload,
not that the VU1 BOOT, vertex loop, or XGKICK now work. Keep
D0 as the default and validate BOOT ARM before BOOT.

References:
- https://github.com/PCSX2/pcsx2/blob/master/pcsx2/Memory.cpp#L393-L403
- https://github.com/glampert/quake2-ps2/blob/main/src/ps2/debug/pipeline_dump.cpp#L25-L28

## 2026-10-10: PCSX2 resets to BIOS immediately after executing the new ELF

The PCSX2 log now shows the correct `host:box3d_ps2.elf` ELF,
entrypoint `0x00102848`, and `ELF ... is executing` at 3.0439s.
PCSX2 reports `# Restart.` at 3.0849s and then boots
`rom0:OSDSYS`. Thus the ELF **was loaded**; this is an
early EE/PS2 reset, not a missing ELF or a confirmed `main return 1`.
The specific faulting instruction is still unknown.

The previous build called `b3Vu1BoxesInit()` at program startup,
before pad/scene creation. That initializes VIF1, sends MPG, reads
EE-mapped VU1 instruction memory, and previously attempted CPU MMIO
recovery. A regression in that path can prevent the entire demo from
starting, even though the default geometry mode is EE-only.

The demo **no longer touches VIF1 or VU1 during program startup**.
Only the ordinary GS/EE demo initializes. This restores a safe path
for displaying stress scenes and measuring PCSS/physics independently
of VU1. The risky CPU-assisted microcode rewrite has also been
removed. The microcode upload must compare correctly through VIF1
MPG before enabling BOOT or VU-only rendering.

On any BENCH scene, TRIANGLE selects:

| Press | HUD stage | Effect |
| --- | --- | --- |
| 0 | VU1 EE SAFE | Startup, no VIF1/VU1 initialization |
| 1 | VU1 ACK | Confirm controller only |
| 2 | VU1 PATH3 | Isolated GS PATH3 completion |
| 3 | VU1 INIT ARM | Confirm before any VIF/VU1 work |
| 4 | VU1 INIT | One-shot initialize VIF1 and upload/verify MPG |
| 5 | VU1 VIF | One-shot camera UNPACK |
| 6 | VU1 BOOT | One-shot minimal MSCAL/FLUSHE |
| 7 | VU1 VUONLY | Full transform with no XGKICK |
| 8 | VU1 BOX 1 | Send one box with XGKICK |

The upper-right HUD identifies this recovery version as
`TRIANGLE VU1R4`, and displays `VU1 NOT INIT` at startup.
**First test only that the new ELF launches and shows the EE boxes.
Do not advance to VU1 INIT before verifying ordinary pages.**
Once normal rendering is confirmed, testing INIT separately can locate
the exact code path that requests a BIOS reset. If the startup reset
persists without pressing TRIANGLE, the fault predates VU1
initialization and requires a different investigation.

Rebuild `sh ps2/build_ee.sh` from a freshly updated branch and boot
the freshly generated `ps2/box3d_ps2.elf` in PCSX2.

## 2026-10-10: VU1R5 split MPG upload from EE micro-memory readback

The EE-only startup (VU1 NOT INIT) is confirmed visible in PCSX2.
A prior ELF unexpectedly returned to the BIOS when VU1 initialized
during startup. The exact faulting instruction is still unknown.

The separate VU1 diagnostic stages now distinguish **VIF1/MPG DMA**
from the potentially problematic **EE memory-mapped readback**.
Do not infer that an uploaded image was verified.

On any BENCH 64/144/256/400 page, press TRIANGLE in order:

| Press | HUD | Operation |
| --- | --- | --- |
| 0 | VU1 EE SAFE | EE only, no VIF1 or micro-memory accesses |
| 1 | VU1 ACK | Pad acknowledgment only |
| 2 | VU1 PATH3 | GS PATH3 flush, no VU1 |
| 3 | VU1 INIT ARM | No VIF1 operations yet |
| 4 | VU1 INIT | Upload MPG and static DMEM via VIF1; **no EE micro-memory readback** |
| 5 | VU1 VERIFY ARM | No VIF1/EE micro-memory operations |
| 6 | VU1 VERIFY | Check VIF idle, then read the EE-mapped instruction window at 0x11008000 |
| 7 | VU1 VIF | Camera UNPACK (requires successful VERIFY) |
| 8 | VU1 BOOT | Minimal one-shot MSCAL (requires successful VERIFY) |

Only a successful VERIFY marks VU1 ready for execution.
Neither INIT nor the appearance of a stage label proves that VU1
executed. The HUD reads `TRIANGLE VU1R5` and reports
`VU1 MPG UPLOADED` after successful INIT. At successful VERIFY,
`VU1 READY DMA` is displayed.

Capture PCSX2's last `BOX3D_VU1_PHASE` line and the EE exception
log if pressing INIT or VERIFY returns to BIOS. This identifies
whether the reset occurs on VIF1 MPG upload or EE micro-memory read.
No direct CPU writes to micro memory are performed. The EE default
remains safe, and PCSS/physics remain unchanged.

This is a diagnostics/safety change, not a claim that MSCAL,
the transform microprogram, or XGKICK now works.

## 2026-10-10: VU1R5 normal-DMA replacement for INIT BIOS resets

The user's PCSX2 log confirmed that INIT ARM completes, but attempting
VU1 INIT results in a BIOS restart. This revision replaces the former
packet2 DMA chain REF/TTE MPG uploader with a contiguous VIF1 **normal
DMA** packet containing NOP/MPG/instruction payload/FLUSHE. The initial
static UNPACK packet uses normal DMA as well. Both are prebuilt and
size-checked before VIF1 is touched. Large per-box chain buffers are
allocated only after VERIFY succeeds.

Reason: an implicit packet2 assertion or malformed REF/TTE chain was
one plausible explanation for the earlier `# Restart.` log, which
did not include an EE faulting PC. This is a replacement of that
potentially failing upload path, **not yet a confirmed root-cause fix**.
The normal-DMA VIF packet format is independently checked over all
even sizes from 2 to 2048 microinstructions by a host test:

```sh
cc -std=c17 -Wall -Wextra -Werror -Ips2 ps2/vu1_vif_stream_test.c -o /tmp/box3d_vif_test
/tmp/box3d_vif_test
```

EE-only startup still initializes neither VIF1 nor VU1.
Follow the VU1R5 sequence and stop at INIT (4 presses) first.
A successful INIT displays `VU1 MPG UPLOADED`; VERIFY is still
required before any MSCAL or XGKICK. If INIT again returns to BIOS,
collect the EE stdout plus PCSX2 log to identify whether the last
line is `mpg_normal_dma_begin` or `mpg_normal_dma_done`.

## 2026-10-10: VIF and BOOT normal-DMA update

The last test reached VU1 READY DMA and selected the VIF stage, then
returned to BIOS at or after VIF. Previously, standalone VIF and BOOT
still used packet2 REF/TTE chains. This update replaces both with the
normal-DMA transport already shown to work for MPG upload.

VIF: 7 camera quadwords, STCYCL / UNPACK V4_32 / FLUSHE. Successful
transfer sets the HUD to `VU1 VIF OK`.

BOOT: one DMA quadword of FLUSH / MSCAL(0) / FLUSHE / NOP. Checks that
the E-bit stub exists before MSCAL. Successful transfer sets HUD to
`VU1 BOOT OK`. No CPU writes to VU1 micro memory.

The VIF/BOOT stream format has host regression coverage. These changes
do not prove VU1 execution or XGKICK works on PCSX2. VUONLY and box
submission still use packet2 chains and should not be tried until BOOT
is verified. PCSX2 on-target execution is not available here.

## 2026-10-11: one-box VUONLY no longer depends on packet2 chains

A PCSX2 run reached the BOOT page, logged a microVU1 program cache at
PC=0, then restarted around 1.7 seconds later. The PCSX2 UI log does
not contain an EE exception PC or the application's BOX3D stdout, so
this **does not prove whether BOOT or the next TRIANGLE / VUONLY
stage** requested the restart. The prior code still used packet2
REF/TTE chains in `b3Vu1BoxesSubmit` beginning at VUONLY.

The single-box, no-XGKICK VUONLY diagnostic now uses a bounded,
contiguous VIF1 normal DMA stream: camera, vertices[8], colors[6],
FLUSH, MSCAL transform PC=2, FLUSHE, NOP. Buffer size 104 u32 words
(26 QWs). The packet is host-regression-tested including exact
VIF command indices. VUONLY also prints `vuonly_normal_dma_begin`
and `vuonly_normal_dma_done`, and displays `VU1 VUONLY OK`
after its DMA completes.

BOOT itself is unchanged. If a reset happens before advancing
beyond BOOT, this change does not claim to solve it. Press only
through BOOT and confirm `VU1 BOOT OK` before selecting VUONLY.
Multi-box XGKICK still uses packet2, is NOT proven, and should not
be attempted until VUONLY is stable.

## 2026-10-11: all VU1 box modes use normal DMA

The tester reached VUONLY but the subsequent XGKICK-enabled BOX 1
stage reset PCSX2 to BIOS. Inspecting the implementation revealed
that BOX 1, BOX 8 and BOX 400 switched back to the earlier packet2
REF/TTE chain path even though INIT, VIF, BOOT and VUONLY used
normal DMA. The legacy submission path is removed.

Each VIF1 normal DMA transfer now contains one shared camera and
per-box vertex/color UNPACKs, FLUSH, MSCAL, FLUSHA and NOP for
XGKICK modes, or FLUSHE for no-XGKICK diagnostics. FLUSHA waits
for the GIF PATH1 output before overwriting the common VU1 DMEM
GIF buffer. A 400-box submission fits in 6,809 QWs. The function
also waits for VIF1 STAT to indicate idle after DMAC completion
before GS HUD PATH3 DMA. It reports failure rather than submitting
an unchecked packet. Logs include `submit_normal_dma_begin` and
`submit_normal_dma_done` upon first use of each mode.

The existing host test now covers byte-exact VUONLY one-box input,
plus 1/8/400-box command layout and FLUSHA. This change does NOT
establish that actual XGKICK/GIF rendering is working on PCSX2.
Test BOX 1 for stability before advancing to 8 and 400.

## 2026-10-11: opt-in Box3D sleep A/B for 400-body VU1 benchmark

A BOX 400 VU1 screenshot measured PHYS 76.6 ms, DRAW 22.7 ms,
GSWAIT 0.0 ms, USE 596%, approximately 10 FPS. At least 77% of
the per-frame work is the Box3D physics step. BENCH scenes
deliberately disable rigid-body sleeping to measure the worst-case
400-awake-body workload, including after the bodies settle.

Press **CROSS** on BENCH 64/144/256/400 only to toggle
`b3World_EnableSleeping(world,flag)` without scene recreation
or changing VU1/PCSS graphics. The initial condition remains
sleep OFF, `SL0`, precisely to preserve the original stress
benchmark. Sleep ON is shown as `SL1` in the page HUD and reported
in `BOX3D_PHYS_SLEEP`. The 60-frame `BOX3D_PERF` log now also
reports `sleep` and `awake`, so faster FPS is not mistaken for
equivalent fully awake physics performance. Use CROSS to restore
the same no-sleep stress load.

Comparison: remain in VU1 BOX 400 at 4 substeps and PCSS. Record
PHYS/DRAW/GSWAIT/PCSX2 FPS at SL0; tap CROSS once, wait for
bodies to settle, then record at SL1; tap again to return SL0.
Optional controls for separate tests: UP toggles S4/S2, and
SQUARE three times chooses SHADOW OFF. Compare one variable at
a time. Normal non-BENCH scenes retain their default sleeping.

No performance gain has yet been measured in PCSX2 for this change,
and this is a workload-mode control rather than a solver optimization.

## 2026-10-11: VU1R8 cache shadow geometry for fully sleeping benchmarks

User measurements on identical BENCH 400, S4, D400, SL1:
SHADOW PCF gave PHYS 0.0 ms, DRAW 22.7 ms, GSWAIT 0.0 ms,
about 30 FPS; SHADOW OFF gave PHYS 0.0 ms, DRAW 3.7 ms,
GSWAIT 0.0 ms, 60 FPS. The extra approximately 19 ms of EE
render work is in the shadow generation/compositing path, not
in the Box3D solver or basic VU1 geometry. PCSS had a similar
DRAW figure on the previous screenshot.

**Opt-in automatic optimization, only on BENCH pages:**
when world sleeping is enabled (SL1), the Box3D awake-body count
is exactly zero, receiver=ground and the stress-scene camera is
fixed, reuse the previously projected shadow polygons, 2D screen
coordinates, mask bounds, union groups and shadow linked lists.
Continue issuing *all* GS shadow mask/composite commands every
frame, preserving correct framebuffer flipping, visual quality and
dynamic PCF/PCSS/legacy filter changes. Do not reuse on any frame
with an awake body, on SL0, or in regular demos. Invalidate the
geometry cache on scene creation and when shadow filter mode changes
(including returning from SHADOW OFF).

The periodic `BOX3D_PCSS_UNION` log includes `shadow_cache=1`
for a reused geometry frame and `shadow_cache=0` for a rebuild.
No VU1 microprogram/DMA or Box3D solver modifications were made.
Runtime performance is not yet verified in PCSX2; this targets CPU
shadow projection/grouping, not the remaining cost of GS commands.
Measure SHADOW PCF/PCSS vs SHADOW OFF after settling at SL1, and
check for matching shadow silhouettes before reporting improvement.

## 2026-10-11: real VU0 micro-mode contact kernel, first gated hardware step

The 400-awake S4 benchmark costs ~76.6ms physics; after sleep 0ms,
a 400-box VU1 scene without shadows reaches 60 FPS. This step starts
a **real VU0 micro-mode solver path**, rather than adding more EE
COP2 macro instructions. It is NOT yet the Box3D world-step backend.

`ps2/vu0_micro_solver.vsm` is a new DVP microprogram stored in
VU0's 4KB micro memory, launched through **VCALLMS 0**, not
the existing `ps2_vu0.c` macro-mode helpers. One launch
processes sixteen 4-lane, independent normal impulse records,
**64 contact point lanes** total. DMEM at **0x11004000**
holds 209 QWs = 3344 bytes (below VU0's 256-QW limit).
The microcode image goes to **0x11000000**. Each 13-QW
record holds 8 inputs and 5 outputs, updating the normal
impulse, delta and three aggregates; association matches
Box3D's 4-lane normal impulse scalar expression. Both
4KB windows differ from VU1's corresponding addresses.

No VU0 memory access at startup. The known-good
EE solver, VU1R8 drawing, S4 and shadow mode are unchanged.
The new **SELECT** control works only in BENCH scenes and
advances one stage per button press:

| SELECT presses | VU0 panel | Hardware action |
| --- | --- | --- |
| 0 | VU0 NOT INIT | none |
| 1 | VU0 ARM | **none**; EE-safe acknowledgment |
| 2 | VU0 UPLOAD OK | VU0 micro image copy to 0x11000000 |
| 3 | VU0 VERIFY OK | read back full uploaded image |
| 4 | VU0 DIFF OK | upload 16 records, VCALLMS 0, compare 320 outputs to reference |

Read PCSX2 stdout lines prefixed `BOX3D_VU0_PHASE`,
`BOX3D_PAD_SELECT`, `BOX3D_VU0_PASS`, and
`BOX3D_VU0_FAIL`. At stage 4 the diagnostic must show
`VU0 MICRO DIFF OK` in the log; if it resets to BIOS, report
which stage caused the reset. Use a fresh ELF, not savestates.

The host ABI/differential reference test runs in CI:
`cc -O2 -std=c17 -Wall -Wextra -Werror -pedantic
ps2/vu0_micro_solver_test.c -lm -o /tmp/b3_vu0_micro_test
&& /tmp/b3_vu0_micro_test`.

**Limitations:** a host test cannot execute the microprogram or
validate VU0 hardware timing. The kernel does not yet apply
impulses to Box3D body velocities or perform broadphase,
friction and all substeps. Full 400-awake 60 FPS requires
benchmarked batched graph-color solve integration and
additional shadow optimizations; VU0 64-lane micro
correctness is only the first hardware milestone.

## 2026-10-11: VU0 micro-mode live contact-normal solver (experimental mode 6)

The user confirmed `VU0 DIFF OK` on BENCH 144 and BENCH 256 in
PCSX2 after VU0 micro upload, readback and the 64-lane numerical
test. This commit makes the first **real Box3D solver call site**
use the VU0 microprogram. The existing solver path remains
available for direct baseline comparison.

- `src/contact_solver_wide.inl`: PS2 SIMD4 normal-impulse and three
  accumulators can now use `b3Vu0MicroNormalSolve4` instead of COP1
  scalar or COP2 macro. The body velocity updates, friction, contact
  graph, broadphase, and other constraint passes remain on EE.
- The VU0 microprogram reads a **1..16 group count** from
  DMEM QW[0].x. Standalone DIFF continues to process all 16 groups.
  The live integration starts with **one group (4 lanes) per VCALLMS**,
  transferring only the necessary data and reading back five output
  QWs. This is explicitly a correctness milestone rather than
  the fast batched solver architecture.
- On failure to run VU0, the contact is recomputed by the scalar
  reference and the global solver mode reverts to 0, avoiding
  consumption of unchecked output.
- BENCH control: after SELECT is pressed four times and `VU0 DIFF OK`
  appears, press DOWN to cycle `0 -> 5 -> 6 -> 0`. Mode 5 is the
  prior COP2 macro path; mode 6 sends the *real* Box3D contact-normal
  computation through VU0 micro. Before DIFF, attempts to enter
  mode 6 are rejected and return to the EE scalar mode 0.
- When mode 6 is active the HUD says `VU0 LIVE MICRO`; the 60-frame
  `BOX3D_PERF` log includes `vu0_micro_calls` to prove that the
  real physics solver used VCALLMS and not just the standalone DIFF.
  The value is a cumulative counter, not a per-frame average.
- VU1R8 microprogram, XGKICK geometry, shadow system and S4/S2
  physics quality remain unchanged. No VU0 microcode is accessed on
  ELF startup, and the safe default remains mode 0.

**CAUTION:** mode 6 may initially perform much *worse* than mode 0
because one VCALLMS and EE/VU0 data exchange is required for
every 4-lane contact solve. It has not been timed or numerically
validated against moving contact bodies in PCSX2. Test BENCH 64
first (S4 SL0), then BENCH 144 and compare `PHYS`, stability and
contacts using DOWN mode 0 versus mode 6. Do not use BOX 400 in
mode 6 until lower contact counts are stable. If the runtime
shows `VU0 LIVE MICRO` with `vu0_micro_calls > 0` but worse
FPS, that is an expected sign that **independent graph-colored
contact batching, fewer synchronization points and EE overlap**
are the necessary next optimization. This does not demonstrate
400-awake 60 FPS yet.

Additional live safety: the first 64 real contact-group executions,
then one call out of each 1024, recompute the five results using the
EE scalar reference and reject large relative disagreement or
non-finite values. A failing call reverts to scalar mode 0 and logs
`BOX3D_VU0_FAIL,live_diff`; the HUD reads the *actual* solver mode,
not a stale pad variable. Because the EE FPU and VU0 round differently,
the guard is tolerant (0.2% relative plus absolute) rather than
bit-exact. This checks the first integration step, not full physics
determinism.
 
## 2026-10-11: VU1 always-on / TRIANGLE VU0 toggle (R10)

The debug stage protocol is removed from the regular unified ELF.
After GS initialization, VIF1 normal-DMA MPG upload and VU1 micro
readback run automatically. Every BENCH scene uses the full VU1
geometry batch without any TRIANGLE presses. If initialization or
DMA submission fails explicitly, render EE fallback solids rather
than leaving an empty picture.

The VU0 micro upload/readback/64-lane differential check also runs
once after GS startup. Its Box3D normal-contact backend remains
**OFF** by default. Each TRIANGLE press toggles only mode 0
(EE scalar) and mode 6 (real VU0 micro via VCALLMS) if startup
validation succeeded. There is no macro-mode 5 or SELECT protocol
in the user-facing controller. The HUD says VU1 AUTO ON and
TRIANGLE VU0 ON/OFF, or a clear failure status.

LEFT/RIGHT change scenes; UP selects S4/S2; CROSS controls BENCH
sleeping; SQUARE changes shadows. A single ELF offers both EE and
VU0 micro performance comparisons while keeping VU1 drawing active.

The VU1 .vsm also gains conservative load/result pipeline spacing
at FTOI0 -> SQI and LQ -> SQI, suspected to contribute to intermittent
missing GIF PATH1 triangles. This needs PCSX2 visual verification.
There is no claim yet that BOX 400 geometry is always visible or that
VU0 mode 6 meets 60 FPS. One VCALLMS per SIMD4 contact remains costly.

Safety refinement: automatically initialize both VU units only
**after one completed EE GS render/present**. Earlier VIF1 init
at startup could cause a BIOS return; successful staged tests used
an already initialized GS/GIF path. Unified megademo ELF now opens
directly to BENCH 400 (page 12/12) and renders one EE frame while
the units are being initialized. All subsequent frames use VU1
unless its verified normal DMA path fails. No multi-press protocol.

## VU0 mode 6 safety: isolated micro execution and bounded live work

R10 accidentally allowed mode 6 to pass the legacy `>=2`, `>=3`
and `>=4` VU0 COP2 macro-dot dispatch conditions. Mode 6 now
runs the real VU0 micro normal-contact kernel without interleaving
the old macro-dot kernels; other contact work stays on EE.

Because the live solver still launches only one SIMD4 record per
VCALLMS, the megademo caps real VU0 micro calls at
`B3_VU0_MICRO_MAX_LIVE_PER_STEP` per **world step** and completes
remaining normal contacts on EE. TRIANGLE's ON state is therefore
explicitly a **VU0+EE mixed** solver, not a fully offloaded solver.
Budget fallback does not disable VU0; an actual micro fault still
disables mode 6. The standalone 16-group/64-lane differential is
unchanged and not subject to the live budget.

VU1 VIF1/MSCAL/XGKICK and GIF PATH3 -> PATH1 -> PATH3 ordering
are unchanged; the mode switch is applied between completed frames
and the world step completes before any draw submission. Host CI
checks the macro/micro dispatch predicate and the existing VU0 ABI.
PCSX2 graphics recovery, 400-awake 60 FPS and real micro throughput
are **not verified** by host tests. Full 64-lane batched live physics
remains future work.

## VU0 one-group VCALLMS correctness guard

The startup 64-lane DIFF executes 16 groups, but a live Box3D
contact uses one group. The VU0 micro loop previously decremented
VI04 immediately before its IBNE, without integer result/use spacing.
This could let the branch see an old count and execute a record after
the requested last record. Four integer-pipeline spacing NOPs now
separate the counter update from the loop branch.

The existing automatic differential check now also exercises **one**
group with different arithmetic inputs, checks all 20 output floats,
and checks that a sentinel in the next group's output is untouched.
It remains a single startup self-check, not another controller stage.
If one-group execution fails or damages the guard, the VU0 backend
is unavailable and TRIANGLE cannot activate it; normal EE and VU1
rendering continue. The HUD also checks the current micro-ready
flag so a live micro failure is no longer labeled READY.

The exact PCSX2 symptom (shadows remain while BOX 400 vanishes after
TRIANGLE) still requires a new-ELF runtime retest. The host tests
cannot validate VU0 integer timing, GIF PATH1 completion or 60 FPS.

## VU0 final-group termination and live fault-safe rendering

The one-group guard exposed that a successful 16-group DIFF is not
proof that the final loop iteration terminates correctly when
the live contact solver requests one record. The VU0 microprogram
now compares the long-stable group counter to one **before**
decrementing. A one-group VCALLMS branches directly to the
E-bit exit without examining the just-updated counter or loading
the next record. Existing 16-group DIFF and one-group/sentinel
startup checks still gate live mode 6.

The HUD displays the current VU0 failure reason instead of only
"VU0 MICRO UNAVAIL". If a live VU0 hardware or numerical
failure disables mode 6 during a physics step, subsequent
geometry uses the original EE GS path rather than relying on
possibly affected VU1/GIF PATH1 output. A normal per-step micro
budget fallback does not disable VU1 or change the solver mode.
No additional pad-controlled diagnostic stages were added.

This is a conservative rendering recovery path, **not** proof of
a GIF synchronization root cause or 60 FPS. PCSX2 validation is
still required to distinguish startup DIFF failure from a
live contact failure and to verify the boxes remain visible.

## VU0 real graph-color batch integration

TRIANGLE mode 6 now invokes the micro normal solver with up to
**16 independent SIMD4 contact groups (64 lanes) per VCALLMS** for
wide convex contact blocks of at least two groups. Each graph color
has disjoint dynamic-body constraints, so a batch does not mix
dependent bodies or different colors. Within each constraint the
normal contact points are still solved and applied in sequence;
rolling/twist/central friction run later on the EE using the
updated body velocities. Overflow/mesh contacts and small blocks
retain the existing single-record path.

The batch copies up to 209 QWs into VU0 DMEM and checks all output
values for finiteness, with periodic scalar comparison of real
contacts. It never commits partial data: any VU0 fault recomputes
the entire pending batch on EE and disables further VU0 execution.
Normal per-step budget exhaustion also falls back to EE but retains
the user-selected mode. The batch budget is 128 VCALLMS per world
step, independent of the existing 32-call single-record budget.

`BOX3D_PERF` includes `vu0_micro_groups` alongside
`vu0_micro_calls`; successful 16-group batching should increase
groups faster than calls. This changes the workload genuinely
offloaded to VU0 rather than only enabling micro mode for a few
individual constraints. VU1/PATH1/PATH3 synchronization and the
TRIANGLE control remain unchanged.

**Unverified:** PS2 toolchain build, PCSX2 numeric equivalence and
long-running stability, 400-awake throughput, and shadowed 60 FPS.
A shadowed 60 FPS frame still requires total physics + draw + GS
wait below 16.67 ms, including optimization beyond the normal
impulse kernel.

The batched normal pass retains each SIMD4 group's EE body state
across successive contact points and scatters velocities once per
group at the end of its batch, avoiding repeated gathers/scatters.
Live validation always checks VU0 output finiteness, while full EE
numeric recomputation is sampled to limit per-frame overhead.
