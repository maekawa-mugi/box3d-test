# PS2 high-resolution native GS experiment

## Deliverables

`sh ps2/build_ee.sh` additionally crosslinks two optional ELFs:

- `ps2/box3d_ps2_samples_pcf_4tap.elf`: **320x224**, native
  GS 3x3 tent using **four bilinear taps**.
- `ps2/box3d_ps2_samples_pcf_progressive.elf`: **640x480 internal
  pixels**, 480p progressive CRT output, the same four-tap filter.

The original validated `ps2/box3d_ps2_samples_pcf.elf` still uses
nine explicit textured-sprite taps for regression comparison.

## Four-tap identity

The old fixed-PCF weights are [6,12,6;12,24,12;6,12,6] (total 96).
This is exactly the separable 3x3 kernel
`[1,2,1]T * [1,2,1] / 16`, multiplied by the old total 96.
Four equal-strength GS TEX1 bilinear samples at UV offsets
`(+/-0.5,+/-0.5)` source texels average to the same 3x3 tent,
because each bilinear tap contains four texels, each with weights
determined by 0.5 in X and Y. Each GS ALPHA FIX=24, four taps total=96.
On the actual PS2 GS, UV 4.4 integer coordinates, 16-bit framebuffer
rounding and clamping can produce small differences at image edges.
This is an *exact algebraic identity for ideal bilinear sampling*, not
a bitwise guarantee of the hardware result. Run
`cc -std=c17 -O2 ps2/pcf_tent4_test.c -lm -o /tmp/pcf_tent4 && /tmp/pcf_tent4`
for the portable reference and memory checks.

## 480p mode and GS VRAM

PS2SDK `graph_set_mode(GRAPH_MODE_NONINTERLACED,
GRAPH_MODE_HDTV_480P,GRAPH_MODE_FRAME,GRAPH_DISABLE)` sets the
progressive HDTV signal. Its 480p timing defines 1440 clock units per
line (720 display sample positions), while the internal framebuffer
is 640 pixels. Therefore the code explicitly programs GS DISPLAY1/2
to horizontal origin 312 and width 1279 GS clocks, displaying exactly
640 pixels at horizontal magnification 2, centered within the nominal
720-pixel active output. This avoids scanout beyond the 640-pixel
buffer.

Allocation without GS page-padding, from 4,194,304 VRAM bytes:

| Surface | Format | Bytes |
| --- | --- | ---: |
| Color front | PSMCT16 | 614,400 |
| Color back | PSMCT16 | 614,400 |
| Depth | PSMZ16 | 614,400 |
| Shadow coverage | PSMCT32 | 1,228,800 |
| **Sum** | | **3,072,000** |

`graph_vram_allocate(...,GRAPH_ALIGN_PAGE)` handles page alignment
and the runtime checks failures. Bilinear sampling is performed from
the PSMCT32 mask while the final subtractive blend lands in the 16-bit
color framebuffer. This means less fine color precision and Z
precision than the low-res PSMCT32/Z32 demos, deliberately traded for
detail. No GSkit, libdraw, C++/ImGui or graphics shader is used.

## Geometry / batching

The original renderer already assembles all GS A+D register writes
into **one packed GIF DMA chain per frame** and waits for GS FINISH
before swapping the read buffer. It does *not* issue one DMA transfer
per triangle. Individual box faces do still require EE perspective
projection and GS command assembly. The progressive variant adds
EE backface culling for convex box hulls before emitting their two
triangles per face (not occlusion culling for arbitrary scenes).
This is only the first part of the eventual geometry-throughput work.

A **VU1 microprogram performing batched vertex transforms is still
pending**. It would require separate compiled VU1 microcode, VIF DMA
setup, vertex/normal stream buffers, and correct double buffering of
VIF/VU1/GIF; no unbuilt VU1 performance gain is claimed.

## Verification and next steps

The first 320x224 fixed-nine-tap PCF ELF is confirmed in PCSX2 by
tester screenshots. **The four-tap and 640x480p variants have not
been crosscompiled or run in PCS​​X2 or on PS2 hardware yet**.
Check the video mode and look for top-left `PCF 4 TAP 480P`,
bottom-right `480P 4TAP`, `BOX3D_GS_VIDEO,480P` on stdout,
actual in-game resolution and eventual frame times. Direct PS2 480p
requires a suitable display/adapter.

Original Box3D desktop sample files contain far more individual
samples than the existing three C17 visual scenes. Full API/rendering
parity across C++, ImGui, ragdolls, mesh primitives, events, joints
and replay is a distinct long-term porting task, not completed by
this display/backface change.

## Expanding the original sample port

The high-resolution-only C17 viewer now cycles **seven** actual physics
scenes in groups Stacking, Shapes, Bodies, Restitution and World. Four
new progressive-only scenes were added after this renderer document
was introduced: Restitution/Varying (its original BOX option with all
40 restitution values), Shapes/Slide Twist, Shapes/Conveyor Belt
(using actual Box3D tangent velocity), and World/Far Stack at its
single-precision origin preset.

The confirmed 320x224 builds still cycle the original three scenes.
This is **not all upstream demos**: see `ps2/SAMPLE_PORTING_STATUS.md`
for an explicit audit of 185 literal `RegisterSample` occurrences
in 21 upstream modules. The 185 count includes repeated registrations
and does not imply there are 185 fully independent C++ classes.

To assess progressive rendering rather than speculate about it,
`BOX3D_GS_GEOMETRY` emits the number of EE-backface-culled and
rendered box faces and the actual total GIF A+D packet qwords. The
GS submission is a *single DMA transfer per frame*, already batched
into 128-bit GIF qwords. **VU1 VIF DMA vertex transformation remains
unimplemented**, and 480p PCSX2/hardware validation is still pending.

## Native GS expanded Kawase and controller update

The separate `box3d_ps2_samples_kawase_wide.elf` (320x224)
adds a disjoint 40x28 eighth-resolution level and the corresponding
ping-pong target, costing **2 x 8192 bytes = 16KiB** of GS 4MiB VRAM
relative to the old two-level pyramid. The path is:

`320x224 -> 160x112 -> 80x56 -> 40x28 (ping-pong) ->
80x56 -> 160x112 -> full-resolution final composite`.

The `box3d_ps2_samples_kawase_wide_progressive.elf` applies the
same **1/8** ratio at 640x480: its stages are 320x240, 160x120,
and 80x60, each with separate source/destination buffers. Its total
page-rounded GS VRAM surface budget is **4,112,384 / 4,194,304 bytes**,
leaving only 81,920 bytes for other GS resources. For the normal
320x224 case, each 40x28 PSMCT32 page requires precisely 8192 bytes.
The Kawase additions involve more GIF register writes, raster passes
and GPU bandwidth; no speedup is implied.

The formerly hardcoded `TEX0` TW/TH for the 320x224 shadow mask is
now calculated using the **ceil(log2(width/height))** of each
active source render target. The correct exponent pair for the
640x480 mask is **10/9**, rather than 9/8. At 40x28 it is
**6/5**. Full logical target dimensions still bound GS CLAMP to
exclude unused padded texture texels.

Both the 480p PCF variant and the two expanded Kawase variants enable
PS2SDK `libpad` navigation. LEFT/RIGHT on port 0/slot 0 switches
immediately to previous/next scene, with edge detection rather than
repeated cycling while held. The six IOP/EE pad module and RPC calls
are based on PS2SDK's own pad demo. If the modules/controller cannot
be opened, the normal 600-frame auto-cycle still works.

The 480p progressive variant (PCF 4tap or expanded Kawase) now has
**eight** scenes including the real
`samples/sample_ragdoll.cpp` `Ragdoll/Box` human:
`CreateHuman()` in the original `shared/human.c` creates 12
Box3D capsule bodies with articulated joint constraints. The native
GS renders axis-aligned-to-body cuboids positioned at the capsule
midpoints rather than full rounded capsule meshes. The old 320px
viewers retain three original scenes, with no fake "full ragdoll"
claim for them.

Portable layout test (no PS2DEV necessary):
```sh
cc -std=c17 -Wall -Wextra -Werror -pedantic \
  ps2/kawase_layout_test.c -o /tmp/kawase_layout
/tmp/kawase_layout
```

**This is new unverified crosscompiler/PCS​​X2 code**, not a
confirmed working 480p ragdoll or extended shadow pipeline. The
first test should verify the left/right pad controls, full-screen
shadow blur without 2x2 lobe artifacts, TEX0 addressing and actual
480p frame output. No CI result is required for this user-requested
work.

## AA1, 12.4 subpixel edges and less-blurry wide shadows (2026-10-10)

A PCSX2 screenshot of the native 480p wide-shadow variant showed
substantial overblur and floor grid lines visible *through* the inclined
opaque slab. These are distinct defects. The grid issue was primarily
caused by using the old Z32 outline depth bias of **+4096** unchanged
in the new Z16 renderer. In Z16, this could move floor grid lines
in front of a closer solid ramp.

The existing **480p** PCF and wide-Kawase renderers (no new ELF) now:
- Compute projected 3D triangle and line XY coordinates in GS **12.4**
  fixed-point. The old implementation truncated 3D positions to
  integer pixels before packing XY2, wasting GS subpixel precision.
  Depth is still interpolated by GS across filled polygons.
- Enable **PRIM.AA1=1 and PRIM.ABE=1** for GS line primitives and
  source-over alpha blending, smoothing the procedural grid and
  the box silhouette outlines. Hardware AA1 applies to *lines*,
  **not antialiased coverage of filled triangle edges**; filled edges
  gain subpixel positioning but remain single-sample GS rasterized.
  No false claim of full-scene MSAA is made.
- Reduce the progressive Z16 line depth bias from **+4096 to +1
  quantum**, preserving ordinary occlusion behind opaque geometry.
  Legacy Z32 versions continue using their original bias.

Only the existing `BOX3D_GS_KAWASE_WIDE` output path additionally
changes the blur composition. It still renders its eighth-resolution
stage, but subtracts the 320x224 or 640x480 **unblurred RAW shadow**
with GS FIX=72 and mixes just four tight (`+/-1px`) wide-blur taps
at FIX=6 each. The total is 96 GS alpha FIX units:
**75% crisp mask + 25% wide blurred mask**. It also reduces the
quarter-to-half upsample offset from 2px to 1px on wide builds.
This keeps contact shadow cores sharp and a low-opacity soft penumbra
without changing the VRAM reservation or generating another ELF.

Build/test the SAME existing targets:
```sh
git pull --ff-only origin experiment/ps2-ee-scalar
sh ps2/build_ee.sh
# ps2/box3d_ps2_samples_kawase_wide_progressive.elf
# ps2/box3d_ps2_samples_pcf_progressive.elf
```

Check that the previously transparent-looking incline now occludes
the checkered grid correctly, that box contours/grid lines are
smoother, and that shadow edges stay in focus while retaining slight
wide softness. Compare PCF against wide Kawase at the same scene.
Check FPS and GS packet overflow logs as before.

**Source-level GS macro and preprocessor checks passed; PS2DEV
cross-link, actual AA1 blending, 480p screenshot and performance
remain unverified in this assistant environment.** The branch is
pushed, but this screenshot belongs to the *pre-fix* executable.

## Fix: GS page-height overlap in 640x480 software-renderer output

The PCSX2 software renderer showed unrelated frame fragments at the
top of the image and black rectangular areas near the bottom in the
480p extended Kawase executable. Inspecting PS2SDK's
`ee/graph/src/graph_vram.c` revealed that `graph_vram_size()`
rounds **framebuffer width and word alignment** but does **not**
round height up to the *physical* GS page height.

GS PSMCT16/PSMZ16 pages have **64x64 pixels** and PSMCT32
pages have **64x32 pixels**, with each page occupying 8192 bytes.
Consequently a logical 640x480 PSMCT16 double buffer needs
a 640x512 allocation for **each** color buffer and the Z buffer.
Similarly 320x240 PSMCT32 half-resolution blur targets require
320x256 allocations. Previously those tail pages could overwrite the
next GS VRAM surface despite the aggregate byte count appearing valid.

The renderer now passes the physical page-rounded height to
`graph_vram_allocate()` for every allocated color/Z/raw/blur surface.
It keeps the original logical width, height, DISPLAY/DISPFB, SCISSOR,
TEX0 TW/TH and all existing ELF names. On 640x480 wide Kawase the
correct allocation totals **4,112,384 / 4,194,304 bytes**, leaving
**81,920 bytes**. The end address is computed from the last allocation
at runtime and emitted as:

```
BOX3D_GS_VRAM,mode=480p_wide,total_bytes=4112384,limit_bytes=4194304,free_bytes=81920
```

`ps2/kawase_layout_test.c` now checks page-height padding and
both the 320x224 and 640x480 actual buffer layouts. No added
ELF variants or rendering features are needed to correct this.

**The 480p software-renderer screenshot precedes this fix; the
revised PS2DEV ELF and PCSX2 render still need tester confirmation.**
