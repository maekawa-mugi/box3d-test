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
