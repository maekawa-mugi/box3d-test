# Box3D upstream sample-port coverage for PS2

The upstream `samples/sample_*.cpp` tree has **21 source files** and
**185 literal `RegisterSample` occurrences** (counting repeated
names, e.g. two Benchmark/Large World entries). The replay file has
no `RegisterSample` registration and is a separate subsystem.
The actual C17 PS2 native-GS visual viewer is not a direct desktop
ImGui/Sokol port. This registry prevents confusing the current subset
with full parity.

| Upstream group | Registered entries | PS2 C17 visual scenes | Port notes |
| --- | ---: | ---: | --- |
| Benchmark | 24 | 4 adapted presets | New Box3D dynamic-grid stress pages: 64/144/256/400 boxes; not literal upstream pyramids |
| Bodies | 12 | 1 | Spinning Book adapted; full body/joint/debug controls pending |
| Character | 4 | 0 | Controller and capsule/mover debug display pending |
| Collision | 14 | 0 | World casts and contact debug geometry pending |
| Compound | 6 | 0 | Compound shape mesh extraction/draw pending |
| Continuous | 10 | 0 | Bullet/TOI/mesh models pending |
| Determinism | 4 | 0 | Input/replay/verifier pending |
| Events | 7 | 0 | Runtime contact-event visualization pending |
| Geometry | 5 | 0 | Generic hull/mesh renderer pending |
| Issues | 13 | 0 | Repro harness and debug drawing pending |
| Joints | 17 | 0 | Joint frame/constraint shapes pending |
| Manifold | 11 | 0 | Contact manifold/normal visualizer pending |
| Mesh | 9 | 0 | Triangle mesh, height field, streaming pending |
| Ragdoll | 5 | 1 | Original shared/human.c 12-bone articulated Box demo; GS capsule proxies still simplified |
| Replay | 0 | 0 | No RegisterSample entry; special non-sample subsystem |
| Restitution | 10 | 1 | Varying BOX mode, 40 boxes; sphere option and controls pending |
| Robustness | 4 | 0 | Extreme scene/depth scaling pending |
| Shapes | 11 | 3 | Inclined Plane adapted; Slide Twist, Conveyor Belt added in progressive |
| Stacking | 14 | 1 | Box Stack 40 cubes; others pending |
| Tree | 1 | 0 | Dynamic tree debug visualization pending |
| World | 4 | 1 | Far Stack origin preset; large-offset slider pending |
| **Total** | **185** | **12 pages (8 visual + 4 stress)** | Only eight visual ports correspond to upstream sample families; stress grids are adapted variants |

The unified 640x480p viewer now contains twelve left/right paginated scenes. Its original eight visual scenes are:
`Stacking/Box Stack` (40 boxes),
`Shapes/Inclined Plane` (adapted),
`Bodies/Spinning Book` (3 angular axes),
`Restitution/Varying` (40 cubes, the original demo's box option),
`Shapes/Slide Twist`,
`Shapes/Conveyor Belt`, `World/Far Stack` (floating-point origin preset), and
`Ragdoll/Box` (original `CreateHuman()`, 12 articulated capsule
bodies and joint constraints; approximate cuboid visual proxies).
The original validated 320x224 visual ELFs intentionally keep their
three-scene loop and do not change behavior.

All eight use real Box3D physical worlds and Box3D body pose queries,
but *scene settings/cameras/materials may be adapted*. Full fidelity
requires each original geometry, controls and dynamic updates to be
compared with its source; some demos must draw non-rigid debug data
like joints, contact manifolds and ray/cast sweeps, and cannot be
ported faithfully by rendering only rigid cuboids.

## Remaining engineering prerequisites

- Native GS triangle meshes and hull faces beyond boxes, persistent
  material/per-face buffers, texture upload and chunked GIF packet DMA.
- A separate PS2SDK VU1/VIF microprogram to transform vertex batches
  with double-buffered VIF DMA and a verified VU1/GS GIF output protocol.
  Currently **no VU1 vertex transforms are integrated**. The
  experimental VU0/macro-mode physics path is a separate concern.
- Lighting/face culling: the **640x480p** variant already skips
  backfacing box faces on EE before emitting their two GS triangles.
  The renderer already packs **the whole frame's GIF A+D writes into
  a single DMA submission**, not one DMA per triangle. It still
  spends EE cycles generating/projecting vertices individually.
- PS2 controller navigation on the updated progressive and expanded
  Kawase builds: LEFT/RIGHT switches scenes immediately. Full menus,
  camera orbit, and per-scene desktop ImGui controls remain pending.
- Visual parity tests and performance budgets for each original
  registration, and targeted fallback options for GS 4 MiB VRAM.

The 640x480p 4tap runner is the staging ground for this work, NOT the
claim that all 185 desktop registrations have been implemented.
Progress should be recorded against this table as new scenes are
verified on actual PCSX2 builds.

## Unified 480p PCSS runtime and box stress scenarios

This viewer **only** builds `ps2/box3d_ps2.elf` with
`BOX3D_PS2_UNIFIED`: 640x480p progressive GS, native PCSS-style
contact-hardening shadows using bilinear 4-tap PCF at contact
and 21 subtexel-filtered taps for raised casters, and a single
D-pad LEFT/RIGHT scene pager. No separate PCF, PCSS or Kawase
sample ELFs are generated. Previous legacy sample ELF artifacts
may still be present on disk after pulling; do not mistake those
for new builds.

After the eight visual scenes, there are four *scaled, adapted*
Box3D broadphase/solver stress presets inspired by the upstream
Benchmark/Large Pyramid and Wide Pyramid workloads:

| Scene page | Dynamic boxes | Arrangement |
| --- | ---: | --- |
| BENCH 64 BOX | 64 | 8x8 grid, staggered drop heights |
| BENCH 144 BOX | 144 | 12x12 grid, staggered drop heights |
| BENCH 256 BOX | 256 | 16x16 grid, staggered drop heights |
| BENCH 400 BOX | 400 | 20x20 grid, staggered drop heights |

These are *not exact* ports of the original desktop C++ Benchmark
classes; their benefit is an explicit scalable workload under the
same real physics and GS PCSS renderer. On stress pages only, tiny
GS outline strokes are omitted to stay within the 15-bit GIF NLOOP
packet maximum; opaque triangles and physics are still processed.
No extra ELF is needed for any stress level. The rasterizer never
claims VU1 transforms have already been implemented.

Telemetry printed every 60 frames:
`BOX3D_PERF,scene=N,frame=...,bodies=...,physics_ms=...,
build_ms=...,gpu_wait_ms=...,vsync_idle_ms=...,work_pct=...,
gif_qwords=...`.
The HUD CPU+GPU USE is the fraction of the **nominal 16.67ms / 60Hz
EE Count budget spent doing work**, excluding deliberate VSync idle.
Note that this is an approximate *combined elapsed* utilization and
includes some CPU waiting for GS completion, not a hardware GPU
percent meter. A PCSX2 display locked at 60fps does not prove that
`work_pct` is small.
