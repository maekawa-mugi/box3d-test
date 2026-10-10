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
| Benchmark | 24 | 0 | Many large stress scenarios need scene-adaptive mesh and batched VU1 path |
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
| Ragdoll | 5 | 0 | Skeleton, capsules, joints pending |
| Replay | 0 | 0 | No RegisterSample entry; special non-sample subsystem |
| Restitution | 10 | 1 | Varying BOX mode, 40 boxes; sphere option and controls pending |
| Robustness | 4 | 0 | Extreme scene/depth scaling pending |
| Shapes | 11 | 3 | Inclined Plane adapted; Slide Twist, Conveyor Belt added in progressive |
| Stacking | 14 | 1 | Box Stack 40 cubes; others pending |
| Tree | 1 | 0 | Dynamic tree debug visualization pending |
| World | 4 | 1 | Far Stack origin preset; large-offset slider pending |
| **Total** | **185** | **7** | Many categories still missing |

The seven progressive viewer scenes are:
`Stacking/Box Stack` (40 boxes),
`Shapes/Inclined Plane` (adapted),
`Bodies/Spinning Book` (3 angular axes),
`Restitution/Varying` (40 cubes, the original demo's box option),
`Shapes/Slide Twist`,
`Shapes/Conveyor Belt`, and
`World/Far Stack` (floating-point origin preset).
The original validated 320x224 visual ELFs intentionally keep their
three-scene loop and do not change behavior.

All seven use real Box3D physical worlds and Box3D body pose queries,
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
- PS2 controller navigation and menus replacing the desktop ImGui
  inspector, along with camera orbit and scene-control interaction.
- Visual parity tests and performance budgets for each original
  registration, and targeted fallback options for GS 4 MiB VRAM.

The 640x480p 4tap runner is the staging ground for this work, NOT the
claim that all 185 desktop registrations have been implemented.
Progress should be recorded against this table as new scenes are
verified on actual PCSX2 builds.
