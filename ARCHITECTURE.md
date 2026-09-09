# CUDA architecture decision

Decision date: 2026-09-05. This is the selected design direction, not a claim that
an unimplemented backend has won a benchmark. Existing measurements are in
[the investigation results](results/2026-09-05/FINDINGS.md).

Build a data-oriented CUDA runtime with topology batches, explicit device memory
pools, and execution selected by workload. Share collision geometry, physical
models, identities, diagnostics, and the environment API. Support separate storage
and scheduling policies for small repeated worlds and large irregular worlds.

## What the measurements justify

* The generic one-thread-per-world engine is useful for small batches of simple
  worlds, but body throughput falls sharply with increasing bodies per world.
* Increasing capacities grows each world from 102,960 to 295,632 bytes and each
  stepping thread's stack from 13,520 to 41,472 bytes in the measured builds.
* The active loose-scene contact solver is serial on the GPU. Existing colored
  kernels are experimental infrastructure, not proof of a scalable solver.
* More substeps alone did not monotonically reduce the measured final errors.
* A lower-complexity cache lookup improved some workloads and regressed others.

These results support separating data from execution. They do not establish the
best tile size, the crossover between thread/warp/block execution, or whether
memory traffic or collision work dominates each case. Those require prototypes.

## Data ownership

| Component | Contents | Lifetime |
| --- | --- | --- |
| Model | Geometry, topology, default mass/material parameters, joint frames, eligible collision pairs | Shared by a topology batch |
| World parameters | Randomized mass, inertia, friction, controls and other overrides | Per world; reset/update invalidates affected caches |
| State | Poses, velocities, forces; articulation coordinates when supported | Persistent device arrays |
| Contact cache | Stable pair/feature identities, impulses or material-specific history | Persistent, with generation checks on reset/reuse |
| Workspace | AABBs, candidate pairs, manifolds, constraint rows, colors, scan buffers | Preallocated device pools, reused each step |
| Diagnostics | Required capacities, overflow, invalid states, solver error, statistics | Device-resident with optional asynchronous readback |

Geometry sharing must not prevent per-world randomization. Cache validity follows
shape generations, material changes, timestep changes, and model revisions.
The long-term GPU representation will not embed maximum-sized body/contact arrays
inside each world. Keep the existing B3World path as a reference during migration.

Use stable `(world_id, body_id)` handles, resolved through layout-aware views.
The RL interface exposes device tensor views and strides. Choose each batch's
storage policy at creation; do not transpose the entire state every timestep or
maintain two competing authoritative copies. Explicit packing for observations
must be included in end-to-end benchmarks.

## Storage and execution paths

**Small repeated worlds.** Bucket compatible topologies and capacities. Prototype
component arrays with the world lane contiguous, conceptually
`state[field][body][world]`, optionally tiled in the world dimension. Neighboring
lanes then access the same field/body in neighboring worlds. Initially preserve
the current constraint order and one-lane-per-world scheduling to isolate the
layout effect. Compare fused kernels and staged kernels without large automatic
arrays. Warp/block-per-world is a competing prototype, not a required default.

**Large or irregular worlds.** Use flat component arrays with bodies contiguous,
world offsets, and compact pair/constraint queues. Parallelize integration over
bodies, collision over pairs, and solving over conflict-free constraint batches.
Island scheduling must not assume that one connected pile fits in one block;
large islands require multiple blocks and explicit synchronization boundaries.

One logical API can cover both paths, but they need different physical layouts
to coalesce their different access patterns. This follows CUDA's documented
importance of adjacent-lane memory accesses; it is not a measured layout speedup
for Box3D yet. [CUDA best practices](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html)

## Collision pipeline

1. Update transforms and each shape AABB once.
2. Generate candidates using a batch-selected broad phase: eligible-pair lists
   for small fixed models; a sorted grid for similarly sized grains; a hierarchy
   for strongly mixed sizes and terrain. Static large surfaces must not be
   replicated into an unbounded number of grid cells.
3. Compact candidates and, when worthwhile, group by shape-pair type. Small
   fixed models can use pre-grouped lists without a runtime sorting pass.
4. Produce bounded manifolds, report required capacity, and match persistent
   contact features. Clipped polygon indices alone are insufficient as a general
   stable identity; track originating geometric features or validated local anchors.
5. Prepare constraint data into compact solver buffers.

Always include world identity in pair/grid keys. Collision masks and topology
changes invalidate precomputed eligibility. A grid must cover every cell overlapped
by an AABB (or use a proven neighbor scheme), deduplicate pairs, and handle size
variation correctly. Grid versus hierarchy remains workload-dependent.
[NVIDIA broad-phase approaches](https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-32-broad-phase-collision-detection-cuda)

## Dynamics and numerical accuracy

Retain Soft Step as the initial general rigid-body numerical baseline. Prototype
colored Gauss-Seidel for larger contact sets: each color has no shared writable
dynamic bodies, including conflicts between joints and contacts. Immutable static
bodies need not create coloring conflicts. Uncolored constraints must be handled
explicitly; reaching a color limit must never silently discard them.

Compare block-Jacobi/delta accumulation as an alternative at matched physical
error. Use a frozen input state and a separate reduction/apply phase; arbitrary
atomic updates to live velocities are not equivalent to Gauss-Seidel. Keep stable
ordering and deterministic reductions available for reproducibility, and measure
their cost. Avoid a mandatory dense matrix scaling quadratically with contacts.

For high-fidelity tree-structured robots, reserve a reduced-coordinate articulation
backend: root state plus joint positions/velocities, with derived link transforms.
This enforces the represented joint kinematics rather than repeatedly correcting
independent link poses. Closed loops, limits, drives, and contact still require
appropriate constraints. PhysX documents this representation and its robotics
advantages; it does not prove a performance result for our implementation.
[PhysX articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/Articulations.html)

Articulation support must include coupled response: an impulse on one link can
affect the entire mechanism. Its contact effective mass cannot use the existing
independent-body inverse inertia formula. Define a solver response interface for
rigid bodies and articulations before implementing that backend. Do not integrate
an articulation and nearby grains independently and treat the resulting contact
as one-way coupling.

Keep hard-contact impulse models distinct from compliant granular laws. The latter
may need tangential displacement history, material calibration, and different
integration timestep limits. Atomistic and deformable simulations require further
models. They can share runtime primitives without claiming identical physics.

## GPU lifecycle and RL integration

Allocate pools at batch creation. Normal stepping uses device counts, fixed launch
topology where possible, and CUDA Graph replay. Pool exhaustion sets explicit
status and required sizes; growth/rebuild happens at a safe boundary outside graph
capture. An overflowed transition is invalid for RL consumption. Retry requires
restoring pre-step state/caches or detecting capacity exhaustion before mutation.
Avoid per-color host readbacks and per-step device heap allocation.

Expose stream-aware `step`, indexed `reset`, action and observation interfaces.
Reset invalidates that world's contact history and articulation caches. Mixed
topologies are grouped once where possible; avoid sorting all worlds every step.
Optional diagnostics should not force synchronization into the normal rollout.

## Implementation order and acceptance gates

1. Add explicit capacity/validity telemetry and trajectory-wide accuracy checks.
   Freeze scene seeds, timestep, material laws, and acceptance limits per workload.
2. Implement model/state separation and the world-contiguous small-batch prototype.
   Keep solver arithmetic/order fixed for the first A/B test. Compare original
   B3World, world-contiguous storage, and a warp/block prototype with identical work.
3. Add the large-world flat storage, grid broad phase, and colored contact solver.
   Test broad-phase completeness against exhaustive pairs on small randomized cases.
4. Implement reduced-coordinate articulation response and mixed-contact coupling.
5. Select dispatch rules from measured error/throughput/memory frontiers, then add
   further shape/material models and generalization tests.

Benchmark both phase timings and complete RL rollouts, including reset/observation
costs, heterogeneous contacts, and domain randomization. Record peak memory and
constraint counts, not only final counts. Compare penetration normalized by size,
joint drift, isolated-system conservation, contact stability, and long-run failure
rates. A faster result that violates its workload's accuracy bound is not a win.

Decided now: data ownership, explicit capacity handling, two storage policies,
shared collision/identity contracts, and space for coupled articulation dynamics.
Deferred to measurements: thread/warp/block thresholds, grid/hierarchy crossover,
kernel fusion, solver iteration policies, and mixed-precision benefits.
