# Puffysics API

## Tiers

**Stable public API** (ordinary users). Definitions, lifecycle, stepping,
queries — see `include/puffysics/puffysics.cuh` PUBLIC banner for the entry list:

- config: `b3_default_body`, `b3_default_shape`, `b3_world_init`
- create: `b3_create_body`, `b3_add_shape`, `b3_create_sphere`,
  `b3_create_capsule`, `b3_create_box`, `b3_create_box_local`,
  `b3_finalize_mass`, `b3_set_inertial`
- joints: `b3_create_weld`, `b3_create_revolute`,
  `b3_joint_enable/set_motor`, `b3_joint_enable/set_limits`,
  `b3_joint_enable/set_spring`, `b3_joint_angle`, `b3_joint_speed`
- run: `b3_step`, `b3_step_kernel`, `b3_find_contacts`, `b3_shape_mass`
- forces: `b3_apply_force`, `b3_apply_torque`, `b3_apply_linear_impulse`
- batches (`b3_batch.cuh`): `B3Batch`, `b3_batch_valid`, `b3_batch_world`,
  `b3_batch_step`, `b3_batch_restore`, `b3_batch_step_gpu`, `b3_batch_restore_gpu`
- capacity/errors: `b3_*_capacity`, `b3_config_signature`,
  `b3_config_matches`, `b3_clear_errors`, `b3_step_stats`,
  `b3_world_error`
- modules keep their prefixes: `nbody_*`, `b3_nbody_*`, `b3_fluid_*`,
  `mjcf_*`, `dat_*`, `dat_gpu_*`, `b3_loose_*`

**Low-level/advanced API** (CUDA/RL performance users): direct `B3World`
field access, device-callable helpers, `B3_*` compile-time flags. Layout
is documented but not frozen; read `docs/CONVENTIONS.md` before depending
on it.

No setters/getters wrap hot fields: direct access is the interface, and
wrappers that protect no invariant are rejected here on purpose.

See [RL_LIBRARY.md](RL_LIBRARY.md) for C/CUDA customization, batch ownership,
masked reset semantics, caller streams, and the native PufferLib example.

## Lifecycle and capacity semantics

- Worlds are **fixed-capacity, append-only**. There is no body/shape/joint
  removal: that is the GPU optimization, not a missing feature. Rebuild
  or re-`init` a world to clear it (`b3_world_init` zeroes everything).
- Creation past capacity returns `-1` (`NULL` from `b3_add_joint`) and
  sets a sticky flag (`body/shape/joint_overflow`). Nothing is written
  out of bounds, with or without `NDEBUG`.
- Contact storage full: pairs with a real manifold are counted in
  `contacts_dropped` and raise `contact_overflow` instead of vanishing.
- Flags are sticky: inspect via `b3_step_stats`/`b3_world_error`, clear
  with `b3_clear_errors` (content untouched).
- Capacities are **compile-time ABI**: `B3_MAX_BODIES/SHAPES/CONTACTS/
  JOINTS` change `sizeof(B3World)`. Every TU in one binary must use one
  build configuration. `b3_config_signature()` packs those four values;
  `b3_config_matches(sig)` rejects a mixed-config world. Shrink them for
  occupancy when scenes allow.
- `b3_set_inertial` overrides mass/COM/diagonal inertia outright (use for
  MJCF armature and friends). `b3_finalize_mass` derives them from shape
  densities with rotation-aware diagonal inertia (parallel-axis included;
  off-diagonals dropped — see `docs/CONVENTIONS.md`).

## Errors

Library calls return failure (`0`, `-1`, `NULL`, `cudaError_t`) — never
`abort()`. Viewers (`play_*`) may exit with a message on fatal GPU
failure; that is viewer policy, not library behavior. `assert()` guards
internal invariants in debug builds only.

## Naming

C prefixes are subsystem namespaces, kept stable: `b3_` core,
`b3_art_` (opt in with `B3_ART_CONTACTS=1`), `b3_nbody_`, `b3_fluid_`,
`b3_loose_`, `nbody_`, `dat_`/`dat_gpu_`, `mjcf_`, `stl_`. No global
rename is planned; any future `Puffysics`-prefixed API ships beside the
old names with a migration note, not as a flag day.

## Determinism

Same inputs + same build config + same entry point = same outputs. No
RNG, no wall-clock, fixed iteration counts. CPU/GPU agreement is
approximate (device `rsqrtf` vs host IEEE divide); tolerances live in the
tests, currently `1e-3` worst-case drift on the n-body probe.

## Compile-time configuration

**Public supported** (override before include): `B3_MAX_BODIES`,
`B3_MAX_SHAPES`, `B3_MAX_CONTACTS`, `B3_MAX_JOINTS`, `B3_JOINT_ITERS`,
`B3_RELAX_ITERS`, `B3_ART_CONTACTS`, `B3_USER_FORCES`. Capacities are
ABI; `b3_config_signature()` reports them.

`B3_ART_CONTACTS` defaults to `0`: independent-body contact effective masses
with the joint solver. Set it to `1` before including the core to enable
articulation-aware contact response and the `b3_art_*` functions, including
`b3_art_step`. This was the default in earlier releases. Changing the mode
can change trajectories and throughput; rebuild every translation unit with
the same definition. The ordinary `b3_step` API is available in either mode.

**Internal / experimental / ablation** (not a supported contract;
`#error` on removed knobs): `B3_MERGE_WARM_CACHE`, `B3_COMPACT_PAIRS`,
`B3_UNCLAMPED_ROTATION`, `B3_STATIC_RESTITUTION`, `B3_PERSISTENT_GS`,
`B3_REUSE_GS_CACHE`, `B3_PACKED_INTEGRATE`, `B3_COUPLED_HINGE`,
`B3_ALTERNATE_JOINT_ORDER`, `B3_POLY_ROTATION`, `B3_RSQRT_MATH`,
`B3_PACKED_GS`, `B3_REVOLUTE_ONLY`, `B3_INTERLEAVE_CONTACTS`,
`B3_SKIP_RESTITUTION`, `B3_TWIST_APPROX`, `B3_CACHE_JOINTS`,
`B3_ABLATE_*`.

## Source distribution

The library distribution includes the headers, editable sources, installation
files, and examples. Tests and benchmarks are maintained separately from the
user installation and are disabled by default in CMake.
