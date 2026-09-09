# Puffysics

Include one file. That file’s functions are the API. Do not include unused
modules. Override `#define`s **before** the include.

| File | Prefix | Pulls |
| --- | --- | --- |
| `puffysics.cuh` | `b3_` | engine; `b3_art.cuh` if `B3_ART_CONTACTS` (default 1) |
| `nbody.cuh` | `nbody_` | particles only |
| `nbody_rigid.cuh` | `b3_nbody_` | `puffysics.cuh` + `nbody.cuh` |
| `ambient.h` | `b3_fluid_` | `puffysics.cuh` |
| `mjcf.h` | `mjcf_` | `puffysics.cuh` + `stl.h` |
| `dat_cloth.h` | `dat_` | `puffysics.cuh` |
| `dat_cloth_gpu.cuh` | `dat_gpu_` | `dat_cloth.h` |
| `b3_loose.cuh` | `b3_loose_` | `puffysics.cuh` |

`box3d.cuh` includes `puffysics.cuh`.

---

## `puffysics.cuh` — rigid world

**Capacities** (before include): `B3_MAX_BODIES` (80), `B3_MAX_SHAPES` (128),
`B3_MAX_CONTACTS` (128), `B3_MAX_JOINTS` (32).

**World values** after `b3_world_init`: write `w->gravity`,
`w->contact_hertz`, `w->contact_damping`, `w->contact_speed`,
`w->restitution_threshold`, `w->max_linear_speed`.

**Body values** on `B3BodyDef` before create, or on `w->bodies[id]` after:
`type` (`B3_STATIC` / `B3_KINEMATIC` / `B3_DYNAMIC`), `position`, `rotation`,
`lin_vel`, `ang_vel`, `linear_damping`, `angular_damping`, `gravity_scale`,
`flags` (`B3_LOCK_LIN_*`, `B3_LOCK_ANG_*`). Each step, write `force` /
`torque` before `b3_step` (they are consumed).

**Shape values** on `B3ShapeDef`: `density`, `friction`, `restitution`,
`rolling`, `category`, `mask`. After create: `w->shapes[id].radius` /
`.half` / those same fields.

| Function | Does |
| --- | --- |
| `b3_default_body()` | Static body at origin, identity rot, `gravity_scale=1`. |
| `b3_default_shape()` | `density=1000`, `friction=0.6`, `restitution=0`, `rolling=0`, all bits collide. |
| `b3_world_init(w)` | Zero world; gravity `(0,-10,0)`, Hertz 30, damping 10, contact speed 3. |
| `b3_create_body(w, &def)` | Append body; returns id. |
| `b3_create_sphere(w, body, c, r, &def)` | Sphere at local `c`, radius `r`. |
| `b3_create_capsule(w, body, half_len, r, &def)` | Capsule along local Y. |
| `b3_create_box(w, body, half, &def)` | Box half-extents. |
| `b3_create_box_local(w, body, half, local_pos, local_rot, &def)` | Offset box. |
| `b3_add_shape(w, body, type, local_pos, local_rot, radius, half, &def)` | Raw shape (`B3_SPHERE` / `B3_CAPSULE` / `B3_BOX`). |
| `b3_finalize_mass(w, body)` | Mass/inertia from shape densities. |
| `b3_set_inertial(w, body, mass, local_com, inertia)` | Override mass, COM, diagonal inertia. |
| `b3_create_weld(w, a, b, anchor_a, anchor_b)` | Weld. Returns `-1` if `B3_REVOLUTE_ONLY`. |
| `b3_create_revolute(w, a, b, anchor_a, anchor_b, axis_a)` | Hinge on `axis_a`. |
| `b3_joint_enable_motor(w, j, on)` / `b3_joint_set_motor(w, j, speed, max_torque)` | Revolute motor. |
| `b3_joint_enable_limit(w, j, on)` / `b3_joint_set_limits(w, j, lo, hi)` | Angle limits (radians). |
| `b3_joint_enable_spring(w, j, on)` / `b3_joint_set_spring(w, j, target, hertz, damping)` | Angle spring. |
| `b3_joint_angle(w, j)` / `b3_joint_speed(w, j)` | Read hinge. |
| `b3_step(w, dt, substeps)` | Advance one env. `dt` and `substeps` are the timestep knobs. |
| `b3_step_kernel<<<blocks,threads>>>(worlds, n, dt, substeps)` | One world per CUDA thread. |

---

## `nbody.cuh` — particle gravity

Does not include the rigid engine. Isolated Plummer-softened all-pairs.
One float pair-force (`nbody_add_source`); energy stays double.

**Values:** `NbodyPoint {x,y,z,mass}` and `NbodyVec {x,y,z}` arrays you own.
`NbodyConfig {gravity, softening}` — `gravity >= 0`, `softening > 0`.
Zero-mass tracers feel gravity but do not source it.

| Function | Does |
| --- | --- |
| `nbody_step(p, v, scratch, n, dt, steps, cfg)` | Kick-drift-kick on host. `scratch` is `n` vectors. |
| `nbody_energy(p, v, n, cfg)` | Host energy (double). |
| `nbody_gpu_init(g, n)` / `nbody_gpu_free(g)` | Device buffers. |
| `nbody_gpu_upload(g, p, v)` / `nbody_gpu_download(g, p, v)` | Host ↔ device. |
| `nbody_gpu_step(g, dt, steps, cfg)` | Same integrator on GPU. After upload, change `p`/`v`/`cfg` then step. |

---

## `nbody_rigid.cuh` — gravity on a `B3World`

Uses `NbodyConfig` from `nbody.cuh`. Point masses at COM; no tidal torque.

**Values:** `cfg.gravity`, `cfg.softening`. Optional `gravitational_mass[body_count]`:
set an entry to let a static body attract; `NULL` uses `1/inv_mass` for dynamics
and 0 for static/kinematic. Zero `w->gravity` unless you also want uniform g.
Write extra `force`/`torque` before `b3_nbody_step`; they are held across substeps.

| Function | Does |
| --- | --- |
| `b3_nbody_apply_forces(w, cfg, masses_or_NULL)` | Add pair gravity to `force` only. |
| `b3_nbody_step(w, dt, substeps, cfg, masses_or_NULL)` | Rebuild gravity each substep, then `b3_step`. |
| `b3_nbody_step_kernel<<<...>>>(worlds, n, dt, substeps, cfg, masses)` | One world per thread. `masses` is world-major, `B3_MAX_BODIES` stride, or `NULL`. |

Do not also put this law in `B3_USER_FORCES`.

---

## `ambient.h` — fluid forces

**Values** on `B3Fluid` (start from `b3_fluid_default()`): `density` (1.2),
`separation_cos` (0 = 90° wake), `mu` (1.8e-5), `drag_scale`, `fric_scale`,
`added_scale` (all 1), `sphere_samples` / `box_face_samples` / `capsule_rings`,
`buoyancy` (1). Sample counts also have `#define B3_FLUID_SPHERE_SAMPLES` (48),
`B3_FLUID_BOX_FACE_SAMPLES` (3), `B3_FLUID_CAPSULE_RINGS` (8) before include.

| Function | Does |
| --- | --- |
| `b3_fluid_default()` | Air-like defaults above. |
| `b3_fluid_state_init(st)` / `b3_fluid_state_reset(st)` | Velocity history for added mass. Reset after teleport. |
| `b3_fluid_step(&fluid, w, &st, h)` | Add buoyancy/drag/skin/added-mass to `force`/`torque`. Call once per substep of length `h`. |

---

## `mjcf.h` — load / spawn XML

**Capacities** (before include): `MJCF_MAX_BODIES`, `MJCF_MAX_JOINTS`,
`MJCF_MAX_GEOMS` (256), `MJCF_MAX_MESHES` (64).

**Values:** XML on disk. After spawn, change pose with the setters below, or
write `w->bodies[s->body_map[i]]`. Collision bits: `MJCF_COL_GROUND`,
`MJCF_COL_SELF`, `MJCF_COL_FEET`, `MJCF_COL_CUBE`, `MJCF_COL_BODY`.

| Function | Does |
| --- | --- |
| `mjcf_load(m, path, z_up)` | Parse MJCF. `z_up=1` remaps Z-up to engine Y-up. 0 on fail (`m->error`). |
| `mjcf_free(m)` | Free STL meshes. |
| `mjcf_spawn(m, w, s, add_ground)` | Create bodies/joints/shapes. `add_ground=1` adds a floor. Maps in `s->body_map` / `s->joint_map`. |
| `mjcf_set_joint_angles(w, m, s, q)` | Set hinge angles from `q[joint_count]`. |
| `mjcf_apply_key(w, m, s, name)` | Apply named keyframe. |
| `mjcf_hold_pose(w, s, n, hertz, damp, torque)` | Spring/motor hold on first `n` joints. |
| `mjcf_sync_body(w, id, pos, rot)` | Teleport one body. |
| `mjcf_rotate_subtree(m, w, s, body, origin, axis, ang)` | Rotate a subtree. |

---

## `dat_cloth.h` — cloth grid + sphere couple

**Values** after `dat_cloth_init` (defaults in parens): `node_mass` (0.2),
`gravity` (−9.81), `damping` (0.03), `relax` (0.6), `iters` (10),
`restitution` (0), `gamma` (0.8, or `#define DAT_GAMMA` before include),
`compliance` (GPU XPBD softness; host `dat_cloth_step` ignores it). Pin nodes
with `c->pinned[k]`. Write `c->pos` / `c->vel` to move
nodes. `W`, `H`, `spacing`, `origin` are set at init.

| Function | Does |
| --- | --- |
| `dat_cloth_init(c, W, H, spacing, origin)` | Alloc `W*H` grid; border pinned. |
| `dat_cloth_free(c)` | Free buffers. |
| `dat_cloth_step(c, h)` | Integrate + stretch constraints. |
| `dat_world_balls(w, bodies, radii, max)` | Collect single-sphere dynamics. |
| `dat_couple(c, w, ball_prev, ball_body, ball_r, nb, h)` | Separate cloth vs those spheres. |
| `dat_couple_velocities(w, ball_before, ball_body, nb, h)` | Push sphere `lin_vel` from COM move. |

---

## `b3_loose.cuh` — runtime-sized free bodies

Train robots stay on `B3World`. This is extra cubes/grains with heap arrays.

**Values** after `b3_loose_init` / `b3_loose_defaults`: `gravity`,
`contact_hertz` (60), `contact_damping` (10), `contact_speed` (3),
`max_linear_speed` (400). Caps are the three `b3_loose_init` arguments.
`#define B3_LOOSE_ITERS` (8) before include.

| Function | Does |
| --- | --- |
| `b3_loose_init(s, cap_bodies, cap_shapes, cap_contacts)` | Alloc. 0 on OOM. |
| `b3_loose_free(s)` | Free. |
| `b3_loose_add_body(s, &def)` / `b3_loose_add_shape(...)` / `b3_loose_finalize_mass(s, body)` | Same meaning as `b3_*`. |
| `b3_loose_add_ground(s)` | Static floor. |
| `b3_loose_spawn_grid(s, nx, ny, nz, size, gap, density, origin)` | Cube lattice. |
| `b3_loose_step(s, dt, substeps)` | Advance loose island. |
| `b3_loose_coupled_step(robot, cubes, xc, xcap, &n_x, dt, substeps)` | Robot `B3World` + loose cubes, shared contacts in `xc`. |
