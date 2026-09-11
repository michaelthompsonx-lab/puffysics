# Puffysics conventions

All claims below are verified against `src/puffysics/*.inl`.

- **Frames**: right-handed, **Y-up**. Gravity default `(0,-10,0)`.
- **Units**: unenforced; SI assumed (meters, kg, seconds, radians).
- **Angles**: radians everywhere (limits, springs, motor speed).
- **Quaternions**: `(v.x, v.y, v.z, s)`, vector-first scalar-last;
  identity is `(0,0,0,1)`. Rotations applied as `q * local`.
- **Shapes**: sphere = `radius`; capsule = `radius` + `half.y`
  half-length along local Y; box = `half` half-extents.
- **Density**: mass per unit volume in length units; `density <= 0`
  shapes contribute no mass.
- **Forces**: write `body.force`/`body.torque` before `b3_step`; they are
  consumed (zeroed in finalize) every step.
- **Friction**: geometric mean, `sqrt(fa*fb)`. Rolling: max. Anisotropic
  friction is not supported.
- **Restitution**: max of the pair; applied only when closing speed
  exceeds `restitution_threshold` (default 1.0) and the contact carried
  normal load. `B3_STATIC_RESTITUTION` changes static-pair behavior.
- **Contacts**: speculative margin `B3_SPECULATIVE` (0.02), linear slop
  `B3_LINEAR_SLOP` (0.005), max 4 points per manifold. Normals point
  from shape A to shape B.
- **Joints**: revolute axis is local Z of `local_rot_a` at creation;
  `b3_joint_angle` returns twist about the hinge axis.
- **Timestep**: `b3_step(w, dt, substeps)` runs `max(substeps,1)` substeps
  of `h = dt/subs`. Contact stiffness is stability-capped:
  `hertz = min(contact_hertz, 0.25/h)` with defaults 30 Hz / damping 10.
- **Speed caps**: `max_linear_speed` (400) clamps linear integration;
  rotation clamped at `B3_MAX_ROTATION` (pi/4) per step.
- **Inertia**: diagonal body inertia. Compound finalize rotates each
  shape diagonal by `local_rot` and shifts by parallel axis to the
  combined COM; off-diagonal terms are dropped (exact for axis-aligned
  compounds).
- **Capacities**: compile-time ABI; see `docs/API.md`. Defaults fit an
  articulated agent plus spare cubes (80/128/128/32).
- **GPU layout**: one `B3World` per thread (`b3_step_kernel`). The world
  is AoS; measured kernel cost is 160 regs/thread with ~59 KB of
  per-thread local (stack) traffic on sm_86 — see profiling notes before
  changing batch layout.
