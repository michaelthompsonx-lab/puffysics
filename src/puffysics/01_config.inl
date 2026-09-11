// ==== puffysics 01_config.inl: INTERNAL: includes, B3_HD/B3_INL, capacities, feature flags ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
// Puffysics — PufferLib physics engine (puffer + physics).
// SPDX-License-Identifier: MIT
// Grew out of a Box3D Soft Step port; the solver is no longer a port.
// Scope: kinematics + ABA/Delassus articulations + primitive contacts
// + weld/revolute joints (motor, spring, limits).
// Out of scope: CCD, sleep, meshes, sensors, events, other joints.
//
// One B3World per environment. Host: build bodies/shapes, copy worlds to
// device, launch b3_step_kernel. Device: b3_step(&worlds[env], dt, 4).
// Override B3_MAX_BODIES / B3_MAX_SHAPES / B3_MAX_CONTACTS before include.
// ABI is B3_* / b3_*. Optional modules keep their file-stem prefix:
// nbody.cuh (nbody_), nbody_rigid.cuh (b3_nbody_), ambient.h (b3_fluid_).

#pragma once

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef __CUDACC__
#define B3_HD __host__ __device__
#define B3_INL __forceinline__
#elif defined(__cplusplus)
#define B3_HD
#define B3_INL inline
#else
// Plain C has no extern-inline model here: every TU gets its own copies.
// This is what makes the header consumable from C without a shim .c file.
#define B3_HD
#define B3_INL static inline
#endif

/* PUBLIC SUPPORTED CONFIGURATION (override before include; ABI-stable meaning):
 *   B3_MAX_BODIES / B3_MAX_SHAPES / B3_MAX_CONTACTS / B3_MAX_JOINTS
 *   B3_CONFIG_SIGNATURE (derived; query via b3_config_signature)
 *   B3_JOINT_ITERS / B3_RELAX_ITERS (positive solver iteration counts)
 *   B3_ART_CONTACTS (0 default; 1 opts into articulation contact response)
 *   B3_USER_FORCES (optional per-substep force hook)
 * Everything else in this file is INTERNAL / EXPERIMENTAL / ABLATION:
 *   B3_MERGE_WARM_CACHE, B3_COMPACT_PAIRS, B3_UNCLAMPED_ROTATION,
 *   B3_STATIC_RESTITUTION, B3_PERSISTENT_GS, B3_REUSE_GS_CACHE,
 *   B3_PACKED_INTEGRATE, B3_COUPLED_HINGE, B3_ALTERNATE_JOINT_ORDER,
 *   B3_POLY_ROTATION, B3_RSQRT_MATH, B3_PACKED_GS, B3_REVOLUTE_ONLY,
 *   B3_INTERLEAVE_CONTACTS, B3_SKIP_RESTITUTION, B3_TWIST_APPROX,
 *   B3_CACHE_JOINTS, B3_ABLATE_*. Not a supported public contract.
 * Defaults fit an articulated agent plus extra rigid cubes / stair boxes. */
#ifndef B3_MAX_BODIES
#define B3_MAX_BODIES 80
#endif
#ifndef B3_MAX_SHAPES
#define B3_MAX_SHAPES 128
#endif
#ifndef B3_MAX_CONTACTS
#define B3_MAX_CONTACTS 128
#endif
#ifndef B3_MAX_JOINTS
#define B3_MAX_JOINTS 32
#endif

/* Compile-time ABI fingerprint. B3_MAX_* change sizeof(B3World); every
 * translation unit in one binary must share this value. Query with
 * b3_config_signature() and reject mixed-config worlds. 16-bit fields
 * cover the supported capacity range. */
#define B3_CONFIG_SIGNATURE ( \
    ((unsigned long long)(B3_MAX_BODIES) & 0xffffull) \
  | (((unsigned long long)(B3_MAX_SHAPES) & 0xffffull) << 16) \
  | (((unsigned long long)(B3_MAX_CONTACTS) & 0xffffull) << 32) \
  | (((unsigned long long)(B3_MAX_JOINTS) & 0xffffull) << 48))

// Experimental: faster on some dense-contact batches, but increases register
// pressure and can slow sparse worlds. Benchmark before enabling.
#ifndef B3_MERGE_WARM_CACHE
#define B3_MERGE_WARM_CACHE 0
#endif
/* 1: compact metadata. 2: also iterate eligible shape bitsets and reject
 * separated spheres from cached centers. Both preserve contact order and use
 * the original narrow phase, including its final separation calculation. */
#ifndef B3_COMPACT_PAIRS
#define B3_COMPACT_PAIRS 0
#endif
#if B3_COMPACT_PAIRS >= 2 && B3_MAX_SHAPES > 32
#error "Bitset broad phase requires at most 32 shapes"
#endif
#ifndef B3_UNCLAMPED_ROTATION
#define B3_UNCLAMPED_ROTATION 0
#endif
#ifndef B3_STATIC_RESTITUTION
#define B3_STATIC_RESTITUTION 0
#endif
#ifndef B3_JOINT_ITERS
#define B3_JOINT_ITERS 2
#endif
#ifndef B3_RELAX_ITERS
#define B3_RELAX_ITERS B3_JOINT_ITERS
#endif
#if B3_JOINT_ITERS < 1 || B3_RELAX_ITERS < 1
#error "Solver iteration counts must be positive"
#endif
#ifndef B3_PERSISTENT_GS
#define B3_PERSISTENT_GS 0
#endif
#ifndef B3_REUSE_GS_CACHE
#define B3_REUSE_GS_CACHE 0
#endif
#ifndef B3_PACKED_INTEGRATE
#define B3_PACKED_INTEGRATE 0
#endif
#if B3_PACKED_INTEGRATE && !B3_PERSISTENT_GS
#error "Packed integration requires persistent packed state"
#endif
#ifndef B3_COUPLED_HINGE
#define B3_COUPLED_HINGE 0
#endif
#ifndef B3_ALTERNATE_JOINT_ORDER
#define B3_ALTERNATE_JOINT_ORDER 0
#endif
#if B3_ALTERNATE_JOINT_ORDER && !defined(B3_PACKED_GS)
#error "Alternate joint ordering experiment requires packed solver"
#endif
#if B3_COUPLED_HINGE && (!defined(B3_PACKED_GS) || !defined(B3_REVOLUTE_ONLY))
#error "Coupled hinge experiment requires packed revolute-only solver"
#endif
#if B3_COUPLED_HINGE && (defined(B3_ABLATE_NO_PERP) || defined(B3_ABLATE_NO_POINT))
#error "Coupled hinge experiment cannot ablate one of its coupled blocks"
#endif
#if B3_PERSISTENT_GS && (!defined(B3_PACKED_GS) || !defined(B3_REVOLUTE_ONLY) \
    || !defined(B3_INTERLEAVE_CONTACTS) || !defined(B3_SKIP_RESTITUTION))
#error "Persistent GS requires packed revolute-only, interleaved contacts, and no restitution"
#endif
#ifndef B3_CONNECT_WORDS
#define B3_CONNECT_WORDS ((B3_MAX_BODIES + 63) / 64)
#endif

#define B3_PI 3.14159265359f
#define B3_LINEAR_SLOP 0.005f
#define B3_SPECULATIVE 0.020f
#define B3_MAX_MANIFOLD 4
#define B3_MAX_ROTATION (0.25f * B3_PI)
#define B3_GYRO_ITERS 1
#define B3_MIN_FRICTION_W 1.0e-10f

// Experimental exponential-map integration; benchmark before enabling.
// Changes integration semantics from normalized Euler, not solver iterations.
#ifndef B3_POLY_ROTATION
#define B3_POLY_ROTATION 0
#endif

// B3_TREE_DUAL (experimental packed tree projection) was removed, with
// B3_TREE_SPRINGS and B3_TREE_OUTER_ITERS. The projected GS sweeps below
// are the only joint path. Defining removed knobs to non-default errors.
#if (defined(B3_TREE_DUAL) && B3_TREE_DUAL) \
    || (defined(B3_TREE_SPRINGS) && B3_TREE_SPRINGS) \
    || (defined(B3_TREE_OUTER_ITERS) && B3_TREE_OUTER_ITERS != 2)
#error "B3_TREE_DUAL was removed; use the default joint solver"
#endif

// B3_JOINT_DUAL (experimental block-tridiagonal dual KKT joint solve)
// was removed; the iterated Gauss-Seidel sweeps below are the only
// joint path. Defining B3_JOINT_DUAL to nonzero is now an error.
#if defined(B3_JOINT_DUAL) && B3_JOINT_DUAL
#error "B3_JOINT_DUAL was removed; use the default joint solver"
#endif

// Experimental: fuse sqrt+divide pairs into the reciprocal-square-root
// intrinsic on device (max 2 ulp vs ~1 ulp for the IEEE pair). Touches
// normalization and cone/velocity clamps only; solver reciprocals of
// effective masses stay exact. Benchmark before enabling.
#ifndef B3_RSQRT_MATH
#define B3_RSQRT_MATH 0
#endif

/* Default 0: independent-body contact effective masses with the joint GS
 * solver, for batched RL throughput. Motors, springs, limits and welds stay
 * available. Set 1 before including the core to opt into Delassus contact
 * mass/apply for jointed worlds and the b3_art_* API. That mode changes the
 * contact response and can be substantially more expensive; it is not a
 * drop-in accuracy/performance equivalent. Worlds without joints use the
 * independent-body path in either mode. */
#ifndef B3_ART_CONTACTS
#define B3_ART_CONTACTS 0
#endif

/* Optional per-substep force law. Define before include. The hook may add
 * to externally applied forces: its contributions are consumed once, then
 * the original force/torque are restored for the next substep. Read current
 * poses as center + delta_pos and delta_rot * rotation. No default overhead. */
#ifndef B3_USER_FORCES
#define B3_USER_FORCES(w, h) ((void)0)
#define B3_HAS_USER_FORCES 0
#else
#define B3_HAS_USER_FORCES 1
#endif
