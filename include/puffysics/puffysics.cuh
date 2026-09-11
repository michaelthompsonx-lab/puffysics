#pragma once
#ifndef PUFFYSICS_CORE_INCLUDED
#define PUFFYSICS_CORE_INCLUDED
// Puffysics development entry point (modular split of puffysics.cuh).
//
// PUBLIC API (stable surface; bodies below are INTERNAL implementation):
//   world: b3_world_init, b3_create_body, b3_add_shape, b3_create_sphere,
//     b3_create_capsule, b3_create_box, b3_create_box_local,
//     b3_finalize_mass, b3_set_inertial, b3_body/shape_capacity,
//     b3_clear_errors, b3_step_stats, b3_world_error,
//     b3_config_signature, b3_config_matches
//   joints: b3_create_weld, b3_create_revolute, b3_joint_enable/set_motor,
//     b3_joint_enable/set_limits, b3_joint_enable/set_spring,
//     b3_joint_angle, b3_joint_speed, b3_joint_capacity
//   step: b3_step, b3_step_kernel, b3_find_contacts, b3_contact_capacity
//   mass: b3_shape_mass
// Topics below map to src/puffysics/*.inl; do not include .inl directly.
#define PUFFYSICS_SPLIT_OK
#include "../../src/puffysics/01_config.inl"
#include "../../src/puffysics/02_types.inl"
#include "../../src/puffysics/03_math.inl"
#include "../../src/puffysics/04_world.inl"
#include "../../src/puffysics/05_collision.inl"
#include "../../src/puffysics/06_joints.inl"
#include "../../src/puffysics/07_packed.inl"
#include "../../src/puffysics/08_contacts.inl"
#include "../../src/puffysics/09_step.inl"
#endif // PUFFYSICS_CORE_INCLUDED
