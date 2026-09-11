#ifndef PUFFYSICS_HOVER_TASK_H
#define PUFFYSICS_HOVER_TASK_H

// Shared, editable C/CUDA task. Keep this configuration identical in every TU.
#ifndef B3_MAX_BODIES
#define B3_MAX_BODIES 2
#endif
#ifndef B3_MAX_SHAPES
#define B3_MAX_SHAPES 2
#endif
#ifndef B3_MAX_CONTACTS
#define B3_MAX_CONTACTS 4
#endif
#ifndef B3_MAX_JOINTS
#define B3_MAX_JOINTS 2
#endif
#ifndef B3_ART_CONTACTS
#define B3_ART_CONTACTS 0
#endif
#include "b3_batch.cuh"

enum { HOVER_OBS_SIZE = 3, HOVER_ACTIONS = 3 };
typedef struct HoverConfig {
    float target_height;
    float thrust; // Acceleration per action level: 0, thrust, 2*thrust.
    float dt;
    int substeps;
    int horizon;
} HoverConfig;

typedef struct HoverTask {
    B3World world;
    HoverConfig config;
    uint32_t rng; // Per-environment state, preserved across episode resets.
    int tick;
    int body;
    float episode_return;
} HoverTask;

B3_HD B3_INL HoverConfig hover_default_config(void) {
    HoverConfig c = {2.0f, 10.0f, 1.0f / 60.0f, 4, 256};
    return c;
}

B3_HD B3_INL void hover_reset(HoverTask* task) {
    b3_world_init(&task->world); // Clears solver caches and applied forces too.
    task->rng = task->rng * 1664525u + 1013904223u;
    float jitter = (float)(task->rng >> 8) * (1.0f / 16777216.0f) - 0.5f;
    B3BodyDef body = b3_default_body();
    body.type = B3_DYNAMIC;
    body.position = b3_v(0, task->config.target_height + jitter, 0);
    body.linear_damping = 0;
    task->body = b3_create_body(&task->world, &body);
    B3ShapeDef shape = b3_default_shape();
    b3_create_sphere(&task->world, task->body, b3_v(0, 0, 0), 0.25f, &shape);
    b3_finalize_mass(&task->world, task->body);
    task->tick = 0;
    task->episode_return = 0;
}

B3_HD B3_INL void hover_init(HoverTask* task, HoverConfig config, uint32_t seed) {
    task->config = config;
    task->rng = seed;
    hover_reset(task);
}

// Edit this to map your policy's actions to forces, torques, or joint motors.
B3_HD B3_INL void hover_apply_action(HoverTask* task, float action) {
    float level = action >= 1.5f ? 2.0f : (action >= 0.5f ? 1.0f : 0.0f);
    B3Body* body = &task->world.bodies[task->body];
    b3_apply_force(&task->world, task->body,
        b3_v(0, level * task->config.thrust / body->inv_mass, 0));
}

B3_HD B3_INL void hover_observe(const HoverTask* task, float* obs) {
    const B3Body* body = &task->world.bodies[task->body];
    obs[0] = body->position.y - task->config.target_height;
    obs[1] = body->lin_vel.y;
    obs[2] = (float)task->tick / (float)task->config.horizon;
}

B3_HD B3_INL float hover_reward(const HoverTask* task) {
    const B3Body* body = &task->world.bodies[task->body];
    float error = fabsf(body->position.y - task->config.target_height);
    return 1.0f - b3_minf(error + 0.1f * fabsf(body->lin_vel.y), 2.0f);
}

B3_HD B3_INL int hover_terminal(const HoverTask* task) {
    float height = task->world.bodies[task->body].position.y;
    return !isfinite(height) || fabsf(height - task->config.target_height) > 3.0f
        || task->tick >= task->config.horizon;
}

// A raw transition; the caller decides when to reset and how to record it.
B3_HD B3_INL void hover_step(HoverTask* task, float action) {
    hover_apply_action(task, action);
    b3_step(&task->world, task->config.dt, task->config.substeps);
    task->tick++;
}
#endif
