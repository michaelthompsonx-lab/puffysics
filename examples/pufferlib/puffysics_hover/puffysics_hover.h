#ifndef PUFFYSICS_HOVER_ENV_H
#define PUFFYSICS_HOVER_ENV_H

// Native CPU adapter; the CUDA adapter includes this same task/Env definition.
#include "hover_task.h"
typedef float obs_t;
#include "pufferenv.h"

#define ACT_SIZES {HOVER_ACTIONS}
#define NUM_ATNS 1
#define OBS_SIZE HOVER_OBS_SIZE

struct Log {
    float perf;
    float score;
    float episode_return;
    float episode_length;
    float n;
};

struct Env {
    Log log; // PufferLib's log reduction requires this first.
    Agent agents[1];
    int num_agents;
    int tag;
    int boundary_reached;
    unsigned int rng; // Native CPU trainer supplies the environment index here.
    HoverTask task;
};

B3_HD B3_INL void hover_env_reset(Env* env) {
    hover_reset(&env->task);
    hover_observe(&env->task, env->agents[0].observations);
    env->agents[0].rewards[0] = 0;
    env->agents[0].terminals[0] = 0;
}

B3_HD B3_INL void hover_env_step(Env* env) {
    HoverTask* task = &env->task;
    hover_step(task, env->agents[0].actions[0]);
    float reward = hover_reward(task);
    int done = hover_terminal(task);
    task->episode_return += reward;
    if (done) {
        env->log.perf += task->tick >= task->config.horizon;
        env->log.score += task->episode_return;
        env->log.episode_return += task->episode_return;
        env->log.episode_length += (float)task->tick;
        env->log.n += 1;
        hover_reset(task);
    }
    // Native PufferLib convention: terminal reward/done with next reset obs.
    hover_observe(task, env->agents[0].observations);
    env->agents[0].rewards[0] = reward;
    env->agents[0].terminals[0] = (float)done;
}

static HoverConfig hover_env_config(Dict* kwargs) {
    HoverConfig config = hover_default_config();
    DictItem* target = dict_find(kwargs, "target_height");
    if (target && isfinite(target->value)) config.target_height = (float)target->value;
    return config;
}

static uint32_t hover_env_seed(Dict* kwargs) {
    DictItem* seed = dict_find(kwargs, "seed");
    if (!seed || !isfinite(seed->value) || seed->value < 0 || seed->value > UINT32_MAX)
        return 1u;
    return (uint32_t)seed->value;
}

void puf_log(Log* log, Dict* out) {
    dict_set(out, "perf", log->perf);
    dict_set(out, "score", log->score);
    dict_set(out, "episode_return", log->episode_return);
    dict_set(out, "episode_length", log->episode_length);
    dict_set(out, "n", log->n);
}

#if PUF_BACKEND == PUF_CPU
void puf_init(Env* env, Dict* kwargs) {
    env->num_agents = 1;
    env->agents[0].action_mask = NULL;
    env->agents[0].policy = 0;
    // CPU trainer sets rng to the environment index before init.
    hover_init(&env->task, hover_env_config(kwargs),
        hover_env_seed(kwargs) + (uint32_t)env->rng * 747796405u);
}
void puf_reset(Env* env) { hover_env_reset(env); }
void puf_step(Env* env) { hover_env_step(env); }
void puf_close(Env* env) { (void)env; }
void puf_render(Env* env) { (void)env; } // Headless training example.
#endif
#endif
