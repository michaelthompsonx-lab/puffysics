#define PUF_BACKEND PUF_GPU
#include <cuda_runtime.h>
typedef float obs_t; // Also required literally in this file by native build.sh.
#include "puffysics_hover.h"

// The local native PufferLib GPU API binds one vector/stream per adapter.
// This adapter state belongs to the trainer, never to the physics library.
static struct {
    Env* envs;
    int count;
    cudaStream_t stream;
} hover_gpu;

static void hover_cuda_check(cudaError_t error) {
    if (error != cudaSuccess) {
        fprintf(stderr, "puffysics_hover: %s\n", cudaGetErrorString(error));
        exit(1); // Example executable policy; library functions return errors.
    }
}

static __global__ void hover_init_kernel(Env* envs, int n, HoverConfig config,
        uint32_t seed, float* obs, float* actions, float* rewards, float* terminals) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    Env* env = &envs[i];
    env->num_agents = 1;
    env->tag = i;
    env->agents[0].observations = obs + (size_t)i * HOVER_OBS_SIZE;
    env->agents[0].actions = actions + i;
    env->agents[0].rewards = rewards + i;
    env->agents[0].terminals = terminals + i;
    hover_init(&env->task, config, seed + (uint32_t)i * 747796405u);
}

static __global__ void hover_reset_kernel(Env* envs, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) hover_env_reset(&envs[i]);
}

static __global__ void hover_step_kernel(Env* envs, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) hover_env_step(&envs[i]);
}

Env* puf_vec_create(int n, Dict* kwargs, obs_t* obs, float* actions,
        float* rewards, float* terminals) {
    if (n <= 0 || hover_gpu.envs) {
        fprintf(stderr, "puffysics_hover requires one nonempty GPU vector\n");
        exit(1);
    }
    hover_gpu.count = n;
    hover_cuda_check(cudaMalloc((void**)&hover_gpu.envs, (size_t)n * sizeof(Env)));
    hover_cuda_check(cudaMemset(hover_gpu.envs, 0, (size_t)n * sizeof(Env)));
    hover_init_kernel<<<((unsigned int)n + 31u) / 32u, 32>>>(hover_gpu.envs, n,
        hover_env_config(kwargs), hover_env_seed(kwargs), obs, actions, rewards, terminals);
    hover_cuda_check(cudaGetLastError());
    hover_cuda_check(cudaDeviceSynchronize()); // Startup only, before capture.
    return hover_gpu.envs;
}

void puf_bind_stream(cudaStream_t stream) { hover_gpu.stream = stream; }
void puf_init(Env*, Dict*) {}
void puf_reset(Env* envs) {
    hover_reset_kernel<<<((unsigned int)hover_gpu.count + 31u) / 32u, 32, 0,
        hover_gpu.stream>>>(envs, hover_gpu.count);
    hover_cuda_check(cudaGetLastError());
}
void puf_step(Env* envs) {
    hover_step_kernel<<<((unsigned int)hover_gpu.count + 31u) / 32u, 32, 0,
        hover_gpu.stream>>>(envs, hover_gpu.count);
    hover_cuda_check(cudaGetLastError());
}
void puf_close(Env* envs) {
    hover_cuda_check(cudaFree(envs));
    hover_gpu.envs = NULL;
    hover_gpu.count = 0;
    hover_gpu.stream = 0;
}
void puf_render(Env*) {} // Headless training example.
