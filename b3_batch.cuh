#ifndef B3_BATCH_CUH
#define B3_BATCH_CUH

// Non-owning batches of independent worlds, including worlds embedded in Env.
// No allocation, global state, threading runtime, or PufferLib dependency.
#include <stddef.h>
#include "puffysics.cuh"
#ifdef __CUDACC__
#include <cuda_runtime.h>
#endif

typedef struct B3Batch {
    void* data;
    size_t stride;       // sizeof(B3World), or sizeof(your Env)
    size_t world_offset; // 0, or offsetof(your Env, world)
    int count;
} B3Batch;

// The caller supplies aligned storage large enough for count * stride bytes.
// Empty batches may have NULL data. Validation never dereferences data.
B3_HD B3_INL int b3_batch_valid(B3Batch batch) {
    if (batch.count < 0) return 0;
    if (batch.count == 0) return 1;
    return batch.data != NULL && batch.stride >= sizeof(B3World)
        && batch.world_offset <= batch.stride - sizeof(B3World)
        && (size_t)batch.count <= SIZE_MAX / batch.stride;
}

// Unchecked hot-path access; batch and index must be valid.
B3_HD B3_INL B3World* b3_batch_world(B3Batch batch, int index) {
    return (B3World*)((unsigned char*)batch.data
        + (size_t)index * batch.stride + batch.world_offset);
}

// CPU batch convenience loop. PufferLib may instead call b3_step directly
// in its own workers. Disjoint worlds can be stepped concurrently.
// active == NULL steps all worlds; otherwise nonzero bytes select worlds.
// Returns 1 on success, 0 on invalid arguments, without partial mutation.
B3_INL int b3_batch_step(B3Batch batch, float dt, int substeps,
        const unsigned char* active) {
    if (!b3_batch_valid(batch) || !isfinite(dt) || dt < 0 || substeps < 1)
        return 0;
    if (dt == 0) return 1;
    for (int i = 0; i < batch.count; i++) {
        if (!active || active[i]) b3_step(b3_batch_world(batch, i), dt, substeps);
    }
    return 1;
}

// Restore selected worlds from count contiguous, immutable world snapshots.
// Snapshots must not overlap destination storage. Copy includes caches and
// diagnostics; capture fresh scenes for episode resets. Env metadata stays put.
B3_INL int b3_batch_restore(B3Batch batch, const B3World* initial,
        const unsigned char* reset) {
    if (!b3_batch_valid(batch) || (batch.count && !initial)) return 0;
    for (int i = 0; i < batch.count; i++) {
        if (!reset || reset[i]) *b3_batch_world(batch, i) = initial[i];
    }
    return 1;
}

#ifdef __CUDACC__
static __global__ void b3_batch_step_kernel(B3Batch batch, float dt,
        int substeps, const unsigned char* active) {
    size_t i = (size_t)blockIdx.x * blockDim.x + threadIdx.x;
    if (i < (size_t)batch.count && (!active || active[i]))
        b3_step(b3_batch_world(batch, (int)i), dt, substeps);
}

static __global__ void b3_batch_restore_kernel(B3Batch batch,
        const B3World* initial, const unsigned char* reset) {
    size_t i = (size_t)blockIdx.x * blockDim.x + threadIdx.x;
    if (i < (size_t)batch.count && (!reset || reset[i]))
        *b3_batch_world(batch, (int)i) = initial[i];
}

// CUDA pointers, including masks/snapshots, must be device-accessible.
// Enqueues on stream without allocation or synchronization; safe in capture.
// Return value reports validation/launch errors. Check execution errors at the
// caller's synchronization boundary. Separate batches may use separate streams.
B3_INL cudaError_t b3_batch_step_gpu(B3Batch batch, float dt, int substeps,
        const unsigned char* active, cudaStream_t stream) {
    if (!b3_batch_valid(batch) || !isfinite(dt) || dt < 0 || substeps < 1)
        return cudaErrorInvalidValue;
    if (!batch.count || dt == 0) return cudaSuccess;
    unsigned int blocks = ((unsigned int)batch.count + 31u) / 32u;
    b3_batch_step_kernel<<<blocks, 32, 0, stream>>>(batch, dt, substeps, active);
    return cudaGetLastError();
}

B3_INL cudaError_t b3_batch_restore_gpu(B3Batch batch,
        const B3World* initial, const unsigned char* reset, cudaStream_t stream) {
    if (!b3_batch_valid(batch) || (batch.count && !initial))
        return cudaErrorInvalidValue;
    if (!batch.count) return cudaSuccess;
    unsigned int blocks = ((unsigned int)batch.count + 31u) / 32u;
    b3_batch_restore_kernel<<<blocks, 32, 0, stream>>>(batch, initial, reset);
    return cudaGetLastError();
}
#endif
#endif
