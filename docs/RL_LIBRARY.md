# C and CUDA library use for RL

Puffysics is a header-only library. Environments own their state and call the
physics functions directly. The same C task functions can run in a CPU worker
or in a CUDA kernel. PufferLib, rendering, rewards, actions, and observations
are outside the physics core.

## Start with the editable native environment

`examples/pufferlib/puffysics_hover/` is a complete headless native environment:

| File | Responsibility |
| --- | --- |
| `hover_task.h` | C/CUDA task state, scene creation, action mapping, observation, reward, termination |
| `puffysics_hover.h` | Native CPU `Env`, `Log`, and `puf_*` adapter |
| `puffysics_hover.cu` | Native GPU buffer binding and fused environment kernels |

The task controls a sphere's vertical thrust with three discrete actions. It
rewards staying near a target height. Its small capacities are an example
configuration; size them for your scene. This is a functional integration
template, not a benchmark or evidence of a trained policy.

These adapters target the native `src/pufferenv.h` interface in the local
PufferLib checkout: CPU `puf_init/reset/step/close/log`, and GPU
`puf_vec_create` / `puf_bind_stream` with `PUF_BACKEND`. They do not target the
older `binding.c` interface or Python vectorization. PufferLib variants may
require adapter changes.

To use in a checkout with that API, from the Puffysics directory:

```sh
# Set these paths for your machine.
PUFFERLIB_DIR=/path/to/PufferLib
cp -r examples/pufferlib/puffysics_hover "$PUFFERLIB_DIR/ocean/"
cp puffysics.cuh b3_batch.cuh "$PUFFERLIB_DIR/ocean/puffysics_hover/"
cp examples/pufferlib/puffysics_hover.ini "$PUFFERLIB_DIR/config/"
cd "$PUFFERLIB_DIR"
bash build.sh puffysics_hover hover_cpu --float
bash build.sh puffysics_hover hover_cuda --cu --float
./hover_cuda train
```

The example uses the two headers copied into its directory. You can also use
an installed include directory via PufferLib's `NVCC_EXTRA` compiler flags.
`--float` keeps the example's float observations and trainer
precision identical. Follow your checkout's CUDA/NCCL/raylib setup instructions
before building the trainer. The example has no renderer; `puf_render` is a no-op.

CPU PufferLib assigns an environment index to `Env.rng` before `puf_init` and
binds the agent buffers afterwards. The example combines that index with the
configured seed. CUDA uses the same seed/index calculation. RNG state lives
in each task and advances on reset, independently of worker scheduling.

The adapter writes observations, rewards, and terminals into PufferLib-owned
buffers. It resets finished environments individually and returns the ending
transition's reward/terminal together with the new episode's observation,
following this native trainer's convention. This API has a single terminal
buffer: horizon expiration and physical failure both set it. It does not
provide Gymnasium-style truncation or final-observation buffers. Add those at
the adapter/trainer boundary if your learning algorithm requires them.

The native GPU API used here binds one vector and one rollout stream per
adapter. `num_buffers=1` is required by that trainer. All rollout work stays on
the bound stream, including automatic reset; step does not allocate,
synchronize, or copy through the host. Only initialization and cleanup own
device allocations. The physics batch API itself has no singleton state and
supports multiple independent batches/streams.

## Call and customize ordinary functions

The sample exposes `hover_init`, `hover_reset`, `hover_apply_action`,
`hover_observe`, `hover_reward`, `hover_terminal`, and `hover_step`. Edit these
functions or call the physics primitives from your own equivalents. No base
class or callback registry is required. `HoverConfig` is stored per task;
different worlds can use different target heights, thrust, timestep, substeps,
and horizons. Supply finite values, positive timestep/substeps/horizon, and a
nonnegative thrust. The sample INI exposes target height and seed; add more
configuration keys in `hover_env_config` as needed.

At the physics layer:

```c
// Within a CPU worker or device kernel, using an initialized world:
b3_apply_force(world, body_id, b3_v(0, thrust, 0));
b3_apply_torque(world, body_id, torque);
b3_step(world, dt, substeps);
// Read world->bodies[body_id].position / lin_vel for observations.
```

Forces and torques act over the entire step, across its substeps, then clear.
Reapply them for each step during action repeat. A force law that depends on
the evolving substep state can use `B3_USER_FORCES(world, h)`, defined before
the first core include. Forward-declare the function before including the
core, then define it afterwards when `B3World` is complete. For CUDA the
function and its declaration must be `__host__ __device__`. Hook additions
are consumed once per substep; the original external force is restored until
the next substep. Read current centers as `center + delta_pos` inside that
hook. Do not create/remove bodies there. Keep hook state per world and apply
the same macro configuration in every translation unit.

To change a solver function itself, edit `src/puffysics/*.inl` (or the relevant
optional module) and regenerate the distribution:

```sh
python3 tools/amalgamate.py amalgamate
python3 tools/amalgamate.py check
```

The generated root and `dist/` headers are distribution artifacts. All
translation units must use the same capacities and feature/hook definitions.
The C functions are provided as source; there is no fixed binary ABI to a
precompiled shared library. See [API.md](API.md) for the public/internal boundary.

## Batches, memory, and reset

Include `b3_batch.cuh` for a non-owning view of contiguous or embedded worlds:

```c
typedef struct MyEnv {
    int episode_tick;
    B3World world;
} MyEnv;

// envs is caller-owned, aligned, initialized storage for count MyEnv objects.
B3Batch batch = {envs, sizeof(MyEnv), offsetof(MyEnv, world), count};
int ok = b3_batch_step(batch, 1.0f / 60.0f, 4, NULL);
```

`b3_batch_world(batch, i)` returns the world pointer and is also device-callable.
The CPU batch function is a serial convenience loop. Use PufferLib's worker
scheduling (or your own disjoint batch slices) for CPU parallel execution.
Each world needs exclusive ownership while stepping or resetting.

| Operation | CPU return | CUDA return |
| --- | --- | --- |
| `b3_batch_step(batch, dt, substeps, active)` | 1 success / 0 invalid | `b3_batch_step_gpu(..., stream)` returns `cudaError_t` |
| `b3_batch_restore(batch, initial, reset)` | 1 success / 0 invalid | `b3_batch_restore_gpu(..., stream)` returns `cudaError_t` |

Masks contain one byte per world: nonzero selects that world; NULL selects
all worlds. GPU data, masks, and snapshots must be device-accessible and live
until their queued work finishes. `initial` contains one contiguous `B3World`
snapshot per environment and must not overlap the destination. Restore copies
the entire world, including caches and diagnostics, while leaving enclosing
environment fields untouched. Use snapshots of freshly initialized scenes for
episode reset. Reset episode counters/RNG/task metadata in your environment;
the sample rebuilds its small scene directly to randomize each new episode.

The batch helpers validate counts, stride/offset bounds, size overflow, finite
nonnegative timestep, and positive substeps. Invalid arguments cause no
mutation or kernel launch. Empty batches and zero timestep are valid no-ops.
They cannot verify allocation size or pointer alignment: supply correctly
aligned arrays, not packed structs. Do not alter masks or snapshots while
queued operations read them. CUDA helpers enqueue one thread per world on
the supplied stream. Check returned launch errors and execution errors at
your normal stream synchronization boundary. They support graph capture
without allocation or hidden synchronization.

For specialized fused kernels, call `b3_step(&envs[i].world, ...)` directly
between action mapping and reward/observation. The sample CUDA adapter does
this. The batch helpers are for applications that enqueue these phases
separately. Existing `b3_step` and `b3_step_kernel` remain available.

For particle gravity, [nbody_batch.cuh](../nbody_batch.cuh) exposes SoA arrays
and a caller stream for separate action/observation kernels. Call `invalidate()`
after externally changing positions, masses, or cached accelerations. It is a
distinct module from rigid-body batching.

## Run the standalone examples

```sh
cmake -S . -B build-library -DPUFFYSICS_BUILD_EXAMPLES=ON
cmake --build build-library
./build-library/examples/minimal_cpu
./build-library/examples/batch_cpu
```

These examples need a C compiler and CMake, without PufferLib or a GPU.
Build the native CUDA environment through PufferLib using the commands above.
Tests and benchmarks are not part of the user installation. GPU runtime
validation and training require an accessible NVIDIA device and driver.
