# Puffysics

Puffysics is a small C/CUDA physics library for building parallel simulation
environments in [PufferLib](https://github.com/PufferAI/PufferLib).

Each environment owns its physics state. Call the same functions from a CPU
worker or a CUDA kernel, and customize the scene, controls, observations, and
rewards for your task. The library is header-only and has no graphics dependency.

## Get running in PufferLib

The included **hover** example teaches an agent to hold a sphere at a target
height using three thrust actions. It includes a shared task implementation,
native CPU and CUDA adapters, and a training config.

You need a working native PufferLib checkout with `src/pufferenv.h` and
`build.sh`. The CUDA adapter uses `PUF_BACKEND`, `puf_vec_create`, and
`puf_bind_stream`; older PufferLib versions may require adapter changes.
Set up PufferLib's training dependencies first. Both commands below use its
CUDA trainer; the CPU version runs the simulation on CPU workers.

From the Puffysics directory, copy the example and its two required headers:

```sh
PUFFERLIB_DIR=/path/to/PufferLib

cp -R examples/pufferlib/puffysics_hover "$PUFFERLIB_DIR/ocean/"
cp puffysics.cuh b3_batch.cuh "$PUFFERLIB_DIR/ocean/puffysics_hover/"
cp examples/pufferlib/puffysics_hover.ini "$PUFFERLIB_DIR/config/"

cd "$PUFFERLIB_DIR"
```

Build and train with GPU simulation:

```sh
bash build.sh puffysics_hover hover_cuda --cu --float
./hover_cuda train
```

Or use CPU simulation:

```sh
bash build.sh puffysics_hover hover_cpu --float
./hover_cpu train
```

The supplied config starts with 4,096 environments. Keep `num_buffers = 1`
for the native GPU adapter. The example is headless; it does not open a viewer.

## Make it your own

Edit `ocean/puffysics_hover/hover_task.h` in your PufferLib checkout:

| Function | What to change |
| --- | --- |
| `hover_reset` | Bodies, shapes, starting state, and reset randomization |
| `hover_apply_action` | How policy actions drive forces, torques, or motors |
| `hover_observe` | What the policy sees |
| `hover_reward` | What the policy learns to optimize |
| `hover_terminal` | When an episode ends |
| `hover_step` | Physics timestep and how actions advance the simulation |

`HoverConfig` holds each environment's target height, thrust, timestep,
substeps, and episode length. The `.ini` file exposes the target height and
seed. Add more settings in `hover_env_config` as needed.

If you change the number of observations or actions, update `HOVER_OBS_SIZE`
and `HOVER_ACTIONS` alongside the corresponding functions. Rebuild the native
binary after changing C/CUDA code.

The adapter files, `puffysics_hover.h` and `puffysics_hover.cu`, connect those
functions to PufferLib's buffers and handle individual episode resets. They
are a starting point for your own native environment.

## Use the physics functions directly

Keep one `B3World` per environment. Create bodies and shapes during setup,
then apply controls and step the world from your environment function:

```c
#include "puffysics.cuh"

// world and body_id come from your scene setup.
b3_apply_force(world, body_id, b3_v(0.0f, thrust, 0.0f));
b3_step(world, 1.0f / 60.0f, 4);
B3Vec3 position = world->bodies[body_id].position;
```

Forces and torques clear after each step, so apply them again on the next
step. Joint motors, limits, and springs are also available.

For a batch of worlds, include `b3_batch.cuh`. It supports worlds embedded in
your own environment structs, optional masks for stepping and reset, and
CUDA launches on a stream you supply. You can also call `b3_step` directly
inside a kernel that computes actions, physics, rewards, and observations.

See the [integration guide](docs/RL_LIBRARY.md) for memory ownership, resets,
streams, and custom force functions, and the [API reference](docs/API.md)
for the available calls.

## Optional CMake installation

Copying the headers is enough for the PufferLib example. For other C/CUDA
projects, you can install the library and use its CMake target:

```sh
cmake -S . -B build
cmake --install build --prefix "$HOME/.local"
```

```cmake
find_package(Puffysics CONFIG REQUIRED)
target_link_libraries(my_environment PRIVATE Puffysics::puffysics)
```

Set `CMAKE_PREFIX_PATH` to the install prefix if CMake does not find it.
There is no separate library binary to compile. To run the standalone CPU
examples without PufferLib:

```sh
cmake -S . -B build -DPUFFYSICS_BUILD_EXAMPLES=ON
cmake --build build
./build/examples/minimal_cpu
./build/examples/batch_cpu
```

## What is included

The core supports spheres, capsules, boxes, and weld/revolute joints with
motors, springs, and limits. Optional modules add N-body gravity, ambient
fluid forces, cloth, MJCF loading, and runtime-sized loose-body islands.

World capacities are fixed at compile time. Set `B3_MAX_BODIES`,
`B3_MAX_SHAPES`, `B3_MAX_CONTACTS`, and `B3_MAX_JOINTS` for your scene before
including the core, and use the same settings in every source file.
See [conventions](docs/CONVENTIONS.md) for units, frames, and memory layout.

To modify the physics implementation, edit `src/puffysics/*.inl` or the
relevant module header, then regenerate the distributable headers:

```sh
python3 tools/amalgamate.py amalgamate
```

Copy the updated headers into your environment and rebuild. Puffysics is
pre-1.0; struct layouts and solver internals may change. Licensed under [MIT](LICENSE).
