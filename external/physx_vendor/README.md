# physx_vendor

ament/colcon vendor package for the [NVIDIA PhysX SDK](https://github.com/NVIDIA-Omniverse/PhysX).

It builds (or stages) the PhysX SDK and installs it to `<prefix>/opt/physx`,
preserving the upstream layout:

```
opt/physx/
  include/                       # PxPhysicsAPI.h, vehicle2/, ...
  bin/linux.x86_64/release/      # libPhysX*_static_64.a
  snippets/                      # snippetvehiclecommon/ (vehicle sample sources)
```

Downstream packages consume it with `find_package(physx_vendor)`, which sets:

- `PHYSX_SDK_DIR` — the SDK root (`<prefix>/opt/physx`)
- `PHYSX_LIB_DIR` — the release static-library directory

`simple_sensor_simulator` already looks this up automatically to enable its
high-fidelity `taiga_x` vehicle model.

## Build

```bash
colcon build --packages-select physx_vendor
```

Clones `NVIDIA-Omniverse/PhysX` (pinned to tag `107.3-physx-5.6.1`) and runs the
upstream `generate_projects.sh <preset>` + CMake build. This downloads
toolchain dependencies via packman (**requires network access**) and compiles
the SDK, which can take a few minutes.

Relevant CMake cache variables (override with `--cmake-args -D...`):

| Variable | Default | Meaning |
|----------|---------|---------|
| `PHYSX_VENDOR_GIT_TAG` | `107.3-physx-5.6.1` | upstream commit/tag to build |
| `PHYSX_VENDOR_PRESET` | `linux-gcc-cpu-only` | `generate_projects.sh` preset. CPU-only + gcc by default (no CUDA toolkit needed); the `taiga_x` vehicle solver runs on CPU. Use `linux-gcc` / `linux-clang` for the GPU build. |
| `PHYSX_VENDOR_GIT_REPOSITORY` | NVIDIA-Omniverse/PhysX | source repository |

## Notes

- PhysX is distributed by NVIDIA under the BSD-3-Clause license.
- The from-source build invokes packman, which fetches binary dependencies from
  the network on first use.
