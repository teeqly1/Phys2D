# phys2d

Header `include/phys2d/Extras.h`,
six implementation modules `src/Extras_*.cpp`, demo `examples/demo_v2.cpp`.

## Build (MSYS2 MINGW64)

```bash
cd /c/
cmake -B build -G "MinGW Makefiles" \ 
-DCMAKE_MAKE_PROGRAM=mingw32-make \ 
-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \ 
-DCMAKE_BUILD_TYPE=Release \ 
-DPHYS2D_NATIVE=ON -DPHYS2D_SIMD_AVX=ON -DPHYS2D_FMA=ON \ 
-DPHYS2D_BIG_BINARY=ON -DPHYS2D_TABLE_MB=34
cmake --build build -j$(nproc)

./build/phys2d_demo_v2.exe --steps 240 # demo of all Subsystems
./build/phys2d_demo_v2.exe --benchmark # 11 scenes + report
./build/phys2d_realtime.exe # realtime window
```

Without CMake:

```bash
g++ -std=c++17 -O3 -march=native -Iinclude examples/demo_v2.cpp src/*.cpp -pthread -o demo_v2.exe
```

## Entry Point: Engine class

`Engine` owns the world and all subsystems; `step(dt)` runs them in the correct order.

```cpp
#include "phys2d/Extras.h"
using namespace phys2d;

Engine engine;
engine.fracture().enabled = true; // fracture
engine.softBodies().enabled = true; // soft bodies
engine.fluid().enabled = true; // SPH
engine.fields().enabled = true; // fields
engine.fields().flags = FIELD_WIND | FIELD_NBODY;

for (int i = 0; i < 600; ++i) engine.step(1.0 / 60.0);
std::puts(engine.statusLine().c_str());
```

All subsystems are enabled/disabled by the `enabled` flag; they are also available individually
(without `Engine`): just call `attach(world)` and `update(dt)`.

## What's implemented

| Scope | Capabilities | Status |
|---|---|---|
| CCD | swept circle/segment, swept AABB, conservative advancement, tunneling rollback | Implemented |
| Rays | `rayCastClosest`, `rayCastAll`, `rayCastShape`, filter by layers | Implemented |
| Joints | weld (soft), motor, prismatic, pulley, gear, catenary rope, breakable | Implemented |
| Fracture | Voronoi fragmentation, radial fracture, momentum threshold, fragment generation | Implemented |
| Soft bodies | mass-spring, co-rotational FEM, hybrid, pressure, rupture, body bond | Implemented |
| Ragdoll | 11 parts, 10 joints + angular constraints | Implemented |
| Fluid | SPH: density, pressure, viscosity, surface tension, waves | Implemented |
| Fields | N-body (Barnes-Hut), electrostatics, magnetism, wind (Perlin), Coriolis, centrifugal, tidal, light pressure | Implemented |
| Heat | Thermal conductivity, friction heating, expansion, phase transitions, piezoelectric effect | Implemented |
| Sound | Impact synthesis, Doppler, mixing, WAV export | Implemented |
| Layers and Materials | Categories/Masks/Groups, 50 materials, 50x50 pair table, anisotropic friction | Implemented |
| Sensors | Trigger zones, Enter/Stay/Exit events | Implemented |
| Islands | Union-find, parallel island solving | Implemented |
| Geometry | Concave subdivision, Bezier, NURBS, terrain, BSP, BVH with shaft optimization | Implemented |
| Import/Export | .obj, JSON states, .b2d (Box2D), .bullet | Implemented |
| Recording | Binary log with timestamp, snapshots, rewind/seek | Implemented |
| Reliability | Pulse clipping, quantization, NaN repair, dead body GC, 64-bit IDs | implemented |
| Accuracy | Predictor-corrector, adaptive stride, higher-order inertia | implemented |
| Templates | `ContactKernel<T, Flags>` — 16 instantiations (float/double x 8 flags) | implemented |
| Tables | 65536-level trigonometry, 1024-level contact cache, 4096-body pool | implemented |
| Tools | command line, scripting language, console, profiler-graph, 11 benchmarks | implemented |

## Approximations and Limitations

| Requirement | How it's done |
|---|---|
| Lua scripting | Native interpreter (variables, expressions, `if`, `repeat`, commands) — no external dependency on Lua |
| Remote console | Console polling a command file instead of a TCP server |
| CUDA | not enabled; parallelism via thread pool and islands |
| OpenGL with shaders, shadow map, 3D parallax | rendering via Win32/GDI (`phys2d_realtime`) and SVG debugging; no shader backend |
| Sound with Doppler | synthesis to buffer and WAV export; no realtime playback |

## Bugs found and fixed (this stage)

1. **Soft weld joint diverged** at small stepping: the `gamma` coefficient was not included

in the effective mass. This only occurred with fast bodies (gears),

including substepping. Fixed in `WeldJoint::prepare`.
2. Perlin noise returned garbage at large coordinates — `(int)std::floor(x)`
overflowed. Wind produced infinite force. Added finiteness check
and coordinate folding.
3. Wind accelerated light chain links: added acceleration limit
`FieldSystem::maxWindAcceleration` (250 m/s² by default).
4. Soft-rigid body coupling**: momentum is limited to the smaller of the two masses,
added checks for non-finite states.

Residual note: in the most dense demo scene (cable + gears + fluid +
soft bodies + all fields simultaneously), `SafetyGuard` still occasionally repairs some
