# GORE LAB — a ready-made physics sandbox game in phys2d

A sandbox game built on top of the engine: ragdolls, destructible boxes, crusher gears,
a soft body, SPH fluid, a rope with a cannonball, and a detailed blood system.

## Files

| File | What is this |
|---|---|
| `include/phys2d/Blood.h` | Blood system API (`BloodSystem` inherits `SimulationSystem`) |
| `src/Blood.cpp` | implementation: drops, decals, puddles, wounds, pulse, coagulation |
| `examples/sandbox_game.cpp` | The game itself: arena, weapons, Win32/GDI + headless rendering |
| `CMakeLists.txt` | Added `phys2d_sandbox` target and `src/Blood.cpp` to the library |

## Build (MSYS2 MINGW64)

Without `-march=native`, until we figured out the `Illegal instruction` on GCC 16:

```bash
cd /d/projects/phys2d
rm -rf build
cmake -B build -G "MinGW Makefiles" -DCMAKE_MAKE_PROGRAM=mingw32-make \ 
-DCMAKE_BUILD_TYPE=RelWithDebInfo \ 
-DPHYS2D_NATIVE=OFF -DPHYS2D_SIMD_AVX=OFF -DPHYS2D_SIMD_SSE=OFF \ 
-DPHYS2D_FMA=OFF -DPHYS2D_BIG_BINARY=OFF
cmake --build build -j$(nproc) --target phys2d_sandbox
./build/phys2d_sandbox.exe
```

Without CMake, in one line:

```bash
g++ -std=c++17 -O2 -Iinclude examples/sandbox_game.cpp src/*.cpp -pthread -lgdi32 -luser32 -o gore_lab.exe
```

On Linux/without Windows, the same source code is built in headless mode:
`./gore_lab --steps 900` runs the scripted slaughter, prints statistics, and
writes `gore_frame.svg` — a vector snapshot of the scene with all the spots.

## Controls

| Key | Action |
|---|---|
| `1` | grab the body with the mouse (spring grab) |
| `2` / `3` / `4` | Spawn crate / barrel / ragdoll victim |
| `5` | blade: cuts flesh with mouse movement |
| `6` / `7` | bullet (straight-through raycast) / shotgun, 9 buckshots |
| `8` | grenade (right mouse button - also a grenade) |
| `9` / `0` | water cannon / blood hose |
| LMB | use tool, hold = continuous |
| wheel | zoom, arrow keys - camera |
| `SPACE` / `S` | pause / single step |
| `R` / `C` | rebuild arena / erase all blood |
| `B` `D` `F` | blood / decals / liquid on-off |
| `T` / `G` | slow-mo x0.25 / disable gravity |
| `[` `]` | solver iterations -2 / +2 |
| `ESC` | exit |

## What's squeezed out of the engine

- ragdolls on brittle joints (`BreakableJointSystem`) — limbs are torn off;
- `FractureSystem` — boxes shatter according to the Voronoi principle from the impulse;
- `SoftBodySystem` — soft body with a tear (`tearStrain = 0.85`);
- SPH fluid with a domain, kinematic crusher gears and a spinner;
- catenary chain with a cast iron core, a pyramid of boxes (stack stability test);
- raycasts with energy loss in flesh (x0.45) and brittle (x0.35);
- explosions via impulse along a radius + fracture initiation.

## Blood System

Model: Blood is a volume in milliliters that flows between three
states: **droplets in the air → decals on surfaces → puddles on the floor**.

- **Wounds** (`addWound` / `sever`) have a reservoir, severity, and coagulation;
arterial blood pulses — `pulseHz = 1.30`, pulse strength `0.85`,
the flow follows a `beat³`, that is, in short bursts, like from a real artery.
- **Drops** — Verlet integration with quadratic drag; the trajectory
is traced by the engine's raycast, so the drop hits the body
it passes through, rather than falling through it at high speed.
- **Decals** are in the local coordinates of the body: the spot moves and rotates with
the body. Shape: A jagged polygon of splatterLobes = 9 petals,
elongated tangentially during a glancing blow, with a "tail" in the direction
of flight and small splatter-like satellites at speeds above castOffSpeed.
- Puddles merge within a radius of poolMergeDist, grow as sqrt(V), and dry in poolDryTime = 70 s.
- Color depends on oxygen saturation (arterial 235 - scarlet, venous
190 - dark) and the degree of drying: fresh → brown crust.
- StainBody sputum smears when moving (smearRate) and
drips down (dripRate) - a blood-soaked Ragdoll leaves traces.

Default limits: 24,000 drops, 7,000 decals, 900 puddles, 256 wounds —
old ones are recycled, memory does not grow.

Main settings: `airDrag`, `stickiness`, `bounceRestitution`,
`castOffSpeed`, `clotRate`, `dryTime`, `poolSpread`, `enableMist`,
`enableCastOff`, `enablePools`, `groundLevel`.

### Minimal usage

```cpp
phys2d::Engine engine{cfg};
phys2d::BloodSystem blood;
blood.attach(engine.world());
blood.groundLevel = 0.02;

blood.addWound(bodyId, worldPoint, worldNormal, /*severity*/0.8, /*arterial*/true);

for (;;) { 
engine.step(dt); 
blood.update(dt); // after the peace step
}
```
