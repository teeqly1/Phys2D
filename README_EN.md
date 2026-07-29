## 📄 README.md

# Phys2D — a physics engine for games, simulations, and wild experiments

**Phys2D** is a high-performance, open-source 2D physics engine written in C++. It's designed for those who want to add realistic physics, blood, destruction, fluids, electricity, soft bodies, and much more to their games.

The engine was written from scratch by a single developer in four months and already outperforms Box2D in a number of benchmarks. It's freely available, but with a simple condition: don't sell it as a standalone product, and if you modify it, please credit the author.

---

## 🔗 Navigation
- **[Original README](README.md)** — original readme in English
- **[Phys2D for games](README_GAME_EN.md)** — creating games with physics, blood, and destruction in English
- **[Phys2D interactive scene](README_REALTIME.md)** — testing physics in real time
- **[Phys2D for games](README_GAME.md)** — creating games with physics, blood, and destruction
- **[Phys2D documentation (English)](DOCS_EN.md)** — full technical documentation in English
- **[Phys2D interactive scene](README_REALTIME.md)** — testing physics in real time

---

## 🚀 What Phys2D Can Do

| Scope | Capabilities |
|---------|------------|
| **Rigid Bodies** | Circles, rectangles, polygons, capsules. SAT, GJK+EPA, continuous collision detection (CCD), beams. |
| **Materials** | 50 built-in materials (steel, glass, concrete, rubber, wood, ice, fabric, nitinol, etc.). 50x50 pair table with friction, recovery, and conductivity. |
| **Joints** | Distance, hinge, spring, cable, angle, weld, motor, prismatic, pulley, gear, catenary, destructible joints. |
| **Destruction** | Voronov fragmentation, radial fracture, precise momentum conservation (up to 10⁻¹⁵), fragment generations. |
| **Soft Bodies** | Mass-spring, co-rotational FEM, hybrid model, internal pressure, bond breaking, interaction with solids. |
| **Fluids (SPH)** | Density, pressure, viscosity, surface tension, waves, bonding with solids. |
| **Electricity** | Water current (resistor network), electrodes, electric shocks, Joule heating, electrokinetics, arc, piezoelectric effect, tribocharging. |
| **Fields** | N-body gravity (Barnes-Hut algorithm), electrostatics, magnetism, wind (Perlin noise), Coriolis, centrifugal force, tidal forces, radiation pressure. |
| **Thermodynamics** | Thermal conductivity, frictional heating, thermal expansion, phase transitions, piezoelectric effect. |
| **Materials** | Fatigue (Baskin), creep (Norton), stress relaxation, viscoelasticity (Kelvin-Voigt), nonlinear elasticity, plasticity, shape memory (Nitinol), impact strength, corrosion, oxidation, radiation, ablation. |
| **Surfaces** | Adhesion, cohesion, capillary bridges, diffusion, osmosis. |
| **Blood** | Drops, decals, puddles, wounds with pulsation, color from oxygen, drying, smudging, dripping, imprints on bodies. |
| **Sensors** | Trigger zones with Enter/Stay/Exit events, layer filtering. |
| **Tools** | JSON serialization, import/export (Box2D, Bullet, OBJ), scripting language, console, profiler, benchmarks, debug rendering. |
| **Optimization** | SIMD (AVX/SSE), thread pool, pin cache, 65536-level trig table, early iteration exit, islands. |

---

## 📦 What's on board

```
phys2d/
├── include/phys2d/ # Header files (API)
│ ├── World.h # Core: bodies, joints, simulation step
│ ├── Body.h # Rigid bodies
│ ├── Shape.h # Shapes (circle, box, polygon, capsule)
│ ├── Collision.h # SAT, GJK+EPA, manifolds, CCD
│ ├── Constraints.h # Basic joints
│ ├── Extras.h # Extended API (everything else)
│ ├── Advanced.h # Advanced Lab (wind, current, materials, fragments)
│ ├── Blood.h # Blood system
│ ├── phys2d_c_api.h # C-API for FFI
│ └── ...
├── src/ # Sources
│ ├── World.cpp
│ ├── Body.cpp
│ ├── Collision.cpp
│ ├── Constraints.cpp
│ ├── Extras_*.cpp # Module breakdown
│ ├── Advanced.cpp
│ ├── Blood.cpp
│ └── ...
├── examples/ # Demos
│ ├── demo_v2.cpp # Full demo of the entire API
│ ├── realtime_scene.cpp # Interactive scene (Win32)
│ ├── advanced_lab.cpp # Advanced Lab (block 1)
│ ├── sandbox_game.cpp # Gore Lab — a sandbox with blood
│ ├── test_scene.cpp # Test scene (1000+ bodies)
│ └── ...
├── README.md # This file
├── README_EN.md # Documentation in English
├── README_GAME.md # For game developers
├── README_REALTIME.md # Interactive scene
├── DOCS.md # Full documentation on features and API
├── LICENSE # License
└── CMakeLists.txt # Build
```

---

## 🛠 Quick Start

### Build (Windows, MinGW)

```bash
cd /d/projects/phys2d
cmake -B build -G "MinGW Makefiles" \
-DCMAKE_MAKE_PROGRAM=mingw32-make \
-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
