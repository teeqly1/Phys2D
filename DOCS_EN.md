
# Phys2D — Full Documentation (English)

Here's a complete guide to the core systems, classes, and methods of Phys2D. The documentation is organized by module for easy reference.

---

## 📚 Table of Contents

1. [Core](#1-core)
2. [Bodies](#2-bodies)
3. [Shapes](#3-shapes)
4. [Collision](#4-collision)
5. [Constraints (Joints)](#5-constraints-joints)
6. [Extras](#6-extras)
7. [Advanced Lab](#7-advanced-lab)
8. [Blood System](#8-blood-system)
9. [Tools](#9-tools)

---

## 1. Core

### World

The main class that manages all bodies, joints, and the simulation loop.

```cpp
#include "phys2d/World.h"

WorldConfig cfg;
cfg.gravity = Vec2(0.0, -9.81);
cfg.substeps = 4;
World world(cfg);

world.step(1.0 / 60.0);
```

#### World Methods

| Method | Description |
|--------|-------------|
| `createBody(const BodyDef&)` | Create a body |
| `destroyBody(BodyId)` | Remove a body |
| `body(BodyId)` | Get a body by ID |
| `bodies()` | List all bodies |
| `step(real dt)` | Simulation step |
| `setGravity(const Vec2&)` | Set gravity |
| `setIntegrator(Integrator)` | Select integrator |
| `toJson(bool pretty)` | Serialize to JSON |
| `fromJson(const std::string&)` | Load from JSON |
| `saveToFile(const std::string&)` | Save to file |
| `loadFromFile(const std::string&)` | Load from file |
| `setBeginContactCallback()` | Contact start callback |
| `setEndContactCallback()` | Contact end callback |

#### WorldConfig

```cpp
struct WorldConfig {
    Vec2 gravity;               // gravity vector
    Integrator integrator;      // SymplecticEuler, Verlet, RK4
    int substeps;               // substeps (default: 4)
    uint32_t modules;           // module bit flags
    SolverConfig solver;        // solver settings
    MediumConfig medium;        // air, fluid, wind
    int threadCount;            // 0 = auto
    size_t arenaBytes;          // >= 30 MB
};
```

#### SolverConfig

```cpp
struct SolverConfig {
    int velocityIterations = 12;
    int positionIterations = 10;
    real baumgarte = 0.20;
    real allowedPenetration = 0.005;
    real restitutionThreshold = 1.0;
};
```

---

## 2. Bodies

### RigidBody

```cpp
struct RigidBody {
    BodyId id;
    BodyType type;              // Static, Dynamic, Kinematic
    Shape shape;
    Material material;
    Vec2 position, velocity, acceleration;
    real angle, angularVelocity, angularAccel;
    real mass, invMass;
    real inertia, invInertia;
    bool sleeping, fixedRotation;

    void applyForce(const Vec2& f);
    void applyForceAtPoint(const Vec2& f, const Vec2& p);
    void applyImpulse(const Vec2& imp);
    void applyImpulseAtPoint(const Vec2& imp, const Vec2& p);
    Vec2 velocityAtPoint(const Vec2& p) const;
    void wake();
    void sleep();
    real kineticEnergy() const;
};
```

### BodyDef

```cpp
struct BodyDef {
    BodyType type = Dynamic;
    Shape shape = Shape::circle(0.5);
    Material material{};
    Vec2 position{0, 0};
    Vec2 velocity{0, 0};
    real angle = 0.0;
    real angularVelocity = 0.0;
    bool fixedRotation = false;
    bool allowSleep = true;
    real gravityScale = 1.0;
    std::string name;
};
```

### Material

```cpp
struct Material {
    real density = 1.0;
    real restitution = 0.35;
    real staticFriction = 0.60;
    real dynamicFriction = 0.40;
    real rollingFriction = 0.02;
    real angularFriction = 0.01;
    real linearDrag = 0.02;
    real quadraticDrag = 0.01;
    real liftCoefficient = 0.0;
    real softness = 0.0;
    real hysteresis = 0.15;
};
```

---

## 3. Shapes

### Shape

```cpp
struct Shape {
    ShapeType type;             // Circle, Polygon, Capsule
    real radius;
    real halfLength;
    std::vector<Vec2> vertices;
    std::vector<Vec2> normals;
    Vec2 centroid;
    real boundingRadius;

    static Shape circle(real r);
    static Shape box(real w, real h);
    static Shape polygon(std::vector<Vec2> pts);
    static Shape capsule(real r, real len);
    static Shape regularPolygon(int n, real r, real phase = 0.0);
    static Shape gear(int teeth, real innerR, real outerR);

    void updateDerived();
    MassData computeMass(real density) const;
    AABB computeAABB(const Transform& xf, real margin = 0.0) const;
    Vec2 supportLocal(const Vec2& dir) const;
    Vec2 supportWorld(const Transform& xf, const Vec2& dir) const;
    bool containsPoint(const Transform& xf, const Vec2& world) const;
};
```

---

## 4. Collision

### Manifold

```cpp
struct Manifold {
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vec2 normal;
    int count;
    ContactPoint points[MAX_MANIFOLD_POINTS];
    real friction;
    real restitution;
    real toi;
    bool touching;
    uint64_t key;
};
```

### ContactPoint

```cpp
struct ContactPoint {
    Vec2 point;
    real penetration;
    real normalImpulse;
    real tangentImpulse;
    real normalMass;
    real tangentMass;
    uint32_t featureId;
};
```

### CCD & Raycasting

```cpp
// Conservative advancement (CCD)
SweepResult conservativeAdvancement(
    const RigidBody& a,
    const RigidBody& b,
    real dt,
    int maxIterations = 32
);

// Raycasts
bool rayCastClosest(World& w, const Vec2& p1, const Vec2& p2, RayHit& out);
int rayCastAll(World& w, const Vec2& p1, const Vec2& p2, std::vector<RayHit>& out);
```

---

## 5. Constraints (Joints)

| Type | Description |
|------|-------------|
| `DistanceConstraint` | Fixed distance between two anchors |
| `RevoluteConstraint` | Pivot joint (shared point) |
| `SpringConstraint` | Spring with stiffness and damping |
| `RopeConstraint` | Tension-only (max length) |
| `AngularConstraint` | Angle limit between bodies |
| `WeldJoint` | Rigid or soft weld |
| `MotorJoint` | Linear/angular motor |
| `PrismaticJoint` | Sliding joint along an axis |
| `PulleyJoint` | Pulley with ratio |
| `GearJoint` | Gear with ratio |

---

## 6. Extras

### FilterRegistry (Layers & Masks)

```cpp
FilterRegistry filters;
filters.set(body, Filter{category: 0x1, mask: 0xFFFF, sensor: false});
filters.install(world);
```

### SensorSystem (Triggers)

```cpp
SensorSystem sensors;
sensors.add(body, 0xFFFF);
sensors.setCallback([](const SensorEvent& e) {
    if (e.phase == SensorPhase::Enter) printf("Entered!\n");
});
```

### FractureSystem (Destruction)

```cpp
FractureSystem fracture;
fracture.makeBreakable(body, 12.0);
fracture.setCallback([](const FractureEvent& e) {
    printf("Shattered into %zu fragments\n", e.fragments.size());
});
```

### SoftBodySystem (Soft Bodies)

```cpp
SoftBody cloth = SoftBody::grid(Vec2(0, 0), 4, 4, 10, 8, 2.0);
cloth.model = SoftModel::Hybrid;
cloth.tearStrain = 0.85;
softBodies.add(cloth);
```

### ParticleSystem (SPH Fluids)

```cpp
ParticleSystem fluid;
fluid.domain = AABB(Vec2(-5, 0), Vec2(5, 5));
fluid.smoothingRadius = 0.35;
fluid.emitBlock(Vec2(-4, 0.2), 8, 2, 0.16);
```

### FieldSystem (Force Fields)

```cpp
FieldSystem fields;
fields.flags = FIELD_WIND | FIELD_ELECTROSTATIC | FIELD_NBODY;
fields.windBase = Vec2(3.0, 0.0);
fields.windTurbulence = 6.0;
```

### ThermalSystem (Thermodynamics)

```cpp
ThermalSystem thermal;
thermal.registry = &properties;
thermal.ambientTemperature = 293.0;
thermal.phaseTransitions = true;
```

### SurfaceRegistry (Surfaces)

```cpp
SurfaceAssignment sa;
sa.anisotropyAxis = Vec2(1, 0);
sa.frictionAlong = 0.05;
sa.frictionAcross = 1.4;
surfaces.set(body, sa);
```

---

## 7. Advanced Lab

### WindSystem

```cpp
WindSystem wind;
wind.setStrength(10.0);
wind.profile.turbulence = 0.25;
wind.configure(body, AeroProfile{dragCoefficient: 1.2, area: -1.0});
```

### ElectricSystem

```cpp
ElectricSystem elec;
elec.fluid = &fluid;
elec.addElectrode(Electrode{Vec2(-4, 0.4), 220.0, 0.6, false});
elec.addElectrode(Electrode{Vec2(4, 0.4), 0.0, 0.6, true});
```

### MaterialLab (Material Science)

```cpp
MaterialLab lab;
lab.registerBody(body, materialSteel());
lab.applyStress(body, 1e8, dt);
lab.onImpact(body, impulse, speed, point);
```

### ImpactFragmentation (Shattering)

```cpp
ImpactFragmentation frag;
frag.lab = &lab;
frag.maxFragments = 9;
ShatterResult r = frag.shatter(body, impactPoint, impactDir);
```

### ContactChemistry (Adhesion/Capillaries)

```cpp
ContactChemistry chem;
chem.enableAdhesion = true;
chem.enableCapillary = true;
chem.setWetness(body, 1.0);
chem.setConcentration(body, 1.0);
```

---

## 8. Blood System

```cpp
BloodSystem blood;
blood.attach(world);
blood.gravity = Vec2(0.0, -9.81);
blood.stickiness = 0.78;
blood.maxDroplets = 24000;
```

### Methods

| Method | Description |
|--------|-------------|
| `emit(p, v, volume, kind, oxygen)` | Emit a single droplet |
| `spray(origin, dir, speed, cone, count, volume)` | Spray blood in a cone |
| `splash(center, speed, count, volume)` | Splash in all directions |
| `addWound(body, point, dir, severity, arterial)` | Add a wound |
| `sever(body, point, dir, severity)` | Sever a limb |
| `stainBody(body, amount)` | Stain a body |
| `update(dt)` | Update the system |

### BloodKind

| Kind | Description |
|------|-------------|
| `Droplet` | Regular drop |
| `Mist` | Fine spray |
| `Gush` | Heavy arterial jet |
| `Clot` | Coagulated chunk |

---

## 9. Tools

### ScriptEngine

```cpp
ScriptEngine script;
script.registerCommand("spawn", [](const auto& args) { ... });
script.execute("spawn circle 0 5 0.5");
```

### Console

```cpp
Console console;
console.script = &script;
console.pushCommand("gravity 0 -9.81");
```

### Profiler

```cpp
const ProfileData& pd = world.profile();
printf("Broad: %.2f ms\n", pd.ms[(size_t)Stage::BroadPhase]);
```

### Benchmarks

```cpp
BenchmarkSuite suite;
auto results = suite.runAll(300, 1.0/60.0);
std::printf("%s", suite.report(results).c_str());
```

### Serialization

```cpp
world.saveToFile("scene.json");
world.loadFromFile("scene.json");
exportBox2D(world, "scene.b2d");
exportBullet(world, "scene.bullet");
```

---

## 📜 Acronyms

| Acronym | Full Name |
|---------|-----------|
| CCD | Continuous Collision Detection |
| SAT | Separating Axis Theorem |
| GJK | Gilbert-Johnson-Keerthi |
| EPA | Expanding Polytope Algorithm |
| SPH | Smoothed Particle Hydrodynamics |
| FEM | Finite Element Method |
| SIMD | Single Instruction, Multiple Data |

---

## 🔗 Links

- [README.md](https://github.com/teeqly1/Phys2D#readme) — Main project page
- [README_GAME.md](https://github.com/teeqly1/Phys2D) — Game creation guide
- [README_REALTIME.md](https://github.com/teeqly1/Phys2D) — Interactive scene
- [DOCS.md](https://github.com/teeqly1/Phys2D/blob/main/DOCS.md) — Russian docs
- [LICENSE](https://github.com/teeqly1/Phys2D/blob/main/LICENSE) — License
- [Author on GitHub](https://github.com/teeqly1)

---

**Last updated:** July 2026

---

Have a build!
```

---
