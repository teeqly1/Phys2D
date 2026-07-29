- **[Documentation in English](DOCS_EN.md)** — original documentation in English

## 📄 DOCS.md

# Phys2D — Complete Documentation

All the main Phys2D systems, classes, and methods are described here. The documentation is organized by module for easy searching.

---

## 📚 Contents

1. [Core](#1-core)
2. [Bodies](#2-bodies)
3. [Shapes](#3-shapes)
4. [Collision](#4-collision)
5. [Constraints](#5-constraints)
6. [Extras](#6-extras)
7. [Advanced Lab](#7-advanced-lab)
8. [Blood System](#8-blood-system)
9. [Tools](#9-tools)

---

## 1. Core

### World

The main class that manages all bodies, joints, and the simulation.

```cpp
#include "phys2d/World.h"

WorldConfig cfg;
cfg.gravity = Vec2(0.0, -9.81);
cfg.substeps = 4;
World world(cfg);

world.step(1.0 / 60.0);
```

**World Methods:**

| Method | Description |
|-------|---------|
| `createBody(const BodyDef&)` | Create a body |
| `destroyBody(BodyId)` | Delete a body |
| `body(BodyId)` | Get a body by ID |
| `bodies()` | List all bodies |
| `step(real dt)` | Simulation step |
| `setGravity(const Vec2&)` | Set gravity |
| `setIntegrator(Integrator)` | Select integrator |
| `toJson(bool pretty)` | Serialize to JSON |
| `fromJson(const std::string&)` | Load from JSON |
| `saveToFile(const std::string&)` | Save to file |
| `loadFromFile(const std::string&)` | Load from file |
| `setBeginContactCallback()` | Begin contact callback |
| `setEndContactCallback()` | End contact callback |

**WorldConfig:**

```cpp
struct WorldConfig {
Vec2 gravity; // gravity
Integrator integrator; // SymplecticEuler, Verlet, RK4
int substeps; // substeps (default 4)
uint32_t modules; // module bit flags
SolverConfig solver; // solver settings
MediumConfig medium; // air, liquid, wind
int threadCount; // 0 = auto
size_t arenaBytes; // >= 30 MB
};
```

**SolverConfig:**

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

###RigidBody

```cpp
struct RigidBody { 
BodyId id; 
BodyType type; // Static, Dynamic, Kinematic 
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

###BodyDef

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
ShapeType type; // Circle, Polygon, Capsule 
real radius; 
real halfLength; 
std::vector<Vec2> vertices; 
std::vector<Vec2> normals; 
Vec2centroid; 
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

## 4. Collisions

### Manifold

```cpp
struct Manifold { 
RigidBody* bodyA; 
RigidBody* bodyB; 
Vec2 normal; 
int count; 
ContactPoint points[MAX_MANIFOLD_POINTS]; 
real friction; 
real restitution 
real toi; 
bool touching; 
uint64_t key;
};
```

###Contact Point

```cpp
struct ContactPoint { 
Vec2 point; 
real penetration;
