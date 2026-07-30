# Phys2D — Unified Documentation

# 1. Introduction

## Version
3.0 (Nova). C++17, `using real = double`, PIMPL, C-API, SIMD, threading.

## Package Structure
```
phys2d/
  include/phys2d/
    Vec2.h            math, real, AABB
    Shape.h           circle, box, polygon, capsule
    Body.h            Material, BodyDef, RigidBody
    World.h           world, WorldConfig, SolverConfig, MediumConfig
    Constraints.h     11 constraint types
    Collision.h       manifolds, narrow-phase
    BroadPhase.h      Grid / SweepAndPrune / Hybrid
    DebugDraw.h       debug geometry
    Profiler.h        ProfileData
    Serialization.h   JSON scenes
    ThreadPool.h      thread pool
    Extras.h          CCD, fracture, softbody, ragdolls, SPH, rays
    Advanced2.h       units, compound, wind, presets, CONFIG.TXT
    Blood.h           blood system
    Nova.h            12 Nova subsystems
    phys2d_c_api.h    flat C interface
  src/                implementation + embedded_tables.S
  examples/           demos and games
```

## Core Concepts

| Concept | Description |
| :--- | :--- |
| `World` | container for bodies, constraints, and modules |
| `RigidBody` | body: type, shape, material, transform, velocities |
| `Shape` | Circle, Box, Polygon, Capsule |
| `Material` | density, restitution, friction, drag |
| `Constraint` | connection between two bodies |
| `SimulationSystem` | module: `attach(world)` + `update(dt)` |

All Extras and Nova modules work the same way: create object, call
`attach(world)`, then call `update(dt)` alongside `world.step(dt)`.

## Units
Meters, kilograms, seconds, radians. Density — kg/m³, force — N,
impulse — N·s, speed in km/h = `length(v) * 3.6`.
Working range for body sizes: 0.05–50 m. Below 0.02 m — need 1/240 step.

## Minimal Example
```cpp
#include "phys2d/World.h"
#include <cstdio>
using namespace phys2d;

int main() {
    World world;
    world.setGravity(Vec2(0, -9.81));

    BodyDef ground;
    ground.type = BodyType::Static;
    ground.shape = Shape::box(100.0, 1.0);
    ground.position = Vec2(0, -0.5);
    world.createBody(ground);

    BodyDef box;
    box.shape = Shape::box(1.0, 1.0);
    box.position = Vec2(0, 10.0);
    box.material.density = 700.0;
    box.material.restitution = 0.2;
    BodyId id = world.createBody(box);

    for (int i = 0; i < 600; ++i) world.step(1.0 / 60.0);
    printf("y = %.3f\n", world.body(id)->position.y);
}
```

---

# 2. Building

## MSYS2 MinGW64
```bash
cd /d/projects/phys2d
mkdir -p build
g++ -std=c++17 -O3 -march=native -Iinclude \
    examples/nova_demo.cpp src/*.cpp -pthread -o build/nova_demo.exe
```

**Important:**
- **Never add `src/embedded_tables.S`** — MinGW assembler gives
  `junk at end of line`; tables are available in C++ form as well
- If binary crashes with `Illegal instruction` — remove `-march=native`
- `-ld3d9` only **after** the source list
- `-pthread` is required

## CMake
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native"
cmake --build build -j
```

Targets: `phys2d`, `test_scene`, `demo_v2`, `adv_demo`, `nova_demo`, `sandbox_opt`,
`gorelab_remake`, `carcrash`, `drive`.

## Flags

| Flag | Purpose |
| :--- | :--- |
| `-O3 -march=native` | maximum speed |
| `-DPHYS2D_D3D9` | enable Direct3D 9 |
| `-DPHYS2D_NO_D3D9` | disable D3D9 |
| `-DPHYS2D_SINGLE_PRECISION` | `real = float` |
| `-DPHYS2D_NO_SIMD` | scalar path |

Don't use `-fvisibility=hidden -ffunction-sections -fdata-sections`
with static library — C-API symbols are lost.

## As a Library
```bash
g++ -std=c++17 -O2 -Iinclude -c src/*.cpp
ar rcs libphys2d.a *.o
g++ -std=c++17 -O2 -Iinclude my_game.cpp libphys2d.a -pthread -o my_game
```

---

# 3. Core Engine API

## Vec2.h
```cpp
using real = double;

struct Vec2 {
    real x = 0, y = 0;
    Vec2();  Vec2(real x, real y);
    Vec2 operator+(const Vec2&) const;  Vec2 operator-(const Vec2&) const;
    Vec2 operator*(real) const;         Vec2 operator/(real) const;
    Vec2& operator+=(const Vec2&);      Vec2& operator-=(const Vec2&);
    Vec2 operator-() const;
};

real dot(const Vec2& a, const Vec2& b);
real cross(const Vec2& a, const Vec2& b);
real length(const Vec2& v);      real lengthSq(const Vec2& v);
Vec2 normalized(const Vec2& v);  Vec2 rotated(const Vec2& v, real angle);
Vec2 perp(const Vec2& v);        Vec2 fromAngle(real angle);
bool isFinite(const Vec2& v);    real clampr(real v, real lo, real hi);

struct AABB { Vec2 min; Vec2 max; };   // fields are min/max, not lo/hi
```

## Shape.h
```cpp
enum class ShapeType : uint8_t { Circle, Box, Polygon, Capsule };

struct Shape {
    static Shape circle(real radius);
    static Shape box(real width, real height);        // FULL dimensions
    static Shape polygon(const std::vector<Vec2>& pts);
    static Shape capsule(real halfLength, real radius);

    ShapeType type;
    real radius, halfLength, boundingRadius;
    std::vector<Vec2> vertices;

    MassData computeMass(real density) const;
    void capsuleSegment(Vec2& a, Vec2& b) const;
    static Vec2 closestPointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b);
};
```

**Important:** `Shape::box(1,1)` — cube with side 1 m, not 2×2.
Polygons must be convex, counter-clockwise winding.

## Body.h
```cpp
struct Material {
    real density          = 1.00;   // kg/m^3
    real restitution      = 0.35;
    real staticFriction   = 0.60;
    real dynamicFriction  = 0.40;
    real rollingFriction  = 0.02;
    real angularFriction  = 0.01;
    real linearDrag       = 0.02;
    real quadraticDrag    = 0.01;
    real liftCoefficient  = 0.00;
    real softness         = 0.00;
    real hysteresis       = 0.15;
};
```

**Densities:** wood 700, concrete 2400, steel 7850, glass 2500, rubber 1100,
ice 917, water 1000, snow 250, flesh 1050.

```cpp
enum class BodyType : uint8_t { Static = 0, Dynamic = 1, Kinematic = 2 };
using BodyId = uint32_t;
constexpr BodyId INVALID_BODY = 0xFFFFFFFFu;

struct BodyDef {
    BodyType type = BodyType::Dynamic;
    Shape shape;  Material material;
    Vec2 position, velocity;
    real angle = 0, angularVelocity = 0;
    bool fixedRotation = false, allowSleep = true;
    real gravityScale = 1.0;
    void* userData = nullptr;
    std::string name;
};

class RigidBody {
public:
    Vec2 position, velocity, force;
    real angle, angularVelocity, torque;
    BodyType type;  Shape shape;  Material material;
    real boundingRadius = 0.5;
    bool fixedRotation, allowSleep;
    real gravityScale;  void* userData;  std::string name;

    void applyForce(const Vec2& f);
    void applyForceAtPoint(const Vec2& f, const Vec2& worldPoint);
    void applyTorque(real t);
    void applyImpulse(const Vec2& j);
    void applyImpulseAtPoint(const Vec2& j, const Vec2& worldPoint);
    void clearForces();
    void wake();   bool awake() const;
    Transform transform() const;
    Vec2 velocityAtPoint(const Vec2& worldPoint) const;
    std::vector<Vec2> worldVertices() const;
};
```

**Sleeping body ignores forces** — call `wake()`.

## World.h
```cpp
class World {
public:
    BodyId createBody(const BodyDef& def);
    void   destroyBody(BodyId id);
    RigidBody* body(BodyId id);   const RigidBody* body(BodyId id) const;
    size_t bodyCount() const;
    std::vector<RigidBody*>& bodies();

    ConstraintId createDistance(BodyId a, BodyId b, Vec2 la, Vec2 lb, real len);
    ConstraintId createRevolute(BodyId a, BodyId b, Vec2 la, Vec2 lb);
    ConstraintId createSpring(BodyId a, BodyId b, Vec2 la, Vec2 lb,
                              real length, real stiffness, real damping);
    ConstraintId createRope(BodyId a, BodyId b, Vec2 la, Vec2 lb, real maxLen);
    ConstraintId createAngular(BodyId a, BodyId b, real targetAngle);
    ConstraintId addConstraint(const Constraint& c);
    void destroyConstraint(ConstraintId id);
    size_t constraintCount() const;
    std::vector<Constraint>& allConstraints();

    void step(real dt);
    void setGravity(const Vec2& g);   Vec2 gravity() const;
    WorldConfig& config();
    void setIntegrator(Integrator i);
    void enableModule(Module m, bool on);   bool moduleEnabled(Module m) const;

    using ContactFilterFn = std::function<bool(const RigidBody&, const RigidBody&)>;
    void setContactFilter(ContactFilterFn fn);
    void setBeginContactCallback(std::function<void(const CollisionEvent&)>);
    void setEndContactCallback(std::function<void(const CollisionEvent&)>);
    void setPostStepCallback(std::function<void(World&, real)>);

    RigidBody* queryPoint(const Vec2& p);
    std::vector<RigidBody*> queryAABB(const AABB& box);
    const ProfileData& profile() const;
    size_t contactCount() const, awakeCount() const, arenaBytes() const;
    real totalEnergy() const;
    const std::vector<Manifold>& manifolds() const;
    const DebugBuffer& debugBuffer() const;   void buildDebugGeometry();

    std::string toJson(bool pretty = true) const;
    bool fromJson(const std::string& json);
    bool saveToFile(const std::string& path) const;
    bool loadFromFile(const std::string& path);
    void clear();
};
```

**Danger:** `bodies()` returns a reference to a vector that reallocates
on `createBody`. Creating or deleting bodies during iteration — SIGSEGV.

### Configuration
```cpp
struct SolverConfig {
    int  velocityIterations   = 12;
    int  positionIterations   = 10;
    real baumgarte            = 0.20;
    real allowedPenetration   = 0.005;
    real maxLinearCorrection  = 0.20;
    real restitutionThreshold = 1.00;
    real schlagerFactor       = 0.80;
    // v3 extensions
    real frictionLoadFactor   = 0.00;
    real stictionSpeed        = 0.05;
    real warmStartDamping     = 0.92;
    real maxPenetration       = 0.25;
    int  maxContactPoints     = 4;
    real restitutionSpeedRef  = 4.00;
    real restitutionFalloff   = 0.55;
};

struct MediumConfig {
    real airDensity = 1.204, fluidDensity = 1000.0;
    real fluidLevel = -1e30;      // water level for Archimedes
    real viscosity = 0.90;  Vec2 wind;
};

struct WorldConfig {
    SolverConfig solver;  MediumConfig medium;
    BroadPhaseMode broadPhase = BroadPhaseMode::Hybrid;
    real gridCellSize = 4.0, aabbMargin = 0.10;
    int  regionCount = 4, threadCount = 0;
    real sleepLinearTolerance = 0.04, sleepAngularTolerance = 0.04;
    real sleepTime = 0.60, wakeImpulseThreshold = 0.20;
    size_t arenaBytes = 32u*1024*1024, maxContacts = 262144, maxBodies = 65536;
};

enum class BroadPhaseMode : uint8_t { Grid = 0, SweepAndPrune = 1, Hybrid = 2 };
```

### Collision Event
```cpp
struct CollisionEvent {
    RigidBody* a;  RigidBody* b;          // pointers, not BodyId
    Vec2 normal, point;
    real penetration, normalImpulse, tangentImpulse, relativeSpeed, timeOfImpact;
    ContactPhase phase;                   // Begin, Persist, End
};
```

## Constraints.h
```cpp
enum class ConstraintType {
    Distance = 0, Revolute, Spring, Rope, Angular,
    Weld, Motor, Prismatic, Pulley, Gear, Catenary
};

struct SpringConstraint { real restLength = 1.0, stiffness = 200.0, damping = 5.0; };
struct RopeConstraint   { real maxLength  = 2.0; };
```

---

# 4. Extras and Advanced2

Common interface for all modules:
```cpp
class SimulationSystem {
public:
    virtual void attach(World& w);
    virtual void update(real dt);
    uint64_t tick() const;
    World* world() const;
};
```

## Rays and Queries
```cpp
struct RayHit { RigidBody* body; Vec2 point, normal; real fraction; };

bool rayCastClosest(World& w, Vec2 from, Vec2 to, RayHit& out);
std::vector<RayHit> rayCastAll(World& w, Vec2 from, Vec2 to);
bool rayCastShape(const Shape& s, const Transform& t, Vec2 from, Vec2 to, RayHit& out);
bool shapeCast(World& w, const Shape& s, Vec2 from, Vec2 to, RayHit& out);
std::vector<RigidBody*> overlapShape(World& w, const Shape& s, const Transform& t);
std::vector<RigidBody*> pointQueryAll(World& w, Vec2 p);
```

## CCD
```cpp
class CcdSystem : public SimulationSystem {
public:
    real speedThreshold = 6.0;   // m/s
    int  maxSubsteps    = 8;
    int  resolvedThisStep() const;
};
```

## Fracture
```cpp
std::vector<Shape> voronoiFracture(const Shape& s, int cells, uint32_t seed);
std::vector<Shape> radialFracture(const Shape& s, Vec2 impact, int rings, int sectors);

class FractureSystem : public SimulationSystem {
public:
    void setThreshold(real impulseJ);
    void markBreakable(BodyId id, real toughness, int shards);
    int  fracturedThisStep() const;
};
```

## Soft Bodies and Ragdolls
```cpp
class SoftBodySystem : public SimulationSystem {
public:
    int  createBlob(Vec2 center, real radius, int nodes, real stiffness);
    int  createBeam(Vec2 a, Vec2 b, int segments, real stiffness);
    void setPressure(int id, real p);
};

class Ragdoll {
public:
    void build(World& w, Vec2 pelvis, real height, real mass);
    BodyId head() const, torso() const, limb(int i) const;
};
```

## Fluids (SPH / PBF)
```cpp
struct FluidParticle { Vec2 position, velocity; real density, pressure, mass; uint8_t kind; };

class ParticleSystem : public SimulationSystem {
public:
    void emitBlock(Vec2 min, Vec2 max, real spacing);
    void emitJet(Vec2 origin, Vec2 dir, real speed, int count);
    size_t count() const;
    std::vector<FluidParticle>& particles();
    real wetnessAt(const Vec2& p, real radius) const;   // fire extinguishing
    void applyExplosion(Vec2 center, real radius, real impulse);
    void clear();
};
```

## Blood (Blood.h)
```cpp
enum class BloodKind { Droplet, Mist, Gush, Clot };

class BloodSystem : public SimulationSystem {
public:
    void spray(Vec2 origin, Vec2 dir, real speed, real coneRad,
               int count, real volumeMl, uint8_t oxygen = 225);
    void addWound(BodyId body, Vec2 local, real flowMlPerSec, real duration);
    void stainBody(BodyId body, Vec2 local, real amount);
    static void colorOf(uint8_t oxygen, real wetness,
                        uint8_t& r, uint8_t& g, uint8_t& b);
};
```

Oxygenation: 225 — arterial bright red, 120 — venous dark red, 40 — dried brown.

## Advanced2.h

| Class | Purpose |
| :--- | :--- |
| `Units` / `UnitScale` | meters and pixels |
| `Compound` | compound body with multiple shapes |
| `MaterialTree` | material hierarchy with inheritance |
| `CollisionRules` | layers and collision masks |
| `ContactTuner` | per-pair friction/restitution tuning |
| `AxisLock` | axis movement restriction |
| `GravityMode` | `Constant`, `Radial`, `Dispersive` |
| `WindField` | wind with gusts and turbulence |
| `MotionInterpolator` | smooth rendering between steps |
| `Diagnostics` | NaN detection, energy balance |
| `StabilityGuard` | solver explosion suppression |
| `FrictionHeat` | friction heating, thermal map |
| `DirtyTracker` | bodies changed per frame |
| `SceneKeeper` | snapshots and rollback |
| `MemoryTracker` | arena and memory peak |
| `PluginHost` | external module loading |
| `Preset` | `Accuracy`, `Speed`, `Balance` |

### CONFIG.TXT
```
preset = Balance
stiction_speed = 0.05
warm_start_damping = 0.92
max_penetration = 0.25
max_contact_points = 4
velocity_iterations = 18
position_iterations = 12
baumgarte = 0.22
gravity = 0 -9.81
thread_count = 0
```

## Renderer (Render.h)
```cpp
enum class RenderBackend { None, Direct3D9 };

struct RenderConfig {
    int width = 1440, height = 860;
    real pixelsPerMeter = 34.0;
    RenderBackend backend = RenderBackend::Direct3D9;
};

class Renderer {
public:
    bool create(const RenderConfig& cfg);
    void beginFrame(uint32_t clearColor);
    void drawWorld(const World& w);
    void drawLine(Vec2 a, Vec2 b, uint32_t color);
    void drawText(int x, int y, const char* text, uint32_t color);
    void endFrame();
    void destroy();
    bool pumpMessages();       // false = window closed
};
```

## C-API (phys2d_c_api.h)
```c
typedef struct p2dWorld p2dWorld;
typedef unsigned int p2dBody;

p2dWorld* p2d_world_create(void);
void      p2d_world_destroy(p2dWorld*);
void      p2d_world_set_gravity(p2dWorld*, double x, double y);
void      p2d_world_step(p2dWorld*, double dt);

p2dBody p2d_create_box(p2dWorld*, double x, double y, double w, double h,
                       double density, int isStatic);
p2dBody p2d_create_circle(p2dWorld*, double x, double y, double r,
                          double density, int isStatic);
void    p2d_body_get_position(p2dWorld*, p2dBody, double* x, double* y);
double  p2d_body_get_angle(p2dWorld*, p2dBody);
void    p2d_body_apply_impulse(p2dWorld*, p2dBody, double jx, double jy);
void    p2d_body_destroy(p2dWorld*, p2dBody);
int     p2d_world_body_count(p2dWorld*);
int     p2d_world_save(p2dWorld*, const char* path);
int     p2d_world_load(p2dWorld*, const char* path);
```

---

# 5. Nova — 12 Subsystems

Header `phys2d/Nova.h`, namespace `phys2d::nova`.

```cpp
struct NovaSuite {
    DepthWorld depth;  ServerAuthority server;  ClientPredictor client;
    InterpolationBuffer interp;  ClothSystem cloth;  GrainSystem grains;
    WeatherSystem weather;  Ocean ocean;  Acoustics audio;
    PowerManager power;  EditorCore editor;  LuaVM lua;

    void attachAll(World& w);
    void update(real dt);       // cloth -> weather -> grains -> audio -> power
    std::string report() const;
};
```

## 1. Materials (48 items)
```cpp
enum class FractureStyle : uint8_t
    { None, Shards, Splinters, Dent, Crumble, Shatter, Tear, Melt };

struct MaterialLook  { uint32_t color, accent; float roughness, gloss, emissive, transparency; };
struct MaterialSound { float baseHz, decay, gain, rolloff, absorption, footstepHz; };
struct MaterialDef   { std::string name; Material phys; MaterialLook look;
                       MaterialSound sound; FractureStyle fracture;
                       real toughness; int shardCount; };

class MaterialLibrary {
public:
    static const MaterialLibrary& instance();
    int count() const;                       // 48
    const MaterialDef& byIndex(int i) const;
    int indexOf(const std::string& name) const;
    const MaterialDef& byName(const std::string& name) const;
    Material physics(int index) const;
    real impactHz(int a, int b, real impulseJ) const;
};
```

**Indices:** 0 water, 1 saltwater, 2 oil, 3 ice, 4 snow, 5 packedsnow, 6 sand,
7 gravel, 8 soil, 9 clay, 10 mud, 11 asphalt, 12 concrete, 13 brick, 14 granite,
15 marble, 16 sandstone, 17 limestone, 18 basalt, 19 glass, 20 temperedglass,
21 ceramic, 22 porcelain, 23 oak, 24 pine, 25 plywood, 26 bamboo, 27 cork,
28 steel, 29 stainless, 30 iron, 31 aluminium, 32 titanium, 33 copper, 34 brass,
35 lead, 36 gold, 37 rubber, 38 softrubber, 39 plastic, 40 nylon, 41 foam,
42 cloth, 43 leather, 44 flesh, 45 bone, 46 wax, 47 lava.

## 2. 2.5D Layers
```cpp
struct LayerInfo { real z, thickness, parallax; uint32_t tint; float fog; bool collides; };

class DepthWorld {
public:
    void attach(World& w);                 // sets collision filter
    void configure(int layer, const LayerInfo& info);
    const LayerInfo& info(int layer) const;
    void setLayer(BodyId id, int layer);   int layer(BodyId id) const;
    void linkLayers(int a, int b, bool collide);
    bool canCollide(int a, int b) const;
    std::vector<BodyId> drawOrder() const;
    real parallax(int layer) const;
    real scaleFor(int layer, real cameraZ = 0) const;   // 20/(20+z-cameraZ)
    size_t trackedBodies() const;
};
```

## 3. Editor
```cpp
struct PaletteItem {
    enum Kind { Box, Circle, Polygon, StaticBox, Ramp, Cloth, Grain, Vehicle };
    std::string name; Kind kind; real w, h; int materialIndex;
};

class EditorCore {
public:
    void attach(World& w);
    void setPaused(bool);   bool paused() const;   void step();
    bool pick(Vec2 worldPos);   void drag(Vec2 target);   void release();
    bool hasSelection() const;  BodyId selection() const;
    void deleteSelection();     void duplicateSelection();
    const std::vector<PaletteItem>& palette() const;
    void addPaletteItem(const PaletteItem& item);
    BodyId spawn(int paletteIndex, Vec2 position);
    void setGravity(Vec2), setGlobalFriction(real), setGlobalRestitution(real);
    void setSolverIterations(int vel, int pos);
    void setTimeScale(real);  real timeScale() const;
    void pushUndo();  bool undo();  bool redo();  int undoDepth() const;
    bool saveScene(const std::string& path);
    bool loadScene(const std::string& path);
    std::string statusLine() const;
};
```

## 4. Procedural Generation
```cpp
struct CaveParams { int width=96, height=48; real cell=0.5, fill=0.45;
                    int smoothPasses=5; uint32_t seed=1337; Vec2 origin; };
struct CityParams { int blocks=8; real streetWidth=6, minBuilding=4, maxBuilding=12,
                    minHeight=6, maxHeight=26, debrisChance=0.25;
                    uint32_t seed=2024; Vec2 origin; };
struct MazeParams { int cols=21, rows=15; real cell=2; uint32_t seed=7;
                    Vec2 origin; bool braid=false; };

class ProcGen {
public:
    static std::vector<uint8_t> caveGrid(const CaveParams&);
    static std::vector<uint8_t> mazeGrid(const MazeParams&);
    static std::vector<BodyId>  buildCave(World&, const CaveParams&);
    static std::vector<BodyId>  buildMaze(World&, const MazeParams&);
    static std::vector<BodyId>  buildCity(World&, const CityParams&);
    static Vec2 findSpawn(const std::vector<uint8_t>& grid, int w, int h,
                          real cell, Vec2 origin);
};
```

## 5. Lua Scripts
```cpp
struct LuaValue {
    enum Type { Nil, Number, Bool, Str };
    bool truthy() const;  double toNumber() const;  std::string toString() const;
};
using LuaNative = std::function<LuaValue(const std::vector<LuaValue>&)>;

class LuaVM {
public:
    void bindWorld(World& w);
    void registerFunction(const std::string& name, LuaNative fn);
    bool doString(const std::string& code, std::string* error = nullptr);
    bool doFile(const std::string& path, std::string* error = nullptr);
    bool hasFunction(const std::string& name) const;
    LuaValue callFunction(const std::string& name, const std::vector<LuaValue>& args);
    void setGlobal(const std::string&, const LuaValue&);
    LuaValue getGlobal(const std::string&) const;
    const std::string& output() const;   void clearOutput();
    int nativeCount() const;             // 28
};
```

**Supported:** `local`, `if/elseif/else`, `while`, `repeat/until`, `for i=a,b,c`,
`function`, `return`, `break`, `do/end`, comments `--`, concatenation `..`.
**Tables `{}` are NOT supported**.

28 built-in functions: `print`, `tostring`, `tonumber`, `math.sin`, `math.cos`,
`math.tan`, `math.sqrt`, `math.abs`, `math.floor`, `math.ceil`, `math.min`,
`math.max`, `math.atan2`, `math.random`, `math.pi`, `material_count`,
`material_name`, `set_gravity`, `body_count`, `spawn_box`, `spawn_circle`,
`spawn_static`, `body_x`, `body_y`, `body_speed`, `apply_force`, `apply_impulse`,
`remove_body`, `step`.

## 6. Cloth, Hair, Ropes
```cpp
class ClothSystem {
public:
    Vec2 gravity = Vec2(0, -9.81);
    void attach(World& w);
    int createFlag(Vec2 topLeft, int cols, int rows, real spacing = 0.18,
                   real stiffness = 1.0);
    int createCurtain(Vec2 topLeft, int cols, int rows, real spacing);
    int createHair(Vec2 root, int nodes, real length, real stiffness = 0.9);
    int createRopeStrand(Vec2 a, Vec2 b, int nodes);
    void pinToBody(int piece, int node, BodyId body, Vec2 local);
    void setWind(Vec2 wind, real turbulence);
    void update(real dt, int iterations = 6);
    void applyImpulse(Vec2 center, real radius, Vec2 impulse);
    int pieceCount() const;  const ClothPiece& piece(int i) const;
    int nodeCount() const;   int tornLinks() const;
};
```

## 7. Grains (DEM) and Weather
```cpp
enum class GrainKind : uint8_t
    { Sand, Gravel, Snow, Spark, Smoke, Ember, Rain, Hail, Dust };

class GrainSystem {
public:
    Vec2 gravity;
    real stiffness = 8000, damping = 12, friction = 0.55, airDrag = 0.02;
    size_t maxCount = 40000;
    real bounds[4] = { -400, -80, 400, 260 };

    void attach(World& w);   void reserve(size_t n);
    void emitPile(Vec2 center, real w, real h, real radius, GrainKind k);
    void emitBurst(Vec2 center, real speed, int count, GrainKind k);
    void emitJet(Vec2 origin, Vec2 dir, real speed, int count, GrainKind k);
    void update(real dt);    void clear();
    void applyExplosion(Vec2 center, real radius, real impulse);
    size_t count() const;    const std::vector<Grain>& grains() const;
    real packedFraction() const;   int contactsLastStep() const;
};

class WeatherSystem {
public:
    enum class Kind { Clear, Rain, Snowfall, Hail, Sandstorm };
    void configure(Kind k, real intensity, Vec2 wind = Vec2(0,0));
    void setArea(Vec2 min, Vec2 max);
    void update(real dt, GrainSystem& grains);
    Kind kind() const;  real intensity() const;
    size_t spawnedTotal() const;   Vec2 wind() const;
};
```

## 8. Vehicles
```cpp
struct WheelSetup { Vec2 anchor; real radius = 0.35, travel = 0.30,
                    stiffnessScale = 1, damperScale = 1;
                    bool driven = true, steered = false, brakes = true; };

class WheeledVehicle {
public:
    struct Config { Vec2 position; real halfWidth = 1.7, halfHeight = 0.42,
                    mass = 1100, driveAccel = 7, topSpeed = 46, brakeAccel = 11,
                    ridePercent = 0.35, damperRatio = 0.75, grip = 1, downforce = 0;
                    int materialIndex = 0; };
    void create(World& w, const Config& cfg, const std::vector<WheelSetup>& wheels);
    void destroy();
    void update(real dt, real throttle, real brake, real lean);
    void postStep();                     // REQUIRED after world.step
    BodyId chassis() const;  const std::vector<WheelState>& wheels() const;
    real speedKmh() const, suspensionTravelUsed() const, odometer() const;
    bool airborne() const;
    real springRate() const, damperRate() const;
};

class TrackedVehicle {   // tracks
public:
    struct Config { Vec2 position; real halfLength = 3, halfHeight = 0.6,
                    mass = 24000; int roadWheels = 6;
                    real wheelRadius = 0.45, trackGrip = 2.2,
                    drivePower = 520000, turretMass = 3000; };
    void create(World&, const Config&);
    void update(real dt, real leftTrack, real rightTrack);
    void postStep();
    BodyId hull() const, turret() const;
    real trackSpeed() const, groundPressure() const;   // kPa
    int roadWheels() const;
};

class Aircraft {
public:
    struct Wing { Vec2 offset; real area = 6, liftSlope = 5.8, stallAngle = 0.28,
                  dragBase = 0.02, controlGain = 0; };
    struct Config { Vec2 position; real mass = 900, halfWidth = 3.2,
                    halfHeight = 0.5, thrust = 5200, airDensity = 1.225;
                    bool rotorcraft = false; real rotorThrust = 14000; };
    void create(World&, const Config&, const std::vector<Wing>& wings);
    void update(real dt, real throttle, real pitchInput);
    BodyId body() const;
    real angleOfAttack() const, lastLift() const, lastDrag() const, airspeed() const;
    bool stalled() const;   void setWind(Vec2);
};
```

**Order in loop:** `update(dt, ...)` → `world.step(dt)` → `postStep()`.

## 9. Ocean
```cpp
class Ocean {
public:
    struct Wave { real amplitude = 0.4, length = 12, speed = 4, phase = 0,
                  steepness = 0.6; };
    real fluidDensity = 1025, dragCoefficient = 1.1, angularDrag = 0.6;

    void attach(World& w);
    void clearWaves();   void addWave(const Wave&);
    void makeSea(int count, real windSpeed, uint32_t seed = 4242);
    void setShore(real x, real slope);   void setLevel(real y);  real level() const;
    real height(real x, real time) const;
    Vec2 surfacePoint(real x, real time) const;
    Vec2 orbitalVelocity(Vec2 p, real time) const;
    real depth(real x) const;   real breakingIntensity(real x, real time) const;
    void applyBuoyancy(real time, real dt);
    size_t floatingBodies() const;   int waveCount() const;
};
```

## 10. Network Physics
```cpp
struct NetBodyState { uint32_t id; float x, y, angle, vx, vy, w; };
struct NetSnapshot  { uint32_t tick; double time;
                      std::vector<NetBodyState> bodies;
                      const NetBodyState* find(uint32_t id) const; };
struct NetInput     { uint32_t tick, client, buttons; float axisX, axisY; };

class SnapshotCodec {
public:
    static std::vector<uint8_t> encode(const NetSnapshot& cur, const NetSnapshot* prev);
    static bool decode(const std::vector<uint8_t>& data, NetSnapshot& out);
    static size_t rawSize(const NetSnapshot&);
    static real posQuantum();   // 1/512 m
    static real angQuantum();   // 1/2048 rad
    static real velQuantum();   // 1/256
};

class ServerAuthority {
public:
    void attach(World&);   void setHistoryLength(int ticks);
    void setInputHandler(NetInputHandler fn);
    void pushInput(const NetInput&);
    void step(real dt);
    NetSnapshot capture() const;
    bool rewind(uint32_t tick);   void restore();
    RigidBody* rewindQueryPoint(Vec2 at);      // lag compensation
    uint32_t tick() const;  size_t historySize() const;
};

class ClientPredictor {
public:
    void attach(World&);   void setInputHandler(NetInputHandler fn);
    void pushLocalInput(const NetInput&);
    void predict(real dt);
    void applySnapshot(const NetSnapshot&);
    real reconciliationError() const;
    void setSmoothing(real);   uint32_t tick() const;   size_t unacked() const;
};

class InterpolationBuffer {
public:
    void setDelay(real seconds);   void push(const NetSnapshot&);
    bool sample(double now, NetSnapshot& out);
    size_t size() const;   void clear();
};
```

## 11. Acoustics
```cpp
struct Voice { Vec2 position; real gain, pitch, delay, pan, reverbTail, frequency;
               int materialA, materialB; };

class Acoustics {
public:
    struct Wall { Vec2 a, b; int material = 0; };
    void attach(World&);
    void setListener(Vec2 position, Vec2 velocity);
    void addWall(const Wall&);   void clearWalls();
    void setSpeedOfSound(real);  void setRoom(real volume, real surface);
    void impact(Vec2 at, real impulseJ, int matA, int matB);
    void footstep(Vec2 at, int material, real weight);
    void update(real dt);
    const std::vector<Voice>& voices() const;
    real reverbTime60() const;
    real dopplerFor(Vec2 sourcePos, Vec2 sourceVel) const;
    int wallCount() const;   real masterLevel() const;
};
```

## 12. Mobile Optimization
```cpp
struct CpuFeatures { bool sse2, sse4, avx, avx2, neon; int cores; std::string isa; };
CpuFeatures detectCpu();
const char* simdBackend();

class PowerManager {
public:
    enum class Profile { Desktop, Mobile, Battery };
    void attach(World&);
    void setProfile(Profile);   Profile profile() const;
    void update(real dt);
    real suggestTimestep() const;      // from 1/120 to 1/15
    int  suggestIterations() const;
    bool shouldRenderFrame() const;
    real activity() const;             // 0..1, fraction of awake bodies
    real estimatedPowerW() const;
    int  skippedFrames() const;
};
```

---

# 6. Code Examples

## 1. World, Ground, and Box Tower
```cpp
World world;
world.setGravity(Vec2(0, -9.81));
world.config().solver.velocityIterations = 18;
world.config().solver.positionIterations = 12;
world.config().solver.baumgarte = 0.22;

BodyDef g; g.type = BodyType::Static;
g.shape = Shape::box(200, 2); g.position = Vec2(0, -1);
g.material.staticFriction = 0.9; g.material.dynamicFriction = 0.75;
world.createBody(g);

for (int i = 0; i < 12; ++i) {
    BodyDef b;
    b.shape = Shape::box(0.8, 0.8);
    b.position = Vec2(0, 0.4 + i * 0.81);
    b.material = MaterialLibrary::instance().physics(23);   // oak
    b.material.restitution = 0.02;
    world.createBody(b);
}
```

## 2. Fixed Timestep with Accumulator
```cpp
const real FIXED = 1.0 / 120.0;
real acc = 0;
while (running) {
    real frame = clampr(realDeltaTime(), 0.0, 0.25);
    acc += frame;
    while (acc >= FIXED) { world.step(FIXED); acc -= FIXED; }
    render(world, acc / FIXED);          // alpha for interpolation
}
```

## 3. Contact Callbacks and Impact Sound
```cpp
nova::Acoustics audio;  audio.attach(world);
audio.setRoom(400, 320);

world.setBeginContactCallback([&](const CollisionEvent& e) {
    if (e.normalImpulse < 4.0) return;
    int ma = matIndexOf(e.a), mb = matIndexOf(e.b);
    audio.impact(e.point, e.normalImpulse, ma, mb);
    if (e.normalImpulse > 120.0) markForFracture(e.a);
});
```

## 4. Safe Body Deletion
```cpp
std::vector<BodyId> doomed;
for (RigidBody* b : world.bodies())
    if (b->position.y < -50.0) doomed.push_back(idOf(b));
for (BodyId id : doomed) world.destroyBody(id);   // only AFTER the loop
```

## 5. Vehicle with Suspension
```cpp
nova::WheeledVehicle car;
nova::WheeledVehicle::Config cfg;
cfg.position = Vec2(0, 2);  cfg.mass = 1100;
cfg.driveAccel = 7;  cfg.topSpeed = 46;  cfg.ridePercent = 0.30;

std::vector<nova::WheelSetup> wheels(2);
wheels[0].anchor = Vec2(-1.2, -0.3);  wheels[0].radius = 0.34;
wheels[1].anchor = Vec2( 1.2, -0.3);  wheels[1].radius = 0.34;
car.create(world, cfg, wheels);

// loop
car.update(dt, throttle, brake, lean);
world.step(dt);
car.postStep();
printf("%.0f km/h, suspension %.0f%%\n",
       car.speedKmh(), car.suspensionTravelUsed() * 100.0);
```

## 6. Tank and Aircraft
```cpp
nova::TrackedVehicle tank;
nova::TrackedVehicle::Config tc;  tc.position = Vec2(-20, 3);
tank.create(world, tc);
tank.update(dt, 1.0, 0.6);          // left/right track

nova::Aircraft plane;
nova::Aircraft::Config pc;  pc.position = Vec2(0, 100);
std::vector<nova::Aircraft::Wing> wings(2);
wings[0].offset = Vec2(0, 0);     wings[0].area = 11.0;
wings[1].offset = Vec2(-3.0, 0);  wings[1].area = 2.2;  wings[1].controlGain = 0.25;
plane.create(world, pc, wings);

real err = 100.0 - world.body(plane.body())->position.y;
real vy  = world.body(plane.body())->velocity.y;
real w   = world.body(plane.body())->angularVelocity;
real pitch = clampr(err * 0.02 - vy * 0.06 - w * 0.30, -0.4, 0.4);
plane.update(dt, 1.0, pitch);
```

## 7. Lua Scene Script
```cpp
nova::LuaVM vm;  vm.bindWorld(world);
std::string err;
bool ok = vm.doString(R"(
  set_gravity(0, -9.81)
  spawn_static(0, -1, 200, 2)
  for i = 0, 9 do
    spawn_box(0, 2 + i * 0.9, 0.8, 0.8)
  end
  step(1/60, 120)
  print("bodies: " .. body_count())
)", &err);
if (!ok) printf("%s\n", err.c_str());
printf("%s", vm.output().c_str());
```

## 8. Level Generation
```cpp
nova::CaveParams cave;  cave.width = 96; cave.height = 48; cave.seed = 1337;
auto grid = nova::ProcGen::caveGrid(cave);
auto ids  = nova::ProcGen::buildCave(world, cave);
Vec2 spawn = nova::ProcGen::findSpawn(grid, cave.width, cave.height,
                                      cave.cell, cave.origin);

nova::MazeParams maze;  maze.braid = true;
nova::ProcGen::buildMaze(world, maze);

nova::CityParams city;  city.blocks = 8;
nova::ProcGen::buildCity(world, city);
```

## 9. Flag, Hair, and Rope
```cpp
nova::ClothSystem cloth;  cloth.attach(world);
int flag = cloth.createFlag(Vec2(-3, 6), 8, 6, 0.20, 1.0);
cloth.createHair(Vec2(2, 5), 12, 1.2, 0.9);
int rope = cloth.createRopeStrand(Vec2(-2, 8), Vec2(2, 8), 12);
cloth.pinToBody(rope, 11, crateId, Vec2(0, 0.4));
cloth.setWind(Vec2(6, 0), 0.4);
cloth.update(dt, 6);
```

## 10. Sand, Explosion, and Weather
```cpp
nova::GrainSystem grains;  grains.attach(world);  grains.reserve(20000);
grains.emitPile(Vec2(4, 0.1), 0.8, 0.6, 0.06, nova::GrainKind::Sand);
grains.emitBurst(Vec2(0, 3), 18.0, 400, nova::GrainKind::Spark);
grains.applyExplosion(Vec2(0, 1), 6.0, 900.0);

nova::WeatherSystem weather;
weather.setArea(Vec2(-60, 0), Vec2(60, 40));
weather.configure(nova::WeatherSystem::Kind::Rain, 0.8, Vec2(-4, 0));
weather.update(dt, grains);
grains.update(dt);
```

## 11. Ocean and Boat
```cpp
nova::Ocean sea;  sea.attach(world);
sea.makeSea(9, 12.0, 77);
sea.setLevel(0.0);
sea.setShore(55.0, 0.06);

BodyDef hull;
hull.shape = Shape::box(6.0, 1.6);
hull.position = Vec2(0, 0.6);
hull.material.density = 320.0;          // lighter than water — floats
BodyId boat = world.createBody(hull);

real t = 0;
for (int i = 0; i < 600; ++i) {
    sea.applyBuoyancy(t, dt);
    world.step(dt);
    t += dt;
}
```

## 12. Networking: Server, Client, Lag Compensation
```cpp
// --- server ---
nova::ServerAuthority srv;  srv.attach(serverWorld);
srv.setHistoryLength(64);
srv.setInputHandler([](World& w, const nova::NetInput& in) {
    RigidBody* p = w.body(in.client);
    if (p) p->applyForce(Vec2(in.axisX * 900.0, 0));
});
srv.pushInput(input);
srv.step(1.0 / 60.0);

nova::NetSnapshot cur = srv.capture();
auto packet = nova::SnapshotCodec::encode(cur, &prev);   // delta
prev = cur;

// shot with lag compensation
if (srv.rewind(shot.tick)) {
    RigidBody* victim = srv.rewindQueryPoint(shot.aimPoint);
    if (victim) applyDamage(victim, 35);
    srv.restore();
}

// --- client ---
nova::ClientPredictor cli;  cli.attach(clientWorld);
cli.setInputHandler(sameHandler);
cli.pushLocalInput(input);
cli.predict(1.0 / 60.0);

nova::NetSnapshot got;
if (nova::SnapshotCodec::decode(packet, got)) cli.applySnapshot(got);

// --- remote players ---
nova::InterpolationBuffer buf;  buf.setDelay(0.1);
buf.push(got);
nova::NetSnapshot shown;
if (buf.sample(nowSeconds, shown)) drawRemotePlayers(shown);
```

## 13. Editor with Mouse
```cpp
nova::EditorCore ed;  ed.attach(world);

if (mouseDown)  ed.pick(mouseWorld);
if (mouseHeld)  ed.drag(mouseWorld);
if (mouseUp)    ed.release();
if (key(VK_DELETE)) ed.deleteSelection();
if (ctrl && key('D')) ed.duplicateSelection();
if (ctrl && key('Z')) ed.undo();
if (key(VK_SPACE)) ed.setPaused(!ed.paused());
if (rightClick) ed.spawn(currentPaletteIndex, mouseWorld);
ed.saveScene("scene.json");
drawText(8, 8, ed.statusLine().c_str());
```

## 14. 2.5D Layers
```cpp
nova::DepthWorld depth;  depth.attach(world);
depth.setLayer(backgroundTree, -1);
depth.setLayer(player,          0);
depth.setLayer(foregroundBush, +1);
depth.linkLayers(-1, +1, false);

for (BodyId id : depth.drawOrder()) {
    int L = depth.layer(id);
    drawBody(world.body(id),
             depth.parallax(L), depth.scaleFor(L, cameraZ),
             depth.info(L).tint, depth.info(L).fog);
}
```

## 15. Save and Load
```cpp
world.saveToFile("save.json");
World loaded;
loaded.loadFromFile("save.json");
std::string compact = world.toJson(false);   // no indentation, for network
```

## 16. Profiling
```cpp
const ProfileData& p = world.profile();
printf("broad %.2f  narrow %.2f  solve %.2f  integrate %.2f ms\n",
       p.broadPhaseMs, p.narrowPhaseMs, p.solverMs, p.integrateMs);
printf("bodies %zu  awake %zu  contacts %zu  arena %zu KB\n",
       world.bodyCount(), world.awakeCount(), world.contactCount(),
       world.arenaBytes() / 1024);
```

---

# 7. Demos, Games, and Benchmarks

## Building Demos
```bash
g++ -std=c++17 -O2 -Iinclude examples/<name>.cpp src/*.cpp -pthread -o build/<name>.exe
```

For graphical demos add `-DPHYS2D_D3D9` and `-ld3d9` at the very end.

## Console Tests

| File | What it does | Keys |
| :--- | :--- | :--- |
| `test_scene.cpp` | stress: 1000 circles, 500 boxes, 200 polygons, 50 capsules | `--steps N` |
| `demo_v2.cpp` | all Extras modules: CCD, fracture, softbody, ragdolls, SPH, rays | `--steps N` |
| `adv_demo.cpp` | Advanced2: compound, wind, layers, thermal map, presets, CONFIG.TXT | `--steps N` |
| `nova_demo.cpp` | all 12 Nova subsystems in sequence | `--steps N`, default 240 |
| `water_test.cpp` | 715 PBF particles, water stability test | `--steps N` |
| `realtime_scene.cpp` | ms/step measurement by solver phases, no graphics | `--steps N` |
| `c_example.c` | C-API usage | — |

## Games and Sandboxes

### GoreLab Remake (`gorelab_remake.cpp`, `gorelab_remake2.cpp`)
People Playground clone: left menu, object spawner, wind settings.
- Types: Ground, Wood, Steel, Glass, Ice, Rubber, Stone, Bomb, Balloon, Human, Debris
- 7 weapons: pistol, shotgun, laser, saw, flamethrower, gravity gun, explosives
- Ragdolls with wounds and blood, limb detachment

**Controls:** LMB — spawn/drag, RMB — delete, scroll — zoom,
`1..9` — object selection, `Space` — pause, `R` — reset, `F1` — help.

### CarCrash (`carcrash.cpp`)
Vehicles with deformable panels, wheel detachment, and broken glass.
```bash
g++ -std=c++17 -O2 -Iinclude examples/carcrash.cpp src/*.cpp -pthread -ld3d9 -o build/carcrash.exe
```

### PHYS2D DRIVE (`drive.cpp`)
Pure driving without explosions, suspension testing.
```bash
./build/drive.exe --map 0|1|2 --car 0..4 --steps N
```

| # | Model | Mass | Acceleration | Max km/h | Drop | Grip |
| :-- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0 | SEDAN | 1100 | 7.0 | 46 | 0.30 | 1.00 |
| 1 | SPORT | 850 | 9.5 | 58 | 0.22 | 1.15 |
| 2 | OFFROAD | 1900 | 6.2 | 36 | 0.46 | 1.25 |
| 3 | MONSTER | 2400 | 5.6 | 32 | 0.62 | 1.35 |
| 4 | TRUCK | 3200 | 4.2 | 28 | 0.36 | 1.05 |

**Controls:** `W`/`Up` — gas, `S`/`Down` — brake and reverse,
`A`/`D` — lean in air, `R` — respawn, `1/2/3` — map, `Tab` — car.

### Sandbox (`sandbox_game.cpp`, `sandbox_opt.cpp`)
`sandbox_opt.cpp` — optimized version: SoA particle buffers,
precomputed sinuses, screen culling, body pooling, draw batching.

### `dx_demo.cpp`
Renderer module demo: window 1440x860, 34 pixels per meter.
Requires `-DPHYS2D_D3D9 ... -ld3d9`.

## Benchmarks

| Scenario | ms/step |
| :--- | :--- |
| baseline, 1750 bodies | ~4.1 |
| cave 96x48 + 200 boxes | ~1.8 |
| DEM 1856 particles | ~2.3 |
| cloth 156 nodes, 6 iterations | ~0.4 |
| network: encoding 40 bodies | ~0.03 |
| full `nova_demo`, 240 steps | 1.44 s total |

**Tips:**
1. `allowSleep = true` for everything not player-controlled.
2. `BroadPhaseMode::Grid` for uniform bodies, `Hybrid` for mixed sizes.
3. Merge static geometry — `ProcGen` does this automatically.
4. `PowerManager` reduces iterations on idle.
5. Update grains and cloth every other step if >5000.
6. `threadCount = 0` — auto-select based on core count.

---

# 8. Troubleshooting

## Build Errors

| Message | Cause and Solution |
| :--- | :--- |
| `junk at end of line` in `embedded_tables.S` | Don't add the file to command line |
| `No such file or directory: examples/dx_demo.cpp` | `unzip -o file.zip -d /d/projects/phys2d`, check `ls examples/` |
| `undefined reference to Direct3DCreate9` | `-ld3d9` before sources. Move to the end |
| `SolverConfig has no member named stictionSpeed` | Add fields from the block below |
| `AABB has no member named 'lo'/'hi'` | fields are `min` and `max` |
| `cannot bind non-const lvalue reference of type uint8_t&` | Create `uint8_t r, g, b;` |
| `-Wformat: %d instead of size_t` | use `%zu` or cast `(int)` |

### Required `SolverConfig` fragment
```cpp
real frictionLoadFactor  = 0.0;
real stictionSpeed       = 0.05;
real warmStartDamping    = 0.92;
real maxPenetration      = 0.25;
int  maxContactPoints    = 4;
real restitutionSpeedRef = 4.0;
real restitutionFalloff  = 0.55;
```

## Runtime Errors

| Symptom | Cause |
| :--- | :--- |
| `Illegal instruction` at startup | Build without `-march=native`, then with `-mavx2`, then with `-DPHYS2D_NO_SIMD` |
| SIGSEGV on spawn | Creating/deleting bodies while iterating `world.bodies()` |
| Process killed by OOM | Spawning inside geometry: check `overlapShape` before creating |
| NaN in positions | Zero density or degenerate polygon. Enable `Diagnostics` |

## Tuning Physics Feel

### "Boxes feel like oil, nothing has weight"
```cpp
d.material = nova::MaterialLibrary::instance().physics(23);   // oak 700 kg/m³
world.config().solver.velocityIterations = 18;
world.config().solver.positionIterations = 12;
world.config().solver.baumgarte = 0.22;
world.config().solver.stictionSpeed = 0.05;
world.config().solver.restitutionThreshold = 1.20;
```

### "Wheels fall off when moving"
`postStep()` not called after `world.step()` or anchor point inside chassis:
```cpp
real clearance = cfg.halfHeight + wheelRadius + 0.05 - anchor.y;
if (restLen < clearance) restLen = clearance;
```

### "Car doesn't move"
Wheels submerged in chassis (see above) or chassis is sleeping.
Always use `allowSleep = false` on vehicle chassis.

### "Water oscillates instead of falling"
Step 1/120, 4 iterations, `eps = 120`, `maxSpeed = 26`, displacement limit `0.30*h`,
XSPH viscosity 0.02, spacing 0.20 m, kernel 0.40 m.

### "Cloth explodes and flies away"
`ClothSystem` ignores bodies with `boundingRadius > 4.0`.

### "Box stack vibrates"
`positionIterations` 12-16, `baumgarte` 0.22, `warmStartDamping` 0.92, `restitution` 0.02.

### "Bullet passes through wall"
```cpp
CcdSystem ccd;  ccd.attach(world);
ccd.speedThreshold = 4.0;   ccd.maxSubsteps = 12;
ccd.update(dt);             // after world.step()
```

---

# 9. Engine Formulas

| Where | Formula |
| :--- | :--- |
| step stability (CFL) | `dt < 0.25 * h / c` |
| friction under load | `mu = mu0 / (1 + k*Fn*(1/ma + 1/mb))` |
| SPH particle mass | `m = rho0 / sum(W)` |
| PBF constraint | `C = rho/rho0 - 1`, `lambda = -C/(sum|gradC|² + eps)` |
| Archimedes | `F = -rho * V * g` |
| fluid drag | `F = 0.5 * rho * Cd * A * |v| * v` |
| suspension spring | `F = k*(restLen - len) - c*v_rel` |
| static sag | `x = m*g/(2k)` |
| critical damping | `c_crit = 2*sqrt(k*m/2)`, working `0.75*c_crit` |
| Gerstner wave | `y = sum A*cos(kx - wt + f)`, `x' = x - sum Q*A*sin(...)` |
| Pierson-Moskowitz spectrum | `L_peak = 2*pi*U²/(g*0.769)` |
| Sabine reverberation | `RT60 = 0.161*V/(S*alpha)` |
| Doppler effect | `f' = f*(c - v_listener)/(c - v_source)` |
| Hertz contact (DEM) | `Fn = k*d^1.5 - c*d_dot` |
| speed in km/h | `length(v) * 3.6` |

---

# Cheat Sheet

1. `Shape::box(w, h)` — FULL dimensions, not half.
2. Sleeping body ignores forces — `wake()`.
3. Don't create or delete bodies while iterating `world.bodies()`.
4. Fixed timestep, preferably 1/120 with accumulator.
5. Vehicles: `update` → `world.step` → `postStep`.
6. Default material density is 1.0 — always set real density.
7. `-ld3d9` only at the end of link line.
8. `src/embedded_tables.S` never passed to g++.
9. For cloth: `boundingRadius > 4.0` is ignored.
10. For water: step 1/120, 4 PBF iterations.
