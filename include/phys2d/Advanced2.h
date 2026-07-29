// phys2d v3 — расширенный блок: составные тела, поверхности, фильтры, управление,
// поля, диагностика, стабильность, запросы, геометрия, сервисные системы.
#pragma once

#include "phys2d/Extras.h"
#include "phys2d/World.h"

#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace phys2d {

// ================================================================ единицы измерения
enum class Units { Meters, Centimeters, Pixels };

struct UnitScale {
    real toMeters = 1.0;
    static UnitScale of(Units u, real pixelsPerMeter = 32.0);
    real len(real v) const { return v * toMeters; }
    Vec2 vec(const Vec2& v) const { return v * toMeters; }
    real toUnits(real meters) const { return meters / toMeters; }
};

// ================================================================ составные тела (COMPOUND SHAPE)
struct CompoundPart {
    Shape shape   = Shape::circle(0.5);
    Vec2  offset;
    real  angle   = 0.0;
    real  density = 1000.0;
};

struct Compound {
    BodyId              root = INVALID_BODY;
    std::vector<BodyId> parts;
    Vec2                center;
    real                mass    = 0.0;
    real                inertia = 0.0;
};

// Суммарная масса/инерция через теорему Штейнера.
MassData compoundMass(const std::vector<CompoundPart>& parts);
Compound createCompound(World& w, const std::vector<CompoundPart>& parts, const Vec2& position,
                        real angle = 0.0, BodyType type = BodyType::Dynamic, real weldStiffness = 0.0);
void     destroyCompound(World& w, Compound& c);

// ================================================================ дерево материалов
enum MaterialField : uint32_t {
    MF_NONE             = 0u,
    MF_DENSITY          = 1u << 0,
    MF_RESTITUTION      = 1u << 1,
    MF_STATIC_FRICTION  = 1u << 2,
    MF_DYNAMIC_FRICTION = 1u << 3,
    MF_ROLLING          = 1u << 4,
    MF_DRAG             = 1u << 5,
    MF_ALL              = 0xFFFFFFFFu
};

class MaterialTree {
public:
    int  add(const std::string& name, const Material& m,
             const std::string& parent = std::string(), uint32_t ownMask = MF_ALL);
    int  id(const std::string& name) const;
    Material resolve(int index) const;
    Material resolve(const std::string& name) const;
    std::vector<std::string> names() const;
    size_t size() const { return m_nodes.size(); }
    void loadStandardLibrary();

private:
    struct Node {
        std::string name;
        Material    mat;
        int         parent = -1;
        uint32_t    mask   = MF_ALL;
    };
    std::vector<Node>                     m_nodes;
    std::unordered_map<std::string, int>  m_index;
};

// ================================================================ профили поверхностей
struct SurfaceProfile {
    uint32_t group = 0u;                    // COLLISION GROUP
    uint32_t mask  = 0xFFFFFFFFu;           // COLLISION MASK
    int      layer = 0;                     // LAYERS

    Vec2 frictionAxis   = Vec2(1.0, 0.0);   // анизотропное трение
    real frictionAlong  = 1.0;
    real frictionAcross = 1.0;
    real oneWayFriction = 0.0;              // ONE-WAY FRICTION
    real spinFriction   = 0.0;              // SPIN FRICTION
    real angleAbsorption = 0.0;             // поглощение по углу удара

    real restitutionAtRest  = -1.0;         // e(v): медленный удар (<0 — выкл)
    real restitutionAtSpeed = -1.0;         // e(v): быстрый удар
};

class SurfaceProfileRegistry {
public:
    SurfaceProfile&       ref(BodyId id);
    const SurfaceProfile* find(BodyId id) const;
    void                  remove(BodyId id);
    size_t                size() const { return m_map.size(); }

private:
    std::unordered_map<BodyId, SurfaceProfile> m_map;
};

// ================================================================ фильтрация столкновений
class CollisionRules {
public:
    using UserFilter = std::function<bool(const RigidBody&, const RigidBody&)>;

    SurfaceProfileRegistry surfaces;

    void   setLayerRule(int a, int b, bool collide);
    bool   layersCollide(int a, int b) const;
    void   setUserFilter(UserFilter f);
    bool   shouldCollide(const RigidBody& a, const RigidBody& b) const;
    void   install(World& w);
    size_t rejected() const { return m_rejected; }

private:
    std::unordered_map<uint64_t, bool> m_layerRules;
    UserFilter                        m_user;
    mutable size_t                    m_rejected = 0;
};

// Доводка контактов: анизотропия, one-way, spin friction, e(v), поглощение по углу.
class ContactTuner {
public:
    SurfaceProfileRegistry* surfaces       = nullptr;
    real                    speedReference = 4.0;

    void   install(World& w);
    void   onContact(const CollisionEvent& e);
    size_t adjusted() const { return m_adjusted; }
    real   energyRemoved() const { return m_energy; }

private:
    size_t m_adjusted = 0;
    real   m_energy   = 0.0;
};

// ================================================================ управление телами
struct AxisLock {
    bool x = false, y = false, rotation = false;
};

struct MotionLimits {
    real maxSpeed        = 0.0;   // HARD CAP линейной скорости
    real maxAngularSpeed = 0.0;
    real maxAngularStep  = 0.0;   // максимальный угол за шаг
};

struct BodyControl {
    Vec2 constantForce;                  // CONSTANT FORCE
    real constantTorque = 0.0;

    bool holdPosition   = false;         // SOFT CONSTRAINT с демпфированием
    Vec2 anchor;
    real holdStiffness  = 100.0;
    real holdDamping    = 10.0;

    bool useTargetVelocity = false;      // целевая скорость
    Vec2 targetVelocity;
    real targetAngular = 0.0;
    real gain          = 8.0;

    AxisLock     lock;                   // AXIS LOCK
    MotionLimits limits;
    real         smoothing = 0.0;        // экспоненциальное сглаживание позиции
};

class ControlSystem {
public:
    BodyControl&       ref(BodyId id);
    const BodyControl* find(BodyId id) const;
    void               remove(BodyId id);
    void               kick(World& w, BodyId id, const Vec2& impulse);
    void               applyBeforeStep(World& w, real dt);
    void               applyAfterStep(World& w, real dt);
    size_t             size() const { return m_map.size(); }

private:
    std::unordered_map<BodyId, BodyControl> m_map;
    std::unordered_map<BodyId, Vec2>       m_smoothed;
};

// ================================================================ поля: гравитация и ветер
enum class GravityMode { Constant, Radial, Dispersive };

struct GravityController {
    GravityMode mode     = GravityMode::Constant;
    Vec2        constant = Vec2(0.0, -9.81);
    Vec2        center;
    real        strength  = 9.81;
    real        falloff   = 2.0;
    real        minRadius = 0.5;
    Vec2        spinCenter;
    real        spinRate  = 0.0;          // центростремительная сила

    void apply(World& w, real dt);
};

struct WindField {
    Vec2 base;
    real gustAmplitude = 0.0;
    real gustFrequency = 0.35;
    real turbulence    = 0.0;
    real time          = 0.0;

    Vec2 at(const Vec2& p) const;
    void apply(World& w, real dt, real dragScale = 1.0);
};

// ================================================================ интерполяция для рендера
struct RenderTransform {
    Vec2 position;
    real angle = 0.0;
};

class MotionInterpolator {
public:
    void            capture(World& w);
    RenderTransform at(BodyId id, real alpha) const;   // alpha>1 — экстраполяция

private:
    struct Frame {
        Vec2 prevP, currP;
        real prevA = 0.0, currA = 0.0;
        Vec2 vel;
        real omega = 0.0;
    };
    std::unordered_map<BodyId, Frame> m_frames;
};

// ================================================================ диагностика
struct RuntimeStats {
    size_t steps = 0;
    real   stepMs = 0.0, avgStepMs = 0.0, maxStepMs = 0.0, fps = 0.0;
    size_t bodies = 0, dynamicBodies = 0, sleeping = 0;
    size_t contacts = 0, contactPoints = 0, constraints = 0, fluidParticles = 0;
    int    velocityIterations = 0, positionIterations = 0;
    real   energy = 0.0, energyDrift = 0.0;
    size_t memoryBytes = 0;
};

size_t estimateMemory(const World& w, size_t fluidParticles = 0);

class Diagnostics {
public:
    bool   visualize    = true;           // тумблер визуализации
    size_t historyLimit = 4096;

    void                beginStep();
    void                endStep(World& w, size_t fluidParticles = 0);
    const RuntimeStats& stats() const { return m_stats; }
    std::string         line() const;
    void                print(std::FILE* out = nullptr) const;
    bool                writeJson(const std::string& path) const;

private:
    RuntimeStats      m_stats;
    real              m_sumMs = 0.0, m_prevEnergy = 0.0;
    long long         m_t0 = 0;
    std::vector<real> m_stepMs, m_energy;
};

// ================================================================ пресеты и конфиг
enum class Preset { Accuracy, Speed, Balance };

void applyPreset(WorldConfig& cfg, Preset p);
void adaptSolver(World& w, int minIterations = 6, int maxIterations = 24);
bool loadConfigTxt(const std::string& path, WorldConfig& cfg);
bool saveConfigTxt(const std::string& path, const WorldConfig& cfg);

// ================================================================ стабильность
class StabilityGuard {
public:
    AABB bounds{Vec2(-1000.0, -1000.0), Vec2(1000.0, 1000.0)};
    bool clampToBounds  = true;
    bool deleteEscapers = false;
    bool rollbackOnNaN  = true;
    real maxSpeed        = 200.0;
    real maxAngularSpeed = 200.0;

    void   snapshot(World& w);
    bool   verify(World& w);
    size_t rollbacks() const { return m_rollbacks; }
    size_t clamps() const { return m_clamps; }
    size_t removed() const { return m_removed; }

private:
    struct Snap {
        BodyId id;
        Vec2   p, v;
        real   a, w;
    };
    std::vector<Snap> m_snap;
    size_t m_rollbacks = 0, m_clamps = 0, m_removed = 0;
};

// ================================================================ запросы
struct ShapeCastHit {
    RigidBody* body = nullptr;
    real       t    = 1.0;
    Vec2       normal;
    Vec2       point;
};

bool shapeCast(World& w, const Shape& s, const Transform& start, const Vec2& translation,
               ShapeCastHit& out, int steps = 32);
std::vector<RigidBody*> overlapShape(World& w, const Shape& s, const Transform& xf);
std::vector<RigidBody*> pointQueryAll(World& w, const Vec2& p);
size_t                  sweptTunnelGuard(World& w, real skin = 0.01);

// ================================================================ геометрические утилиты
bool  segmentIntersect(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2, Vec2& out);
bool  pointInPolygon(const std::vector<Vec2>& poly, const Vec2& p);
Vec2  projectOnSegment(const Vec2& p, const Vec2& a, const Vec2& b);
std::vector<Vec2> removeCollinear(std::vector<Vec2> pts, real tol = 1e-6);
std::vector<Vec2> sortCounterClockwise(std::vector<Vec2> pts);
bool  isConvex(const std::vector<Vec2>& pts);
Shape repairShape(Shape s);
Shape randomConvexPolygon(uint32_t& seed, int vertices = 6, real radius = 0.5);
void  reshapeBody(World& w, BodyId id, const Shape& s);

// ================================================================ нагрев от трения
class FrictionHeat {
public:
    real ambient  = 293.0;
    real heatRate = 0.05;
    real coolRate = 0.4;

    void install(World& w);
    void update(real dt);
    real temperature(BodyId id) const;
    real hottest() const;

private:
    std::unordered_map<BodyId, real> m_pending;
    std::unordered_map<BodyId, real> m_temp;
};

// ================================================================ сервисные системы
class DirtyTracker {
public:
    void scan(World& w, real epsilon = 1e-4);
    void markDirty(BodyId id);
    bool dirty(BodyId id) const;
    void clear();
    const std::vector<BodyId>& list() const { return m_list; }

private:
    std::vector<BodyId>                m_list;
    std::unordered_map<BodyId, int>    m_set;
    std::unordered_map<BodyId, Vec2>   m_last;
};

class SceneKeeper {
public:
    int         intervalSteps = 0;        // 0 — автосохранение выключено
    std::string path = "autosave.json";

    void   tick(World& w);
    size_t saves() const { return m_saves; }
    size_t steps() const { return m_steps; }

private:
    size_t m_saves = 0, m_steps = 0;
    int    m_counter = 0;
};

size_t removeOrphanStatics(World& w);

struct MemoryTracker {
    static void*  alloc(size_t bytes);           // ZERO-INIT + учёт
    static void   release(void* p, size_t bytes);
    static size_t liveBytes();
    static size_t peakBytes();
    static size_t allocations();
    static std::string report();
};

class PluginHost {
public:
    ~PluginHost();
    bool load(const std::string& path);          // входной символ: phys2d_plugin_init
    void unloadAll();
    const std::vector<std::string>& names() const { return m_names; }
    const std::string& lastError() const { return m_error; }

private:
    std::vector<void*>       m_handles;
    std::vector<std::string> m_names;
    std::string              m_error;
};

} // namespace phys2d
