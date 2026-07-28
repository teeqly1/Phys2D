// phys2d v2 - extension layer.
//
//   * collision layers / masks / groups, sensors, multi-material surfaces
//   * joints: weld, motor, prismatic, pulley, gear, catenary rope, breakable, soft limits
//   * continuous collision detection (swept circle, conservative advancement) and raycasts
//   * fracture (voronoi) and impulse driven fragmentation
//   * soft bodies: mass-spring and co-rotational FEM triangle meshes with tearing, ragdolls
//   * SPH fluid particles with viscosity, surface tension and waves
//   * force fields: barnes-hut n-body, magnetism, electrostatics, perlin wind, coriolis,
//     centrifugal, tidal and light pressure
//   * thermal model: conduction, expansion, temperature dependent coefficients, phase change
//   * runtime services: 64-bit handles, object pools, GC, event log with rewind, safety guard,
//     islands, console scripting, command line, benchmarks and reports
//   * geometry services: concave decomposition, bezier/NURBS, terrain, BSP, BVH, OBJ import,
//     Box2D / Bullet export
//
// Everything here is additive: the core World keeps working exactly as before and every system
// is opt-in. Systems share one base hierarchy so they can be registered in a single Engine.
#pragma once

#include "World.h"

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace phys2d {

// ================================================================ 1. base hierarchy
// Object -> Named -> Trackable -> SimulationSystem -> ForceSystem -> concrete systems.
class Object {
public:
    Object();
    virtual ~Object();
    uint64_t uid() const { return m_uid; }
    virtual const char* className() const { return "Object"; }

protected:
    uint64_t m_uid = 0;
};

class Named : public Object {
public:
    std::string name;
    const char* className() const override { return "Named"; }
};

class Trackable : public Named {
public:
    bool     enabled = true;
    double   lastMs = 0.0;      // время последнего update, мс
    double   averageMs = 0.0;
    uint64_t updates = 0;
    const char* className() const override { return "Trackable"; }
};

class SimulationSystem : public Trackable {
public:
    virtual void attach(World& w) { m_world = &w; }
    virtual void update(real dt) = 0;
    void         tick(real dt);                  // update + замер времени
    World*       world() const { return m_world; }
    const char*  className() const override { return "SimulationSystem"; }

protected:
    World* m_world = nullptr;
};

class ForceSystem : public SimulationSystem {
public:
    real strengthScale = 1.0;
    const char* className() const override { return "ForceSystem"; }

protected:
    void addForce(RigidBody& b, const Vec2& f) const;
};

// ================================================================ 2. filtering / materials
struct Filter {
    uint32_t category = 0x0001;        // к каким слоям принадлежит тело
    uint32_t mask     = 0xFFFFFFFFu;   // с какими слоями сталкивается
    int32_t  group    = 0;             // >0 всегда сталкиваются, <0 никогда
    bool     sensor   = false;         // не влияет на физику, только события
};

class FilterRegistry : public Named {
public:
    void   set(BodyId id, const Filter& f);
    Filter get(BodyId id) const;
    void   remove(BodyId id);
    bool   shouldCollide(BodyId a, BodyId b) const;
    void   install(World& w);          // ставит World::setContactFilter
    size_t size() const { return m_map.size(); }

private:
    std::unordered_map<BodyId, Filter> m_map;
};

// Поверхностный материал в справочной таблице (50 материалов, 50x50 пар).
struct SurfaceMaterial {
    const char* name = "";
    real density = 1.0, restitution = 0.3, friction = 0.5;
    real conductivity = 20.0, heatCapacity = 800.0, expansion = 1.2e-5;
    real meltingPoint = 1500.0, youngModulus = 7.0e10, tensileStrength = 2.0e8;
};

struct MaterialPair {
    real friction = 0.5, restitution = 0.3, conductance = 10.0, adhesion = 0.0;
};

const SurfaceMaterial& materialById(int id);              // 0..49
int                    materialByName(const char* name);  // -1 если нет
const MaterialPair&    materialPair(int a, int b);        // таблица 50x50
int                    materialCount();

// Разные материалы на разных гранях одного тела + анизотропное трение.
struct SurfaceAssignment {
    std::vector<int> edgeMaterial;      // индекс материала на каждое ребро
    int              defaultMaterial = 0;
    Vec2             anisotropyAxis{1.0, 0.0};   // локальная ось
    real             frictionAlong = 1.0;        // множитель вдоль оси
    real             frictionAcross = 1.0;       // множитель поперёк
};

class SurfaceRegistry : public SimulationSystem {
public:
    void set(BodyId id, const SurfaceAssignment& s);
    const SurfaceAssignment* get(BodyId id) const;
    void update(real dt) override;      // анизотропное трение по текущим контактам
    const char* className() const override { return "SurfaceRegistry"; }

private:
    std::unordered_map<BodyId, SurfaceAssignment> m_map;
};

// ---- сенсоры и триггеры
enum class SensorPhase : uint8_t { Enter = 0, Stay = 1, Exit = 2 };

struct SensorEvent {
    BodyId      sensor = INVALID_BODY;
    BodyId      other  = INVALID_BODY;
    SensorPhase phase  = SensorPhase::Enter;
    Vec2        point;
    real        overlap = 0.0;
};

using SensorCallback = std::function<void(const SensorEvent&)>;

class SensorSystem : public SimulationSystem {
public:
    void add(BodyId id, uint32_t mask = 0xFFFFFFFFu);
    void remove(BodyId id);
    void setCallback(SensorCallback cb) { m_cb = std::move(cb); }
    void setFilterRegistry(FilterRegistry* fr) { m_filters = fr; }
    void update(real dt) override;
    size_t overlapCount() const { return m_current.size(); }
    const char* className() const override { return "SensorSystem"; }

private:
    std::unordered_map<BodyId, uint32_t>  m_sensors;
    std::unordered_set<uint64_t>          m_current, m_previous;
    SensorCallback                        m_cb;
    FilterRegistry*                       m_filters = nullptr;
};

// ================================================================ 3. joints v2
// Мягкие ограничения: stiffness/damping -> биас и softness по схеме soft-constraint.
struct SoftParams {
    real frequencyHz = 0.0;   // 0 => жёсткое ограничение
    real dampingRatio = 1.0;
};

class WeldJoint : public Constraint {
public:
    real referenceAngle = 0.0;
    SoftParams soft;
    ConstraintType type() const override { return ConstraintType::Weld; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    void solvePosition(real dt, real schlagerFactor) override;

private:
    Mat22 m_K;
    Vec2  m_linImpulse;
    real  m_angMass = 0.0, m_angImpulse = 0.0;
    real  m_gamma = 0.0, m_bias = 0.0;
};

class MotorJoint : public Constraint {
public:
    real targetSpeed = 0.0;      // рад/с (угловой) либо м/с (линейный)
    real maxTorque   = 100.0;
    bool linear      = false;    // true => линейный мотор вдоль axis
    Vec2 axis{1.0, 0.0};
    ConstraintType type() const override { return ConstraintType::Motor; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    real appliedTorque() const { return m_impulse; }

private:
    real m_mass = 0.0, m_impulse = 0.0;
};

class PrismaticJoint : public Constraint {
public:
    Vec2 localAxisA{1.0, 0.0};   // ось скольжения в системе A
    real referenceAngle = 0.0;
    bool enableLimit = false;
    real lowerTranslation = -1.0, upperTranslation = 1.0;
    bool enableMotor = false;
    real motorSpeed = 0.0, maxMotorForce = 0.0;
    SoftParams softLimit;        // мягкий упор вместо жёсткого
    ConstraintType type() const override { return ConstraintType::Prismatic; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    void solvePosition(real dt, real schlagerFactor) override;
    real translation() const { return m_translation; }

private:
    Vec2 m_axis, m_perp;
    real m_s1 = 0.0, m_s2 = 0.0, m_a1 = 0.0, m_a2 = 0.0;
    real m_perpMass = 0.0, m_angMass = 0.0, m_axialMass = 0.0;
    real m_perpImpulse = 0.0, m_angImpulse = 0.0, m_motorImpulse = 0.0, m_limitImpulse = 0.0;
    real m_translation = 0.0;
    int  m_limitState = 0;
};

class PulleyJoint : public Constraint {
public:
    Vec2 groundA, groundB;       // мировые точки подвеса
    real ratio = 1.0;            // передаточное отношение
    real totalLength = 4.0;      // lenA + ratio * lenB
    ConstraintType type() const override { return ConstraintType::Pulley; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    void solvePosition(real dt, real schlagerFactor) override;

private:
    Vec2 m_uA, m_uB;
    real m_mass = 0.0, m_impulse = 0.0, m_C = 0.0;
};

class GearJoint : public Constraint {
public:
    real ratio = 1.0;            // omegaA + ratio * omegaB = 0
    ConstraintType type() const override { return ConstraintType::Gear; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    void solvePosition(real dt, real schlagerFactor) override;

private:
    real m_mass = 0.0, m_impulse = 0.0, m_C = 0.0;
};

// Верёвка с провисанием: цепная линия между двумя точками.
std::vector<Vec2> catenaryPoints(const Vec2& a, const Vec2& b, real ropeLength, int samples = 24);

struct RopeChain {
    std::vector<BodyId>      links;
    std::vector<Constraint*> joints;
};

// Строит физическую верёвку из сегментов по цепной линии.
RopeChain buildCatenaryRope(World& w, const Vec2& a, const Vec2& b, real ropeLength,
                            int segments = 16, real thickness = 0.08, real density = 1.0);

// Разрушаемые джойнты.
struct JointBreakEvent {
    Constraint* joint = nullptr;
    real        force = 0.0;
    real        threshold = 0.0;
};

class BreakableJointSystem : public SimulationSystem {
public:
    void watch(Constraint* c, real breakForce);
    void setCallback(std::function<void(const JointBreakEvent&)> cb) { m_cb = std::move(cb); }
    void update(real dt) override;
    int  brokenCount() const { return m_broken; }
    const char* className() const override { return "BreakableJointSystem"; }

private:
    std::vector<std::pair<Constraint*, real>>   m_watch;
    std::function<void(const JointBreakEvent&)> m_cb;
    int m_broken = 0;
};

// Удобные фабрики (тела ищутся в мире по id).
WeldJoint*      createWeld     (World& w, BodyId a, BodyId b, const Vec2& worldAnchor, const SoftParams& s = {});
MotorJoint*     createMotor    (World& w, BodyId a, BodyId b, real targetSpeed, real maxTorque);
PrismaticJoint* createPrismatic(World& w, BodyId a, BodyId b, const Vec2& worldAnchor, const Vec2& worldAxis);
PulleyJoint*    createPulley   (World& w, BodyId a, BodyId b, const Vec2& groundA, const Vec2& groundB,
                                const Vec2& localA, const Vec2& localB, real ratio);
GearJoint*      createGear     (World& w, BodyId a, BodyId b, real ratio);

// ================================================================ 4. CCD и лучи
struct SweepResult {
    bool       hit = false;
    real       t = 1.0;             // доля шага [0..1]
    Vec2       point, normal;
    RigidBody* body = nullptr;
    int        iterations = 0;
};

struct RayHit {
    RigidBody* body = nullptr;
    Vec2       point, normal;
    real       fraction = 1.0;
};

// Аналитический swept circle против отрезка / AABB.
bool sweptCircleSegment(const Vec2& c0, const Vec2& c1, real r,
                        const Vec2& a, const Vec2& b, real& t, Vec2& normal);
bool sweptCircleAABB(const Vec2& c0, const Vec2& c1, real r, const AABB& box, real& t, Vec2& normal);

// Conservative advancement / continuous GJK для произвольных выпуклых форм.
SweepResult conservativeAdvancement(const RigidBody& a, const RigidBody& b, real dt,
                                    int maxIterations = 32, real tolerance = 1e-4);

class CcdSystem : public SimulationSystem {
public:
    real speedThreshold = 6.0;     // м за шаг относительно размера тела
    int  maxSubsteps = 8;
    void update(real dt) override; // откатывает быстрые тела к моменту касания
    int  resolvedThisStep() const { return m_resolved; }
    const char* className() const override { return "CcdSystem"; }

private:
    int m_resolved = 0;
};

// Лучи: ближайший и все пересечения с фильтром по слоям.
bool rayCastClosest(World& w, const Vec2& p1, const Vec2& p2, RayHit& out,
                    const FilterRegistry* filters = nullptr, uint32_t mask = 0xFFFFFFFFu);
int  rayCastAll(World& w, const Vec2& p1, const Vec2& p2, std::vector<RayHit>& out,
                const FilterRegistry* filters = nullptr, uint32_t mask = 0xFFFFFFFFu);
bool rayCastShape(const RigidBody& b, const Vec2& p1, const Vec2& p2, RayHit& out);

// ================================================================ 5. разрушение
// Разбиение выпуклой формы диаграммой Вороного по набору точек.
std::vector<Shape> voronoiFracture(const Shape& shape, const std::vector<Vec2>& sites);
std::vector<Shape> radialFracture(const Shape& shape, const Vec2& impactLocal, int pieces, uint32_t seed = 1);

struct FractureEvent {
    BodyId              original = INVALID_BODY;
    std::vector<BodyId> fragments;
    Vec2                impactPoint;
    real                impulse = 0.0;
};

class FractureSystem : public SimulationSystem {
public:
    real impulseThreshold = 12.0;   // импульс в точке контакта для разлома
    int  pieces = 5;
    int  maxFragmentsPerStep = 24;
    real minFragmentArea = 0.02;
    int  maxGeneration = 2;         // сколько раз осколок может ломаться дальше
    void setCallback(std::function<void(const FractureEvent&)> cb) { m_cb = std::move(cb); }
    void makeBreakable(BodyId id, real threshold = -1.0);
    void update(real dt) override;
    int  fracturesTotal() const { return m_total; }
    const char* className() const override { return "FractureSystem"; }

    // Ручной разлом.
    std::vector<BodyId> fracture(BodyId id, const Vec2& impactWorld, int pieces);

private:
    struct Entry { real threshold; int generation; };
    std::unordered_map<BodyId, Entry>            m_breakable;
    std::function<void(const FractureEvent&)>    m_cb;
    int m_total = 0;
};

// ================================================================ 6. мягкие тела
struct SoftNode {
    Vec2 position, previous, velocity, force;
    real mass = 1.0, invMass = 1.0;
    real temperature = 293.0;
    bool pinned = false;
};

struct SoftLink {
    int  a = 0, b = 0;
    real restLength = 1.0;
    real stiffness = 800.0;
    real damping = 4.0;
    real breakStrain = 0.0;   // 0 => не рвётся
    bool broken = false;
};

struct SoftTriangle {
    int  a = 0, b = 0, c = 0;
    real restArea = 0.0;
    Mat22 invRest;            // для co-rotational FEM
    bool broken = false;
};

enum class SoftModel : uint8_t { MassSpring = 0, FEM = 1, Hybrid = 2 };

class SoftBody : public Named {
public:
    std::vector<SoftNode>     nodes;
    std::vector<SoftLink>     links;
    std::vector<SoftTriangle> triangles;

    SoftModel model = SoftModel::Hybrid;
    real youngModulus = 4.0e4;   // FEM
    real poissonRatio = 0.30;
    real damping = 6.0;
    real pressure = 0.0;         // внутреннее давление (надувные тела)
    real tearStrain = 0.0;       // 0 => не рвётся
    real friction = 0.4;
    real restitution = 0.1;
    bool selfCollision = false;
    real nodeRadius = 0.06;

    // Построение
    static SoftBody grid(const Vec2& origin, real w, real h, int nx, int ny, real totalMass);
    static SoftBody fromPolygon(const std::vector<Vec2>& poly, real spacing, real totalMass);
    static SoftBody rope(const Vec2& a, const Vec2& b, int segments, real totalMass);

    void  rebuildRest();
    Vec2  centroid() const;
    real  area() const;
    AABB  bounds() const;
    int   componentCount() const;   // связность после разрывов
    const char* className() const override { return "SoftBody"; }
};

class SoftBodySystem : public SimulationSystem {
public:
    int  solverIterations = 8;
    Vec2 gravity{0.0, -9.81};
    bool coupleWithRigid = true;

    SoftBody* add(SoftBody body);
    void      clear();
    size_t    count() const { return m_bodies.size(); }
    SoftBody* at(size_t i) { return m_bodies[i].get(); }
    void      update(real dt) override;
    int       tearsThisStep() const { return m_tears; }
    const char* className() const override { return "SoftBodySystem"; }

private:
    void integrate(SoftBody& sb, real dt);
    void solveLinks(SoftBody& sb, real dt);
    void solveFem(SoftBody& sb, real dt);
    void collideRigid(SoftBody& sb, real dt);

    std::vector<std::unique_ptr<SoftBody>> m_bodies;
    int m_tears = 0;
};

// ---- ragdoll из твёрдых тел с угловыми ограничениями
struct RagdollConfig {
    Vec2 position{0.0, 0.0};
    real scale = 1.0;
    real density = 1.0;
    real jointFriction = 0.05;
};

struct Ragdoll {
    BodyId head = INVALID_BODY, torso = INVALID_BODY, pelvis = INVALID_BODY;
    BodyId armL = INVALID_BODY, armR = INVALID_BODY, forearmL = INVALID_BODY, forearmR = INVALID_BODY;
    BodyId thighL = INVALID_BODY, thighR = INVALID_BODY, shinL = INVALID_BODY, shinR = INVALID_BODY;
    std::vector<BodyId>      parts;
    std::vector<Constraint*> joints;
};

Ragdoll createRagdoll(World& w, const RagdollConfig& cfg);

// ================================================================ 7. SPH частицы
struct FluidParticle {
    Vec2 position, velocity, force;
    real density = 0.0, pressure = 0.0;
    real temperature = 293.0;
    real mass = 1.0;
    uint32_t material = 0;
    bool active = true;
};

class ParticleSystem : public SimulationSystem {
public:
    real smoothingRadius = 0.35;
    real restDensity = 1000.0;
    real stiffness = 800.0;        // уравнение состояния
    real viscosity = 12.0;
    real surfaceTension = 0.8;
    real particleMass = 1.0;
    Vec2 gravity{0.0, -9.81};
    real boundaryDamping = 0.4;
    AABB domain;
    bool coupleWithRigid = true;

    void   emitBlock(const Vec2& origin, real w, real h, real spacing);
    void   emit(const Vec2& p, const Vec2& v);
    void   clear();
    size_t count() const { return m_particles.size(); }
    const std::vector<FluidParticle>& particles() const { return m_particles; }
    real   surfaceHeight(real x) const;      // волны на поверхности
    void   update(real dt) override;
    const char* className() const override { return "ParticleSystem"; }

private:
    void buildGrid();
    void computeDensity();
    void computeForces();
    void integrate(real dt);

    std::vector<FluidParticle> m_particles;
    std::unordered_map<uint64_t, std::vector<int>> m_grid;
    std::vector<real> m_waveHeight, m_waveVelocity;
};

// ================================================================ 8. поля и термика
struct BodyPhysics {
    real charge = 0.0;              // Кл
    real magneticMoment = 0.0;      // А*м^2
    real temperature = 293.0;       // К
    real heatCapacity = 900.0;
    real conductivity = 30.0;
    real expansion = 1.2e-5;        // 1/К
    real meltingPoint = 1400.0;
    real emissivity = 0.6;
    real piezoCoefficient = 0.0;    // Кл/Н
    real accumulatedCharge = 0.0;
    bool molten = false;
    real baseScale = 1.0;
};

class BodyPhysicsRegistry : public Named {
public:
    BodyPhysics&       ref(BodyId id);
    const BodyPhysics* find(BodyId id) const;
    void               remove(BodyId id);
    std::unordered_map<BodyId, BodyPhysics>&       all()       { return m_map; }
    const std::unordered_map<BodyId, BodyPhysics>& all() const { return m_map; }

private:
    std::unordered_map<BodyId, BodyPhysics> m_map;
};

// Perlin noise 2D (для турбулентного ветра и волн).
real perlinNoise(real x, real y, uint32_t seed = 0);
real fractalNoise(real x, real y, int octaves = 4, real persistence = 0.5, uint32_t seed = 0);

enum FieldFlag : uint32_t {
    FIELD_NBODY        = 1u << 0,   // гравитация N тел (Barnes-Hut)
    FIELD_MAGNETIC     = 1u << 1,
    FIELD_ELECTROSTATIC= 1u << 2,
    FIELD_WIND         = 1u << 3,   // ветер с турбулентностью
    FIELD_CORIOLIS     = 1u << 4,
    FIELD_CENTRIFUGAL  = 1u << 5,
    FIELD_TIDAL        = 1u << 6,
    FIELD_LIGHT        = 1u << 7,
    FIELD_ALL          = 0xFFFFFFFFu
};

struct PointSource {
    Vec2 position;
    real strength = 1.0;    // масса / заряд / светимость
    real radius = 0.0;
};

class FieldSystem : public ForceSystem {
public:
    uint32_t flags = 0;
    BodyPhysicsRegistry* registry = nullptr;

    real gravitationalConstant = 6.674e-11;
    real nbodyTheta = 0.7;              // критерий Barnes-Hut
    real coulombConstant = 8.988e9;
    real magneticConstant = 1.0e-7;
    Vec2 windBase{4.0, 0.0};
    real windTurbulence = 3.0;
    real windScale = 0.15;
    real maxWindAcceleration = 250.0;   // предел ускорения от ветра, м/с^2
    real frameOmega = 0.0;              // вращающаяся система отсчёта
    Vec2 frameCenter;
    std::vector<PointSource> tidalSources;
    std::vector<PointSource> lightSources;

    void update(real dt) override;
    int  nbodyNodes() const { return m_nodes; }
    const char* className() const override { return "FieldSystem"; }

private:
    void applyNBody();
    void applyCharges();
    void applyWind(real t);
    void applyFrame();
    void applyTidal();
    void applyLight();

    real m_time = 0.0;
    int  m_nodes = 0;
};

class ThermalSystem : public SimulationSystem {
public:
    BodyPhysicsRegistry* registry = nullptr;
    real ambientTemperature = 293.0;
    real ambientCoupling = 0.02;
    real contactCoupling = 1.0;
    real frictionHeating = 0.25;        // доля работы трения в тепло
    bool thermalExpansion = true;
    bool phaseTransitions = true;
    bool temperatureDependentCoefficients = true;

    void update(real dt) override;
    int  moltenCount() const { return m_molten; }
    const char* className() const override { return "ThermalSystem"; }

private:
    int m_molten = 0;
};

// ================================================================ 9. звук
struct AudioEvent {
    real time = 0.0;
    real frequency = 220.0;
    real amplitude = 0.5;
    real duration = 0.25;
    real doppler = 1.0;
    Vec2 position;
};

class AudioSystem : public SimulationSystem {
public:
    int  sampleRate = 44100;
    Vec2 listener{0.0, 0.0};
    Vec2 listenerVelocity;
    real speedOfSound = 340.0;
    real impulseThreshold = 0.5;
    real masterGain = 0.6;

    void update(real dt) override;
    void addEvent(const AudioEvent& e);
    const std::vector<AudioEvent>& events() const { return m_events; }
    const std::vector<float>&      buffer()  const { return m_buffer; }
    bool writeWav(const std::string& path) const;
    void clear();
    const char* className() const override { return "AudioSystem"; }

private:
    std::vector<AudioEvent> m_events;
    std::vector<float>      m_buffer;
    real                    m_time = 0.0;
};

// ================================================================ 10. runtime
// 64-битные хэндлы: индекс + поколение, быстрый поиск через хэш-таблицу.
struct Handle64 {
    uint64_t value = 0;
    uint32_t index() const { return (uint32_t)(value & 0xFFFFFFFFu); }
    uint32_t generation() const { return (uint32_t)(value >> 32); }
    bool     valid() const { return value != 0; }
    bool operator==(const Handle64& o) const { return value == o.value; }
};

class HandleRegistry : public Named {
public:
    Handle64 acquire(BodyId id);
    BodyId   resolve(Handle64 h) const;
    Handle64 handleOf(BodyId id) const;
    void     release(Handle64 h);
    size_t   liveCount() const { return m_toBody.size(); }

private:
    std::unordered_map<uint64_t, BodyId>   m_toBody;
    std::unordered_map<BodyId, uint64_t>   m_toHandle;
    std::vector<uint32_t>                  m_generation;
    std::vector<uint32_t>                  m_free;
};

// Пул объектов фиксированного размера (по умолчанию 4096) с дефрагментацией.
template <typename T, size_t N = 4096>
class ObjectPool {
public:
    ObjectPool() : m_storage(N), m_used(N, false) {
        m_free.reserve(N);
        for (size_t i = N; i-- > 0;) m_free.push_back((uint32_t)i);
    }
    T* acquire() {
        if (m_free.empty()) return nullptr;
        const uint32_t i = m_free.back();
        m_free.pop_back();
        m_used[i] = true;
        ++m_live;
        return &m_storage[i];
    }
    void release(T* p) {
        if (!p) return;
        const size_t i = (size_t)(p - m_storage.data());
        if (i >= N || !m_used[i]) return;
        m_used[i] = false;
        m_storage[i] = T{};
        m_free.push_back((uint32_t)i);
        --m_live;
    }
    // Сжимает список свободных слотов, чтобы выдача шла плотно по памяти.
    void defragment() {
        m_free.clear();
        for (size_t i = N; i-- > 0;)
            if (!m_used[i]) m_free.push_back((uint32_t)i);
    }
    size_t capacity() const { return N; }
    size_t live() const { return m_live; }
    size_t bytes() const { return N * sizeof(T) + m_used.size() + m_free.capacity() * 4; }

private:
    std::vector<T>       m_storage;
    std::vector<bool>    m_used;
    std::vector<uint32_t> m_free;
    size_t               m_live = 0;
};

// Сборщик мусора: удаляет мёртвые тела (вне домена, NaN, помеченные).
class GarbageCollector : public SimulationSystem {
public:
    AABB domain;
    real interval = 0.5;
    bool removeNaN = true;
    void mark(BodyId id);
    void update(real dt) override;
    int  collectedTotal() const { return m_total; }
    const char* className() const override { return "GarbageCollector"; }

private:
    std::unordered_set<BodyId> m_marked;
    real m_timer = 0.0;
    int  m_total = 0;
};

// Защита от взрывов: клиппинг скоростей и импульсов, квантование позиций.
class SafetyGuard : public SimulationSystem {
public:
    real maxLinearSpeed = 120.0;
    real maxAngularSpeed = 60.0;
    real maxAcceleration = 4000.0;
    real quantizationSpeed = 0.02;    // ниже — позиции квантуются
    real quantizationStep = 1e-4;
    bool repairNaN = true;
    void update(real dt) override;
    int  clampedThisStep() const { return m_clamped; }
    int  repairedTotal() const { return m_repaired; }
    const char* className() const override { return "SafetyGuard"; }

private:
    int m_clamped = 0, m_repaired = 0;
};

// Бинарный лог событий с таймстампами + перемотка снапшотов.
enum class LogEventType : uint16_t {
    Step = 0, ContactBegin = 1, ContactEnd = 2, BodyCreated = 3, BodyDestroyed = 4,
    JointBroken = 5, Fracture = 6, Custom = 7
};

#pragma pack(push, 1)
struct LogRecord {
    uint64_t timestampNs = 0;
    uint64_t frame = 0;
    uint16_t type = 0;
    uint32_t a = 0, b = 0;
    double   x = 0.0, y = 0.0, value = 0.0;
};
#pragma pack(pop)

struct Snapshot {
    uint64_t frame = 0;
    real     time = 0.0;
    std::vector<BodyId> ids;
    std::vector<Vec2>   positions, velocities;
    std::vector<real>   angles, angularVelocities;
};

class EventRecorder : public SimulationSystem {
public:
    int  snapshotInterval = 1;      // кадров между снапшотами
    int  maxSnapshots = 600;
    bool recordContacts = true;

    void update(real dt) override;
    void push(LogEventType type, uint32_t a, uint32_t b, real x, real y, real value);
    bool saveBinary(const std::string& path) const;
    bool loadBinary(const std::string& path);
    const std::vector<LogRecord>& records() const { return m_records; }

    // Перемотка: применяет сохранённый снапшот к миру.
    bool rewind(int frames = 1);
    bool seek(uint64_t frame);
    size_t snapshotCount() const { return m_snapshots.size(); }
    uint64_t frame() const { return m_frame; }
    const char* className() const override { return "EventRecorder"; }

private:
    void capture();
    void restore(const Snapshot& s);

    std::vector<LogRecord> m_records;
    std::deque<Snapshot>   m_snapshots;
    uint64_t               m_frame = 0;
    real                   m_time = 0.0;
};

// Разбиение на острова + параллельная обработка систем по островам.
struct Island {
    std::vector<RigidBody*> bodies;
    int contacts = 0;
};

class IslandSolver : public SimulationSystem {
public:
    bool parallel = true;
    int  minIslandSize = 8;
    void update(real dt) override;
    const std::vector<Island>& islands() const { return m_islands; }
    size_t islandCount() const { return m_islands.size(); }
    int    largestIsland() const;
    const char* className() const override { return "IslandSolver"; }

private:
    std::vector<Island> m_islands;
};

// ================================================================ 11. геометрия
// Разбиение вогнутого многоугольника на выпуклые части.
std::vector<std::vector<Vec2>> decomposeConcave(const std::vector<Vec2>& polygon);
std::vector<Shape>             shapesFromConcave(const std::vector<Vec2>& polygon);

// Кривые: Безье, Catmull-Rom, рациональные B-сплайны (NURBS).
Vec2 bezierPoint(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, real t);
std::vector<Vec2> sampleBezier(const std::vector<Vec2>& controlPoints, int samples);
std::vector<Vec2> sampleNurbs(const std::vector<Vec2>& controlPoints, const std::vector<real>& weights,
                              int degree, int samples);

// Ландшафт: статическая цепочка сегментов из карты высот.
struct Terrain {
    std::vector<Vec2>   points;
    std::vector<BodyId> segments;
};
Terrain buildTerrain(World& w, const std::vector<real>& heights, real x0, real dx, real thickness = 0.4);
Terrain buildNoiseTerrain(World& w, real x0, real x1, real dx, real amplitude, uint32_t seed = 7);

// BSP-дерево для статической геометрии.
class BspTree : public Named {
public:
    struct Node {
        Vec2 point, normal;
        int  front = -1, back = -1;
        bool leaf = false;
        bool solid = false;
    };
    void build(const std::vector<std::pair<Vec2, Vec2>>& segments);
    bool pointSolid(const Vec2& p) const;
    int  depth() const;
    size_t nodeCount() const { return m_nodes.size(); }

private:
    int  buildRecursive(std::vector<std::pair<Vec2, Vec2>> segs, int depth);
    std::vector<Node> m_nodes;
};

// BVH по AABB с shaft-оптимизацией (объединение по «шахте» между узлами).
class Bvh : public Named {
public:
    struct Node {
        AABB box;
        int  left = -1, right = -1;
        int  body = -1;
    };
    void build(const std::vector<RigidBody*>& bodies);
    void query(const AABB& box, std::vector<int>& out) const;
    int  raycast(const Vec2& p1, const Vec2& p2, std::vector<int>& out) const;
    size_t nodeCount() const { return m_nodes.size(); }
    int    height() const;
    real   shaftCost() const { return m_shaftCost; }

private:
    int build(std::vector<int>& indices, int begin, int end);
    std::vector<Node>       m_nodes;
    std::vector<RigidBody*> m_bodies;
    real                    m_shaftCost = 0.0;
};

// Импорт мешей из .obj (проекция на XY + выпуклая декомпозиция).
struct ObjMesh {
    std::vector<Vec2>              vertices;
    std::vector<std::vector<int>>  faces;
};
bool importObj(const std::string& path, ObjMesh& out);
std::vector<Shape> shapesFromObj(const ObjMesh& mesh, real scale = 1.0);

// Экспорт сцены в форматы других движков.
bool exportBox2D(const World& w, const std::string& path);   // JSON в стиле b2d-json
bool exportBullet(const World& w, const std::string& path);  // текстовый .bullet-совместимый дамп

// ================================================================ 12. tooling
struct CommandLineOptions {
    int         steps = 600;
    real        dt = 1.0 / 60.0;
    int         threads = 0;
    int         bodies = 1000;
    bool        headless = true;
    bool        profile = true;
    bool        runBenchmark = false;
    std::string scene;
    std::string outputJson;
    std::string report;
    std::string script;
    std::unordered_map<std::string, std::string> extra;
};
CommandLineOptions parseCommandLine(int argc, char** argv);
std::string commandLineHelp();

// Встроенный скриптовый интерпретатор (совместимое подмножество Lua-подобного
// синтаксиса: присваивания, вызовы команд, арифметика, if/while). Нужен для
// живой настройки параметров без пересборки.
class ScriptEngine : public SimulationSystem {
public:
    void   attach(World& w) override;
    void   registerCommand(const std::string& name,
                           std::function<std::string(const std::vector<std::string>&)> fn);
    std::string execute(const std::string& source);   // возвращает вывод
    bool   executeFile(const std::string& path, std::string* out = nullptr);
    void   setVariable(const std::string& name, real value);
    real   variable(const std::string& name) const;
    void   update(real dt) override;
    const char* className() const override { return "ScriptEngine"; }

private:
    std::string runLine(const std::string& line);
    real        evaluate(const std::string& expr) const;

    std::unordered_map<std::string, real> m_vars;
    std::unordered_map<std::string, std::function<std::string(const std::vector<std::string>&)>> m_cmds;
};

// Консоль: очередь команд из stdin / файла / сети, исполняется между шагами.
class Console : public SimulationSystem {
public:
    ScriptEngine* script = nullptr;
    void  pushCommand(const std::string& cmd);
    bool  pollFile(const std::string& path);   // читает и очищает файл команд
    void  update(real dt) override;
    const std::vector<std::string>& log() const { return m_log; }
    const char* className() const override { return "Console"; }

private:
    std::vector<std::string> m_queue, m_log;
};

// Бенчмарк: минимум 10 сцен + отчёт.
struct BenchmarkResult {
    std::string scene;
    int    bodies = 0;
    int    steps = 0;
    double totalMs = 0.0, avgStepMs = 0.0, maxStepMs = 0.0, p95StepMs = 0.0;
    double broadMs = 0.0, narrowMs = 0.0, solveMs = 0.0;
    double contactsAvg = 0.0;
    double energyDrift = 0.0;
    bool   stable = true;
};

class BenchmarkSuite : public Named {
public:
    using SceneBuilder = std::function<void(World&)>;
    BenchmarkSuite();                                  // регистрирует 10 стандартных сцен
    void addScene(const std::string& name, SceneBuilder fn);
    std::vector<BenchmarkResult> runAll(int steps = 300, real dt = 1.0 / 60.0);
    BenchmarkResult run(const std::string& name, int steps, real dt);
    std::string report(const std::vector<BenchmarkResult>& results) const;   // markdown
    std::string csv(const std::vector<BenchmarkResult>& results) const;
    std::vector<std::string> sceneNames() const;

private:
    std::vector<std::pair<std::string, SceneBuilder>> m_scenes;
};

// График профайлера в реальном времени (кольцевой буфер + ASCII/линии для рендера).
class ProfilerGraph : public SimulationSystem {
public:
    int capacity = 240;
    void update(real dt) override;
    const std::vector<float>& series(Stage s) const;
    std::string asciiPlot(Stage s, int width = 60, int height = 8) const;
    double peak(Stage s) const;
    const char* className() const override { return "ProfilerGraph"; }

private:
    std::array<std::vector<float>, (size_t)Stage::Count> m_series;
};

// ================================================================ 13. движок-агрегатор
// Собирает все системы в один объект с единым порядком обновления.
class Engine : public Named {
public:
    explicit Engine(const WorldConfig& cfg = WorldConfig{});
    ~Engine();

    World& world() { return m_world; }
    const World& world() const { return m_world; }

    FilterRegistry&      filters()   { return m_filters; }
    SurfaceRegistry&     surfaces()  { return m_surfaces; }
    SensorSystem&        sensors()   { return m_sensors; }
    CcdSystem&           ccd()       { return m_ccd; }
    FractureSystem&      fracture()  { return m_fracture; }
    SoftBodySystem&      softBodies(){ return m_soft; }
    ParticleSystem&      fluid()     { return m_fluid; }
    FieldSystem&         fields()    { return m_fields; }
    ThermalSystem&       thermal()   { return m_thermal; }
    AudioSystem&         audio()     { return m_audio; }
    SafetyGuard&         safety()    { return m_safety; }
    GarbageCollector&    gc()        { return m_gc; }
    EventRecorder&       recorder()  { return m_recorder; }
    IslandSolver&        islands()   { return m_islands; }
    BreakableJointSystem& breakables(){ return m_breakables; }
    BodyPhysicsRegistry& properties(){ return m_props; }
    HandleRegistry&      handles()   { return m_handles; }
    ScriptEngine&        script()    { return m_script; }
    Console&             console()   { return m_console; }
    ProfilerGraph&       graph()     { return m_graph; }

    void enable(SimulationSystem& sys, bool on) { sys.enabled = on; }
    void step(real dt);
    void step() { step(m_world.config().fixedTimeStep); }
    std::string statusLine() const;

private:
    World                 m_world;
    FilterRegistry        m_filters;
    SurfaceRegistry       m_surfaces;
    SensorSystem          m_sensors;
    CcdSystem             m_ccd;
    FractureSystem        m_fracture;
    SoftBodySystem        m_soft;
    ParticleSystem        m_fluid;
    FieldSystem           m_fields;
    ThermalSystem         m_thermal;
    AudioSystem           m_audio;
    SafetyGuard           m_safety;
    GarbageCollector      m_gc;
    EventRecorder         m_recorder;
    IslandSolver          m_islands;
    BreakableJointSystem  m_breakables;
    BodyPhysicsRegistry   m_props;
    HandleRegistry        m_handles;
    ScriptEngine          m_script;
    Console               m_console;
    ProfilerGraph         m_graph;
    std::vector<SimulationSystem*> m_order;
};

// ================================================================ 14. диагностика
// Многослойная обработка ошибок: код + сообщение + стек вызовов движка.
enum class ErrorCode : int {
    None = 0, InvalidArgument, NotFound, OutOfMemory, NumericFailure,
    GeometryFailure, IoFailure, Unsupported
};

struct ErrorFrame {
    const char* function = "";
    const char* file = "";
    int         line = 0;
    std::string detail;
};

class ErrorStack {
public:
    static ErrorStack& instance();
    void        push(ErrorCode code, const ErrorFrame& frame);
    void        clear();
    bool        hasError() const { return m_code != ErrorCode::None; }
    ErrorCode   code() const { return m_code; }
    std::string trace() const;
    const std::vector<ErrorFrame>& frames() const { return m_frames; }

private:
    ErrorCode               m_code = ErrorCode::None;
    std::vector<ErrorFrame> m_frames;
};

#define PHYS2D_ERROR(code, detail) \
    ::phys2d::ErrorStack::instance().push((code), ::phys2d::ErrorFrame{__func__, __FILE__, __LINE__, (detail)})

#if defined(PHYS2D_DEBUG_ASSERTS)
#define PHYS2D_ASSERT(cond, detail)                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            PHYS2D_ERROR(::phys2d::ErrorCode::InvalidArgument, (detail));      \
            ::phys2d::phys2dAssertFailed(#cond, __FILE__, __LINE__, (detail)); \
        }                                                                      \
    } while (0)
#else
#define PHYS2D_ASSERT(cond, detail) ((void)0)
#endif

void phys2dAssertFailed(const char* expr, const char* file, int line, const char* detail);

// ================================================================ 15. таблицы и шаблоны
// Тригонометрия по таблице на 65536 градаций (быстрее std::sin для игр).
real tableSin(real angle);
real tableCos(real angle);
void tableSinCos(real angle, real& s, real& c);
size_t trigTableEntries();
size_t staticTableBytes();

// Кэш контактных точек (1024 слота, инвалидация по повороту).
struct CachedContact {
    uint64_t key = 0;
    Vec2     point, normal;
    real     angleA = 0.0, angleB = 0.0;
    real     normalImpulse = 0.0, tangentImpulse = 0.0;
    uint32_t frame = 0;
    bool     valid = false;
};

class ContactPointCache : public Named {
public:
    static constexpr size_t kSlots = 1024;
    void  store(uint64_t key, const CachedContact& c);
    const CachedContact* lookup(uint64_t key, real angleA, real angleB, real angleTolerance = 0.02) const;
    void  invalidateOlderThan(uint32_t frame);
    void  clear();
    size_t occupancy() const;
    size_t bytes() const { return kSlots * sizeof(CachedContact); }

private:
    std::array<CachedContact, kSlots> m_slots{};
};

// Все алгоритмы шаблонизированы под float/double и 8 комбинаций флагов.
enum KernelFlag : int {
    KERNEL_FRICTION   = 1 << 0,
    KERNEL_RESTITUTION= 1 << 1,
    KERNEL_WARMSTART  = 1 << 2
};

template <typename T>
struct VecT {
    T x = T(0), y = T(0);
    VecT() = default;
    VecT(T x_, T y_) : x(x_), y(y_) {}
    VecT operator+(const VecT& o) const { return VecT(x + o.x, y + o.y); }
    VecT operator-(const VecT& o) const { return VecT(x - o.x, y - o.y); }
    VecT operator*(T s) const { return VecT(x * s, y * s); }
    T    dot(const VecT& o) const { return x * o.x + y * o.y; }
    T    cross(const VecT& o) const { return x * o.y - y * o.x; }
    T    length() const;
    VecT normalized() const;
};

template <typename T, int Flags>
struct ContactKernel {
    T restitution = T(0.3), friction = T(0.5);
    T normalMass = T(1), tangentMass = T(1);
    T normalImpulse = T(0), tangentImpulse = T(0);

    // Решает одну контактную точку; ветви убираются на этапе компиляции по Flags.
    void solve(VecT<T>& va, T& wa, VecT<T>& vb, T& wb,
               const VecT<T>& ra, const VecT<T>& rb,
               const VecT<T>& normal, T invMassA, T invMassB, T invIA, T invIB, T bias);
    static const char* signature();
};

// 8 комбинаций флагов * 2 типа = 16 явных инстанциаций (см. Extras_runtime.cpp).
extern template struct ContactKernel<float, 0>;
extern template struct ContactKernel<float, 1>;
extern template struct ContactKernel<float, 2>;
extern template struct ContactKernel<float, 3>;
extern template struct ContactKernel<float, 4>;
extern template struct ContactKernel<float, 5>;
extern template struct ContactKernel<float, 6>;
extern template struct ContactKernel<float, 7>;
extern template struct ContactKernel<double, 0>;
extern template struct ContactKernel<double, 1>;
extern template struct ContactKernel<double, 2>;
extern template struct ContactKernel<double, 3>;
extern template struct ContactKernel<double, 4>;
extern template struct ContactKernel<double, 5>;
extern template struct ContactKernel<double, 6>;
extern template struct ContactKernel<double, 7>;

const char* kernelSignature(int flags, bool doublePrecision);

// Инерциальные тензоры высших порядков (моменты площади 2-4 порядка).
struct HigherOrderInertia {
    real m0 = 0.0;              // площадь
    real mx = 0.0, my = 0.0;    // первые моменты
    real mxx = 0.0, myy = 0.0, mxy = 0.0;
    real mxxx = 0.0, myyy = 0.0, mxxy = 0.0, mxyy = 0.0;
    real m4 = 0.0;              // след 4-го порядка
};
HigherOrderInertia computeHigherOrderInertia(const Shape& s);

// Пересчёт массы при изменении формы.
void rebuildMass(RigidBody& b, real density);
void setShape(RigidBody& b, const Shape& s, real density);

// Предиктор-корректор (улучшенная стабильность интегрирования).
void predictorCorrectorStep(World& w, real dt, int correctorPasses = 1);

// Адаптивная точность по локальной ошибке (метод удвоения шага).
real adaptiveStep(World& w, real dt, real tolerance = 1e-3, int maxHalvings = 3);

} // namespace phys2d
