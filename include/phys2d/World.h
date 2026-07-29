// phys2d — мир (PIMPL): интеграторы, solver, широкая/узкая фаза, события, профайлинг.
#pragma once

#include "Body.h"
#include "Collision.h"
#include "Constraints.h"
#include "BroadPhase.h"
#include "Profiler.h"
#include "DebugDraw.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace phys2d {

// Переключаемые методы интегрирования.
enum class Integrator : uint8_t {
    SymplecticEuler = 0,   // симплектический Эйлер
    Verlet          = 1,   // Верлет для положений
    RK4             = 2    // Рунге-Кутта 4 порядка для скоростей
};

// Отключаемые модули движка (битовые флаги).
enum ModuleFlag : uint32_t {
    MOD_GRAVITY            = 1u << 0,
    MOD_COLLISION          = 1u << 1,
    MOD_FRICTION           = 1u << 2,   // трение Кулона
    MOD_RESTITUTION        = 1u << 3,
    MOD_WARM_STARTING      = 1u << 4,
    MOD_POSITION_CORRECTION= 1u << 5,   // позиционная коррекция
    MOD_BAUMGARTE          = 1u << 6,
    MOD_CONSTRAINTS        = 1u << 7,
    MOD_SLEEPING           = 1u << 8,   // сонные тела с авто-пробуждением
    MOD_AIR_DRAG           = 1u << 9,   // линейное + квадратичное сопротивление
    MOD_AERODYNAMICS       = 1u << 10,  // подъёмная сила
    MOD_BUOYANCY           = 1u << 11,  // плавучесть
    MOD_VISCOSITY          = 1u << 12,  // вязкость среды
    MOD_ROLLING_FRICTION   = 1u << 13,  // трение качения
    MOD_ANGULAR_FRICTION   = 1u << 14,
    MOD_PRECESSION         = 1u << 15,  // прецессия несимметричных тел
    MOD_SOFT_BODY          = 1u << 16,  // упругая деформация
    MOD_MULTITHREADING     = 1u << 17,
    MOD_SUBSTEPPING        = 1u << 18,
    MOD_ADAPTIVE_TIMESTEP  = 1u << 19,
    MOD_TOI                = 1u << 20,  // бинарный поиск времени контакта
    MOD_SAT_CACHE          = 1u << 21,
    MOD_MASS_SORTING       = 1u << 22,  // сортировка тел по массе
    MOD_HYSTERESIS         = 1u << 23,
    MOD_DEBUG_DRAW         = 1u << 24,
    MOD_PROFILING          = 1u << 25,
    MOD_EVENTS             = 1u << 26,
    MOD_TRAILS             = 1u << 27,

    MOD_NONE    = 0u,
    MOD_DEFAULT = MOD_GRAVITY | MOD_COLLISION | MOD_FRICTION | MOD_RESTITUTION |
                  MOD_WARM_STARTING | MOD_POSITION_CORRECTION | MOD_BAUMGARTE |
                  MOD_CONSTRAINTS | MOD_SLEEPING | MOD_AIR_DRAG | MOD_ROLLING_FRICTION |
                  MOD_ANGULAR_FRICTION | MOD_MULTITHREADING | MOD_SUBSTEPPING |
                  MOD_SAT_CACHE | MOD_MASS_SORTING | MOD_HYSTERESIS | MOD_PROFILING |
                  MOD_EVENTS,
    MOD_ALL     = 0xFFFFFFFFu
};

struct SolverConfig {
    int  velocityIterations   = 12;      // итерации скоростного solver'а
    int  positionIterations   = 10;      // минимум 10 итераций позиционной коррекции
    int  maxIterations        = 100;     // жёсткий потолок итераций
    real baumgarte            = 0.20;    // коэффициент стабилизации Baumgarte
    real allowedPenetration   = 0.005;   // slop
    real maxLinearCorrection  = 0.20;
    real restitutionThreshold = 1.0;     // порог восстановления (м/с)
    real schlagerFactor       = 0.80;    // метод Шлагера для наложения ограничений
    real velocityTolerance    = 1e-5;    // ранний выход итераций
    real frictionLoadFactor  = 0.0;
    real stictionSpeed       = 0.05;
    real warmStartDamping    = 0.92;
    real maxPenetration      = 0.25;
    int  maxContactPoints    = 4;
    real restitutionSpeedRef = 4.0;
    real restitutionFalloff  = 0.55;
};

struct MediumConfig {
    real airDensity   = 1.204;           // плотность воздуха
    real fluidDensity = 1000.0;          // плотность жидкости
    real fluidLevel   = -1e30;           // уровень жидкости по Y (ниже — плавучесть)
    real viscosity    = 0.90;            // вязкость среды
    Vec2 wind;                           // ветер
};

struct WorldConfig {
    Vec2         gravity{0.0, -9.81};
    Integrator   integrator     = Integrator::SymplecticEuler;
    SolverConfig solver{};
    MediumConfig medium{};
    uint32_t     modules        = MOD_DEFAULT;

    real  fixedTimeStep         = 1.0 / 60.0;
    int   substeps              = 4;          // substepping с дробными шагами
    int   maxSubsteps           = 16;
    real  minTimeStep           = 1.0 / 960.0;// динамическое изменение шага
    real  maxTimeStep           = 1.0 / 30.0;
    real  adaptiveSpeedLimit    = 24.0;       // м/с — выше шаг дробится

    BroadPhaseMode broadPhase   = BroadPhaseMode::Hybrid;
    real  gridCellSize          = 4.0;
    real  aabbMargin            = 0.10;       // запас AABB
    int   regionCount           = 4;          // разделение сцены на регионы
    int   threadCount           = 0;          // 0 = hardware_concurrency

    real  sleepLinearTolerance  = 0.04;
    real  sleepAngularTolerance = 0.04;
    real  sleepTime             = 0.60;
    real  wakeImpulseThreshold  = 0.20;

    size_t arenaBytes           = 32u * 1024u * 1024u;  // >= 30 МБ под данные физики
    size_t maxContacts          = 262144;
    size_t maxBodies            = 65536;
};

// ---------------------------------------------------------------- События
enum class ContactPhase : uint8_t { Begin = 0, Persist = 1, End = 2 };

struct CollisionEvent {
    RigidBody*   a = nullptr;
    RigidBody*   b = nullptr;
    Vec2         normal;
    Vec2         point;
    real         penetration     = 0.0;
    real         normalImpulse   = 0.0;
    real         tangentImpulse  = 0.0;
    real         relativeSpeed   = 0.0;
    real         timeOfImpact    = 1.0;
    ContactPhase phase           = ContactPhase::Begin;
};

using CollisionCallback = std::function<void(const CollisionEvent&)>;

// ---------------------------------------------------------------- World
class World {
public:
    explicit World(const WorldConfig& cfg = WorldConfig{});
    ~World();
    World(const World&)            = delete;
    World& operator=(const World&) = delete;
    World(World&&) noexcept;
    World& operator=(World&&) noexcept;

    // ---- тела
    BodyId      createBody(const BodyDef& def);
    void        destroyBody(BodyId id);
    RigidBody*  body(BodyId id);
    const RigidBody* body(BodyId id) const;
    size_t      bodyCount() const;
    std::vector<RigidBody*>& bodies();

    // ---- ограничения
    DistanceConstraint* createDistance (BodyId a, BodyId b, const Vec2& la, const Vec2& lb, real length);
    RevoluteConstraint* createRevolute (BodyId a, BodyId b, const Vec2& la, const Vec2& lb);
    SpringConstraint*   createSpring   (BodyId a, BodyId b, const Vec2& la, const Vec2& lb,
                                        real length, real stiffness, real damping);
    RopeConstraint*     createRope     (BodyId a, BodyId b, const Vec2& la, const Vec2& lb, real maxLength);
    AngularConstraint*  createAngular  (BodyId a, BodyId b, real minAngle, real maxAngle);
    Constraint*         addConstraint(std::unique_ptr<Constraint> c);   // custom joints (Extras.h)
    std::vector<Constraint*> allConstraints();
    void                destroyConstraint(Constraint* c);
    size_t              constraintCount() const;

    // ---- шаг симуляции
    void step(real dt);
    void step();                     // фиксированный шаг из конфига

    // ---- настройки
    WorldConfig&       config();
    const WorldConfig& config() const;
    void  setGravity(const Vec2& g);
    Vec2  gravity() const;
    void  setIntegrator(Integrator i);
    Integrator integrator() const;

    void  enableModule(uint32_t flag, bool on);
    bool  moduleEnabled(uint32_t flag) const;
    void  setModules(uint32_t mask);
    uint32_t modules() const;

    // ---- события столкновений
    void setBeginContactCallback(CollisionCallback cb);
    void setPersistContactCallback(CollisionCallback cb);
    void setEndContactCallback(CollisionCallback cb);
    // Фильтр контактов (слои, маски, сенсоры) — см. Extras.h
    using ContactFilterFn = std::function<bool(const RigidBody&, const RigidBody&)>;
    void setContactFilter(ContactFilterFn fn);
    // Пост-шаговый хук: вызывается в конце каждого step().
    void setPostStepCallback(std::function<void(real)> fn);


    // ---- профайлинг и статистика
    const ProfileData& profile() const;
    size_t contactCount() const;
    size_t awakeCount() const;
    size_t arenaBytes() const;         // фактически выделено под данные физики
    real   totalEnergy() const;
    const std::vector<Manifold>& manifolds() const;

    // ---- дебаг-визуализация
    DebugDrawBuffer&       debugBuffer();
    const DebugDrawBuffer& debugBuffer() const;
    void                   buildDebugGeometry();

    // ---- запросы
    RigidBody*              queryPoint(const Vec2& p);
    std::vector<RigidBody*> queryAABB(const AABB& box);

    // ---- сериализация JSON
    std::string toJson(bool pretty = true) const;
    bool        fromJson(const std::string& json, std::string* error = nullptr);
    bool        saveToFile(const std::string& path) const;
    bool        loadFromFile(const std::string& path, std::string* error = nullptr);

    void clear();

private:
    struct Impl;
    std::unique_ptr<Impl> m;   // PIMPL — внутренние детали скрыты
};

} // namespace phys2d
