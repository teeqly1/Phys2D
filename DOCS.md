- **[Documentation in English](DOCS_EN.md)** — original documentation in English

## 📄 DOCS.md


# Phys2D — Полная документация

Здесь описаны все основные системы, классы и методы Phys2D. Документация разбита по модулям для удобного поиска.

---

## 📚 Содержание

1. [Ядро (Core)](#1-ядро-core)
2. [Твёрдые тела (Bodies)](#2-твёрдые-тела-bodies)
3. [Формы (Shapes)](#3-формы-shapes)
4. [Столкновения (Collision)](#4-столкновения-collision)
5. [Джойнты (Constraints)](#5-джойнты-constraints)
6. [Расширения (Extras)](#6-расширения-extras)
7. [Advanced Lab](#7-advanced-lab)
8. [Система крови (Blood)](#8-система-крови-blood)
9. [Инструменты](#9-инструменты)

---

## 1. Ядро (Core)

### World

Главный класс, управляющий всеми телами, джойнтами и симуляцией.

```cpp
#include "phys2d/World.h"

WorldConfig cfg;
cfg.gravity = Vec2(0.0, -9.81);
cfg.substeps = 4;
World world(cfg);

world.step(1.0 / 60.0);
```

**Методы World:**

| Метод | Описание |
|-------|----------|
| `createBody(const BodyDef&)` | Создать тело |
| `destroyBody(BodyId)` | Удалить тело |
| `body(BodyId)` | Получить тело по ID |
| `bodies()` | Список всех тел |
| `step(real dt)` | Шаг симуляции |
| `setGravity(const Vec2&)` | Установить гравитацию |
| `setIntegrator(Integrator)` | Выбрать интегратор |
| `toJson(bool pretty)` | Сериализация в JSON |
| `fromJson(const std::string&)` | Загрузка из JSON |
| `saveToFile(const std::string&)` | Сохранить в файл |
| `loadFromFile(const std::string&)` | Загрузить из файла |
| `setBeginContactCallback()` | Коллбэк начала контакта |
| `setEndContactCallback()` | Коллбэк конца контакта |

**WorldConfig:**

```cpp
struct WorldConfig {
    Vec2 gravity;               // гравитация
    Integrator integrator;      // SymplecticEuler, Verlet, RK4
    int substeps;               // подшаги (по умолчанию 4)
    uint32_t modules;           // битовые флаги модулей
    SolverConfig solver;        // настройки солвера
    MediumConfig medium;        // воздух, жидкость, ветер
    int threadCount;            // 0 = авто
    size_t arenaBytes;          // >= 30 МБ
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

## 2. Твёрдые тела (Bodies)

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

## 3. Формы (Shapes)

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

## 4. Столкновения (Collision)

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

### CCD и лучи

```cpp
// Консервативное продвижение (CCD)
SweepResult conservativeAdvancement(
    const RigidBody& a,
    const RigidBody& b,
    real dt,
    int maxIterations = 32
);

// Лучи
bool rayCastClosest(World& w, const Vec2& p1, const Vec2& p2, RayHit& out);
int rayCastAll(World& w, const Vec2& p1, const Vec2& p2, std::vector<RayHit>& out);
```

---

## 5. Джойнты (Constraints)

| Тип | Описание |
|-----|----------|
| `DistanceConstraint` | Жёсткая связь с фиксированным расстоянием |
| `RevoluteConstraint` | Шарнир (точка вращения) |
| `SpringConstraint` | Пружина (жёсткость + демпфирование) |
| `RopeConstraint` | Трос (только на растяжение) |
| `AngularConstraint` | Угловое ограничение |
| `WeldJoint` | Сварка (жёсткая или мягкая) |
| `MotorJoint` | Линейный/угловой мотор |
| `PrismaticJoint` | Поступательное движение |
| `PulleyJoint` | Блок с передаточным числом |
| `GearJoint` | Шестерня |

---

## 6. Расширения (Extras)

### FilterRegistry (слои и маски)

```cpp
FilterRegistry filters;
filters.set(body, Filter{category: 0x1, mask: 0xFFFF, sensor: false});
filters.install(world);
```

### SensorSystem (сенсоры/триггеры)

```cpp
SensorSystem sensors;
sensors.add(body, 0xFFFF);
sensors.setCallback([](const SensorEvent& e) {
    if (e.phase == SensorPhase::Enter) printf("Вошёл!\n");
});
```

### FractureSystem (разрушение)

```cpp
FractureSystem fracture;
fracture.makeBreakable(body, 12.0);
fracture.setCallback([](const FractureEvent& e) {
    printf("Разрушено на %zu осколков\n", e.fragments.size());
});
```

### SoftBodySystem (мягкие тела)

```cpp
SoftBody cloth = SoftBody::grid(Vec2(0, 0), 4, 4, 10, 8, 2.0);
cloth.model = SoftModel::Hybrid;
cloth.tearStrain = 0.85;
softBodies.add(cloth);
```

### ParticleSystem (SPH жидкости)

```cpp
ParticleSystem fluid;
fluid.domain = AABB(Vec2(-5, 0), Vec2(5, 5));
fluid.smoothingRadius = 0.35;
fluid.emitBlock(Vec2(-4, 0.2), 8, 2, 0.16);
```

### FieldSystem (поля сил)

```cpp
FieldSystem fields;
fields.flags = FIELD_WIND | FIELD_ELECTROSTATIC | FIELD_NBODY;
fields.windBase = Vec2(3.0, 0.0);
fields.windTurbulence = 6.0;
```

### ThermalSystem (термодинамика)

```cpp
ThermalSystem thermal;
thermal.registry = &properties;
thermal.ambientTemperature = 293.0;
thermal.phaseTransitions = true;
```

### SurfaceRegistry (поверхности)

```cpp
SurfaceAssignment sa;
sa.anisotropyAxis = Vec2(1, 0);
sa.frictionAlong = 0.05;
sa.frictionAcross = 1.4;
surfaces.set(body, sa);
```

---

## 7. Advanced Lab

### WindSystem (ветер)

```cpp
WindSystem wind;
wind.setStrength(10.0);
wind.profile.turbulence = 0.25;
wind.configure(body, AeroProfile{dragCoefficient: 1.2, area: -1.0});
```

### ElectricSystem (электричество)

```cpp
ElectricSystem elec;
elec.fluid = &fluid;
elec.addElectrode(Electrode{Vec2(-4, 0.4), 220.0, 0.6, false});
elec.addElectrode(Electrode{Vec2(4, 0.4), 0.0, 0.6, true});
```

### MaterialLab (материаловедение)

```cpp
MaterialLab lab;
lab.registerBody(body, materialSteel());
lab.applyStress(body, 1e8, dt);
lab.onImpact(body, impulse, speed, point);
```

### ImpactFragmentation (осколки)

```cpp
ImpactFragmentation frag;
frag.lab = &lab;
frag.maxFragments = 9;
ShatterResult r = frag.shatter(body, impactPoint, impactDir);
```

### ContactChemistry (адгезия/капилляры)

```cpp
ContactChemistry chem;
chem.enableAdhesion = true;
chem.enableCapillary = true;
chem.setWetness(body, 1.0);
chem.setConcentration(body, 1.0);
```

---

## 8. Система крови (Blood)

```cpp
BloodSystem blood;
blood.attach(world);
blood.gravity = Vec2(0.0, -9.81);
blood.stickiness = 0.78;
blood.maxDroplets = 24000;
```

**Методы:**

| Метод | Описание |
|-------|----------|
| `emit(p, v, volume, kind, oxygen)` | Выпустить каплю крови |
| `spray(origin, dir, speed, cone, count, volume)` | Распылить кровь веером |
| `splash(center, speed, count, volume)` | Разбрызгать во все стороны |
| `addWound(body, point, dir, severity, arterial)` | Добавить рану |
| `sever(body, point, dir, severity)` | Отсечь конечность |
| `stainBody(body, amount)` | Запачкать тело |
| `update(dt)` | Обновить систему |

**BloodKind:**
- `Droplet` — обычная капля
- `Mist` — мелкий туман
- `Gush` — тяжёлая артериальная струя
- `Clot` — сгусток

---

## 9. Инструменты

### ScriptEngine (скриптовый движок)

```cpp
ScriptEngine script;
script.registerCommand("spawn", [](const auto& args) { ... });
script.execute("spawn circle 0 5 0.5");
```

### Console (консоль)

```cpp
Console console;
console.script = &script;
console.pushCommand("gravity 0 -9.81");
```

### Profiler (профайлер)

```cpp
const ProfileData& pd = world.profile();
printf("Broad: %.2f ms\n", pd.ms[(size_t)Stage::BroadPhase]);
```

### Бенчмарки

```cpp
BenchmarkSuite suite;
auto results = suite.runAll(300, 1.0/60.0);
std::printf("%s", suite.report(results).c_str());
```

### Сериализация

```cpp
world.saveToFile("scene.json");
world.loadFromFile("scene.json");
exportBox2D(world, "scene.b2d");
exportBullet(world, "scene.bullet");
```

---

## 📜 Сокращения

| Сокращение | Расшифровка |
|------------|-------------|
| CCD | Continuous Collision Detection |
| SAT | Separating Axis Theorem |
| GJK | Gilbert-Johnson-Keerthi |
| EPA | Expanding Polytope Algorithm |
| SPH | Smoothed Particle Hydrodynamics |
| FEM | Finite Element Method |
| SIMD | Single Instruction, Multiple Data |

---

## 🔗 Ссылки

- [README.md](README.md) — главная страница проекта
- [README_GAME.md](README_GAME.md) — создание игр
- [README_REALTIME.md](README_REALTIME.md) — интерактивная сцена
- [README_EN.md](README_EN.md) — документация на английском
- [Лицензия](LICENSE)
- [Автор на GitHub](https://github.com/teeqly1)

---

*Последнее обновление: июль 2026*
```

---

Have a build!
