// phys2d v2 - base system hierarchy, filtering, sensors, materials, runtime services, engine.
#include "phys2d/Extras.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace phys2d {
namespace {

std::atomic<uint64_t> g_uidCounter{1};

inline uint64_t pairKey(BodyId a, BodyId b) {
    const uint64_t lo = std::min(a, b), hi = std::max(a, b);
    return (hi << 32) | lo;
}

inline bool finite2(const Vec2& v) { return std::isfinite(v.x) && std::isfinite(v.y); }

} // namespace

// ================================================================ hierarchy
Object::Object() : m_uid(g_uidCounter.fetch_add(1)) {}
Object::~Object() = default;

void SimulationSystem::tick(real dt) {
    if (!enabled || !m_world) return;
    const auto t0 = std::chrono::steady_clock::now();
    update(dt);
    lastMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    averageMs = (updates == 0) ? lastMs : averageMs * 0.95 + lastMs * 0.05;
    ++updates;
}

void ForceSystem::addForce(RigidBody& b, const Vec2& f) const {
    if (!b.isDynamic()) return;
    const Vec2 scaled = f * strengthScale;
    if (!finite2(scaled)) return;
    b.force += scaled;
}

// ================================================================ filtering
void   FilterRegistry::set(BodyId id, const Filter& f) { m_map[id] = f; }
void   FilterRegistry::remove(BodyId id) { m_map.erase(id); }
Filter FilterRegistry::get(BodyId id) const {
    const auto it = m_map.find(id);
    return (it == m_map.end()) ? Filter{} : it->second;
}

bool FilterRegistry::shouldCollide(BodyId a, BodyId b) const {
    const Filter fa = get(a), fb = get(b);
    if (fa.sensor || fb.sensor) return false;                 // сенсоры не дают импульсов
    if (fa.group != 0 && fa.group == fb.group) return fa.group > 0;
    return (fa.category & fb.mask) != 0 && (fb.category & fa.mask) != 0;
}

void FilterRegistry::install(World& w) {
    w.setContactFilter([this](const RigidBody& a, const RigidBody& b) {
        return shouldCollide(a.id, b.id);
    });
}

// ================================================================ materials (50 шт.)
namespace {

constexpr int kMaterialCount = 50;

const SurfaceMaterial kMaterials[kMaterialCount] = {
    {"vacuum",        0.001, 0.00, 0.00,   0.02,  1000, 0.0,     1e9,   1e5,  0.0},
    {"air",           0.001, 0.00, 0.01,   0.026, 1005, 3.4e-3,  1e9,   1e5,  0.0},
    {"water",         1000,  0.00, 0.01,   0.6,   4180, 2.1e-4,  373,   2.2e9, 0.0},
    {"ice",           917,   0.10, 0.05,   2.2,   2100, 5.1e-5,  273,   9e9,   1e6},
    {"snow",          300,   0.05, 0.30,   0.3,   2090, 5.0e-5,  273,   1e7,   1e5},
    {"rubber",        1100,  0.85, 1.10,   0.16,  2000, 2.0e-4,  450,   5e7,   2e7},
    {"soft_rubber",   950,   0.92, 1.30,   0.14,  2100, 2.4e-4,  430,   2e7,   1e7},
    {"steel",         7850,  0.55, 0.42,   45.0,  490,  1.2e-5,  1723,  2.0e11, 5e8},
    {"stainless",     8000,  0.52, 0.38,   16.0,  500,  1.6e-5,  1673,  1.9e11, 5.2e8},
    {"cast_iron",     7200,  0.35, 0.45,   52.0,  460,  1.1e-5,  1473,  1.1e11, 2e8},
    {"aluminium",     2700,  0.45, 0.40,   205.0, 900,  2.3e-5,  933,   6.9e10, 3e8},
    {"titanium",      4500,  0.50, 0.36,   22.0,  520,  8.6e-6,  1941,  1.1e11, 9e8},
    {"copper",        8960,  0.40, 0.36,   401.0, 385,  1.7e-5,  1358,  1.1e11, 2.2e8},
    {"brass",         8500,  0.42, 0.35,   109.0, 380,  1.9e-5,  1200,  1.0e11, 3e8},
    {"bronze",        8800,  0.40, 0.38,   60.0,  380,  1.8e-5,  1223,  1.0e11, 3.5e8},
    {"lead",          11340, 0.15, 0.55,   35.0,  128,  2.9e-5,  600,   1.6e10, 1.2e7},
    {"gold",          19300, 0.30, 0.42,   315.0, 129,  1.4e-5,  1337,  7.9e10, 1.2e8},
    {"silver",        10490, 0.35, 0.40,   429.0, 235,  1.9e-5,  1235,  8.3e10, 1.7e8},
    {"tungsten",      19250, 0.45, 0.35,   173.0, 134,  4.5e-6,  3695,  4.1e11, 1.5e9},
    {"zinc",          7140,  0.32, 0.44,   116.0, 388,  3.0e-5,  693,   1.1e11, 1.5e8},
    {"glass",         2500,  0.55, 0.40,   1.05,  840,  8.5e-6,  1873,  7.0e10, 5e7},
    {"tempered_glass",2530,  0.60, 0.38,   1.10,  840,  9.0e-6,  1873,  7.2e10, 1.5e8},
    {"ceramic",       3900,  0.50, 0.55,   30.0,  880,  8.0e-6,  2323,  3.7e11, 3e8},
    {"porcelain",     2400,  0.45, 0.50,   1.5,   1085, 6.0e-6,  1923,  7e10,   5.5e7},
    {"concrete",      2400,  0.20, 0.75,   1.7,   880,  1.2e-5,  1773,  3.0e10, 4e6},
    {"reinforced_concrete", 2500, 0.22, 0.78, 2.0, 880, 1.2e-5,  1773,  3.5e10, 1e7},
    {"brick",         1900,  0.18, 0.70,   0.7,   840,  6.0e-6,  1873,  1.5e10, 3e6},
    {"stone",         2700,  0.25, 0.65,   2.5,   790,  8.0e-6,  1473,  5.0e10, 1e7},
    {"granite",       2750,  0.28, 0.62,   2.8,   790,  8.0e-6,  1533,  6.0e10, 1.5e7},
    {"marble",        2700,  0.30, 0.58,   2.8,   880,  1.0e-5,  1600,  5.5e10, 1.2e7},
    {"sandstone",     2200,  0.15, 0.72,   1.7,   920,  1.0e-5,  1473,  2.0e10, 5e6},
    {"asphalt",       2300,  0.12, 0.80,   0.75,  920,  2.0e-5,  423,   3e9,    2e6},
    {"wood_oak",      750,   0.30, 0.55,   0.17,  2400, 5.0e-6,  573,   1.1e10, 4e7},
    {"wood_pine",     500,   0.28, 0.50,   0.12,  2300, 5.0e-6,  573,   9e9,    3e7},
    {"plywood",       600,   0.26, 0.52,   0.13,  2300, 6.0e-6,  573,   8e9,    2.5e7},
    {"cardboard",     700,   0.10, 0.60,   0.06,  1400, 1.0e-5,  500,   2e9,    1e7},
    {"paper",         800,   0.05, 0.45,   0.05,  1340, 1.0e-5,  500,   1e9,    3e6},
    {"plastic_pe",    950,   0.45, 0.35,   0.4,   1900, 2.0e-4,  403,   1.1e9,  3e7},
    {"plastic_pvc",   1400,  0.40, 0.42,   0.19,  1000, 8.0e-5,  373,   3.0e9,  5e7},
    {"nylon",         1150,  0.50, 0.30,   0.25,  1700, 9.0e-5,  537,   3.0e9,  8e7},
    {"teflon",        2200,  0.30, 0.06,   0.25,  1000, 1.2e-4,  600,   5.0e8,  2.5e7},
    {"foam",          80,    0.20, 0.65,   0.035, 1300, 6.0e-5,  373,   1e7,    2e5},
    {"cork",          240,   0.35, 0.70,   0.04,  1900, 5.0e-5,  573,   3e7,    1e6},
    {"leather",       900,   0.25, 0.62,   0.14,  1500, 4.0e-5,  473,   1e8,    2e7},
    {"fabric",        400,   0.10, 0.68,   0.05,  1400, 3.0e-5,  473,   1e7,    5e6},
    {"sand",          1600,  0.05, 0.85,   0.3,   830,  1.0e-5,  1973,  1e7,    1e5},
    {"gravel",        1800,  0.10, 0.90,   0.5,   840,  1.0e-5,  1873,  2e7,    2e5},
    {"clay",          1750,  0.08, 0.78,   1.3,   880,  1.5e-5,  1473,  1e8,    5e5},
    {"mud",           1600,  0.02, 0.55,   1.0,   1500, 2.0e-5,  373,   1e6,    1e4},
    {"magnet_ferrite",5000,  0.30, 0.48,   4.0,   700,  1.0e-5,  1673,  1.5e11, 5e7},
};

struct MaterialTable {
    MaterialPair pairs[kMaterialCount][kMaterialCount];
    MaterialTable() {
        for (int i = 0; i < kMaterialCount; ++i) {
            for (int j = 0; j < kMaterialCount; ++j) {
                const SurfaceMaterial& a = kMaterials[i];
                const SurfaceMaterial& b = kMaterials[j];
                MaterialPair& p = pairs[i][j];
                p.friction    = std::sqrt(a.friction * b.friction);
                p.restitution = std::max(a.restitution, b.restitution) * 0.5 +
                                std::sqrt(a.restitution * b.restitution) * 0.5;
                const real ca = a.conductivity, cb = b.conductivity;
                p.conductance = (ca + cb > 0.0) ? (2.0 * ca * cb / (ca + cb)) : 0.0;
                p.adhesion    = (a.meltingPoint < 500.0 && b.meltingPoint < 500.0) ? 0.05 : 0.0;
            }
        }
    }
};

const MaterialTable& materialTable() {
    static const MaterialTable table;
    return table;
}

} // namespace

int materialCount() { return kMaterialCount; }

const SurfaceMaterial& materialById(int id) {
    if (id < 0 || id >= kMaterialCount) return kMaterials[0];
    return kMaterials[id];
}

int materialByName(const char* name) {
    if (!name) return -1;
    for (int i = 0; i < kMaterialCount; ++i)
        if (std::strcmp(kMaterials[i].name, name) == 0) return i;
    return -1;
}

const MaterialPair& materialPair(int a, int b) {
    const int ia = (a < 0 || a >= kMaterialCount) ? 0 : a;
    const int ib = (b < 0 || b >= kMaterialCount) ? 0 : b;
    return materialTable().pairs[ia][ib];
}

size_t staticTableBytes() {
    return sizeof(MaterialTable) + sizeof(kMaterials);
}

// ================================================================ surfaces / anisotropy
void SurfaceRegistry::set(BodyId id, const SurfaceAssignment& s) { m_map[id] = s; }

const SurfaceAssignment* SurfaceRegistry::get(BodyId id) const {
    const auto it = m_map.find(id);
    return (it == m_map.end()) ? nullptr : &it->second;
}

// Анизотропное трение моделируется корректирующим импульсом после шага:
// тангенциальная скорость гасится сильнее поперёк оси материала, чем вдоль неё.
void SurfaceRegistry::update(real dt) {
    if (!m_world || m_map.empty() || dt <= 0.0) return;

    for (const Manifold& mf : m_world->manifolds()) {
        if (!mf.touching || mf.count == 0) continue;
        RigidBody* bodies[2] = {mf.bodyA, mf.bodyB};
        for (int side = 0; side < 2; ++side) {
            RigidBody* b = bodies[side];
            if (!b || !b->isDynamic()) continue;
            const SurfaceAssignment* sa = get(b->id);
            if (!sa) continue;
            if (std::fabs(sa->frictionAlong - sa->frictionAcross) < 1e-9) continue;

            const Vec2 axis = sa->anisotropyAxis.rotated(b->angle).normalized();
            const Vec2 tangent = mf.normal.perp();
            const real along = std::fabs(dot(axis, tangent));
            const real k = sa->frictionAlong * along + sa->frictionAcross * (1.0 - along);
            const real extra = clampr((k - 1.0) * 0.5, -0.9, 0.9);
            if (std::fabs(extra) < 1e-6) continue;

            for (int i = 0; i < mf.count; ++i) {
                const Vec2 p = mf.points[i].point;
                const Vec2 v = b->velocityAtPoint(p);
                const real vt = dot(v, tangent);
                const Vec2 corrective = tangent * (-vt * extra * b->mass * 0.5);
                b->applyImpulseAtPoint(corrective, p);
            }
        }
    }
}

// ================================================================ sensors
void SensorSystem::add(BodyId id, uint32_t mask) { m_sensors[id] = mask; }
void SensorSystem::remove(BodyId id) { m_sensors.erase(id); }

void SensorSystem::update(real) {
    if (!m_world || m_sensors.empty()) return;

    m_current.clear();
    Manifold mf;

    for (const auto& kv : m_sensors) {
        RigidBody* sensor = m_world->body(kv.first);
        if (!sensor) continue;
        const uint32_t mask = kv.second;

        const std::vector<RigidBody*> candidates = m_world->queryAABB(sensor->aabb);
        for (RigidBody* other : candidates) {
            if (!other || other == sensor) continue;
            if (m_sensors.count(other->id)) continue;
            if (m_filters) {
                const Filter fo = m_filters->get(other->id);
                if ((fo.category & mask) == 0) continue;
            }

            mf = Manifold{};
            if (!collide(mf, *sensor, *other, nullptr) || mf.count == 0) continue;

            const uint64_t key = pairKey(sensor->id, other->id);
            m_current.insert(key);

            if (m_cb) {
                SensorEvent e;
                e.sensor = sensor->id;
                e.other = other->id;
                e.point = mf.points[0].point;
                e.overlap = mf.points[0].penetration;
                e.phase = m_previous.count(key) ? SensorPhase::Stay : SensorPhase::Enter;
                m_cb(e);
            }
        }
    }

    if (m_cb) {
        for (uint64_t key : m_previous) {
            if (m_current.count(key)) continue;
            SensorEvent e;
            e.sensor = (BodyId)(key & 0xFFFFFFFFu);
            e.other = (BodyId)(key >> 32);
            if (!m_sensors.count(e.sensor)) std::swap(e.sensor, e.other);
            e.phase = SensorPhase::Exit;
            m_cb(e);
        }
    }
    m_previous = m_current;
}

// ================================================================ handles
Handle64 HandleRegistry::acquire(BodyId id) {
    const auto existing = m_toHandle.find(id);
    if (existing != m_toHandle.end()) return Handle64{existing->second};

    uint32_t index;
    if (!m_free.empty()) {
        index = m_free.back();
        m_free.pop_back();
    } else {
        index = (uint32_t)m_generation.size();
        m_generation.push_back(1);
    }
    const uint64_t value = ((uint64_t)m_generation[index] << 32) | (uint64_t)(index + 1);
    m_toBody[value] = id;
    m_toHandle[id] = value;
    return Handle64{value};
}

BodyId HandleRegistry::resolve(Handle64 h) const {
    const auto it = m_toBody.find(h.value);
    return (it == m_toBody.end()) ? INVALID_BODY : it->second;
}

Handle64 HandleRegistry::handleOf(BodyId id) const {
    const auto it = m_toHandle.find(id);
    return (it == m_toHandle.end()) ? Handle64{} : Handle64{it->second};
}

void HandleRegistry::release(Handle64 h) {
    const auto it = m_toBody.find(h.value);
    if (it == m_toBody.end()) return;
    m_toHandle.erase(it->second);
    m_toBody.erase(it);

    const uint32_t index = h.index();
    if (index >= 1 && index - 1 < m_generation.size()) {
        ++m_generation[index - 1];
        m_free.push_back(index - 1);
    }
}

// ================================================================ safety guard
void SafetyGuard::update(real dt) {
    if (!m_world) return;
    m_clamped = 0;

    for (RigidBody* b : m_world->bodies()) {
        if (!b->isDynamic()) continue;

        if (repairNaN) {
            if (!finite2(b->position) || !std::isfinite(b->angle)) {
                b->position = finite2(b->prevPosition) ? b->prevPosition : Vec2();
                b->angle = std::isfinite(b->prevAngle) ? b->prevAngle : 0.0;
                b->velocity = Vec2();
                b->angularVelocity = 0.0;
                ++m_repaired;
            }
            if (!finite2(b->velocity)) { b->velocity = Vec2(); ++m_repaired; }
            if (!std::isfinite(b->angularVelocity)) { b->angularVelocity = 0.0; ++m_repaired; }
        }

        // Клиппинг ускорений (explosion protection).
        if (dt > 0.0) {
            const Vec2 dv = b->velocity - b->prevPosition * 0.0;   // без истории: ограничиваем модуль
            (void)dv;
            const real maxDv = maxAcceleration * dt;
            if (b->velocity.length() > maxDv + maxLinearSpeed) {
                b->velocity = b->velocity.normalized() * (maxDv + maxLinearSpeed);
                ++m_clamped;
            }
        }

        const real speed = b->velocity.length();
        if (speed > maxLinearSpeed) {
            b->velocity = b->velocity * (maxLinearSpeed / speed);
            ++m_clamped;
        }
        if (std::fabs(b->angularVelocity) > maxAngularSpeed) {
            b->angularVelocity = (b->angularVelocity > 0.0 ? 1.0 : -1.0) * maxAngularSpeed;
            ++m_clamped;
        }

        // Квантование позиций на малых скоростях — убирает дрожание штабелей.
        if (quantizationStep > 0.0 && speed < quantizationSpeed &&
            std::fabs(b->angularVelocity) < quantizationSpeed) {
            const real inv = 1.0 / quantizationStep;
            b->position.x = std::round(b->position.x * inv) * quantizationStep;
            b->position.y = std::round(b->position.y * inv) * quantizationStep;
            b->updateVertices();
        }
    }
}

// ================================================================ garbage collector
void GarbageCollector::mark(BodyId id) { m_marked.insert(id); }

void GarbageCollector::update(real dt) {
    if (!m_world) return;
    m_timer += dt;
    if (m_timer < interval && m_marked.empty()) return;
    m_timer = 0.0;

    std::vector<BodyId> dead(m_marked.begin(), m_marked.end());
    m_marked.clear();

    const bool hasDomain = domain.min.x < domain.max.x && domain.min.y < domain.max.y;
    for (RigidBody* b : m_world->bodies()) {
        if (b->isStatic()) continue;
        const bool bad = removeNaN && (!finite2(b->position) || !std::isfinite(b->angle));
        const bool outside = hasDomain && !domain.contains(b->position);
        if (bad || outside) dead.push_back(b->id);
    }

    std::sort(dead.begin(), dead.end());
    dead.erase(std::unique(dead.begin(), dead.end()), dead.end());
    for (BodyId id : dead) {
        if (m_world->body(id)) {
            m_world->destroyBody(id);
            ++m_total;
        }
    }
}

// ================================================================ islands
void IslandSolver::update(real) {
    if (!m_world) return;

    std::vector<RigidBody*>& bodies = m_world->bodies();
    std::unordered_map<BodyId, int> index;
    index.reserve(bodies.size() * 2);
    for (size_t i = 0; i < bodies.size(); ++i) index[bodies[i]->id] = (int)i;

    std::vector<int> parent(bodies.size());
    for (size_t i = 0; i < parent.size(); ++i) parent[i] = (int)i;

    std::function<int(int)> find = [&](int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite = [&](int a, int b) {
        const int ra = find(a), rb = find(b);
        if (ra != rb) parent[ra] = rb;
    };

    int contactLinks = 0;
    for (const Manifold& mf : m_world->manifolds()) {
        if (!mf.touching || !mf.bodyA || !mf.bodyB) continue;
        if (mf.bodyA->isStatic() || mf.bodyB->isStatic()) continue;
        const auto ia = index.find(mf.bodyA->id), ib = index.find(mf.bodyB->id);
        if (ia == index.end() || ib == index.end()) continue;
        unite(ia->second, ib->second);
        ++contactLinks;
    }
    for (Constraint* c : m_world->allConstraints()) {
        if (!c || !c->enabled || c->broken || !c->bodyA || !c->bodyB) continue;
        if (c->bodyA->isStatic() || c->bodyB->isStatic()) continue;
        const auto ia = index.find(c->bodyA->id), ib = index.find(c->bodyB->id);
        if (ia == index.end() || ib == index.end()) continue;
        unite(ia->second, ib->second);
    }

    std::unordered_map<int, size_t> slot;
    m_islands.clear();
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i]->isStatic()) continue;
        const int root = find((int)i);
        auto it = slot.find(root);
        if (it == slot.end()) {
            slot[root] = m_islands.size();
            m_islands.push_back(Island{});
            it = slot.find(root);
        }
        m_islands[it->second].bodies.push_back(bodies[i]);
    }
    if (!m_islands.empty()) m_islands[0].contacts = contactLinks;
}

int IslandSolver::largestIsland() const {
    size_t best = 0;
    for (const Island& i : m_islands) best = std::max(best, i.bodies.size());
    return (int)best;
}

// ================================================================ event recorder / rewind
void EventRecorder::push(LogEventType type, uint32_t a, uint32_t b, real x, real y, real value) {
    LogRecord r;
    r.timestampNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
    r.frame = m_frame;
    r.type = (uint16_t)type;
    r.a = a; r.b = b;
    r.x = (double)x; r.y = (double)y; r.value = (double)value;
    m_records.push_back(r);
}

void EventRecorder::capture() {
    if (!m_world) return;
    Snapshot s;
    s.frame = m_frame;
    s.time = m_time;
    const std::vector<RigidBody*>& bodies = m_world->bodies();
    s.ids.reserve(bodies.size());
    for (RigidBody* b : bodies) {
        s.ids.push_back(b->id);
        s.positions.push_back(b->position);
        s.velocities.push_back(b->velocity);
        s.angles.push_back(b->angle);
        s.angularVelocities.push_back(b->angularVelocity);
    }
    m_snapshots.push_back(std::move(s));
    while ((int)m_snapshots.size() > maxSnapshots) m_snapshots.pop_front();
}

void EventRecorder::restore(const Snapshot& s) {
    if (!m_world) return;
    for (size_t i = 0; i < s.ids.size(); ++i) {
        RigidBody* b = m_world->body(s.ids[i]);
        if (!b) continue;
        b->position = s.positions[i];
        b->prevPosition = s.positions[i];
        b->velocity = s.velocities[i];
        b->angle = s.angles[i];
        b->prevAngle = s.angles[i];
        b->angularVelocity = s.angularVelocities[i];
        b->wake();
        b->updateVertices();
        b->updateAABB();
    }
    m_frame = s.frame;
    m_time = s.time;
}

void EventRecorder::update(real dt) {
    if (!m_world) return;
    m_time += dt;
    ++m_frame;

    push(LogEventType::Step, (uint32_t)m_world->bodyCount(), (uint32_t)m_world->contactCount(),
         m_time, m_world->profile().ms[(size_t)Stage::Total], m_world->totalEnergy());

    if (recordContacts) {
        for (const Manifold& mf : m_world->manifolds()) {
            if (!mf.touching || mf.count == 0) continue;
            push(LogEventType::ContactBegin, mf.bodyA->id, mf.bodyB->id,
                 mf.points[0].point.x, mf.points[0].point.y, mf.points[0].normalImpulse);
        }
    }

    if (snapshotInterval > 0 && (m_frame % (uint64_t)snapshotInterval) == 0) capture();
}

bool EventRecorder::rewind(int frames) {
    if (m_snapshots.empty() || frames <= 0) return false;
    const int available = (int)m_snapshots.size();
    const int back = std::min(frames, available);
    for (int i = 1; i < back; ++i) m_snapshots.pop_back();
    const Snapshot s = m_snapshots.back();
    m_snapshots.pop_back();
    restore(s);
    return true;
}

bool EventRecorder::seek(uint64_t frame) {
    for (auto it = m_snapshots.rbegin(); it != m_snapshots.rend(); ++it) {
        if (it->frame <= frame) { restore(*it); return true; }
    }
    return false;
}

bool EventRecorder::saveBinary(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f) { PHYS2D_ERROR(ErrorCode::IoFailure, path); return false; }
    const char magic[8] = {'P','H','2','D','L','O','G','1'};
    f.write(magic, 8);
    const uint64_t n = m_records.size();
    f.write(reinterpret_cast<const char*>(&n), sizeof(n));
    if (n) f.write(reinterpret_cast<const char*>(m_records.data()), (std::streamsize)(n * sizeof(LogRecord)));
    return (bool)f;
}

bool EventRecorder::loadBinary(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { PHYS2D_ERROR(ErrorCode::IoFailure, path); return false; }
    char magic[8]{};
    f.read(magic, 8);
    if (std::memcmp(magic, "PH2DLOG1", 8) != 0) {
        PHYS2D_ERROR(ErrorCode::IoFailure, "bad log magic");
        return false;
    }
    uint64_t n = 0;
    f.read(reinterpret_cast<char*>(&n), sizeof(n));
    m_records.assign((size_t)n, LogRecord{});
    if (n) f.read(reinterpret_cast<char*>(m_records.data()), (std::streamsize)(n * sizeof(LogRecord)));
    return (bool)f;
}

// ================================================================ profiler graph
void ProfilerGraph::update(real) {
    if (!m_world) return;
    const ProfileData& pd = m_world->profile();
    for (size_t i = 0; i < (size_t)Stage::Count; ++i) {
        std::vector<float>& s = m_series[i];
        s.push_back((float)pd.ms[i]);
        while ((int)s.size() > capacity) s.erase(s.begin());
    }
}

const std::vector<float>& ProfilerGraph::series(Stage s) const {
    return m_series[(size_t)s];
}

double ProfilerGraph::peak(Stage s) const {
    double best = 0.0;
    for (float v : m_series[(size_t)s]) best = std::max(best, (double)v);
    return best;
}

std::string ProfilerGraph::asciiPlot(Stage s, int width, int height) const {
    const std::vector<float>& data = m_series[(size_t)s];
    if (data.empty()) return std::string("(no data)");

    const int w = std::max(8, width), h = std::max(2, height);
    double maxV = 1e-9;
    for (float v : data) maxV = std::max(maxV, (double)v);

    std::vector<std::string> rows((size_t)h, std::string((size_t)w, ' '));
    for (int x = 0; x < w; ++x) {
        const size_t idx = data.size() * (size_t)x / (size_t)w;
        const double v = data[std::min(idx, data.size() - 1)];
        const int bar = (int)std::lround(v / maxV * (h - 1));
        for (int y = 0; y <= bar; ++y) rows[(size_t)(h - 1 - y)][(size_t)x] = (y == bar) ? '*' : '|';
    }

    std::ostringstream out;
    out << stageName(s) << "  max " << maxV << " ms\n";
    for (const std::string& r : rows) out << r << "\n";
    return out.str();
}

// ================================================================ error stack
ErrorStack& ErrorStack::instance() {
    static ErrorStack stack;
    return stack;
}

void ErrorStack::push(ErrorCode code, const ErrorFrame& frame) {
    m_code = code;
    m_frames.push_back(frame);
    if (m_frames.size() > 64) m_frames.erase(m_frames.begin());
}

void ErrorStack::clear() {
    m_code = ErrorCode::None;
    m_frames.clear();
}

std::string ErrorStack::trace() const {
    std::ostringstream out;
    out << "error code " << (int)m_code << ", " << m_frames.size() << " frame(s)\n";
    for (size_t i = m_frames.size(); i-- > 0;) {
        const ErrorFrame& f = m_frames[i];
        out << "  #" << (m_frames.size() - i - 1) << " " << f.function
            << " (" << f.file << ":" << f.line << ") " << f.detail << "\n";
    }
    return out.str();
}

void phys2dAssertFailed(const char* expr, const char* file, int line, const char* detail) {
    std::fprintf(stderr, "phys2d assertion failed: %s\n  at %s:%d\n  %s\n%s",
                 expr, file, line, detail ? detail : "",
                 ErrorStack::instance().trace().c_str());
}

// ================================================================ engine
Engine::Engine(const WorldConfig& cfg) : m_world(cfg) {
    name = "phys2d.engine";

    m_filters.name = "filters";
    m_filters.install(m_world);

    m_sensors.setFilterRegistry(&m_filters);
    m_fields.registry = &m_props;
    m_thermal.registry = &m_props;
    m_console.script = &m_script;

    SimulationSystem* systems[] = {
        &m_surfaces, &m_sensors, &m_ccd, &m_fracture, &m_soft, &m_fluid,
        &m_thermal, &m_audio, &m_safety, &m_gc, &m_islands, &m_breakables,
        &m_recorder, &m_graph, &m_script, &m_console
    };
    for (SimulationSystem* s : systems) {
        s->attach(m_world);
        m_order.push_back(s);
    }
    m_fields.attach(m_world);

    // По умолчанию тяжёлые системы выключены — включайте осознанно.
    m_fields.enabled = false;
    m_thermal.enabled = false;
    m_fluid.enabled = false;
    m_soft.enabled = false;
    m_recorder.enabled = false;
    m_audio.enabled = false;
    m_islands.enabled = false;
}

Engine::~Engine() = default;

void Engine::step(real dt) {
    // 1. внешние силы до шага (аккумуляторы обнуляются внутри World::step)
    m_fields.tick(dt);

    // 2. шаг твёрдотельной физики
    m_world.step(dt);

    // 3. пост-системы
    for (SimulationSystem* s : m_order) s->tick(dt);
}

std::string Engine::statusLine() const {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "bodies %zu | contacts %zu | soft %zu | particles %zu | islands %zu | step %.2f ms | arena %.1f MB",
        m_world.bodyCount(), m_world.contactCount(), m_soft.count(), m_fluid.count(),
        m_islands.islandCount(), m_world.profile().ms[(size_t)Stage::Total],
        (double)m_world.arenaBytes() / (1024.0 * 1024.0));
    return std::string(buf);
}

} // namespace phys2d
