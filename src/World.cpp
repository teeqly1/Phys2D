#include "phys2d/World.h"
#include "phys2d/ThreadPool.h"
#include "phys2d/Serialization.h"

#include <algorithm>
#include <deque>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <cstring>

namespace phys2d {

// ================================================================ PIMPL
struct World::Impl {
    WorldConfig cfg;

    // ---- тела (deque — стабильные адреса)
    std::deque<RigidBody>                    storage;
    std::vector<RigidBody*>                  bodyList;
    std::unordered_map<BodyId, RigidBody*>   byId;
    BodyId                                   nextId = 1;

    // ---- ограничения
    std::vector<ConstraintPtr> constraints;

    // ---- широкая / узкая фаза
    BroadPhase             broad;
    SatAxisCache           satCache;
    std::vector<BroadPair> pairs;
    LockFreeList<Manifold> contactBuffer;      // безблокировочный список контактов
    std::vector<Manifold>  manifoldList;

    // ---- warm starting
    struct WarmEntry {
        int      count = 0;
        uint32_t fid[MAX_MANIFOLD_POINTS]{};
        real     ni[MAX_MANIFOLD_POINTS]{};
        real     ti[MAX_MANIFOLD_POINTS]{};
    };
    std::unordered_map<uint64_t, WarmEntry> warm;
    std::unordered_set<uint64_t>            touchingPrev;
    std::unordered_set<uint64_t>            touchingNow;

    // ---- события
    CollisionCallback onBegin, onPersist, onEnd;
    World::ContactFilterFn      contactFilter;   // слои / маски / сенсоры
    std::function<void(real)>   postStep;        // внешние системы (Extras)

    // ---- профайлинг / дебаг
    Profiler        profiler;
    ProfileData     prof;
    DebugDrawBuffer debug;

    // ---- многопоточность
    std::unique_ptr<ThreadPool> pool;

    // ---- память под данные физики (>= 30 МБ)
    std::vector<uint8_t> arena;

    explicit Impl(const WorldConfig& c) : cfg(c) {
        broad.mode       = cfg.broadPhase;
        broad.aabbMargin = cfg.aabbMargin;
        broad.regionCount = cfg.regionCount;
        broad.setCellSize(cfg.gridCellSize);

        pool = std::make_unique<ThreadPool>(cfg.threadCount);
        contactBuffer.reserve(cfg.maxContacts);
        manifoldList.reserve(cfg.maxContacts);
        pairs.reserve(cfg.maxContacts);
        warm.reserve(cfg.maxContacts);

        // Гарантированное выделение памяти под данные физики (страницы трогаем,
        // чтобы память действительно была зарезервирована и резидентна).
        const size_t bytes = std::max<size_t>(cfg.arenaBytes, 30u * 1024u * 1024u);
        arena.assign(bytes, 0u);
        for (size_t i = 0; i < arena.size(); i += 4096) arena[i] = (uint8_t)(i & 0xFF);
    }

    bool mod(uint32_t f) const { return (cfg.modules & f) != 0; }

    // ------------------------------------------------------------ силы
    Vec2 dragForce(const RigidBody& b, const Vec2& v) const {
        Vec2 f;
        const Vec2 rel = v - cfg.medium.wind;
        const real speed = rel.length();
        if (speed < EPSILON) return f;
        const bool submerged = b.position.y < cfg.medium.fluidLevel;
        const real rho = submerged ? cfg.medium.fluidDensity : cfg.medium.airDensity;
        const real refLen = std::max(b.boundingRadius, 1e-3);

        if (mod(MOD_AIR_DRAG)) {
            // Линейное (Стокс) + квадратичное (Ньютон) сопротивление.
            f -= rel * (b.material.linearDrag * b.mass);
            f -= rel * (b.material.quadraticDrag * rho * refLen * speed);
        }
        if (mod(MOD_VISCOSITY) && submerged) {
            f -= rel * (cfg.medium.viscosity * b.mass);
        }
        if (mod(MOD_AERODYNAMICS) && b.material.liftCoefficient > 0.0) {
            // Подъёмная сила: перпендикулярно потоку, знак по углу атаки.
            const Vec2 flow = rel / speed;
            const Vec2 chord = Vec2::fromAngle(b.angle);
            const real aoa = std::asin(clampr(cross(flow, chord), -1.0, 1.0));
            const real lift = 0.5 * rho * speed * speed * refLen *
                              b.material.liftCoefficient * std::sin(2.0 * aoa);
            f += flow.perp() * lift;
        }
        return f;
    }

    Vec2 buoyancyForce(const RigidBody& b) const {
        if (!mod(MOD_BUOYANCY)) return Vec2();
        const real level = cfg.medium.fluidLevel;
        const real top = b.aabb.max.y, bottom = b.aabb.min.y;
        if (bottom >= level) return Vec2();
        // Доля погружённого объёма (по AABB) и выталкивающая сила Архимеда.
        const real h = std::max(top - bottom, EPSILON);
        const real frac = clampr((level - bottom) / h, 0.0, 1.0);
        const real area = b.shape.computeMass(1.0).area;
        const real displaced = area * frac;
        return -cfg.gravity * (cfg.medium.fluidDensity * displaced);
    }

    void applyPrecession(RigidBody& b, real h) const {
        if (!mod(MOD_PRECESSION) || b.invInertia == 0.0) return;
        // Модель прецессии для несимметричных тел: осевая асимметрия создаёт
        // колебательный момент, пропорциональный omega^2.
        const Vec2 ext = b.aabb.extents();
        const real asym = (std::max(ext.x, ext.y) - std::min(ext.x, ext.y)) /
                          std::max(std::max(ext.x, ext.y), EPSILON);
        if (asym < 1e-3) return;
        const real wobble = asym * 0.05 * b.angularVelocity * std::fabs(b.angularVelocity);
        b.angularVelocity -= wobble * b.invInertia * b.inertia * h;
    }

    // Упругая деформация мягких тел: узлы оболочки на пружинах к исходной форме.
    void relaxSoftBody(RigidBody& b, real h) const {
        if (!mod(MOD_SOFT_BODY) || b.material.softness <= 0.0) return;
        if (b.restVertices.size() != b.localVertices.size()) return;
        const real k = 1.0 / std::max(b.material.softness, 1e-6);
        const real c = 2.0 * std::sqrt(k);
        for (size_t i = 0; i < b.localVertices.size(); ++i) {
            const Vec2 x = b.localVertices[i] - b.restVertices[i];
            Vec2& v = b.vertexVelocity[i];
            v += (x * -k - v * c) * h;
            b.localVertices[i] += v * h;
        }
        b.shape.vertices = b.localVertices;
        b.shape.updateDerived();
        b.updateVertices();
    }

    void deformOnImpact(RigidBody& b, const Vec2& point, const Vec2& normal, real impulse) const {
        if (!mod(MOD_SOFT_BODY) || b.material.softness <= 0.0) return;
        if (b.localVertices.size() < 3) return;
        const Vec2 local = b.transform().invApply(point);
        size_t best = 0; real bestD = BIG;
        for (size_t i = 0; i < b.localVertices.size(); ++i) {
            const real d = (b.localVertices[i] - local).lengthSq();
            if (d < bestD) { bestD = d; best = i; }
        }
        const Vec2 nLocal = b.transform().invApplyR(normal);
        b.vertexVelocity[best] += nLocal * (impulse * b.material.softness * b.invMass);
    }

    // ------------------------------------------------------------ интегрирование
    void integrateVelocities(real h) {
        for (RigidBody* pb : bodyList) {
            RigidBody& b = *pb;
            if (!b.isDynamic() || b.sleeping) continue;

            Vec2 fConst = b.force;
            if (mod(MOD_GRAVITY)) fConst += cfg.gravity * (b.mass * b.gravityScale);
            fConst += buoyancyForce(b);

            auto accel = [&](const Vec2& v) -> Vec2 {
                return (fConst + dragForce(b, v)) * b.invMass;
            };

            switch (cfg.integrator) {
                case Integrator::RK4: {
                    // RK4 для скоростей: dv/dt = a(v)
                    const Vec2 v0 = b.velocity;
                    const Vec2 k1 = accel(v0);
                    const Vec2 k2 = accel(v0 + k1 * (0.5 * h));
                    const Vec2 k3 = accel(v0 + k2 * (0.5 * h));
                    const Vec2 k4 = accel(v0 + k3 * h);
                    b.acceleration = (k1 + k2 * 2.0 + k3 * 2.0 + k4) / 6.0;
                    b.velocity = v0 + b.acceleration * h;
                    break;
                }
                case Integrator::Verlet:
                case Integrator::SymplecticEuler:
                default: {
                    b.acceleration = accel(b.velocity);
                    b.velocity += b.acceleration * h;      // симплектический шаг по скорости
                    break;
                }
            }

            b.angularAccel = b.torque * b.invInertia;
            b.angularVelocity += b.angularAccel * h;
            if (mod(MOD_ANGULAR_FRICTION))
                b.angularVelocity /= (1.0 + b.material.angularFriction * h);   // угловое трение
            applyPrecession(b, h);
        }
    }

    void integratePositions(real h) {
        for (RigidBody* pb : bodyList) {
            RigidBody& b = *pb;
            if (b.isStatic() || b.sleeping) continue;

            if (cfg.integrator == Integrator::Verlet) {
                // Верлет для положений: x(n+1) = x(n) + v*h + 0.5*a*h^2
                const Vec2 next = b.position + b.velocity * h + b.acceleration * (0.5 * h * h);
                b.prevPosition = b.position;
                b.position = next;
            } else {
                b.prevPosition = b.position;
                b.position += b.velocity * h;
            }
            b.prevAngle = b.angle;
            b.angle += b.angularVelocity * h;

            b.updateVertices();
            relaxSoftBody(b, h);

            if (mod(MOD_TRAILS)) {
                b.trail.push_back(b.position);
                if ((int)b.trail.size() > debug.trailLength) b.trail.erase(b.trail.begin());
            }
        }
    }

    // ------------------------------------------------------------ узкая фаза
    void narrowPhase(real h) {
        contactBuffer.clear();
        const size_t n = pairs.size();
        const bool mt = mod(MOD_MULTITHREADING) && pool && pool->threadCount() > 1 && n > 256;

        auto work = [&](size_t from, size_t to) {
            Manifold m;
            for (size_t i = from; i < to; ++i) {
                RigidBody* a = pairs[i].a;
                RigidBody* b = pairs[i].b;
                if (contactFilter && !contactFilter(*a, *b)) continue;
                m = Manifold{};
                if (!collide(m, *a, *b, mod(MOD_SAT_CACHE) ? &satCache : nullptr)) continue;
                if (mod(MOD_TOI))
                    m.toi = timeOfImpact(*a, *b, h, 20);   // бинарный поиск времени контакта
                contactBuffer.push(m);                     // безблокировочная запись
            }
        };

        if (mt) pool->parallelFor(n, 64, work);
        else if (n) work(0, n);

        manifoldList.assign(contactBuffer.data(), contactBuffer.data() + contactBuffer.size());

        // Сортировка по массе — тяжёлые контакты решаются раньше (стабильность штабелей).
        if (mod(MOD_MASS_SORTING)) {
            std::sort(manifoldList.begin(), manifoldList.end(),
                      [](const Manifold& l, const Manifold& r) {
                          const real ml = std::max(l.bodyA->mass, l.bodyB->mass);
                          const real mr = std::max(r.bodyA->mass, r.bodyB->mass);
                          if (ml != mr) return ml > mr;
                          return l.key < r.key;
                      });
        }
    }

    // ------------------------------------------------------------ подготовка контактов
    void prepareContacts(real h) {
        const real invH = (h > EPSILON) ? 1.0 / h : 0.0;

        for (Manifold& m : manifoldList) {
            RigidBody& a = *m.bodyA;
            RigidBody& b = *m.bodyB;
            const Vec2 t = m.normal.perp();

            // Гистерезис: потери энергии растут с скоростью удара.
            real restitution = mod(MOD_RESTITUTION) ? m.restitution : 0.0;

            for (int i = 0; i < m.count; ++i) {
                ContactPoint& cp = m.points[i];
                cp.rA = cp.point - a.position;
                cp.rB = cp.point - b.position;

                const real kn = a.invMass + b.invMass +
                                a.invInertia * sqr(cross(cp.rA, m.normal)) +
                                b.invInertia * sqr(cross(cp.rB, m.normal));
                const real kt = a.invMass + b.invMass +
                                a.invInertia * sqr(cross(cp.rA, t)) +
                                b.invInertia * sqr(cross(cp.rB, t));
                cp.normalMass  = (kn > EPSILON) ? 1.0 / kn : 0.0;
                cp.tangentMass = (kt > EPSILON) ? 1.0 / kt : 0.0;

                // Относительная скорость в точке контакта
                const Vec2 dv = b.velocityAtPoint(cp.point) - a.velocityAtPoint(cp.point);
                cp.relVelNormal = dot(dv, m.normal);

                real e = restitution;
                if (mod(MOD_HYSTERESIS)) {
                    const real hyst = 0.5 * (a.material.hysteresis + b.material.hysteresis);
                    e *= 1.0 - clampr(hyst * std::fabs(cp.relVelNormal) / 10.0, 0.0, 1.0);
                }
                // Restitution threshold: медленные контакты без отскока
                if (cp.relVelNormal > -cfg.solver.restitutionThreshold) e = 0.0;
                cp.velocityBias = -e * cp.relVelNormal;

                // Baumgarte
                cp.positionBias = mod(MOD_BAUMGARTE)
                    ? cfg.solver.baumgarte * invH *
                      std::max(0.0, cp.penetration - cfg.solver.allowedPenetration)
                    : 0.0;

                cp.normalImpulse = 0.0;
                cp.tangentImpulse = 0.0;
            }

            // Матрица масс контактного пространства (блок 2x2 для двух точек)
            if (m.count == 2) {
                const ContactPoint& c1 = m.points[0];
                const ContactPoint& c2 = m.points[1];
                const real k11 = a.invMass + b.invMass +
                                 a.invInertia * sqr(cross(c1.rA, m.normal)) +
                                 b.invInertia * sqr(cross(c1.rB, m.normal));
                const real k22 = a.invMass + b.invMass +
                                 a.invInertia * sqr(cross(c2.rA, m.normal)) +
                                 b.invInertia * sqr(cross(c2.rB, m.normal));
                const real k12 = a.invMass + b.invMass +
                                 a.invInertia * cross(c1.rA, m.normal) * cross(c2.rA, m.normal) +
                                 b.invInertia * cross(c1.rB, m.normal) * cross(c2.rB, m.normal);
                m.K = Mat22(k11, k12, k12, k22);
            } else {
                m.K = Mat22(m.points[0].normalMass, 0.0, 0.0, 0.0);
            }

            // Warm starting — восстановление накопленных импульсов с прошлого шага
            if (mod(MOD_WARM_STARTING)) {
                auto it = warm.find(m.key);
                if (it != warm.end()) {
                    const WarmEntry& w = it->second;
                    for (int i = 0; i < m.count; ++i) {
                        for (int j = 0; j < w.count; ++j) {
                            if (w.fid[j] == m.points[i].featureId) {
                                m.points[i].normalImpulse  = w.ni[j];
                                m.points[i].tangentImpulse = w.ti[j];
                                break;
                            }
                        }
                    }
                }
                for (int i = 0; i < m.count; ++i) {
                    const ContactPoint& cp = m.points[i];
                    const Vec2 P = m.normal * cp.normalImpulse + t * cp.tangentImpulse;
                    applyImpulse(a, b, cp, -P);
                }
            }
        }
    }

    static void applyImpulse(RigidBody& a, RigidBody& b, const ContactPoint& cp, const Vec2& impulseOnA) {
        a.velocity        += impulseOnA * a.invMass;
        a.angularVelocity += a.invInertia * cross(cp.rA, impulseOnA);
        b.velocity        -= impulseOnA * b.invMass;
        b.angularVelocity -= b.invInertia * cross(cp.rB, impulseOnA);
    }

    // ------------------------------------------------------------ скоростной solver
    void solveVelocities(real h) {
        const int iters = std::min(std::max(cfg.solver.velocityIterations, 1), cfg.solver.maxIterations);
        const real invH = (h > EPSILON) ? 1.0 / h : 0.0;
        prof.velocityIterationsUsed = 0;

        for (int iter = 0; iter < iters; ++iter) {
            real maxDelta = 0.0;
            ++prof.velocityIterationsUsed;

            for (Manifold& m : manifoldList) {
                RigidBody& a = *m.bodyA;
                RigidBody& b = *m.bodyB;
                const Vec2 t = m.normal.perp();

                for (int i = 0; i < m.count; ++i) {
                    ContactPoint& cp = m.points[i];

                    // --- нормальный импульс (accumulated impulses + Baumgarte)
                    Vec2 dv = b.velocityAtPoint(cp.point) - a.velocityAtPoint(cp.point);
                    real vn = dot(dv, m.normal);
                    real lambda = cp.normalMass * (-(vn) + cp.velocityBias + cp.positionBias);
                    const real oldN = cp.normalImpulse;
                    cp.normalImpulse = std::max(oldN + lambda, 0.0);   // одностороннее ограничение
                    lambda = cp.normalImpulse - oldN;
                    maxDelta = std::max(maxDelta, std::fabs(lambda));
                    applyImpulse(a, b, cp, m.normal * -lambda);

                    // --- трение Кулона (статическое / динамическое)
                    if (mod(MOD_FRICTION)) {
                        dv = b.velocityAtPoint(cp.point) - a.velocityAtPoint(cp.point);
                        const real vt = dot(dv, t);
                        real lambdaT = -cp.tangentMass * vt;

                        // Коэффициент трения зависит от нормальной силы: mu = mu0 / (1 + k*Fn)
                        const real Fn = cp.normalImpulse * invH;
                        const bool sticking = std::fabs(vt) < 1e-3;
                        const real mu0 = sticking ? m.staticFriction : m.dynamicFriction;
                        const real mu  = mu0 / (1.0 + cfg.solver.frictionLoadFactor * Fn);
                        const real maxF = mu * cp.normalImpulse;

                        const real oldT = cp.tangentImpulse;
                        cp.tangentImpulse = clampr(oldT + lambdaT, -maxF, maxF);
                        lambdaT = cp.tangentImpulse - oldT;
                        applyImpulse(a, b, cp, t * -lambdaT);
                    }

                    // --- трение качения (окружности и капсулы)
                    if (mod(MOD_ROLLING_FRICTION) && iter == iters - 1) {
                        applyRollingFriction(a, cp.normalImpulse, m);
                        applyRollingFriction(b, cp.normalImpulse, m);
                    }

                    // --- упругая деформация мягких тел
                    if (iter == iters - 1 && mod(MOD_SOFT_BODY)) {
                        deformOnImpact(a, cp.point, -m.normal, cp.normalImpulse);
                        deformOnImpact(b, cp.point,  m.normal, cp.normalImpulse);
                    }
                }
            }

            // Ограничения решаются в том же итеративном цикле
            if (mod(MOD_CONSTRAINTS))
                for (ConstraintPtr& c : constraints)
                    if (c->enabled && !c->broken) c->solveVelocity(h);

            if (maxDelta < cfg.solver.velocityTolerance) break;   // ранний выход
        }
    }

    void applyRollingFriction(RigidBody& b, real normalImpulse, const Manifold&) const {
        if (!b.isDynamic() || b.invInertia == 0.0) return;
        if (b.shape.type == ShapeType::Polygon) return;
        const real r = std::max(b.shape.radius, 1e-6);
        const real maxDw = b.material.rollingFriction * normalImpulse * b.invInertia * r;
        if (b.angularVelocity > 0.0) b.angularVelocity = std::max(0.0, b.angularVelocity - maxDw);
        else                         b.angularVelocity = std::min(0.0, b.angularVelocity + maxDw);
    }

    // ------------------------------------------------------------ позиционная коррекция
    void solvePositions(real h) {
        if (!mod(MOD_POSITION_CORRECTION)) return;
        const int iters = std::min(std::max(cfg.solver.positionIterations, 10),  // минимум 10
                                  cfg.solver.maxIterations);
        prof.positionIterationsUsed = 0;

        for (int iter = 0; iter < iters; ++iter) {
            real maxPen = 0.0;
            ++prof.positionIterationsUsed;

            for (Manifold& m : manifoldList) {
                RigidBody& a = *m.bodyA;
                RigidBody& b = *m.bodyB;
                const real wsum = a.invMass + b.invMass;
                if (wsum < EPSILON) continue;

                for (int i = 0; i < m.count; ++i) {
                    ContactPoint& cp = m.points[i];
                    const real pen = cp.penetration - cfg.solver.allowedPenetration;
                    if (pen <= 0.0) continue;
                    maxPen = std::max(maxPen, pen);

                    const real corr = std::min(cfg.solver.baumgarte * pen, cfg.solver.maxLinearCorrection);
                    const Vec2 delta = m.normal * (corr / wsum);
                    if (a.isDynamic()) { a.position -= delta * a.invMass; }
                    if (b.isDynamic()) { b.position += delta * b.invMass; }
                    cp.penetration -= corr;
                }
            }

            if (mod(MOD_CONSTRAINTS))
                for (ConstraintPtr& c : constraints)
                    if (c->enabled && !c->broken) c->solvePosition(h, cfg.solver.schlagerFactor);

            if (maxPen < cfg.solver.allowedPenetration * 0.25) break;
        }

        for (RigidBody* b : bodyList)
            if (!b->isStatic()) b->updateVertices();
    }

    // ------------------------------------------------------------ сон / пробуждение
    void updateSleeping(real h) {
        if (!mod(MOD_SLEEPING)) return;

        // Авто-пробуждение при контакте с активным телом
        for (const Manifold& m : manifoldList) {
            RigidBody& a = *m.bodyA;
            RigidBody& b = *m.bodyB;
            real imp = 0.0;
            for (int i = 0; i < m.count; ++i) imp += m.points[i].normalImpulse;
            if (imp < cfg.wakeImpulseThreshold) continue;
            if (a.sleeping && !b.sleeping && !b.isStatic()) a.wake();
            if (b.sleeping && !a.sleeping && !a.isStatic()) b.wake();
        }

        for (RigidBody* pb : bodyList) {
            RigidBody& b = *pb;
            if (b.isStatic() || !b.allowSleep) continue;
            const bool slow = b.velocity.length() < cfg.sleepLinearTolerance &&
                              std::fabs(b.angularVelocity) < cfg.sleepAngularTolerance;
            if (slow) {
                b.sleepTimer += h;
                if (b.sleepTimer > cfg.sleepTime && !b.sleeping) b.sleep();
            } else {
                b.sleepTimer = 0.0;
                b.sleeping = false;
            }
        }
    }

    // ------------------------------------------------------------ события + warm cache
    void dispatchEvents() {
        touchingNow.clear();
        for (const Manifold& m : manifoldList) {
            if (!m.touching) continue;
            touchingNow.insert(m.key);

            WarmEntry w;
            w.count = m.count;
            for (int i = 0; i < m.count; ++i) {
                w.fid[i] = m.points[i].featureId;
                w.ni[i]  = m.points[i].normalImpulse;
                w.ti[i]  = m.points[i].tangentImpulse;
            }
            warm[m.key] = w;

            if (!mod(MOD_EVENTS)) continue;
            const bool isNew = touchingPrev.find(m.key) == touchingPrev.end();
            const CollisionCallback& cb = isNew ? onBegin : onPersist;
            if (!cb) continue;
            CollisionEvent ev;
            ev.a = m.bodyA; ev.b = m.bodyB;
            ev.normal = m.normal;
            ev.point = m.points[0].point;
            ev.penetration = m.points[0].penetration;
            ev.timeOfImpact = m.toi;
            ev.phase = isNew ? ContactPhase::Begin : ContactPhase::Persist;
            for (int i = 0; i < m.count; ++i) {
                ev.normalImpulse  += m.points[i].normalImpulse;
                ev.tangentImpulse += m.points[i].tangentImpulse;
            }
            ev.relativeSpeed = m.points[0].relVelNormal;
            cb(ev);
        }

        if (mod(MOD_EVENTS) && onEnd) {
            for (uint64_t key : touchingPrev) {
                if (touchingNow.find(key) != touchingNow.end()) continue;
                CollisionEvent ev;
                const BodyId ida = (BodyId)(key >> 32);
                const BodyId idb = (BodyId)(key & 0xFFFFFFFFu);
                auto ia = byId.find(ida), ib = byId.find(idb);
                ev.a = (ia != byId.end()) ? ia->second : nullptr;
                ev.b = (ib != byId.end()) ? ib->second : nullptr;
                ev.phase = ContactPhase::End;
                onEnd(ev);
            }
        }

        // Чистка кэша warm starting для исчезнувших пар
        if (warm.size() > cfg.maxContacts) {
            for (auto it = warm.begin(); it != warm.end();)
                it = (touchingNow.find(it->first) == touchingNow.end()) ? warm.erase(it) : ++it;
        }
        touchingPrev = touchingNow;
    }

    // ------------------------------------------------------------ один substep
    void substep(real h) {
        { ScopedStage s(profiler, prof, Stage::IntegrateForces);
          integrateVelocities(h); }

        { ScopedStage s(profiler, prof, Stage::BroadPhase);
          broad.mode = cfg.broadPhase;
          broad.aabbMargin = cfg.aabbMargin;
          broad.regionCount = cfg.regionCount;
          broad.updateAABBs(bodyList, h);
          if (mod(MOD_COLLISION))
              broad.computePairs(bodyList, pairs, mod(MOD_MULTITHREADING), pool ? pool->threadCount() : 1);
          else
              pairs.clear();
          prof.broadPairs += (int)pairs.size(); }

        { ScopedStage s(profiler, prof, Stage::NarrowPhase);
          narrowPhase(h);
          prof.contacts += (int)manifoldList.size();
          for (const Manifold& m : manifoldList) prof.contactPoints += m.count; }

        { ScopedStage s(profiler, prof, Stage::Constraints);
          if (mod(MOD_CONSTRAINTS))
              for (ConstraintPtr& c : constraints)
                  if (c->enabled && !c->broken) c->prepare(h); }

        { ScopedStage s(profiler, prof, Stage::SolveVelocity);
          prepareContacts(h);
          solveVelocities(h); }

        { ScopedStage s(profiler, prof, Stage::IntegratePositions);
          integratePositions(h); }

        { ScopedStage s(profiler, prof, Stage::SolvePosition);
          solvePositions(h); }

        { ScopedStage s(profiler, prof, Stage::Sleeping);
          updateSleeping(h);
          dispatchEvents(); }

        for (RigidBody* b : bodyList) b->clearForces();
    }
};

// ================================================================ World
World::World(const WorldConfig& cfg) : m(std::make_unique<Impl>(cfg)) {}
World::~World() = default;
World::World(World&&) noexcept = default;
World& World::operator=(World&&) noexcept = default;

BodyId World::createBody(const BodyDef& def) {
    m->storage.emplace_back();
    RigidBody& b = m->storage.back();

    b.id            = m->nextId++;
    b.type          = def.type;
    b.name          = def.name;
    b.userData      = def.userData;
    b.shape         = def.shape;
    b.material      = def.material;
    b.position      = def.position;
    b.prevPosition  = def.position;
    b.velocity      = def.velocity;
    b.angle         = def.angle;
    b.prevAngle     = def.angle;
    b.angularVelocity = def.angularVelocity;
    b.fixedRotation = def.fixedRotation;
    b.allowSleep    = def.allowSleep;
    b.gravityScale  = def.gravityScale;
    b.localVertices = b.shape.vertices;
    b.restVertices  = b.shape.vertices;
    b.vertexVelocity.assign(b.shape.vertices.size(), Vec2());

    b.setType(def.type);
    b.updateVertices();
    b.updateAABB(m->cfg.aabbMargin);

    m->bodyList.push_back(&b);
    m->byId[b.id] = &b;
    return b.id;
}

void World::destroyBody(BodyId id) {
    auto it = m->byId.find(id);
    if (it == m->byId.end()) return;
    RigidBody* b = it->second;

    m->constraints.erase(
        std::remove_if(m->constraints.begin(), m->constraints.end(),
                       [b](const ConstraintPtr& c) { return c->bodyA == b || c->bodyB == b; }),
        m->constraints.end());

    m->bodyList.erase(std::remove(m->bodyList.begin(), m->bodyList.end(), b), m->bodyList.end());
    m->byId.erase(it);
    b->id = INVALID_BODY;
}

RigidBody* World::body(BodyId id) {
    auto it = m->byId.find(id);
    return (it == m->byId.end()) ? nullptr : it->second;
}
const RigidBody* World::body(BodyId id) const {
    auto it = m->byId.find(id);
    return (it == m->byId.end()) ? nullptr : it->second;
}
size_t World::bodyCount() const { return m->bodyList.size(); }
std::vector<RigidBody*>& World::bodies() { return m->bodyList; }

// ---------------------------------------------------------------- ограничения
DistanceConstraint* World::createDistance(BodyId a, BodyId b, const Vec2& la, const Vec2& lb, real length) {
    auto c = std::make_unique<DistanceConstraint>();
    c->bodyA = body(a); c->bodyB = body(b);
    c->localAnchorA = la; c->localAnchorB = lb;
    c->worldAnchorB = lb;
    c->restLength = length;
    DistanceConstraint* raw = c.get();
    m->constraints.push_back(std::move(c));
    return raw;
}
RevoluteConstraint* World::createRevolute(BodyId a, BodyId b, const Vec2& la, const Vec2& lb) {
    auto c = std::make_unique<RevoluteConstraint>();
    c->bodyA = body(a); c->bodyB = body(b);
    c->localAnchorA = la; c->localAnchorB = lb; c->worldAnchorB = lb;
    RevoluteConstraint* raw = c.get();
    m->constraints.push_back(std::move(c));
    return raw;
}
SpringConstraint* World::createSpring(BodyId a, BodyId b, const Vec2& la, const Vec2& lb,
                                      real length, real stiffness, real damping) {
    auto c = std::make_unique<SpringConstraint>();
    c->bodyA = body(a); c->bodyB = body(b);
    c->localAnchorA = la; c->localAnchorB = lb; c->worldAnchorB = lb;
    c->restLength = length; c->stiffness = stiffness; c->damping = damping;
    SpringConstraint* raw = c.get();
    m->constraints.push_back(std::move(c));
    return raw;
}
RopeConstraint* World::createRope(BodyId a, BodyId b, const Vec2& la, const Vec2& lb, real maxLength) {
    auto c = std::make_unique<RopeConstraint>();
    c->bodyA = body(a); c->bodyB = body(b);
    c->localAnchorA = la; c->localAnchorB = lb; c->worldAnchorB = lb;
    c->maxLength = maxLength;
    RopeConstraint* raw = c.get();
    m->constraints.push_back(std::move(c));
    return raw;
}
AngularConstraint* World::createAngular(BodyId a, BodyId b, real minAngle, real maxAngle) {
    auto c = std::make_unique<AngularConstraint>();
    c->bodyA = body(a); c->bodyB = body(b);
    c->minAngle = minAngle; c->maxAngle = maxAngle;
    AngularConstraint* raw = c.get();
    m->constraints.push_back(std::move(c));
    return raw;
}
void World::destroyConstraint(Constraint* c) {
    m->constraints.erase(
        std::remove_if(m->constraints.begin(), m->constraints.end(),
                       [c](const ConstraintPtr& p) { return p.get() == c; }),
        m->constraints.end());
}
size_t World::constraintCount() const { return m->constraints.size(); }
Constraint* World::addConstraint(std::unique_ptr<Constraint> c) {
    if (!c) return nullptr;
    Constraint* raw = c.get();
    m->constraints.push_back(std::move(c));
    return raw;
}
std::vector<Constraint*> World::allConstraints() {
    std::vector<Constraint*> out;
    out.reserve(m->constraints.size());
    for (ConstraintPtr& c : m->constraints) out.push_back(c.get());
    return out;
}
void World::setContactFilter(ContactFilterFn fn) { m->contactFilter = std::move(fn); }
void World::setPostStepCallback(std::function<void(real)> fn) { m->postStep = std::move(fn); }

// ---------------------------------------------------------------- шаг
void World::step(real dt) {
    if (dt <= 0.0) return;
    m->prof.reset();
    m->profiler.enabled = m->mod(MOD_PROFILING);
    const auto stepStart = Profiler::Clock::now();

    // Динамическое изменение шага интегрирования
    real stepDt = dt;
    int  substeps = m->mod(MOD_SUBSTEPPING) ? std::max(1, m->cfg.substeps) : 1;

    if (m->mod(MOD_ADAPTIVE_TIMESTEP)) {
        real maxSpeed = 0.0;
        for (const RigidBody* b : m->bodyList)
            if (b->isDynamic() && !b->sleeping) maxSpeed = std::max(maxSpeed, b->velocity.length());
        if (maxSpeed > m->cfg.adaptiveSpeedLimit) {
            const real scale = maxSpeed / m->cfg.adaptiveSpeedLimit;
            substeps = std::min((int)std::ceil(substeps * scale), m->cfg.maxSubsteps);
        }
        stepDt = clampr(dt, m->cfg.minTimeStep * substeps, m->cfg.maxTimeStep);
    }

    const real h = stepDt / (real)substeps;   // substepping с дробными шагами
    m->prof.substepsUsed = substeps;
    m->prof.dtUsed = stepDt;

    for (int i = 0; i < substeps; ++i) m->substep(h);

    m->prof.bodies = (int)m->bodyList.size();
    m->prof.awakeBodies = 0;
    m->prof.sleepingBodies = 0;
    for (const RigidBody* b : m->bodyList) {
        if (b->isStatic()) continue;
        if (b->sleeping) ++m->prof.sleepingBodies; else ++m->prof.awakeBodies;
    }
    m->prof.satCacheHits = (int)m->satCache.size();

    if (m->mod(MOD_DEBUG_DRAW)) {
        ScopedStage s(m->profiler, m->prof, Stage::DebugDraw);
        buildDebugGeometry();
    }

    if (m->postStep) m->postStep(stepDt);

    m->prof.ms[(size_t)Stage::Total] =
        std::chrono::duration<double, std::milli>(Profiler::Clock::now() - stepStart).count();
    Profiler::commit(m->prof);
}

void World::step() { step(m->cfg.fixedTimeStep); }

// ---------------------------------------------------------------- настройки
WorldConfig&       World::config()       { return m->cfg; }
const WorldConfig& World::config() const { return m->cfg; }
void  World::setGravity(const Vec2& g)   { m->cfg.gravity = g; }
Vec2  World::gravity() const             { return m->cfg.gravity; }
void  World::setIntegrator(Integrator i) { m->cfg.integrator = i; }
Integrator World::integrator() const     { return m->cfg.integrator; }

void World::enableModule(uint32_t flag, bool on) {
    if (on) m->cfg.modules |= flag; else m->cfg.modules &= ~flag;
}
bool World::moduleEnabled(uint32_t flag) const { return (m->cfg.modules & flag) != 0; }
void World::setModules(uint32_t mask) { m->cfg.modules = mask; }
uint32_t World::modules() const { return m->cfg.modules; }

void World::setBeginContactCallback(CollisionCallback cb)   { m->onBegin = std::move(cb); }
void World::setPersistContactCallback(CollisionCallback cb) { m->onPersist = std::move(cb); }
void World::setEndContactCallback(CollisionCallback cb)     { m->onEnd = std::move(cb); }

const ProfileData& World::profile() const { return m->prof; }
size_t World::contactCount() const { return m->manifoldList.size(); }
size_t World::awakeCount() const { return (size_t)m->prof.awakeBodies; }
size_t World::arenaBytes() const {
    return m->arena.size() + m->contactBuffer.bytes() +
           m->bodyList.capacity() * sizeof(RigidBody*) +
           m->storage.size() * sizeof(RigidBody);
}
real World::totalEnergy() const {
    real e = 0.0;
    for (const RigidBody* b : m->bodyList) {
        if (!b->isDynamic()) continue;
        e += b->kineticEnergy();
        e += -m->cfg.gravity.y * b->mass * b->position.y;
    }
    return e;
}
const std::vector<Manifold>& World::manifolds() const { return m->manifoldList; }

DebugDrawBuffer&       World::debugBuffer()       { return m->debug; }
const DebugDrawBuffer& World::debugBuffer() const { return m->debug; }

void World::buildDebugGeometry() {
    DebugDrawBuffer& d = m->debug;
    d.clear();

    const DebugColor cShape{160, 200, 255, 255};
    const DebugColor cSleep{110, 110, 130, 255};
    const DebugColor cStatic{120, 220, 150, 255};
    const DebugColor cAabb{80, 90, 110, 200};
    const DebugColor cContact{255, 90, 70, 255};
    const DebugColor cNormal{255, 200, 60, 255};
    const DebugColor cForce{120, 255, 180, 255};
    const DebugColor cTrail{90, 160, 255, 160};
    const DebugColor cSat{220, 120, 255, 220};

    for (const RigidBody* b : m->bodyList) {
        const DebugColor col = b->isStatic() ? cStatic : (b->sleeping ? cSleep : cShape);

        if (d.has(DBG_SHAPES)) {
            if (b->shape.type == ShapeType::Circle) {
                d.circle(b->position, b->shape.radius, col, DBG_SHAPES);
                d.line(b->position, b->position + Vec2::fromAngle(b->angle, b->shape.radius), col, DBG_SHAPES);
            } else if (b->shape.type == ShapeType::Capsule) {
                Vec2 a0, a1;
                b->shape.capsuleSegment(b->transform(), a0, a1);
                const Vec2 n = (a1 - a0).normalized().perp() * b->shape.radius;
                d.line(a0 + n, a1 + n, col, DBG_SHAPES);
                d.line(a0 - n, a1 - n, col, DBG_SHAPES);
                d.circle(a0, b->shape.radius, col, DBG_SHAPES);
                d.circle(a1, b->shape.radius, col, DBG_SHAPES);
            } else {
                const std::vector<Vec2>& v = b->worldVertices;
                for (size_t i = 0; i < v.size(); ++i)
                    d.line(v[i], v[(i + 1) % v.size()], col, DBG_SHAPES);
            }
        }
        if (d.has(DBG_AABB))   d.box(b->aabb, cAabb, DBG_AABB);            // отображение AABB
        if (d.has(DBG_FORCES)) d.line(b->position, b->position + b->velocity * d.forceScale * 20.0,
                                      cForce, DBG_FORCES);                 // рисование сил
        if (d.has(DBG_TRAILS))                                             // траектории
            for (size_t i = 1; i < b->trail.size(); ++i)
                d.line(b->trail[i - 1], b->trail[i], cTrail, DBG_TRAILS);
    }

    for (const Manifold& mf : m->manifoldList) {
        for (int i = 0; i < mf.count; ++i) {
            d.point(mf.points[i].point, 0.06, cContact, DBG_CONTACTS);      // контакты
            d.line(mf.points[i].point, mf.points[i].point + mf.normal * d.normalScale,
                   cNormal, DBG_NORMALS);
        }
        if (d.has(DBG_SAT_AXES) && mf.satAxis >= 0) {                       // оси SAT
            const Vec2 c = mf.points[0].point;
            d.line(c - mf.satAxisDir * 0.5, c + mf.satAxisDir * 0.5, cSat, DBG_SAT_AXES);
        }
    }

    if (d.has(DBG_JOINTS))
        for (const ConstraintPtr& c : m->constraints)
            d.line(c->anchorAWorld(), c->anchorBWorld(), DebugColor{255, 255, 120, 255}, DBG_JOINTS);
}

// ---------------------------------------------------------------- запросы
RigidBody* World::queryPoint(const Vec2& p) {
    for (RigidBody* b : m->bodyList)
        if (b->aabb.contains(p) && b->shape.containsPoint(b->transform(), p)) return b;
    return nullptr;
}
std::vector<RigidBody*> World::queryAABB(const AABB& box) {
    std::vector<RigidBody*> out;
    for (RigidBody* b : m->bodyList)
        if (b->aabb.overlaps(box)) out.push_back(b);
    return out;
}

void World::clear() {
    m->constraints.clear();
    m->bodyList.clear();
    m->byId.clear();
    m->storage.clear();
    m->manifoldList.clear();
    m->pairs.clear();
    m->warm.clear();
    m->touchingPrev.clear();
    m->satCache.clear();
    m->nextId = 1;
}

// ---------------------------------------------------------------- JSON
static JsonValue vecToJson(const Vec2& v) {
    JsonValue o = JsonValue::makeObject();
    o.set("x", v.x); o.set("y", v.y);
    return o;
}
static Vec2 jsonToVec(const JsonValue* o) {
    if (!o) return Vec2();
    return Vec2(o->numberOr("x", 0.0), o->numberOr("y", 0.0));
}

std::string World::toJson(bool pretty) const {
    JsonValue root = JsonValue::makeObject();
    root.set("version", 1);

    JsonValue cfg = JsonValue::makeObject();
    cfg.set("gravity", vecToJson(m->cfg.gravity));
    cfg.set("integrator", (int)m->cfg.integrator);
    cfg.set("modules", (double)m->cfg.modules);
    cfg.set("fixedTimeStep", m->cfg.fixedTimeStep);
    cfg.set("substeps", m->cfg.substeps);
    cfg.set("gridCellSize", m->cfg.gridCellSize);
    cfg.set("aabbMargin", m->cfg.aabbMargin);
    cfg.set("velocityIterations", m->cfg.solver.velocityIterations);
    cfg.set("positionIterations", m->cfg.solver.positionIterations);
    cfg.set("baumgarte", m->cfg.solver.baumgarte);
    cfg.set("restitutionThreshold", m->cfg.solver.restitutionThreshold);
    cfg.set("schlagerFactor", m->cfg.solver.schlagerFactor);
    cfg.set("fluidLevel", m->cfg.medium.fluidLevel);
    cfg.set("fluidDensity", m->cfg.medium.fluidDensity);
    cfg.set("airDensity", m->cfg.medium.airDensity);
    cfg.set("viscosity", m->cfg.medium.viscosity);
    cfg.set("wind", vecToJson(m->cfg.medium.wind));
    root.set("config", cfg);

    JsonValue bodies = JsonValue::makeArray();
    for (const RigidBody* b : m->bodyList) {
        JsonValue jb = JsonValue::makeObject();
        jb.set("id", (double)b->id);
        jb.set("name", b->name);
        jb.set("type", (int)b->type);
        jb.set("shapeType", (int)b->shape.type);
        jb.set("radius", b->shape.radius);
        jb.set("halfLength", b->shape.halfLength);
        jb.set("width", b->shape.width);
        jb.set("height", b->shape.height);
        jb.set("position", vecToJson(b->position));
        jb.set("velocity", vecToJson(b->velocity));
        jb.set("angle", b->angle);
        jb.set("angularVelocity", b->angularVelocity);
        jb.set("mass", b->mass);
        jb.set("inertia", b->inertia);
        jb.set("fixedRotation", b->fixedRotation);
        jb.set("allowSleep", b->allowSleep);
        jb.set("sleeping", b->sleeping);
        jb.set("gravityScale", b->gravityScale);

        JsonValue mat = JsonValue::makeObject();
        mat.set("density", b->material.density);
        mat.set("restitution", b->material.restitution);
        mat.set("staticFriction", b->material.staticFriction);
        mat.set("dynamicFriction", b->material.dynamicFriction);
        mat.set("rollingFriction", b->material.rollingFriction);
        mat.set("angularFriction", b->material.angularFriction);
        mat.set("linearDrag", b->material.linearDrag);
        mat.set("quadraticDrag", b->material.quadraticDrag);
        mat.set("liftCoefficient", b->material.liftCoefficient);
        mat.set("softness", b->material.softness);
        mat.set("hysteresis", b->material.hysteresis);
        jb.set("material", mat);

        JsonValue verts = JsonValue::makeArray();
        for (const Vec2& v : b->localVertices) verts.push(vecToJson(v));
        jb.set("vertices", verts);

        bodies.push(jb);
    }
    root.set("bodies", bodies);

    JsonValue joints = JsonValue::makeArray();
    for (const ConstraintPtr& c : m->constraints) {
        JsonValue jc = JsonValue::makeObject();
        jc.set("type", (int)c->type());
        jc.set("bodyA", (double)(c->bodyA ? c->bodyA->id : INVALID_BODY));
        jc.set("bodyB", (double)(c->bodyB ? c->bodyB->id : INVALID_BODY));
        jc.set("localAnchorA", vecToJson(c->localAnchorA));
        jc.set("localAnchorB", vecToJson(c->localAnchorB));
        switch (c->type()) {
            case ConstraintType::Distance:
                jc.set("restLength", ((DistanceConstraint*)c.get())->restLength); break;
            case ConstraintType::Spring: {
                auto* s = (SpringConstraint*)c.get();
                jc.set("restLength", s->restLength);
                jc.set("stiffness", s->stiffness);
                jc.set("damping", s->damping);
                break;
            }
            case ConstraintType::Rope:
                jc.set("maxLength", ((RopeConstraint*)c.get())->maxLength); break;
            case ConstraintType::Angular: {
                auto* an = (AngularConstraint*)c.get();
                jc.set("minAngle", an->minAngle);
                jc.set("maxAngle", an->maxAngle);
                break;
            }
            default: break;
        }
        joints.push(jc);
    }
    root.set("constraints", joints);
    return root.dump(pretty);
}

bool World::fromJson(const std::string& json, std::string* error) {
    JsonValue root;
    if (!JsonValue::parse(json, root, error)) return false;
    clear();

    if (const JsonValue* c = root.find("config")) {
        m->cfg.gravity      = jsonToVec(c->find("gravity"));
        m->cfg.integrator   = (Integrator)(int)c->numberOr("integrator", 0);
        m->cfg.modules      = (uint32_t)c->numberOr("modules", MOD_DEFAULT);
        m->cfg.fixedTimeStep = c->numberOr("fixedTimeStep", 1.0 / 60.0);
        m->cfg.substeps     = (int)c->numberOr("substeps", 4);
        m->cfg.gridCellSize = c->numberOr("gridCellSize", 4.0);
        m->cfg.aabbMargin   = c->numberOr("aabbMargin", 0.1);
        m->cfg.solver.velocityIterations = (int)c->numberOr("velocityIterations", 12);
        m->cfg.solver.positionIterations = (int)c->numberOr("positionIterations", 10);
        m->cfg.solver.baumgarte = c->numberOr("baumgarte", 0.2);
        m->cfg.solver.restitutionThreshold = c->numberOr("restitutionThreshold", 1.0);
        m->cfg.solver.schlagerFactor = c->numberOr("schlagerFactor", 0.8);
        m->cfg.medium.fluidLevel   = c->numberOr("fluidLevel", -1e30);
        m->cfg.medium.fluidDensity = c->numberOr("fluidDensity", 1000.0);
        m->cfg.medium.airDensity   = c->numberOr("airDensity", 1.204);
        m->cfg.medium.viscosity    = c->numberOr("viscosity", 0.9);
        m->cfg.medium.wind         = jsonToVec(c->find("wind"));
        m->broad.setCellSize(m->cfg.gridCellSize);
    }

    std::unordered_map<uint32_t, BodyId> idMap;

    if (const JsonValue* arr = root.find("bodies")) {
        for (const JsonValue& jb : arr->arr) {
            BodyDef def;
            def.type = (BodyType)(int)jb.numberOr("type", 1);
            def.name = jb.stringOr("name", "");
            const int st = (int)jb.numberOr("shapeType", 0);
            const real r = jb.numberOr("radius", 0.5);
            if (st == (int)ShapeType::Circle) {
                def.shape = Shape::circle(r);
            } else if (st == (int)ShapeType::Capsule) {
                def.shape = Shape::capsule(r, jb.numberOr("halfLength", 0.5) * 2.0);
            } else {
                std::vector<Vec2> verts;
                if (const JsonValue* vj = jb.find("vertices"))
                    for (const JsonValue& v : vj->arr) verts.push_back(jsonToVec(&v));
                if (verts.size() >= 3) def.shape = Shape::polygon(verts);
                else def.shape = Shape::box(jb.numberOr("width", 1.0), jb.numberOr("height", 1.0));
            }
            def.position = jsonToVec(jb.find("position"));
            def.velocity = jsonToVec(jb.find("velocity"));
            def.angle = jb.numberOr("angle", 0.0);
            def.angularVelocity = jb.numberOr("angularVelocity", 0.0);
            def.fixedRotation = jb.boolOr("fixedRotation", false);
            def.allowSleep = jb.boolOr("allowSleep", true);
            def.gravityScale = jb.numberOr("gravityScale", 1.0);

            if (const JsonValue* mat = jb.find("material")) {
                def.material.density         = mat->numberOr("density", 1.0);
                def.material.restitution     = mat->numberOr("restitution", 0.35);
                def.material.staticFriction  = mat->numberOr("staticFriction", 0.6);
                def.material.dynamicFriction = mat->numberOr("dynamicFriction", 0.4);
                def.material.rollingFriction = mat->numberOr("rollingFriction", 0.02);
                def.material.angularFriction = mat->numberOr("angularFriction", 0.01);
                def.material.linearDrag      = mat->numberOr("linearDrag", 0.02);
                def.material.quadraticDrag   = mat->numberOr("quadraticDrag", 0.01);
                def.material.liftCoefficient = mat->numberOr("liftCoefficient", 0.0);
                def.material.softness        = mat->numberOr("softness", 0.0);
                def.material.hysteresis      = mat->numberOr("hysteresis", 0.15);
            }

            const BodyId newId = createBody(def);
            idMap[(uint32_t)jb.numberOr("id", 0.0)] = newId;
            if (RigidBody* nb = body(newId)) {
                nb->sleeping = jb.boolOr("sleeping", false);
                const real mass = jb.numberOr("mass", nb->mass);
                const real inertia = jb.numberOr("inertia", nb->inertia);
                if (nb->isDynamic()) nb->setMassData(mass, inertia);
            }
        }
    }

    if (const JsonValue* arr = root.find("constraints")) {
        for (const JsonValue& jc : arr->arr) {
            const uint32_t oa = (uint32_t)jc.numberOr("bodyA", 0.0);
            const uint32_t ob = (uint32_t)jc.numberOr("bodyB", 0.0);
            auto ia = idMap.find(oa), ib = idMap.find(ob);
            if (ia == idMap.end() || ib == idMap.end()) continue;
            const Vec2 la = jsonToVec(jc.find("localAnchorA"));
            const Vec2 lb = jsonToVec(jc.find("localAnchorB"));
            switch ((ConstraintType)(int)jc.numberOr("type", 0)) {
                case ConstraintType::Distance:
                    createDistance(ia->second, ib->second, la, lb, jc.numberOr("restLength", 1.0)); break;
                case ConstraintType::Revolute:
                    createRevolute(ia->second, ib->second, la, lb); break;
                case ConstraintType::Spring:
                    createSpring(ia->second, ib->second, la, lb,
                                 jc.numberOr("restLength", 1.0),
                                 jc.numberOr("stiffness", 200.0),
                                 jc.numberOr("damping", 5.0)); break;
                case ConstraintType::Rope:
                    createRope(ia->second, ib->second, la, lb, jc.numberOr("maxLength", 2.0)); break;
                case ConstraintType::Angular:
                    createAngular(ia->second, ib->second,
                                  jc.numberOr("minAngle", -PI * 0.25),
                                  jc.numberOr("maxAngle",  PI * 0.25)); break;
            }
        }
    }
    return true;
}

bool World::saveToFile(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << toJson(true);
    return f.good();
}

bool World::loadFromFile(const std::string& path, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { if (error) *error = "cannot open file"; return false; }
    std::ostringstream ss;
    ss << f.rdbuf();
    return fromJson(ss.str(), error);
}

} // namespace phys2d
