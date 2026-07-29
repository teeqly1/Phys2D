// phys2d v3 — реализация расширенного блока.
#include "phys2d/Advanced2.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

namespace phys2d {

static inline real clamp01(real v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
static inline double nowSeconds() {
    using namespace std::chrono;
    return duration_cast<duration<double> >(steady_clock::now().time_since_epoch()).count();
}

// ================================================================ единицы
UnitScale UnitScale::of(Units u, real pixelsPerMeter) {
    UnitScale s;
    switch (u) {
        case Units::Meters:      s.toMeters = 1.0; break;
        case Units::Centimeters: s.toMeters = 0.01; break;
        case Units::Pixels:      s.toMeters = 1.0 / std::max((real)1e-6, pixelsPerMeter); break;
    }
    return s;
}

// ================================================================ составные тела
MassData compoundMass(const std::vector<CompoundPart>& parts) {
    MassData total;
    total.mass = 0.0; total.inertia = 0.0; total.area = 0.0; total.center = Vec2();
    for (const CompoundPart& p : parts) {
        MassData md = p.shape.computeMass(p.density);
        Vec2 c = p.offset + Vec2(std::cos(p.angle) * md.center.x - std::sin(p.angle) * md.center.y,
                                 std::sin(p.angle) * md.center.x + std::cos(p.angle) * md.center.y);
        total.mass   += md.mass;
        total.area   += md.area;
        total.center += c * md.mass;
    }
    if (total.mass > 0.0) total.center /= total.mass;
    for (const CompoundPart& p : parts) {
        MassData md = p.shape.computeMass(p.density);
        Vec2 c = p.offset + md.center;
        total.inertia += md.inertia + md.mass * distanceSq(c, total.center);
    }
    return total;
}

Compound createCompound(World& w, const std::vector<CompoundPart>& parts, const Vec2& position,
                        real angle, BodyType type, real weldStiffness) {
    Compound out;
    if (parts.empty()) return out;

    MassData md = compoundMass(parts);
    out.center  = md.center;
    out.mass    = md.mass;
    out.inertia = md.inertia;

    const real ca = std::cos(angle), sa = std::sin(angle);
    for (size_t i = 0; i < parts.size(); ++i) {
        const CompoundPart& p = parts[i];
        Vec2 local = p.offset - md.center;
        Vec2 world = position + Vec2(ca * local.x - sa * local.y, sa * local.x + ca * local.y);

        BodyDef d;
        d.type             = type;
        d.shape            = p.shape;
        d.position         = world;
        d.angle            = angle + p.angle;
        d.material.density = p.density;
        BodyId id = w.createBody(d);
        if (i == 0) out.root = id;
        out.parts.push_back(id);
    }

    for (size_t i = 1; i < out.parts.size(); ++i) {
        RigidBody* a = w.body(out.parts[0]);
        RigidBody* b = w.body(out.parts[i]);
        if (!a || !b) continue;
        Vec2 mid = (a->position + b->position) * 0.5;
        Vec2 la  = mid - a->position;
        Vec2 lb  = mid - b->position;
        if (weldStiffness > 0.0) {
            w.createSpring(out.parts[0], out.parts[i], la, lb, 0.0, weldStiffness, weldStiffness * 0.08);
        } else {
            w.createRevolute(out.parts[0], out.parts[i], la, lb);
            w.createAngular(out.parts[0], out.parts[i], -0.01, 0.01);
        }
    }
    return out;
}

void destroyCompound(World& w, Compound& c) {
    for (BodyId id : c.parts) w.destroyBody(id);
    c.parts.clear();
    c.root = INVALID_BODY;
}

// ================================================================ дерево материалов
int MaterialTree::add(const std::string& name, const Material& m, const std::string& parent, uint32_t ownMask) {
    Node n;
    n.name   = name;
    n.mat    = m;
    n.parent = parent.empty() ? -1 : id(parent);
    n.mask   = ownMask;
    m_nodes.push_back(n);
    int index = (int)m_nodes.size() - 1;
    m_index[name] = index;
    return index;
}

int MaterialTree::id(const std::string& name) const {
    auto it = m_index.find(name);
    return it == m_index.end() ? -1 : it->second;
}

Material MaterialTree::resolve(int index) const {
    if (index < 0 || index >= (int)m_nodes.size()) return Material{};
    std::vector<int> chain;
    for (int i = index, guard = 0; i >= 0 && guard < 64; i = m_nodes[i].parent, ++guard) chain.push_back(i);
    Material out = m_nodes[chain.back()].mat;
    for (size_t k = chain.size(); k-- > 0; ) {
        const Node& n = m_nodes[chain[k]];
        if (n.mask & MF_DENSITY)          out.density         = n.mat.density;
        if (n.mask & MF_RESTITUTION)      out.restitution     = n.mat.restitution;
        if (n.mask & MF_STATIC_FRICTION)  out.staticFriction  = n.mat.staticFriction;
        if (n.mask & MF_DYNAMIC_FRICTION) out.dynamicFriction = n.mat.dynamicFriction;
        if (n.mask & MF_ROLLING) {
            out.rollingFriction = n.mat.rollingFriction;
            out.angularFriction = n.mat.angularFriction;
        }
        if (n.mask & MF_DRAG) {
            out.linearDrag    = n.mat.linearDrag;
            out.quadraticDrag = n.mat.quadraticDrag;
        }
    }
    return out;
}

Material MaterialTree::resolve(const std::string& name) const { return resolve(id(name)); }

std::vector<std::string> MaterialTree::names() const {
    std::vector<std::string> out;
    out.reserve(m_nodes.size());
    for (const Node& n : m_nodes) out.push_back(n.name);
    return out;
}

void MaterialTree::loadStandardLibrary() {
    Material base;
    base.density = 1000.0; base.restitution = 0.2;
    base.staticFriction = 0.6; base.dynamicFriction = 0.45;
    add("generic", base);

    Material solid = base; solid.restitution = 0.15; solid.linearDrag = 0.02;
    add("solid", solid, "generic");

    Material metal = solid; metal.density = 7800.0; metal.restitution = 0.25;
    metal.staticFriction = 0.55; metal.dynamicFriction = 0.42;
    add("metal", metal, "solid");
    Material steel = metal; steel.density = 7850.0;  add("steel", steel, "metal");
    Material alu   = metal; alu.density   = 2700.0;  add("aluminium", alu, "metal");
    Material lead  = metal; lead.density  = 11340.0; lead.restitution = 0.05;
    add("lead", lead, "metal");

    Material wood = solid; wood.density = 550.0; wood.restitution = 0.28;
    wood.staticFriction = 0.62; wood.dynamicFriction = 0.48;
    add("wood", wood, "solid");
    Material balsa = wood; balsa.density = 160.0; add("balsa", balsa, "wood");
    Material oak   = wood; oak.density   = 760.0; add("oak", oak, "wood");

    Material stone = solid; stone.density = 2600.0; stone.restitution = 0.10;
    stone.staticFriction = 0.80; stone.dynamicFriction = 0.65;
    add("stone", stone, "solid");
    Material concrete = stone; concrete.density = 2400.0; add("concrete", concrete, "stone");

    Material glass = solid; glass.density = 2500.0; glass.restitution = 0.18;
    glass.staticFriction = 0.35; glass.dynamicFriction = 0.28;
    add("glass", glass, "solid");

    Material rubber = solid; rubber.density = 1100.0; rubber.restitution = 0.82;
    rubber.staticFriction = 1.10; rubber.dynamicFriction = 0.95;
    add("rubber", rubber, "solid");

    Material ice = solid; ice.density = 917.0; ice.restitution = 0.12;
    ice.staticFriction = 0.08; ice.dynamicFriction = 0.04;
    add("ice", ice, "solid");

    Material flesh = base; flesh.density = 1050.0; flesh.restitution = 0.04;
    flesh.staticFriction = 0.92; flesh.dynamicFriction = 0.80; flesh.softness = 0.55;
    add("flesh", flesh, "generic");
    Material bone = flesh; bone.density = 1900.0; bone.softness = 0.05;
    bone.restitution = 0.16;
    add("bone", bone, "flesh");
}

// ================================================================ поверхности и фильтрация
SurfaceProfile& SurfaceProfileRegistry::ref(BodyId id) { return m_map[id]; }
const SurfaceProfile* SurfaceProfileRegistry::find(BodyId id) const {
    auto it = m_map.find(id);
    return it == m_map.end() ? nullptr : &it->second;
}
void SurfaceProfileRegistry::remove(BodyId id) { m_map.erase(id); }

static inline uint64_t layerKey(int a, int b) {
    uint32_t x = (uint32_t)(a < b ? a : b), y = (uint32_t)(a < b ? b : a);
    return ((uint64_t)x << 32) | y;
}

void CollisionRules::setLayerRule(int a, int b, bool collide) { m_layerRules[layerKey(a, b)] = collide; }

bool CollisionRules::layersCollide(int a, int b) const {
    auto it = m_layerRules.find(layerKey(a, b));
    return it == m_layerRules.end() ? true : it->second;
}

void CollisionRules::setUserFilter(UserFilter f) { m_user = std::move(f); }

bool CollisionRules::shouldCollide(const RigidBody& a, const RigidBody& b) const {
    const SurfaceProfile* sa = surfaces.find(a.id);
    const SurfaceProfile* sb = surfaces.find(b.id);
    if (sa && sb) {
        if (sa->group != 0 && sa->group == sb->group) {
            if ((sa->mask & sb->group) == 0 || (sb->mask & sa->group) == 0) { ++m_rejected; return false; }
        } else {
            if (sb->group != 0 && (sa->mask & sb->group) == 0) { ++m_rejected; return false; }
            if (sa->group != 0 && (sb->mask & sa->group) == 0) { ++m_rejected; return false; }
        }
        if (!layersCollide(sa->layer, sb->layer)) { ++m_rejected; return false; }
    }
    if (m_user && !m_user(a, b)) { ++m_rejected; return false; }
    return true;
}

void CollisionRules::install(World& w) {
    CollisionRules* self = this;
    w.setContactFilter([self](const RigidBody& a, const RigidBody& b) { return self->shouldCollide(a, b); });
}

void ContactTuner::install(World& w) {
    ContactTuner* self = this;
    w.setPersistContactCallback([self](const CollisionEvent& e) { self->onContact(e); });
}

void ContactTuner::onContact(const CollisionEvent& e) {
    if (!surfaces || !e.a || !e.b) return;
    RigidBody* a = e.a;
    RigidBody* b = e.b;
    const SurfaceProfile* pa = surfaces->find(a->id);
    const SurfaceProfile* pb = surfaces->find(b->id);
    if (!pa && !pb) return;

    const Vec2 n = e.normal;
    const Vec2 t(-n.y, n.x);
    const real invSum = a->invMass + b->invMass;
    if (invSum <= 0.0) return;

    Vec2 rv = b->velocityAtPoint(e.point) - a->velocityAtPoint(e.point);
    real vt = dot(rv, t);
    real vn = dot(rv, n);

    SurfaceProfile sa = pa ? *pa : SurfaceProfile{};
    SurfaceProfile sb = pb ? *pb : SurfaceProfile{};

    // ---- анизотропное трение
    real aniso = 1.0;
    for (int k = 0; k < 2; ++k) {
        const SurfaceProfile* p = k == 0 ? pa : pb;
        if (!p) continue;
        if (std::fabs(p->frictionAlong - 1.0) < 1e-6 && std::fabs(p->frictionAcross - 1.0) < 1e-6) continue;
        Vec2 axis = p->frictionAxis.normalized();
        real c    = std::fabs(dot(t, axis));
        aniso *= p->frictionAlong * c + p->frictionAcross * (1.0 - c);
    }

    // ---- трение только в одну сторону
    real oneWay = 0.0;
    for (int k = 0; k < 2; ++k) {
        const SurfaceProfile* p = k == 0 ? pa : pb;
        if (!p || p->oneWayFriction <= 0.0) continue;
        Vec2 axis = p->frictionAxis.normalized();
        if (dot(rv, axis) < 0.0) oneWay = std::max(oneWay, p->oneWayFriction);
    }

    real extraTangent = 0.0;
    if (std::fabs(aniso - 1.0) > 1e-6) extraTangent += (aniso - 1.0);
    extraTangent += oneWay;

    if (std::fabs(extraTangent) > 1e-9 && std::fabs(vt) > 1e-6) {
        real ra = cross(e.point - a->position, t), rb = cross(e.point - b->position, t);
        real kt = invSum + a->invInertia * ra * ra + b->invInertia * rb * rb;
        if (kt > 1e-12) {
            real limit = std::fabs(e.normalImpulse) * std::max((real)0.0, extraTangent);
            real jt    = clampr(-vt / kt, -limit, limit);
            Vec2 imp   = t * jt;
            a->applyImpulseAtPoint(imp * -1.0, e.point);
            b->applyImpulseAtPoint(imp, e.point);
            m_energy += std::fabs(jt * vt);
            ++m_adjusted;
        }
    }

    // ---- SPIN FRICTION
    real spin = std::max(sa.spinFriction, sb.spinFriction);
    if (spin > 0.0) {
        real dw = b->angularVelocity - a->angularVelocity;
        real k  = a->invInertia + b->invInertia;
        if (k > 1e-12 && std::fabs(dw) > 1e-6) {
            real limit = std::fabs(e.normalImpulse) * spin * 0.05;
            real j     = clampr(-dw / k, -limit, limit);
            a->applyAngularImpulse(-j);
            b->applyAngularImpulse(j);
            ++m_adjusted;
        }
    }

    // ---- поглощение энергии по углу удара
    real absorb = std::max(sa.angleAbsorption, sb.angleAbsorption);
    if (absorb > 0.0 && vn < 0.0) {
        real speed = rv.length();
        if (speed > 1e-6) {
            real cosA = std::fabs(vn) / speed;
            real k    = absorb * (1.0 - cosA);
            Vec2 imp  = rv * (-k / invSum);
            a->applyImpulseAtPoint(imp * -1.0, e.point);
            b->applyImpulseAtPoint(imp, e.point);
            m_energy += std::fabs(k * speed);
            ++m_adjusted;
        }
    }

    // ---- динамическое e(v)
    real eRest  = sa.restitutionAtRest  >= 0.0 ? sa.restitutionAtRest  : sb.restitutionAtRest;
    real eSpeed = sa.restitutionAtSpeed >= 0.0 ? sa.restitutionAtSpeed : sb.restitutionAtSpeed;
    if (eRest >= 0.0 && eSpeed >= 0.0 && e.relativeSpeed < -0.5) {
        real s      = clamp01(std::fabs(e.relativeSpeed) / std::max((real)0.1, speedReference));
        real target = eRest + (eSpeed - eRest) * s * s;
        real mixed  = 0.5 * (a->material.restitution + b->material.restitution);
        real delta  = target - mixed;
        if (std::fabs(delta) > 1e-4) {
            Vec2 imp = n * (delta * std::fabs(e.relativeSpeed) / invSum);
            a->applyImpulseAtPoint(imp * -1.0, e.point);
            b->applyImpulseAtPoint(imp, e.point);
            ++m_adjusted;
        }
    }
}

// ================================================================ управление телами
BodyControl& ControlSystem::ref(BodyId id) { return m_map[id]; }
const BodyControl* ControlSystem::find(BodyId id) const {
    auto it = m_map.find(id);
    return it == m_map.end() ? nullptr : &it->second;
}
void ControlSystem::remove(BodyId id) { m_map.erase(id); m_smoothed.erase(id); }

void ControlSystem::kick(World& w, BodyId id, const Vec2& impulse) {
    if (RigidBody* b = w.body(id)) { b->wake(); b->applyImpulse(impulse); }
}

void ControlSystem::applyBeforeStep(World& w, real dt) {
    (void)dt;
    for (auto& kv : m_map) {
        RigidBody* b = w.body(kv.first);
        if (!b) continue;
        BodyControl& c = kv.second;

        if (c.constantForce.lengthSq() > 0.0)  b->applyForce(c.constantForce);
        if (std::fabs(c.constantTorque) > 0.0) b->applyTorque(c.constantTorque);

        if (c.holdPosition && b->invMass > 0.0) {
            Vec2 d = c.anchor - b->position;
            b->applyForce(d * (c.holdStiffness * b->mass) - b->velocity * (c.holdDamping * b->mass));
            b->wake();
        }

        if (c.useTargetVelocity) {
            if (b->type == BodyType::Kinematic || b->invMass <= 0.0) {
                b->velocity        = c.targetVelocity;
                b->angularVelocity = c.targetAngular;
            } else {
                Vec2 dv = c.targetVelocity - b->velocity;
                b->applyForce(dv * (c.gain * b->mass));
                b->applyTorque((c.targetAngular - b->angularVelocity) * c.gain * b->inertia);
            }
            b->wake();
        }
    }
}

void ControlSystem::applyAfterStep(World& w, real dt) {
    for (auto& kv : m_map) {
        RigidBody* b = w.body(kv.first);
        if (!b) continue;
        BodyControl& c = kv.second;

        if (c.lock.x)        b->velocity.x = 0.0;
        if (c.lock.y)        b->velocity.y = 0.0;
        if (c.lock.rotation) { b->angularVelocity = 0.0; b->angle = b->prevAngle; }

        if (c.limits.maxSpeed > 0.0) {
            real sp = b->velocity.length();
            if (sp > c.limits.maxSpeed) b->velocity *= c.limits.maxSpeed / sp;
        }
        if (c.limits.maxAngularSpeed > 0.0)
            b->angularVelocity = clampr(b->angularVelocity, -c.limits.maxAngularSpeed, c.limits.maxAngularSpeed);
        if (c.limits.maxAngularStep > 0.0 && dt > 0.0) {
            real maxOmega = c.limits.maxAngularStep / dt;
            b->angularVelocity = clampr(b->angularVelocity, -maxOmega, maxOmega);
        }

        if (c.smoothing > 0.0) {
            auto it = m_smoothed.find(kv.first);
            if (it == m_smoothed.end()) m_smoothed[kv.first] = b->position;
            else {
                real k = clamp01(c.smoothing);
                it->second = it->second * k + b->position * (1.0 - k);
                b->position = it->second;
                b->updateVertices();
                b->updateAABB(w.config().aabbMargin);
            }
        }
    }
}

// ================================================================ гравитация и ветер
void GravityController::apply(World& w, real dt) {
    (void)dt;
    if (mode == GravityMode::Constant) { w.setGravity(constant); return; }
    w.setGravity(Vec2());
    for (RigidBody* b : w.bodies()) {
        if (!b || b->invMass <= 0.0) continue;
        Vec2 d   = center - b->position;
        real r   = std::max(minRadius, d.length());
        Vec2 dir = d / r;
        real mag = strength / std::pow(r, falloff);
        if (mode == GravityMode::Dispersive) dir = dir * -1.0;
        b->applyForce(dir * (mag * b->mass * b->gravityScale));

        if (spinRate != 0.0) {
            Vec2 rv = b->position - spinCenter;
            b->applyForce(rv * (-b->mass * spinRate * spinRate));
        }
    }
}

Vec2 WindField::at(const Vec2& p) const {
    Vec2 v = base;
    if (gustAmplitude != 0.0 && base.lengthSq() > 1e-12) {
        real ph = time * gustFrequency * 6.2831853;
        v += base.normalized() * (gustAmplitude * (0.5 * std::sin(ph) + 0.5 * std::sin(ph * 2.37 + 1.1)));
    }
    if (turbulence != 0.0) {
        real s = std::sin(p.x * 0.7 + time * 1.3) * std::cos(p.y * 0.9 - time * 0.7);
        real c = std::cos(p.x * 0.5 - time * 0.9) * std::sin(p.y * 0.6 + time * 1.1);
        v += Vec2(s, c) * turbulence;
    }
    return v;
}

void WindField::apply(World& w, real dt, real dragScale) {
    time += dt;
    const real rho = w.config().medium.airDensity;
    for (RigidBody* b : w.bodies()) {
        if (!b || b->invMass <= 0.0 || b->sleeping) continue;
        Vec2 flow = at(b->position) - b->velocity;
        real sp   = flow.length();
        if (sp < 1e-4) continue;
        Vec2 dir = flow / sp;

        if (b->worldVertices.size() >= 3) {
            const std::vector<Vec2>& vs = b->worldVertices;
            const size_t n = vs.size();
            for (size_t i = 0; i < n; ++i) {
                Vec2 p0 = vs[i], p1 = vs[(i + 1) % n];
                Vec2 e   = p1 - p0;
                real len = e.length();
                if (len < 1e-9) continue;
                Vec2 nrm(e.y / len, -e.x / len);
                real proj = dot(dir, nrm);
                if (proj <= 0.0) continue;
                real q = 0.5 * rho * sp * sp * len * proj * dragScale;
                b->applyForceAtPoint(nrm * q, (p0 + p1) * 0.5);
            }
        } else {
            real area = 2.0 * b->boundingRadius;
            b->applyForce(dir * (0.5 * rho * sp * sp * area * dragScale));
        }
    }
}

// ================================================================ интерполяция
void MotionInterpolator::capture(World& w) {
    for (RigidBody* b : w.bodies()) {
        if (!b) continue;
        Frame& f = m_frames[b->id];
        f.prevP = f.currP; f.prevA = f.currA;
        f.currP = b->position; f.currA = b->angle;
        f.vel   = b->velocity; f.omega = b->angularVelocity;
    }
}

RenderTransform MotionInterpolator::at(BodyId id, real alpha) const {
    RenderTransform out;
    auto it = m_frames.find(id);
    if (it == m_frames.end()) return out;
    const Frame& f = it->second;
    if (alpha <= 1.0) {
        out.position = f.prevP + (f.currP - f.prevP) * alpha;
        out.angle    = f.prevA + (f.currA - f.prevA) * alpha;
    } else {
        real extra   = alpha - 1.0;
        out.position = f.currP + f.vel * extra;
        out.angle    = f.currA + f.omega * extra;
    }
    return out;
}

// ================================================================ диагностика
size_t estimateMemory(const World& w, size_t fluidParticles) {
    size_t bytes = w.arenaBytes();
    bytes += w.bodyCount() * (sizeof(RigidBody) + 12 * sizeof(Vec2));
    bytes += w.manifolds().size() * sizeof(Manifold);
    bytes += w.constraintCount() * 256;
    bytes += fluidParticles * sizeof(FluidParticle);
    return bytes;
}

void Diagnostics::beginStep() { m_t0 = (long long)(nowSeconds() * 1e6); }

void Diagnostics::endStep(World& w, size_t fluidParticles) {
    real ms = (real)((long long)(nowSeconds() * 1e6) - m_t0) / 1000.0;
    if (ms < 0.0) ms = 0.0;

    m_stats.steps++;
    m_stats.stepMs    = ms;
    m_stats.maxStepMs = std::max(m_stats.maxStepMs, ms);
    m_sumMs += ms;
    m_stats.avgStepMs = m_sumMs / (real)m_stats.steps;
    m_stats.fps       = ms > 1e-6 ? 1000.0 / ms : 0.0;

    m_stats.bodies = w.bodyCount();
    size_t dyn = 0, sleep = 0;
    for (RigidBody* b : w.bodies()) {
        if (!b) continue;
        if (b->isDynamic()) ++dyn;
        if (b->sleeping)    ++sleep;
    }
    m_stats.dynamicBodies = dyn;
    m_stats.sleeping      = sleep;
    m_stats.contacts      = w.contactCount();
    size_t pts = 0;
    for (const Manifold& mf : w.manifolds()) pts += (size_t)mf.count;
    m_stats.contactPoints      = pts;
    m_stats.constraints        = w.constraintCount();
    m_stats.fluidParticles     = fluidParticles;
    m_stats.velocityIterations = w.config().solver.velocityIterations;
    m_stats.positionIterations = w.config().solver.positionIterations;

    real energy = w.totalEnergy();
    m_stats.energyDrift = m_stats.steps > 1 ? energy - m_prevEnergy : 0.0;
    m_prevEnergy        = energy;
    m_stats.energy      = energy;
    m_stats.memoryBytes = estimateMemory(w, fluidParticles);

    if (m_stepMs.size() < historyLimit) { m_stepMs.push_back(ms); m_energy.push_back(energy); }
}

std::string Diagnostics::line() const {
    char buf[360];
    std::snprintf(buf, sizeof(buf),
        "fps %6.1f  step %6.3f ms (avg %6.3f, max %6.3f)  bodies %zu (dyn %zu, sleep %zu)  "
        "contacts %zu/%zu  joints %zu  fluid %zu  iters %d/%d  E %.3f (dE %+.4f)  mem %.2f MB",
        (double)m_stats.fps, (double)m_stats.stepMs, (double)m_stats.avgStepMs, (double)m_stats.maxStepMs,
        m_stats.bodies, m_stats.dynamicBodies, m_stats.sleeping,
        m_stats.contacts, m_stats.contactPoints, m_stats.constraints, m_stats.fluidParticles,
        m_stats.velocityIterations, m_stats.positionIterations,
        (double)m_stats.energy, (double)m_stats.energyDrift,
        (double)m_stats.memoryBytes / (1024.0 * 1024.0));
    return std::string(buf);
}

void Diagnostics::print(std::FILE* out) const {
    std::fprintf(out ? out : stdout, "%s\n", line().c_str());
}

bool Diagnostics::writeJson(const std::string& path) const {
    std::ofstream f(path);
    if (!f) return false;
    f << "{\n  \"steps\": " << m_stats.steps
      << ",\n  \"avg_step_ms\": " << m_stats.avgStepMs
      << ",\n  \"max_step_ms\": " << m_stats.maxStepMs
      << ",\n  \"fps\": " << m_stats.fps
      << ",\n  \"bodies\": " << m_stats.bodies
      << ",\n  \"dynamic_bodies\": " << m_stats.dynamicBodies
      << ",\n  \"sleeping\": " << m_stats.sleeping
      << ",\n  \"contacts\": " << m_stats.contacts
      << ",\n  \"contact_points\": " << m_stats.contactPoints
      << ",\n  \"constraints\": " << m_stats.constraints
      << ",\n  \"fluid_particles\": " << m_stats.fluidParticles
      << ",\n  \"energy\": " << m_stats.energy
      << ",\n  \"energy_drift\": " << m_stats.energyDrift
      << ",\n  \"memory_bytes\": " << m_stats.memoryBytes
      << ",\n  \"step_ms\": [";
    for (size_t i = 0; i < m_stepMs.size(); ++i) f << (i ? "," : "") << m_stepMs[i];
    f << "],\n  \"energy_history\": [";
    for (size_t i = 0; i < m_energy.size(); ++i) f << (i ? "," : "") << m_energy[i];
    f << "]\n}\n";
    return true;
}

// ================================================================ пресеты и конфиг
void applyPreset(WorldConfig& cfg, Preset p) {
    switch (p) {
        case Preset::Accuracy:
            cfg.solver.velocityIterations = 24;
            cfg.solver.positionIterations = 18;
            cfg.solver.baumgarte          = 0.25;
            cfg.solver.allowedPenetration = 0.002;
            cfg.substeps                  = 8;
            cfg.maxSubsteps               = 24;
            break;
        case Preset::Speed:
            cfg.solver.velocityIterations = 6;
            cfg.solver.positionIterations = 3;
            cfg.solver.baumgarte          = 0.15;
            cfg.solver.allowedPenetration = 0.010;
            cfg.substeps                  = 1;
            cfg.maxSubsteps               = 4;
            break;
        case Preset::Balance:
        default:
            cfg.solver.velocityIterations = 12;
            cfg.solver.positionIterations = 8;
            cfg.solver.baumgarte          = 0.20;
            cfg.solver.allowedPenetration = 0.005;
            cfg.substeps                  = 4;
            cfg.maxSubsteps               = 16;
            break;
    }
}

void adaptSolver(World& w, int minIterations, int maxIterations) {
    const size_t bodies   = std::max<size_t>(1, w.awakeCount());
    const size_t contacts = w.contactCount();
    const real   load     = (real)contacts / (real)bodies;

    int iters = (int)std::lround((double)minIterations + (double)load * 4.0);
    iters = (int)clampr((real)iters, (real)minIterations, (real)maxIterations);

    WorldConfig& cfg = w.config();
    cfg.solver.velocityIterations = iters;
    cfg.solver.positionIterations = std::max(2, iters / 2);

    real vmax = 0.0;
    for (RigidBody* b : w.bodies())
        if (b && !b->sleeping) vmax = std::max(vmax, b->velocity.length());
    int subs = 1 + (int)((vmax * cfg.fixedTimeStep) / 0.15);
    cfg.substeps = std::min(cfg.maxSubsteps, std::max(1, subs));
}

bool loadConfigTxt(const std::string& path, WorldConfig& cfg) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq), v = line.substr(eq + 1);
        auto trim = [](std::string& s) {
            while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
            while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) s.pop_back();
        };
        trim(k); trim(v);
        real x = (real)std::atof(v.c_str());
        if      (k == "gravity_x")           cfg.gravity.x = x;
        else if (k == "gravity_y")           cfg.gravity.y = x;
        else if (k == "velocity_iterations") cfg.solver.velocityIterations = (int)x;
        else if (k == "position_iterations") cfg.solver.positionIterations = (int)x;
        else if (k == "baumgarte")           cfg.solver.baumgarte = x;
        else if (k == "allowed_penetration") cfg.solver.allowedPenetration = x;
        else if (k == "friction_load")       cfg.solver.frictionLoadFactor = x;
        else if (k == "stiction_speed")      cfg.solver.stictionSpeed = x;
        else if (k == "warm_start_damping")  cfg.solver.warmStartDamping = x;
        else if (k == "max_penetration")     cfg.solver.maxPenetration = x;
        else if (k == "max_contact_points")  cfg.solver.maxContactPoints = (int)x;
        else if (k == "substeps")            cfg.substeps = (int)x;
        else if (k == "max_substeps")        cfg.maxSubsteps = (int)x;
        else if (k == "fixed_time_step")     cfg.fixedTimeStep = x;
        else if (k == "grid_cell_size")      cfg.gridCellSize = x;
        else if (k == "aabb_margin")         cfg.aabbMargin = x;
        else if (k == "thread_count")        cfg.threadCount = (int)x;
        else if (k == "air_density")         cfg.medium.airDensity = x;
        else if (k == "fluid_density")       cfg.medium.fluidDensity = x;
        else if (k == "fluid_level")         cfg.medium.fluidLevel = x;
        else if (k == "wind_x")              cfg.medium.wind.x = x;
        else if (k == "wind_y")              cfg.medium.wind.y = x;
        else if (k == "preset") {
            if      (v == "accuracy") applyPreset(cfg, Preset::Accuracy);
            else if (v == "speed")    applyPreset(cfg, Preset::Speed);
            else if (v == "balance")  applyPreset(cfg, Preset::Balance);
        }
    }
    return true;
}

bool saveConfigTxt(const std::string& path, const WorldConfig& cfg) {
    std::ofstream f(path);
    if (!f) return false;
    f << "# phys2d config\n";
    f << "gravity_x = "           << cfg.gravity.x << "\n";
    f << "gravity_y = "           << cfg.gravity.y << "\n";
    f << "velocity_iterations = " << cfg.solver.velocityIterations << "\n";
    f << "position_iterations = " << cfg.solver.positionIterations << "\n";
    f << "baumgarte = "           << cfg.solver.baumgarte << "\n";
    f << "allowed_penetration = " << cfg.solver.allowedPenetration << "\n";
    f << "friction_load = "       << cfg.solver.frictionLoadFactor << "\n";
    f << "stiction_speed = "      << cfg.solver.stictionSpeed << "\n";
    f << "warm_start_damping = "  << cfg.solver.warmStartDamping << "\n";
    f << "max_penetration = "     << cfg.solver.maxPenetration << "\n";
    f << "max_contact_points = "  << cfg.solver.maxContactPoints << "\n";
    f << "substeps = "            << cfg.substeps << "\n";
    f << "max_substeps = "        << cfg.maxSubsteps << "\n";
    f << "fixed_time_step = "     << cfg.fixedTimeStep << "\n";
    f << "grid_cell_size = "      << cfg.gridCellSize << "\n";
    f << "aabb_margin = "         << cfg.aabbMargin << "\n";
    f << "thread_count = "        << cfg.threadCount << "\n";
    f << "air_density = "         << cfg.medium.airDensity << "\n";
    f << "fluid_density = "       << cfg.medium.fluidDensity << "\n";
    f << "fluid_level = "         << cfg.medium.fluidLevel << "\n";
    f << "wind_x = "              << cfg.medium.wind.x << "\n";
    f << "wind_y = "              << cfg.medium.wind.y << "\n";
    return true;
}

// ================================================================ стабильность
void StabilityGuard::snapshot(World& w) {
    m_snap.clear();
    m_snap.reserve(w.bodyCount());
    for (RigidBody* b : w.bodies()) {
        if (!b) continue;
        Snap s;
        s.id = b->id; s.p = b->position; s.v = b->velocity;
        s.a = b->angle; s.w = b->angularVelocity;
        m_snap.push_back(s);
    }
}

bool StabilityGuard::verify(World& w) {
    bool bad = false;
    std::vector<BodyId> doomed;
    for (RigidBody* b : w.bodies()) {
        if (!b) continue;
        const bool nan = !b->position.isFinite() || !b->velocity.isFinite() ||
                         !std::isfinite(b->angle) || !std::isfinite(b->angularVelocity);
        if (nan) { bad = true; continue; }

        // нормализация угла (wrap angle): без неё угол накапливает огромные значения
        if (std::fabs(b->angle) > 1e4) {
            b->angle = std::fmod(b->angle, (real)6.283185307179586);
            b->prevAngle = b->angle;
            b->updateVertices();
            b->updateAABB(w.config().aabbMargin);
        }

        real sp = b->velocity.length();
        if (maxSpeed > 0.0 && sp > maxSpeed) { b->velocity *= maxSpeed / sp; ++m_clamps; }
        if (maxAngularSpeed > 0.0 && std::fabs(b->angularVelocity) > maxAngularSpeed) {
            b->angularVelocity = clampr(b->angularVelocity, -maxAngularSpeed, maxAngularSpeed);
            ++m_clamps;
        }

        const bool outside = b->position.x < bounds.min.x || b->position.x > bounds.max.x ||
                             b->position.y < bounds.min.y || b->position.y > bounds.max.y;
        if (outside && b->isDynamic()) {
            if (deleteEscapers) { doomed.push_back(b->id); continue; }
            if (clampToBounds) {
                b->position.x = clampr(b->position.x, bounds.min.x, bounds.max.x);
                b->position.y = clampr(b->position.y, bounds.min.y, bounds.max.y);
                b->velocity  *= 0.2;
                b->updateVertices();
                b->updateAABB(w.config().aabbMargin);
                ++m_clamps;
            }
        }
    }
    for (BodyId id : doomed) { w.destroyBody(id); ++m_removed; }

    if (bad && rollbackOnNaN) {
        for (const Snap& s : m_snap) {
            RigidBody* b = w.body(s.id);
            if (!b) continue;
            b->position = s.p; b->velocity = s.v;
            b->angle = s.a;    b->angularVelocity = s.w;
            b->updateVertices();
            b->updateAABB(w.config().aabbMargin);
        }
        ++m_rollbacks;
    }
    return !bad;
}

// ================================================================ запросы
bool shapeCast(World& w, const Shape& s, const Transform& start, const Vec2& translation,
               ShapeCastHit& out, int steps) {
    steps = std::max(1, steps);
    Shape probe = s;
    probe.updateDerived();

    for (int i = 0; i <= steps; ++i) {
        real t = (real)i / (real)steps;
        Transform xf(start.p + translation * t, start.angle);
        AABB box = probe.computeAABB(xf, 0.0);
        for (RigidBody* b : w.bodies()) {
            if (!b) continue;
            if (!b->fatAABB.overlaps(box)) continue;
            GjkEpaResult r = gjkEpa(probe, xf, b->shape, b->transform());
            if (r.hit) {
                out.body   = b;
                out.t      = t;
                out.normal = r.normal;
                out.point  = r.witnessB;
                return true;
            }
        }
    }
    out.body = nullptr;
    out.t    = 1.0;
    return false;
}

std::vector<RigidBody*> overlapShape(World& w, const Shape& s, const Transform& xf) {
    std::vector<RigidBody*> out;
    Shape probe = s;
    probe.updateDerived();
    AABB box = probe.computeAABB(xf, 0.0);
    for (RigidBody* b : w.bodies()) {
        if (!b || !b->fatAABB.overlaps(box)) continue;
        if (gjkEpa(probe, xf, b->shape, b->transform()).hit) out.push_back(b);
    }
    return out;
}

std::vector<RigidBody*> pointQueryAll(World& w, const Vec2& p) {
    std::vector<RigidBody*> out;
    for (RigidBody* b : w.bodies()) {
        if (!b || !b->aabb.contains(p)) continue;
        if (b->shape.containsPoint(b->transform(), p)) out.push_back(b);
    }
    return out;
}

size_t sweptTunnelGuard(World& w, real skin) {
    size_t fixed = 0;
    for (RigidBody* b : w.bodies()) {
        if (!b || !b->isDynamic() || b->sleeping) continue;
        Vec2 delta = b->position - b->prevPosition;
        if (delta.length() < b->boundingRadius * 0.9) continue;

        RayHit hit;
        if (!rayCastClosest(w, b->prevPosition, b->position, hit)) continue;
        if (!hit.body || hit.body == b || hit.body->isDynamic()) continue;

        b->position = hit.point + hit.normal * (b->boundingRadius + skin);
        real vn = dot(b->velocity, hit.normal);
        if (vn < 0.0) b->velocity -= hit.normal * (vn * (1.0 + b->material.restitution));
        b->updateVertices();
        b->updateAABB(w.config().aabbMargin);
        ++fixed;
    }
    return fixed;
}

// ================================================================ геометрия
bool segmentIntersect(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2, Vec2& out) {
    Vec2 r = p2 - p1, s = q2 - q1;
    real denom = cross(r, s);
    if (std::fabs(denom) < 1e-12) return false;
    real t = cross(q1 - p1, s) / denom;
    real u = cross(q1 - p1, r) / denom;
    if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) return false;
    out = p1 + r * t;
    return true;
}

bool pointInPolygon(const std::vector<Vec2>& poly, const Vec2& p) {
    bool inside = false;
    const size_t n = poly.size();
    if (n < 3) return false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[j];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-30) + a.x))
            inside = !inside;
    }
    return inside;
}

Vec2 projectOnSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    real len2 = ab.lengthSq();
    if (len2 < 1e-18) return a;
    return a + ab * clamp01(dot(p - a, ab) / len2);
}

std::vector<Vec2> removeCollinear(std::vector<Vec2> pts, real tol) {
    if (pts.size() < 3) return pts;
    std::vector<Vec2> out;
    const size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const Vec2& prev = pts[(i + n - 1) % n];
        const Vec2& cur  = pts[i];
        const Vec2& next = pts[(i + 1) % n];
        if (distanceSq(cur, prev) < tol * tol) continue;
        if (std::fabs(cross(cur - prev, next - cur)) < tol) continue;
        out.push_back(cur);
    }
    return out.size() >= 3 ? out : pts;
}

std::vector<Vec2> sortCounterClockwise(std::vector<Vec2> pts) {
    if (pts.size() < 3) return pts;
    Vec2 c;
    for (const Vec2& p : pts) c += p;
    c /= (real)pts.size();
    std::sort(pts.begin(), pts.end(), [&](const Vec2& a, const Vec2& b) {
        return std::atan2(a.y - c.y, a.x - c.x) < std::atan2(b.y - c.y, b.x - c.x);
    });
    return pts;
}

bool isConvex(const std::vector<Vec2>& pts) {
    const size_t n = pts.size();
    if (n < 3) return false;
    int sign = 0;
    for (size_t i = 0; i < n; ++i) {
        Vec2 a = pts[(i + 1) % n] - pts[i];
        Vec2 b = pts[(i + 2) % n] - pts[(i + 1) % n];
        real z = cross(a, b);
        if (std::fabs(z) < 1e-12) continue;
        int s = z > 0.0 ? 1 : -1;
        if (sign == 0) sign = s;
        else if (s != sign) return false;
    }
    return sign != 0;
}

Shape repairShape(Shape s) {
    if (s.vertices.size() >= 3) {
        std::vector<Vec2> v = removeCollinear(sortCounterClockwise(s.vertices));
        if (!isConvex(v)) v = convexHull(v);
        if (v.size() >= 3) s = Shape::polygon(v);
        else               s = Shape::circle(std::max((real)0.02, s.boundingRadius));
    }
    s.updateDerived();
    return s;
}

Shape randomConvexPolygon(uint32_t& seed, int vertices, real radius) {
    auto rnd = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return (real)((seed >> 8) & 0xFFFFFF) / (real)0xFFFFFF;
    };
    vertices = std::max(3, std::min(16, vertices));
    std::vector<Vec2> pts;
    pts.reserve((size_t)vertices);
    for (int i = 0; i < vertices; ++i) {
        real ang = 6.2831853 * ((real)i + rnd() * 0.7) / (real)vertices;
        real r   = radius * (0.55 + 0.45 * rnd());
        pts.push_back(Vec2(std::cos(ang) * r, std::sin(ang) * r));
    }
    return repairShape(Shape::polygon(convexHull(pts)));
}

void reshapeBody(World& w, BodyId id, const Shape& s) {
    RigidBody* b = w.body(id);
    if (!b) return;
    b->shape = repairShape(s);
    b->computeMassFromShape();
    b->updateVertices();
    b->updateAABB(w.config().aabbMargin);
    b->wake();
}

// ================================================================ нагрев от трения
void FrictionHeat::install(World& w) {
    FrictionHeat* self = this;
    w.setPersistContactCallback([self](const CollisionEvent& e) {
        if (!e.a || !e.b) return;
        real q = std::fabs(e.tangentImpulse) * std::fabs(e.relativeSpeed);
        self->m_pending[e.a->id] += q;
        self->m_pending[e.b->id] += q;
    });
}

void FrictionHeat::update(real dt) {
    for (auto& kv : m_pending) {
        auto res = m_temp.emplace(kv.first, ambient);
        res.first->second += kv.second * heatRate;
        kv.second = 0.0;
    }
    for (auto& kv : m_temp) kv.second -= (kv.second - ambient) * coolRate * dt;
}

real FrictionHeat::temperature(BodyId id) const {
    auto it = m_temp.find(id);
    return it == m_temp.end() ? ambient : it->second;
}

real FrictionHeat::hottest() const {
    real best = ambient;
    for (const auto& kv : m_temp) best = std::max(best, kv.second);
    return best;
}

// ================================================================ сервис
void DirtyTracker::scan(World& w, real epsilon) {
    m_list.clear();
    m_set.clear();
    for (RigidBody* b : w.bodies()) {
        if (!b) continue;
        auto it = m_last.find(b->id);
        if (it == m_last.end() || distanceSq(it->second, b->position) > epsilon * epsilon) {
            m_last[b->id] = b->position;
            markDirty(b->id);
        }
    }
}

void DirtyTracker::markDirty(BodyId id) {
    if (m_set.count(id)) return;
    m_set[id] = 1;
    m_list.push_back(id);
}

bool DirtyTracker::dirty(BodyId id) const { return m_set.count(id) != 0; }
void DirtyTracker::clear() { m_list.clear(); m_set.clear(); }

void SceneKeeper::tick(World& w) {
    ++m_steps;
    if (intervalSteps <= 0) return;
    if (++m_counter < intervalSteps) return;
    m_counter = 0;

    std::ofstream f(path);
    if (!f) return;
    f << "{\n  \"step\": " << m_steps << ",\n  \"bodies\": [\n";
    bool first = true;
    for (RigidBody* b : w.bodies()) {
        if (!b) continue;
        if (!first) f << ",\n";
        first = false;
        f << "    {\"id\": " << b->id
          << ", \"type\": " << (int)b->type
          << ", \"x\": "  << b->position.x << ", \"y\": " << b->position.y
          << ", \"a\": "  << b->angle
          << ", \"vx\": " << b->velocity.x << ", \"vy\": " << b->velocity.y
          << ", \"w\": "  << b->angularVelocity
          << ", \"m\": "  << b->mass << "}";
    }
    f << "\n  ]\n}\n";
    ++m_saves;
}

size_t removeOrphanStatics(World& w) {
    std::vector<Constraint*> cs = w.allConstraints();
    std::unordered_map<const RigidBody*, int> linked;
    for (Constraint* c : cs) {
        if (!c) continue;
        if (c->bodyA) linked[c->bodyA] = 1;
        if (c->bodyB) linked[c->bodyB] = 1;
    }
    std::vector<BodyId> doomed;
    for (RigidBody* b : w.bodies()) {
        if (!b || !b->isStatic()) continue;
        if (linked.count(b)) continue;
        bool touched = false;
        for (const Manifold& mf : w.manifolds())
            if (mf.bodyA == b || mf.bodyB == b) { touched = true; break; }
        if (!touched) doomed.push_back(b->id);
    }
    for (BodyId id : doomed) w.destroyBody(id);
    return doomed.size();
}

namespace {
std::mutex g_memMutex;
size_t g_live = 0, g_peak = 0, g_allocs = 0;
}

void* MemoryTracker::alloc(size_t bytes) {
    void* p = std::calloc(1, bytes);
    if (!p) return nullptr;
    std::lock_guard<std::mutex> lock(g_memMutex);
    g_live += bytes;
    if (g_live > g_peak) g_peak = g_live;
    ++g_allocs;
    return p;
}

void MemoryTracker::release(void* p, size_t bytes) {
    if (!p) return;
    {
        std::lock_guard<std::mutex> lock(g_memMutex);
        g_live = bytes > g_live ? 0 : g_live - bytes;
    }
    std::free(p);
}

size_t MemoryTracker::liveBytes()   { std::lock_guard<std::mutex> l(g_memMutex); return g_live; }
size_t MemoryTracker::peakBytes()   { std::lock_guard<std::mutex> l(g_memMutex); return g_peak; }
size_t MemoryTracker::allocations() { std::lock_guard<std::mutex> l(g_memMutex); return g_allocs; }

std::string MemoryTracker::report() {
    char buf[192];
    std::snprintf(buf, sizeof(buf), "memory: live %.3f MB, peak %.3f MB, allocations %zu",
                  (double)liveBytes() / (1024.0 * 1024.0),
                  (double)peakBytes() / (1024.0 * 1024.0), allocations());
    return std::string(buf);
}

PluginHost::~PluginHost() { unloadAll(); }

bool PluginHost::load(const std::string& path) {
#if defined(_WIN32)
    HMODULE h = LoadLibraryA(path.c_str());
    if (!h) { m_error = "LoadLibrary failed: " + path; return false; }
    m_handles.push_back((void*)h);
#else
    void* h = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!h) { const char* e = dlerror(); m_error = e ? e : "dlopen failed"; return false; }
    m_handles.push_back(h);
#endif
    m_names.push_back(path);
    return true;
}

void PluginHost::unloadAll() {
    for (void* h : m_handles) {
#if defined(_WIN32)
        FreeLibrary((HMODULE)h);
#else
        dlclose(h);
#endif
    }
    m_handles.clear();
    m_names.clear();
}

} // namespace phys2d
