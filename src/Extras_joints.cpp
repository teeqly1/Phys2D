// phys2d v2 - joints: weld, motor, prismatic, pulley, gear, catenary rope, breakable.
#include "phys2d/Extras.h"

#include <algorithm>
#include <cmath>

namespace phys2d {
namespace {

inline real invMassOf(const RigidBody* b) { return b ? b->invMass : 0.0; }
inline real invInertiaOf(const RigidBody* b) { return b ? b->invInertia : 0.0; }
inline Vec2 velocityOf(const RigidBody* b, const Vec2& p) { return b ? b->velocityAtPoint(p) : Vec2(); }
inline real omegaOf(const RigidBody* b) { return b ? b->angularVelocity : 0.0; }

// soft-constraint коэффициенты: частота + демпфирование -> gamma и bias.
void softCoefficients(const SoftParams& s, real mass, real C, real dt, real& gamma, real& bias) {
    gamma = 0.0;
    bias = 0.0;
    if (s.frequencyHz <= 0.0 || mass <= 0.0 || dt <= 0.0) return;

    const real omega = 2.0 * PI * s.frequencyHz;
    const real d = 2.0 * mass * s.dampingRatio * omega;   // демпфирование
    const real k = mass * omega * omega;                  // жёсткость
    gamma = dt * (d + dt * k);
    gamma = (gamma != 0.0) ? 1.0 / gamma : 0.0;
    bias = C * dt * k * gamma;
}

void applyLinear(RigidBody* a, RigidBody* b, const Vec2& impulse, const Vec2& pa, const Vec2& pb) {
    if (a && a->invMass > 0.0) {
        a->velocity -= impulse * a->invMass;
        a->angularVelocity -= a->invInertia * cross(pa - a->position, impulse);
    }
    if (b && b->invMass > 0.0) {
        b->velocity += impulse * b->invMass;
        b->angularVelocity += b->invInertia * cross(pb - b->position, impulse);
    }
}

} // namespace

// ================================================================ weld
void WeldJoint::prepare(real dt) {
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const Vec2 ra = pa - (bodyA ? bodyA->position : pa);
    const Vec2 rb = pb - (bodyB ? bodyB->position : pb);

    const real ma = invMassOf(bodyA), mb = invMassOf(bodyB);
    const real ia = invInertiaOf(bodyA), ib = invInertiaOf(bodyB);

    m_K.m00 = ma + mb + ra.y * ra.y * ia + rb.y * rb.y * ib;
    m_K.m01 = -ra.y * ra.x * ia - rb.y * rb.x * ib;
    m_K.m10 = m_K.m01;
    m_K.m11 = ma + mb + ra.x * ra.x * ia + rb.x * rb.x * ib;

    const real angMassInv = ia + ib;
    m_angMass = (angMassInv > 0.0) ? 1.0 / angMassInv : 0.0;

    const real angleA = bodyA ? bodyA->angle : 0.0;
    const real angleB = bodyB ? bodyB->angle : 0.0;
    const real C = angleB - angleA - referenceAngle;
    softCoefficients(soft, m_angMass, C, dt, m_gamma, m_bias);

    // gamma обязан входить в эффективную массу, иначе при малом dt решение расходится.
    const real angDenom = angMassInv + m_gamma;
    m_angMass = (angDenom > 0.0) ? 1.0 / angDenom : 0.0;

    m_linImpulse = Vec2();
    m_angImpulse = 0.0;
}

void WeldJoint::solveVelocity(real) {
    // угловая часть
    {
        const real Cdot = omegaOf(bodyB) - omegaOf(bodyA);
        const real lambda = -m_angMass * (Cdot + m_bias + m_gamma * m_angImpulse);
        m_angImpulse += lambda;
        if (bodyA) bodyA->angularVelocity -= invInertiaOf(bodyA) * lambda;
        if (bodyB) bodyB->angularVelocity += invInertiaOf(bodyB) * lambda;
    }
    // линейная часть: точки совпадают
    {
        const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
        const Vec2 Cdot = velocityOf(bodyB, pb) - velocityOf(bodyA, pa);
        const Vec2 lambda = m_K.solve(Cdot * -1.0);
        m_linImpulse += lambda;
        impulseAccum = m_linImpulse.length();
        applyLinear(bodyA, bodyB, lambda, pa, pb);
    }
}

void WeldJoint::solvePosition(real, real schlagerFactor) {
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const Vec2 C = pb - pa;
    const real ma = invMassOf(bodyA), mb = invMassOf(bodyB);
    const real total = ma + mb;
    if (total <= 0.0) return;

    const Vec2 correction = C * (schlagerFactor / total);
    if (bodyA && ma > 0.0) { bodyA->position += correction * ma; bodyA->updateVertices(); }
    if (bodyB && mb > 0.0) { bodyB->position -= correction * mb; bodyB->updateVertices(); }

    if (soft.frequencyHz <= 0.0 && bodyA && bodyB) {
        const real angleError = bodyB->angle - bodyA->angle - referenceAngle;
        const real ia = bodyA->invInertia, ib = bodyB->invInertia;
        const real sum = ia + ib;
        if (sum > 0.0) {
            bodyA->angle += angleError * schlagerFactor * (ia / sum);
            bodyB->angle -= angleError * schlagerFactor * (ib / sum);
            bodyA->updateVertices();
            bodyB->updateVertices();
        }
    }
}

// ================================================================ motor
void MotorJoint::prepare(real) {
    if (linear) {
        const real m = invMassOf(bodyA) + invMassOf(bodyB);
        m_mass = (m > 0.0) ? 1.0 / m : 0.0;
    } else {
        const real i = invInertiaOf(bodyA) + invInertiaOf(bodyB);
        m_mass = (i > 0.0) ? 1.0 / i : 0.0;
    }
    m_impulse = 0.0;
}

void MotorJoint::solveVelocity(real dt) {
    if (dt <= 0.0 || m_mass <= 0.0) return;
    const real maxImpulse = maxTorque * dt;

    if (linear) {
        const Vec2 axisW = (bodyA ? axis.rotated(bodyA->angle) : axis).normalized();
        const real Cdot = dot((bodyB ? bodyB->velocity : Vec2()) - (bodyA ? bodyA->velocity : Vec2()), axisW);
        real lambda = m_mass * (targetSpeed - Cdot);
        const real old = m_impulse;
        m_impulse = clampr(old + lambda, -maxImpulse, maxImpulse);
        lambda = m_impulse - old;
        const Vec2 impulse = axisW * lambda;
        if (bodyA) bodyA->velocity -= impulse * bodyA->invMass;
        if (bodyB) bodyB->velocity += impulse * bodyB->invMass;
    } else {
        const real Cdot = omegaOf(bodyB) - omegaOf(bodyA);
        real lambda = m_mass * (targetSpeed - Cdot);
        const real old = m_impulse;
        m_impulse = clampr(old + lambda, -maxImpulse, maxImpulse);
        lambda = m_impulse - old;
        if (bodyA) bodyA->angularVelocity -= invInertiaOf(bodyA) * lambda;
        if (bodyB) bodyB->angularVelocity += invInertiaOf(bodyB) * lambda;
    }
    impulseAccum = std::fabs(m_impulse);
}

// ================================================================ prismatic
void PrismaticJoint::prepare(real dt) {
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const Vec2 d = pb - pa;

    m_axis = (bodyA ? localAxisA.rotated(bodyA->angle) : localAxisA).normalized();
    m_perp = m_axis.perp();

    const Vec2 ra = pa - (bodyA ? bodyA->position : pa);
    const Vec2 rb = pb - (bodyB ? bodyB->position : pb);

    m_s1 = cross(d + ra, m_perp);
    m_s2 = cross(rb, m_perp);
    m_a1 = cross(d + ra, m_axis);
    m_a2 = cross(rb, m_axis);

    const real ma = invMassOf(bodyA), mb = invMassOf(bodyB);
    const real ia = invInertiaOf(bodyA), ib = invInertiaOf(bodyB);

    const real kPerp = ma + mb + ia * m_s1 * m_s1 + ib * m_s2 * m_s2;
    m_perpMass = (kPerp > 0.0) ? 1.0 / kPerp : 0.0;

    const real kAng = ia + ib;
    m_angMass = (kAng > 0.0) ? 1.0 / kAng : 0.0;

    const real kAxial = ma + mb + ia * m_a1 * m_a1 + ib * m_a2 * m_a2;
    m_axialMass = (kAxial > 0.0) ? 1.0 / kAxial : 0.0;

    m_translation = dot(d, m_axis);

    m_limitState = 0;
    if (enableLimit) {
        if (m_translation <= lowerTranslation) m_limitState = -1;
        else if (m_translation >= upperTranslation) m_limitState = 1;
    }

    m_perpImpulse = 0.0;
    m_angImpulse = 0.0;
    m_limitImpulse = 0.0;
    (void)dt;
}

void PrismaticJoint::solveVelocity(real dt) {
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();

    // мотор вдоль оси
    if (enableMotor && dt > 0.0 && m_axialMass > 0.0) {
        const Vec2 dv = velocityOf(bodyB, pb) - velocityOf(bodyA, pa);
        const real Cdot = dot(dv, m_axis);
        real lambda = m_axialMass * (motorSpeed - Cdot);
        const real maxImpulse = maxMotorForce * dt;
        const real old = m_motorImpulse;
        m_motorImpulse = clampr(old + lambda, -maxImpulse, maxImpulse);
        lambda = m_motorImpulse - old;
        applyLinear(bodyA, bodyB, m_axis * lambda, pa, pb);
    }

    // ограничение хода (жёсткое либо мягкое)
    if (m_limitState != 0 && m_axialMass > 0.0) {
        const Vec2 dv = velocityOf(bodyB, pb) - velocityOf(bodyA, pa);
        const real Cdot = dot(dv, m_axis);
        const real C = (m_limitState < 0) ? (m_translation - lowerTranslation)
                                          : (m_translation - upperTranslation);
        real gamma = 0.0, bias = 0.0;
        softCoefficients(softLimit, m_axialMass, C, dt, gamma, bias);

        real lambda = -m_axialMass * (Cdot + bias + gamma * m_limitImpulse);
        const real old = m_limitImpulse;
        m_limitImpulse = (m_limitState < 0) ? std::max(old + lambda, 0.0)
                                            : std::min(old + lambda, 0.0);
        lambda = m_limitImpulse - old;
        applyLinear(bodyA, bodyB, m_axis * lambda, pa, pb);
    }

    // поперечное и угловое ограничение
    {
        const Vec2 dv = velocityOf(bodyB, pb) - velocityOf(bodyA, pa);
        const real Cdot = dot(dv, m_perp);
        const real lambda = -m_perpMass * Cdot;
        m_perpImpulse += lambda;
        applyLinear(bodyA, bodyB, m_perp * lambda, pa, pb);
    }
    {
        const real Cdot = omegaOf(bodyB) - omegaOf(bodyA);
        const real lambda = -m_angMass * Cdot;
        m_angImpulse += lambda;
        if (bodyA) bodyA->angularVelocity -= invInertiaOf(bodyA) * lambda;
        if (bodyB) bodyB->angularVelocity += invInertiaOf(bodyB) * lambda;
    }
    impulseAccum = std::fabs(m_perpImpulse) + std::fabs(m_limitImpulse);
}

void PrismaticJoint::solvePosition(real, real schlagerFactor) {
    if (!bodyA && !bodyB) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const Vec2 d = pb - pa;
    const real perpError = dot(d, m_perp);

    const real ma = invMassOf(bodyA), mb = invMassOf(bodyB);
    const real total = ma + mb;
    if (total <= 0.0) return;

    const Vec2 correction = m_perp * (perpError * schlagerFactor / total);
    if (bodyA && ma > 0.0) { bodyA->position += correction * ma; bodyA->updateVertices(); }
    if (bodyB && mb > 0.0) { bodyB->position -= correction * mb; bodyB->updateVertices(); }

    if (enableLimit && softLimit.frequencyHz <= 0.0) {
        const real t = dot(d, m_axis);
        real violation = 0.0;
        if (t < lowerTranslation) violation = t - lowerTranslation;
        else if (t > upperTranslation) violation = t - upperTranslation;
        if (violation != 0.0) {
            const Vec2 fix = m_axis * (violation * schlagerFactor / total);
            if (bodyA && ma > 0.0) { bodyA->position += fix * ma; bodyA->updateVertices(); }
            if (bodyB && mb > 0.0) { bodyB->position -= fix * mb; bodyB->updateVertices(); }
        }
    }
}

// ================================================================ pulley
void PulleyJoint::prepare(real) {
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    Vec2 uA = pa - groundA;
    Vec2 uB = pb - groundB;
    const real lenA = uA.length(), lenB = uB.length();

    m_uA = (lenA > 1e-9) ? uA / lenA : Vec2();
    m_uB = (lenB > 1e-9) ? uB / lenB : Vec2();
    m_C = totalLength - lenA - ratio * lenB;

    const Vec2 ra = pa - (bodyA ? bodyA->position : pa);
    const Vec2 rb = pb - (bodyB ? bodyB->position : pb);
    const real ruA = cross(ra, m_uA);
    const real ruB = cross(rb, m_uB);

    const real mA = invMassOf(bodyA) + invInertiaOf(bodyA) * ruA * ruA;
    const real mB = invMassOf(bodyB) + invInertiaOf(bodyB) * ruB * ruB;
    const real k = mA + ratio * ratio * mB;
    m_mass = (k > 0.0) ? 1.0 / k : 0.0;
    m_impulse = 0.0;
}

void PulleyJoint::solveVelocity(real) {
    if (m_mass <= 0.0) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const Vec2 vA = velocityOf(bodyA, pa);
    const Vec2 vB = velocityOf(bodyB, pb);

    const real Cdot = -dot(m_uA, vA) - ratio * dot(m_uB, vB);
    const real lambda = -m_mass * Cdot;
    m_impulse += lambda;
    impulseAccum = std::fabs(m_impulse);

    const Vec2 impulseA = m_uA * -lambda;
    const Vec2 impulseB = m_uB * (-ratio * lambda);
    if (bodyA && bodyA->invMass > 0.0) {
        bodyA->velocity += impulseA * bodyA->invMass;
        bodyA->angularVelocity += bodyA->invInertia * cross(pa - bodyA->position, impulseA);
    }
    if (bodyB && bodyB->invMass > 0.0) {
        bodyB->velocity += impulseB * bodyB->invMass;
        bodyB->angularVelocity += bodyB->invInertia * cross(pb - bodyB->position, impulseB);
    }
}

void PulleyJoint::solvePosition(real, real schlagerFactor) {
    if (m_mass <= 0.0 || std::fabs(m_C) < 1e-6) return;
    const real ma = invMassOf(bodyA), mb = invMassOf(bodyB);
    const real total = ma + ratio * mb;
    if (total <= 0.0) return;

    const real correction = m_C * schlagerFactor / total;
    if (bodyA && ma > 0.0) { bodyA->position += m_uA * (correction * ma); bodyA->updateVertices(); }
    if (bodyB && mb > 0.0) { bodyB->position += m_uB * (correction * ratio * mb); bodyB->updateVertices(); }
}

// ================================================================ gear
void GearJoint::prepare(real) {
    const real i = invInertiaOf(bodyA) + ratio * ratio * invInertiaOf(bodyB);
    m_mass = (i > 0.0) ? 1.0 / i : 0.0;
    const real angleA = bodyA ? bodyA->angle : 0.0;
    const real angleB = bodyB ? bodyB->angle : 0.0;
    m_C = angleA + ratio * angleB;
    m_impulse = 0.0;
}

void GearJoint::solveVelocity(real) {
    if (m_mass <= 0.0) return;
    const real Cdot = omegaOf(bodyA) + ratio * omegaOf(bodyB);
    const real lambda = -m_mass * Cdot;
    m_impulse += lambda;
    impulseAccum = std::fabs(m_impulse);
    if (bodyA) bodyA->angularVelocity += invInertiaOf(bodyA) * lambda;
    if (bodyB) bodyB->angularVelocity += invInertiaOf(bodyB) * lambda * ratio;
}

void GearJoint::solvePosition(real, real schlagerFactor) {
    if (m_mass <= 0.0) return;
    const real ia = invInertiaOf(bodyA), ib = invInertiaOf(bodyB);
    const real sum = ia + ratio * ratio * ib;
    if (sum <= 0.0) return;

    const real angleA = bodyA ? bodyA->angle : 0.0;
    const real angleB = bodyB ? bodyB->angle : 0.0;
    const real C = angleA + ratio * angleB - m_C;
    const real correction = -C * schlagerFactor / sum;
    if (bodyA && ia > 0.0) { bodyA->angle += correction * ia; bodyA->updateVertices(); }
    if (bodyB && ib > 0.0) { bodyB->angle += correction * ratio * ib; bodyB->updateVertices(); }
}

// ================================================================ catenary rope
std::vector<Vec2> catenaryPoints(const Vec2& a, const Vec2& b, real ropeLength, int samples) {
    std::vector<Vec2> out;
    const int n = std::max(2, samples);
    out.reserve((size_t)n);

    const real dx = b.x - a.x;
    const real dy = b.y - a.y;
    const real span = std::sqrt(dx * dx + dy * dy);

    // Натянутая верёвка — прямая линия.
    if (ropeLength <= span * 1.0001 || span < 1e-9) {
        for (int i = 0; i < n; ++i) out.push_back(Vec2::lerp(a, b, (real)i / (real)(n - 1)));
        return out;
    }

    // Подбор параметра a цепной линии: 2a*sinh(h/(2a)) = sqrt(L^2 - v^2)
    const real h = std::fabs(dx);
    const real target = std::sqrt(std::max(0.0, ropeLength * ropeLength - dy * dy));
    real lo = 1e-4, hi = std::max(1.0, h * 10.0);
    for (int i = 0; i < 80; ++i) {
        const real mid = 0.5 * (lo + hi);
        const real value = 2.0 * mid * std::sinh(h / (2.0 * mid));
        if (value > target) lo = mid; else hi = mid;
    }
    const real p = 0.5 * (lo + hi);

    for (int i = 0; i < n; ++i) {
        const real t = (real)i / (real)(n - 1);
        const real x = a.x + dx * t;
        // провисание относительно хорды
        const real u = (t - 0.5) * h;
        const real sag = p * (std::cosh(u / p) - std::cosh(h / (2.0 * p)));
        const real y = a.y + dy * t + sag;
        out.push_back(Vec2(x, y));
    }
    return out;
}

RopeChain buildCatenaryRope(World& w, const Vec2& a, const Vec2& b, real ropeLength,
                            int segments, real thickness, real density) {
    RopeChain chain;
    const int n = std::max(2, segments);
    const std::vector<Vec2> pts = catenaryPoints(a, b, ropeLength, n + 1);

    BodyId prev = INVALID_BODY;
    for (int i = 0; i < n; ++i) {
        const Vec2 p0 = pts[(size_t)i], p1 = pts[(size_t)i + 1];
        const Vec2 mid = (p0 + p1) * 0.5;
        const Vec2 d = p1 - p0;
        const real len = std::max((real)1e-3, d.length());

        BodyDef def;
        def.type = BodyType::Dynamic;
        def.shape = Shape::capsule(thickness, len);
        def.position = mid;
        def.angle = std::atan2(d.y, d.x);
        def.allowSleep = false;
        def.material.density = density;
        def.material.restitution = 0.05;
        def.name = "rope_link";
        const BodyId id = w.createBody(def);
        chain.links.push_back(id);

        if (prev != INVALID_BODY) {
            chain.joints.push_back(w.createRevolute(prev, id, Vec2(len * 0.5, 0.0), Vec2(-len * 0.5, 0.0)));
        }
        prev = id;
    }
    return chain;
}

// ================================================================ breakable joints
void BreakableJointSystem::watch(Constraint* c, real breakForce) {
    if (!c) return;
    c->breakForce = breakForce;
    m_watch.emplace_back(c, breakForce);
}

void BreakableJointSystem::update(real dt) {
    if (dt <= 0.0) return;
    for (auto& entry : m_watch) {
        Constraint* c = entry.first;
        if (!c || c->broken || entry.second <= 0.0) continue;
        const real force = std::fabs(c->impulseAccum) / dt;
        if (force > entry.second) {
            c->broken = true;
            c->enabled = false;
            ++m_broken;
            if (m_cb) m_cb(JointBreakEvent{c, force, entry.second});
        }
    }
}

// ================================================================ factories
WeldJoint* createWeld(World& w, BodyId a, BodyId b, const Vec2& worldAnchor, const SoftParams& s) {
    RigidBody* ba = w.body(a);
    RigidBody* bb = w.body(b);
    if (!ba || !bb) { PHYS2D_ERROR(ErrorCode::NotFound, "createWeld: body missing"); return nullptr; }

    auto joint = std::make_unique<WeldJoint>();
    joint->bodyA = ba;
    joint->bodyB = bb;
    joint->localAnchorA = ba->transform().invApply(worldAnchor);
    joint->localAnchorB = bb->transform().invApply(worldAnchor);
    joint->worldAnchorB = worldAnchor;
    joint->referenceAngle = bb->angle - ba->angle;
    joint->soft = s;
    return static_cast<WeldJoint*>(w.addConstraint(std::move(joint)));
}

MotorJoint* createMotor(World& w, BodyId a, BodyId b, real targetSpeed, real maxTorque) {
    RigidBody* ba = w.body(a);
    RigidBody* bb = w.body(b);
    if (!ba || !bb) { PHYS2D_ERROR(ErrorCode::NotFound, "createMotor: body missing"); return nullptr; }

    auto joint = std::make_unique<MotorJoint>();
    joint->bodyA = ba;
    joint->bodyB = bb;
    joint->targetSpeed = targetSpeed;
    joint->maxTorque = maxTorque;
    return static_cast<MotorJoint*>(w.addConstraint(std::move(joint)));
}

PrismaticJoint* createPrismatic(World& w, BodyId a, BodyId b, const Vec2& worldAnchor, const Vec2& worldAxis) {
    RigidBody* ba = w.body(a);
    RigidBody* bb = w.body(b);
    if (!ba || !bb) { PHYS2D_ERROR(ErrorCode::NotFound, "createPrismatic: body missing"); return nullptr; }

    auto joint = std::make_unique<PrismaticJoint>();
    joint->bodyA = ba;
    joint->bodyB = bb;
    joint->localAnchorA = ba->transform().invApply(worldAnchor);
    joint->localAnchorB = bb->transform().invApply(worldAnchor);
    joint->worldAnchorB = worldAnchor;
    joint->localAxisA = worldAxis.normalized().rotated(-ba->angle);
    joint->referenceAngle = bb->angle - ba->angle;
    return static_cast<PrismaticJoint*>(w.addConstraint(std::move(joint)));
}

PulleyJoint* createPulley(World& w, BodyId a, BodyId b, const Vec2& groundA, const Vec2& groundB,
                          const Vec2& localA, const Vec2& localB, real ratio) {
    RigidBody* ba = w.body(a);
    RigidBody* bb = w.body(b);
    if (!ba || !bb) { PHYS2D_ERROR(ErrorCode::NotFound, "createPulley: body missing"); return nullptr; }

    auto joint = std::make_unique<PulleyJoint>();
    joint->bodyA = ba;
    joint->bodyB = bb;
    joint->localAnchorA = localA;
    joint->localAnchorB = localB;
    joint->groundA = groundA;
    joint->groundB = groundB;
    joint->ratio = ratio;
    joint->totalLength = (ba->transform().apply(localA) - groundA).length() +
                         ratio * (bb->transform().apply(localB) - groundB).length();
    return static_cast<PulleyJoint*>(w.addConstraint(std::move(joint)));
}

GearJoint* createGear(World& w, BodyId a, BodyId b, real ratio) {
    RigidBody* ba = w.body(a);
    RigidBody* bb = w.body(b);
    if (!ba || !bb) { PHYS2D_ERROR(ErrorCode::NotFound, "createGear: body missing"); return nullptr; }

    auto joint = std::make_unique<GearJoint>();
    joint->bodyA = ba;
    joint->bodyB = bb;
    joint->ratio = ratio;
    return static_cast<GearJoint*>(w.addConstraint(std::move(joint)));
}

} // namespace phys2d
