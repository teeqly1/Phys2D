#include "phys2d/Constraints.h"

namespace phys2d {

// ---------------------------------------------------------------- base
void Constraint::applyPair(const Vec2& impulse, const Vec2& pa, const Vec2& pb) {
    if (bodyA && bodyA->isDynamic()) {
        bodyA->velocity        -= impulse * bodyA->invMass;
        bodyA->angularVelocity -= bodyA->invInertia * cross(pa - bodyA->position, impulse);
        bodyA->wake();
    }
    if (bodyB && bodyB->isDynamic()) {
        bodyB->velocity        += impulse * bodyB->invMass;
        bodyB->angularVelocity += bodyB->invInertia * cross(pb - bodyB->position, impulse);
        bodyB->wake();
    }
}

Vec2 Constraint::relativeVelocity(const Vec2& pa, const Vec2& pb) const {
    Vec2 va, vb;
    if (bodyA) va = bodyA->velocityAtPoint(pa);
    if (bodyB) vb = bodyB->velocityAtPoint(pb);
    return vb - va;
}

real Constraint::effectiveMass(const Vec2& n, const Vec2& pa, const Vec2& pb) const {
    real k = 0.0;
    if (bodyA && bodyA->isDynamic()) {
        const Vec2 ra = pa - bodyA->position;
        k += bodyA->invMass + bodyA->invInertia * sqr(cross(ra, n));
    }
    if (bodyB && bodyB->isDynamic()) {
        const Vec2 rb = pb - bodyB->position;
        k += bodyB->invMass + bodyB->invInertia * sqr(cross(rb, n));
    }
    return (k > EPSILON) ? 1.0 / k : 0.0;
}

// ---------------------------------------------------------------- жёсткая связь
void DistanceConstraint::prepare(real) {
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    Vec2 d = pb - pa;
    const real len = d.normalize();
    m_axis  = (len > EPSILON) ? d : Vec2(0.0, 1.0);
    m_error = len - restLength;
    m_mass  = effectiveMass(m_axis, pa, pb);
}

void DistanceConstraint::solveVelocity(real) {
    if (!enabled || broken) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const real vn = dot(relativeVelocity(pa, pb), m_axis);
    const real lambda = -m_mass * vn;
    impulseAccum += lambda;                       // accumulated impulse
    if (breakForce > 0.0 && std::fabs(impulseAccum) > breakForce) { broken = true; return; }
    applyPair(m_axis * lambda, pa, pb);
}

void DistanceConstraint::solvePosition(real, real schlagerFactor) {
    if (!enabled || broken) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    Vec2 d = pb - pa;
    const real len = d.normalize();
    if (len < EPSILON) return;
    const real err = len - restLength;
    // Метод Шлагера: прямая проекция позиций с весами по обратным массам.
    const real wa = (bodyA && bodyA->isDynamic()) ? bodyA->invMass : 0.0;
    const real wb = (bodyB && bodyB->isDynamic()) ? bodyB->invMass : 0.0;
    const real wsum = wa + wb;
    if (wsum < EPSILON) return;
    const Vec2 corr = d * (schlagerFactor * err / wsum);
    if (bodyA && bodyA->isDynamic()) { bodyA->position += corr * wa; bodyA->updateVertices(); }
    if (bodyB && bodyB->isDynamic()) { bodyB->position -= corr * wb; bodyB->updateVertices(); }
}

// ---------------------------------------------------------------- шарнир
void RevoluteConstraint::prepare(real) {
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const real ima = (bodyA && bodyA->isDynamic()) ? bodyA->invMass : 0.0;
    const real imb = (bodyB && bodyB->isDynamic()) ? bodyB->invMass : 0.0;
    const real iia = (bodyA && bodyA->isDynamic()) ? bodyA->invInertia : 0.0;
    const real iib = (bodyB && bodyB->isDynamic()) ? bodyB->invInertia : 0.0;
    const Vec2 ra = bodyA ? pa - bodyA->position : Vec2();
    const Vec2 rb = bodyB ? pb - bodyB->position : Vec2();

    // Матрица эффективных масс 2x2
    m_K = Mat22(ima + imb + iia * ra.y * ra.y + iib * rb.y * rb.y,
                       -iia * ra.x * ra.y - iib * rb.x * rb.y,
                       -iia * ra.x * ra.y - iib * rb.x * rb.y,
                ima + imb + iia * ra.x * ra.x + iib * rb.x * rb.x);
    m_impulse = Vec2();
}

void RevoluteConstraint::solveVelocity(real) {
    if (!enabled || broken) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const Vec2 dv = relativeVelocity(pa, pb);
    const Vec2 lambda = m_K.solve(-dv);
    m_impulse += lambda;
    impulseAccum = m_impulse.length();
    if (breakForce > 0.0 && impulseAccum > breakForce) { broken = true; return; }
    applyPair(lambda, pa, pb);
}

void RevoluteConstraint::solvePosition(real, real schlagerFactor) {
    if (!enabled || broken) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const Vec2 err = pb - pa;
    const real wa = (bodyA && bodyA->isDynamic()) ? bodyA->invMass : 0.0;
    const real wb = (bodyB && bodyB->isDynamic()) ? bodyB->invMass : 0.0;
    const real wsum = wa + wb;
    if (wsum < EPSILON) return;
    const Vec2 corr = err * (schlagerFactor / wsum);
    if (bodyA && bodyA->isDynamic()) { bodyA->position += corr * wa; bodyA->updateVertices(); }
    if (bodyB && bodyB->isDynamic()) { bodyB->position -= corr * wb; bodyB->updateVertices(); }
}

// ---------------------------------------------------------------- пружина
void SpringConstraint::prepare(real) {}

void SpringConstraint::solveVelocity(real dt) {
    if (!enabled || broken) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    Vec2 d = pb - pa;
    const real len = d.normalize();
    if (len < EPSILON) return;

    const real x  = len - restLength;
    const real vn = dot(relativeVelocity(pa, pb), d);
    const real force = -stiffness * x - damping * vn;    // гармоническая пружина с демпфированием
    const real lambda = force * dt;
    impulseAccum = lambda;
    if (breakForce > 0.0 && std::fabs(force) > breakForce) { broken = true; return; }
    applyPair(d * lambda, pa, pb);
}

// ---------------------------------------------------------------- трос
void RopeConstraint::prepare(real) {
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    Vec2 d = pb - pa;
    const real len = d.normalize();
    m_axis   = (len > EPSILON) ? d : Vec2(0.0, 1.0);
    m_error  = len - maxLength;
    m_active = m_error > 0.0;                 // трос работает только на растяжение
    m_mass   = effectiveMass(m_axis, pa, pb);
}

void RopeConstraint::solveVelocity(real) {
    if (!enabled || broken || !m_active) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    const real vn = dot(relativeVelocity(pa, pb), m_axis);
    if (vn < 0.0) return;                     // сближаются — трос свободен
    const real lambda = -m_mass * vn;
    impulseAccum += lambda;
    applyPair(m_axis * lambda, pa, pb);
}

void RopeConstraint::solvePosition(real, real schlagerFactor) {
    if (!enabled || broken) return;
    const Vec2 pa = anchorAWorld(), pb = anchorBWorld();
    Vec2 d = pb - pa;
    const real len = d.normalize();
    const real err = len - maxLength;
    if (err <= 0.0 || len < EPSILON) return;
    const real wa = (bodyA && bodyA->isDynamic()) ? bodyA->invMass : 0.0;
    const real wb = (bodyB && bodyB->isDynamic()) ? bodyB->invMass : 0.0;
    const real wsum = wa + wb;
    if (wsum < EPSILON) return;
    const Vec2 corr = d * (schlagerFactor * err / wsum);
    if (bodyA && bodyA->isDynamic()) { bodyA->position += corr * wa; bodyA->updateVertices(); }
    if (bodyB && bodyB->isDynamic()) { bodyB->position -= corr * wb; bodyB->updateVertices(); }
}

// ---------------------------------------------------------------- угловое ограничение
void AngularConstraint::prepare(real) {
    const real iia = (bodyA && bodyA->isDynamic()) ? bodyA->invInertia : 0.0;
    const real iib = (bodyB && bodyB->isDynamic()) ? bodyB->invInertia : 0.0;
    const real k = iia + iib;
    m_mass = (k > EPSILON) ? 1.0 / k : 0.0;

    const real angleA = bodyA ? bodyA->angle : 0.0;
    const real angleB = bodyB ? bodyB->angle : 0.0;
    const real rel = angleB - angleA - referenceAngle;

    if (rel < minAngle)      { m_limitState = -1; m_error = rel - minAngle; }
    else if (rel > maxAngle) { m_limitState =  1; m_error = rel - maxAngle; }
    else                     { m_limitState =  0; m_error = 0.0; }
}

void AngularConstraint::solveVelocity(real dt) {
    if (!enabled || broken) return;
    const real wa = bodyA ? bodyA->angularVelocity : 0.0;
    const real wb = bodyB ? bodyB->angularVelocity : 0.0;

    // Мотор
    if (maxMotorTorque > 0.0) {
        real lambda = m_mass * (motorSpeed - (wb - wa));
        const real maxImp = maxMotorTorque * dt;
        lambda = clampr(lambda, -maxImp, maxImp);
        if (bodyA && bodyA->isDynamic()) bodyA->angularVelocity -= bodyA->invInertia * lambda;
        if (bodyB && bodyB->isDynamic()) bodyB->angularVelocity += bodyB->invInertia * lambda;
    }

    if (m_limitState == 0) return;
    const real relW = (bodyB ? bodyB->angularVelocity : 0.0) - (bodyA ? bodyA->angularVelocity : 0.0);
    real lambda = -m_mass * relW;
    if ((m_limitState < 0 && lambda < 0.0) || (m_limitState > 0 && lambda > 0.0)) lambda = 0.0;
    impulseAccum += lambda;
    if (bodyA && bodyA->isDynamic()) bodyA->angularVelocity -= bodyA->invInertia * lambda;
    if (bodyB && bodyB->isDynamic()) bodyB->angularVelocity += bodyB->invInertia * lambda;
}

void AngularConstraint::solvePosition(real, real schlagerFactor) {
    if (!enabled || broken || m_limitState == 0) return;
    const real iia = (bodyA && bodyA->isDynamic()) ? bodyA->invInertia : 0.0;
    const real iib = (bodyB && bodyB->isDynamic()) ? bodyB->invInertia : 0.0;
    const real k = iia + iib;
    if (k < EPSILON) return;
    const real corr = schlagerFactor * m_error / k;
    if (bodyA && bodyA->isDynamic()) { bodyA->angle += iia * corr; bodyA->updateVertices(); }
    if (bodyB && bodyB->isDynamic()) { bodyB->angle -= iib * corr; bodyB->updateVertices(); }
}

} // namespace phys2d
