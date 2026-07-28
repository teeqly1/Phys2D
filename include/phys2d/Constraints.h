// phys2d — ограничения (joints).
#pragma once

#include "Body.h"
#include <memory>

namespace phys2d {

enum class ConstraintType : uint8_t {
    Distance = 0,   // жёсткая связь с заданным расстоянием
    Revolute = 1,   // шарнирное соединение
    Spring   = 2,   // пружина (жёсткость + демпфирование)
    Rope     = 3,   // трос с максимальной длиной
    Angular  = 4,
    Weld      = 5,
    Motor     = 6,
    Prismatic = 7,
    Pulley    = 8,
    Gear      = 9,
    Catenary  = 10
};

// Базовое ограничение. Скоростное решение + позиционное (метод Шлагера,
// stabilization by direct position projection с масштабированием по обратным массам).
class Constraint {
public:
    virtual ~Constraint() = default;

    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;      // nullptr => привязка к миру (anchorB — мировая точка)
    Vec2       localAnchorA, localAnchorB;
    Vec2       worldAnchorB;         // используется когда bodyB == nullptr
    bool       enabled  = true;
    bool       collideConnected = false;
    real       impulseAccum = 0.0;   // accumulated impulse
    real       breakForce = 0.0;     // 0 => неразрушаемое
    bool       broken = false;

    virtual ConstraintType type() const = 0;
    virtual void prepare(real dt) = 0;
    virtual void solveVelocity(real dt) = 0;
    virtual void solvePosition(real dt, real schlagerFactor) {(void)dt; (void)schlagerFactor;}

    Vec2 anchorAWorld() const { return bodyA ? bodyA->transform().apply(localAnchorA) : localAnchorA; }
    Vec2 anchorBWorld() const { return bodyB ? bodyB->transform().apply(localAnchorB) : worldAnchorB; }

protected:
    void applyPair(const Vec2& impulse, const Vec2& pa, const Vec2& pb);
    Vec2 relativeVelocity(const Vec2& pa, const Vec2& pb) const;
    real effectiveMass(const Vec2& n, const Vec2& pa, const Vec2& pb) const;
};

// Жёсткая связь: |pa - pb| == restLength
class DistanceConstraint : public Constraint {
public:
    real restLength = 1.0;
    ConstraintType type() const override { return ConstraintType::Distance; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    void solvePosition(real dt, real schlagerFactor) override;
private:
    Vec2 m_axis; real m_mass = 0.0, m_error = 0.0;
};

// Шарнир: pa == pb (2 DOF)
class RevoluteConstraint : public Constraint {
public:
    ConstraintType type() const override { return ConstraintType::Revolute; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    void solvePosition(real dt, real schlagerFactor) override;
private:
    Mat22 m_K;
    Vec2  m_impulse;
};

// Пружина: F = -k*(|d| - L0) - c*v
class SpringConstraint : public Constraint {
public:
    real restLength = 1.0;
    real stiffness  = 200.0;   // жёсткость
    real damping    = 5.0;     // демпфирование
    ConstraintType type() const override { return ConstraintType::Spring; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
};

// Трос: |pa - pb| <= maxLength (одностороннее ограничение)
class RopeConstraint : public Constraint {
public:
    real maxLength = 2.0;
    ConstraintType type() const override { return ConstraintType::Rope; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    void solvePosition(real dt, real schlagerFactor) override;
private:
    Vec2 m_axis; real m_mass = 0.0, m_error = 0.0; bool m_active = false;
};

// Угловое ограничение: minAngle <= (angleB - angleA) <= maxAngle
class AngularConstraint : public Constraint {
public:
    real minAngle = -PI * 0.25;
    real maxAngle =  PI * 0.25;
    real referenceAngle = 0.0;
    real motorSpeed = 0.0;
    real maxMotorTorque = 0.0;
    ConstraintType type() const override { return ConstraintType::Angular; }
    void prepare(real dt) override;
    void solveVelocity(real dt) override;
    void solvePosition(real dt, real schlagerFactor) override;
private:
    real m_mass = 0.0, m_error = 0.0;
    int  m_limitState = 0;   // -1 нижний, +1 верхний, 0 свободно
};

using ConstraintPtr = std::unique_ptr<Constraint>;

} // namespace phys2d
