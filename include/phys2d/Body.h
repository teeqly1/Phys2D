// phys2d — твёрдое тело.
#pragma once

#include "Shape.h"
#include <string>
#include <cstdint>

namespace phys2d {

using BodyId = uint32_t;
constexpr BodyId INVALID_BODY = 0xFFFFFFFFu;

enum class BodyType : uint8_t { Static = 0, Dynamic = 1, Kinematic = 2 };

struct Material {
    real density            = 1.0;
    real restitution        = 0.35;   // коэффициент восстановления
    real staticFriction     = 0.60;   // трение Кулона, статическое
    real dynamicFriction    = 0.40;   // трение Кулона, динамическое
    real rollingFriction    = 0.02;   // трение качения (окружности/капсулы)
    real angularFriction    = 0.01;   // угловое трение среды
    real linearDrag         = 0.02;   // линейное сопротивление воздуха  (F = -k*v)
    real quadraticDrag      = 0.01;   // квадратичное сопротивление       (F = -k*|v|*v)
    real liftCoefficient    = 0.0;    // аэродинамика: подъёмная сила
    real softness           = 0.0;    // 0 = абсолютно твёрдое, >0 = мягкое тело
    real hysteresis         = 0.15;   // гистерезис упругого столкновения (потери энергии)
};

struct BodyDef {
    BodyType  type = BodyType::Dynamic;
    Shape     shape = Shape::circle(0.5);
    Material  material{};
    Vec2      position;
    Vec2      velocity;
    real      angle           = 0.0;
    real      angularVelocity = 0.0;
    bool      fixedRotation   = false;
    bool      allowSleep      = true;
    real      gravityScale    = 1.0;
    void*     userData        = nullptr;
    std::string name;
};

class RigidBody {
public:
    // ------- идентификация
    BodyId      id   = INVALID_BODY;
    BodyType    type = BodyType::Dynamic;
    std::string name;
    void*       userData = nullptr;

    // ------- геометрия
    Shape             shape;
    std::vector<Vec2> localVertices;    // локальные вершины
    std::vector<Vec2> worldVertices;    // глобальные вершины (обновляются каждый шаг)
    Vec2              centerOfMass;     // центр масс (мировые координаты)
    Vec2              localCenter;      // центр масс (локальные)
    AABB              aabb;             // текущий AABB
    AABB              fatAABB;          // AABB с запасом (broad phase)
    real              boundingRadius = 0.5;  // радиус описанной окружности

    // ------- кинематика
    Vec2 position;
    Vec2 prevPosition;                  // для интегратора Верлета
    Vec2 velocity;
    Vec2 acceleration;
    real angle           = 0.0;
    real angularVelocity = 0.0;
    real angularAccel    = 0.0;
    real prevAngle       = 0.0;

    // ------- масса и инерция
    real mass = 1.0, invMass = 1.0;
    real inertia = 1.0, invInertia = 1.0;
    bool fixedRotation = false;

    // ------- материал
    Material material{};
    real gravityScale = 1.0;

    // ------- аккумуляторы сил
    Vec2 force;
    real torque = 0.0;

    // ------- сон / пробуждение
    bool sleeping   = false;
    bool allowSleep = true;
    real sleepTimer = 0.0;

    // ------- мягкое тело (упругая деформация)
    std::vector<Vec2> restVertices;     // недеформированная форма
    std::vector<Vec2> vertexVelocity;   // скорости узлов оболочки

    // ------- траектория (дебаг)
    std::vector<Vec2> trail;

    // ---------------------------------------------------------------- API
    void  setMassData(real m, real I);
    void  computeMassFromShape();
    void  setType(BodyType t);

    bool  isDynamic()   const { return type == BodyType::Dynamic; }
    bool  isStatic()    const { return type == BodyType::Static; }
    bool  isKinematic() const { return type == BodyType::Kinematic; }
    bool  isAwake()     const { return !sleeping; }

    Transform transform() const { return Transform(position, angle); }

    void applyForce(const Vec2& f)                          { force += f; wake(); }
    void applyForceAtPoint(const Vec2& f, const Vec2& p)    { force += f; torque += cross(p - position, f); wake(); }
    void applyTorque(real t)                                { torque += t; wake(); }
    void applyImpulse(const Vec2& imp)                      { velocity += imp * invMass; wake(); }
    void applyImpulseAtPoint(const Vec2& imp, const Vec2& p) {
        velocity        += imp * invMass;
        angularVelocity += invInertia * cross(p - position, imp);
        wake();
    }
    void applyAngularImpulse(real imp) { angularVelocity += invInertia * imp; wake(); }

    // Относительная скорость тела в мировой точке p.
    Vec2 velocityAtPoint(const Vec2& p) const { return velocity + cross(angularVelocity, p - position); }

    void wake()  { if (type != BodyType::Static) { sleeping = false; sleepTimer = 0.0; } }
    void sleep() { sleeping = true; velocity = Vec2(); angularVelocity = 0.0; }

    void clearForces() { force = Vec2(); torque = 0.0; }

    void updateVertices();     // локальные -> глобальные + центр масс
    void updateAABB(real margin = 0.0);
    real kineticEnergy() const {
        return 0.5 * mass * velocity.lengthSq() + 0.5 * inertia * sqr(angularVelocity);
    }
};

} // namespace phys2d
