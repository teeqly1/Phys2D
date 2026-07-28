// phys2d — узкая фаза: SAT, GJK, EPA, манифолды, время столкновения.
#pragma once

#include "Body.h"
#include <unordered_map>
#include <cstdint>

namespace phys2d {

constexpr int MAX_MANIFOLD_POINTS = 2;

// Точка контакта с накопленными импульсами (accumulated impulses / warm starting).
struct ContactPoint {
    Vec2 point;                    // точка контакта (мировые координаты)
    real penetration    = 0.0;     // глубина проникновения
    Vec2 rA, rB;                   // плечи контакта относительно центров масс
    real normalImpulse  = 0.0;     // накопленный нормальный импульс
    real tangentImpulse = 0.0;     // накопленный тангенциальный импульс
    real normalMass     = 0.0;     // 1 / (эффективная масса по нормали)
    real tangentMass    = 0.0;
    real velocityBias   = 0.0;     // restitution bias
    real positionBias   = 0.0;     // Baumgarte bias
    real relVelNormal   = 0.0;     // относительная скорость в точке контакта (нормаль)
    uint32_t featureId  = 0;       // идентификатор фичи для сопоставления между шагами
};

struct Manifold {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    Vec2  normal;                              // нормаль контакта (от A к B)
    int   count = 0;
    ContactPoint points[MAX_MANIFOLD_POINTS];
    real  friction    = 0.0;                   // смешанное трение (текущее)
    real  staticFriction = 0.0, dynamicFriction = 0.0;
    real  restitution = 0.0;
    real  toi         = 1.0;                   // время столкновения в долях шага [0..1]
    Mat22 K;                                   // матрица масс контактного пространства (2x2, две точки)
    uint64_t key      = 0;                     // ключ пары (для warm starting / событий)
    bool  touching    = false;
    int   satAxis     = -1;                    // победившая ось SAT (для кэша и дебага)
    Vec2  satAxisDir;
};

inline uint64_t makePairKey(BodyId a, BodyId b) {
    return (a < b) ? ((uint64_t)a << 32) | b : ((uint64_t)b << 32) | a;
}

// -------------------------------------------------------------- GJK / EPA
struct GjkEpaResult {
    bool hit       = false;
    real distance  = 0.0;    // >0 если формы разделены
    real depth     = 0.0;    // глубина проникновения (EPA)
    Vec2 normal;             // нормаль от A к B
    Vec2 witnessA, witnessB; // ближайшие точки
    int  gjkIterations = 0;
    int  epaIterations = 0;
};

// GJK по ядрам форм + EPA при пересечении; радиусы скругления учитываются.
GjkEpaResult gjkEpa(const Shape& sa, const Transform& xa,
                    const Shape& sb, const Transform& xb);

// -------------------------------------------------------------- Кэш осей SAT
class SatAxisCache {
public:
    void   store(uint64_t key, int axisIndex, bool fromB);
    bool   lookup(uint64_t key, int& axisIndex, bool& fromB) const;
    void   clear() { m_cache.clear(); }
    size_t size() const { return m_cache.size(); }
private:
    struct Entry { int axis; bool fromB; };
    std::unordered_map<uint64_t, Entry> m_cache;
};

// -------------------------------------------------------------- Детекторы
bool collideCircleCircle   (Manifold& m, RigidBody& a, RigidBody& b);
bool collideCirclePolygon  (Manifold& m, RigidBody& circle, RigidBody& poly);   // GJK/EPA
bool collidePolygonPolygon (Manifold& m, RigidBody& a, RigidBody& b, SatAxisCache* cache);
bool collideCircleCapsule  (Manifold& m, RigidBody& circle, RigidBody& capsule);
bool collidePolygonCapsule (Manifold& m, RigidBody& poly, RigidBody& capsule);
bool collideCapsuleCapsule (Manifold& m, RigidBody& a, RigidBody& b);

// Главный диспетчер: выбирает алгоритм по паре типов форм.
bool collide(Manifold& m, RigidBody& a, RigidBody& b, SatAxisCache* cache = nullptr);

// Бинарный поиск времени контакта в интервале [0..dt]; возвращает долю шага.
real timeOfImpact(const RigidBody& a, const RigidBody& b, real dt, int iterations = 24);

} // namespace phys2d
