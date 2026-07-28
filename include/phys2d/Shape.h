// phys2d — формы: окружность, прямоугольник, выпуклый многоугольник, капсула.
#pragma once

#include "Vec2.h"
#include <vector>
#include <cstdint>

namespace phys2d {

enum class ShapeType : uint8_t { Circle = 0, Polygon = 1, Capsule = 2 };

struct MassData {
    real mass    = 0.0;
    real inertia = 0.0;   // относительно центра масс
    Vec2 center;          // центр масс в локальных координатах
    real area    = 0.0;
};

// Единая форма: выпуклое ядро (точка / отрезок / многоугольник) + радиус скругления.
//   Circle  : ядро = 1 точка,  radius > 0
//   Capsule : ядро = отрезок [-halfLength..+halfLength] по X, radius > 0
//   Polygon : ядро = выпуклая оболочка вершин, radius = 0
struct Shape {
    ShapeType         type       = ShapeType::Circle;
    real              radius     = 0.5;   // радиус окружности / капсулы
    real              halfLength = 0.0;   // половина длины капсулы
    real              width = 0.0, height = 0.0;   // для прямоугольника (справочно / сериализация)
    bool              isBox      = false;

    std::vector<Vec2> vertices;   // ЛОКАЛЬНЫЕ вершины (CCW)
    std::vector<Vec2> normals;    // нормали рёбер (локальные)

    Vec2  centroid;               // центроид формы
    real  boundingRadius = 0.5;   // радиус описанной окружности

    // ---- фабрики
    static Shape circle(real r);
    static Shape box(real w, real h);
    static Shape polygon(std::vector<Vec2> pts);   // произвольные точки -> выпуклая оболочка
    static Shape capsule(real r, real len);
    static Shape regularPolygon(int n, real r, real phase = 0.0);
    static Shape gear(int teeth, real innerR, real outerR);

    void     updateDerived();                                  // нормали, центроид, радиус
    MassData computeMass(real density) const;                  // интегрирование по площади
    AABB     computeAABB(const Transform& xf, real margin = 0.0) const;
    Vec2     supportLocal(const Vec2& dirLocal) const;         // опорная точка ядра (без radius)
    Vec2     supportWorld(const Transform& xf, const Vec2& dirWorld) const;
    bool     containsPoint(const Transform& xf, const Vec2& world) const;

    // Концы осевого отрезка капсулы в мировых координатах.
    void     capsuleSegment(const Transform& xf, Vec2& a, Vec2& b) const;
};

// Геометрические утилиты, общие для узкой фазы.
Vec2 closestPointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b, real* tOut = nullptr);
real segmentSegmentDistance(const Vec2& p1, const Vec2& q1,
                            const Vec2& p2, const Vec2& q2,
                            Vec2& c1, Vec2& c2);
std::vector<Vec2> convexHull(std::vector<Vec2> pts);

} // namespace phys2d
