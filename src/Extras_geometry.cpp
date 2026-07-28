// phys2d v2 - CCD, raycasts, decomposition, curves, terrain, BSP, BVH, import/export.
#include "phys2d/Extras.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace phys2d {
namespace {

inline bool solveQuadratic(real a, real b, real c, real& t0, real& t1) {
    if (std::fabs(a) < 1e-12) {
        if (std::fabs(b) < 1e-12) return false;
        t0 = t1 = -c / b;
        return true;
    }
    const real disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return false;
    const real sq = std::sqrt(disc);
    t0 = (-b - sq) / (2.0 * a);
    t1 = (-b + sq) / (2.0 * a);
    if (t0 > t1) std::swap(t0, t1);
    return true;
}

bool rayCircle(const Vec2& p1, const Vec2& p2, const Vec2& center, real radius, real& t, Vec2& normal) {
    const Vec2 d = p2 - p1;
    const Vec2 m = p1 - center;
    real t0 = 0.0, t1 = 0.0;
    if (!solveQuadratic(d.lengthSq(), 2.0 * dot(m, d), m.lengthSq() - radius * radius, t0, t1)) return false;
    const real hit = (t0 >= 0.0) ? t0 : t1;
    if (hit < 0.0 || hit > 1.0) return false;
    t = hit;
    normal = (p1 + d * hit - center).normalized();
    return true;
}

bool rayPolygon(const Vec2& p1, const Vec2& p2, const std::vector<Vec2>& verts, real& t, Vec2& normal) {
    if (verts.size() < 2) return false;
    real best = 2.0;
    Vec2 bestNormal;
    bool found = false;

    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % verts.size()];
        const Vec2 r = p2 - p1;
        const Vec2 s = b - a;
        const real denom = cross(r, s);
        if (std::fabs(denom) < 1e-12) continue;
        const real tt = cross(a - p1, s) / denom;
        const real uu = cross(a - p1, r) / denom;
        if (tt >= 0.0 && tt <= 1.0 && uu >= 0.0 && uu <= 1.0 && tt < best) {
            best = tt;
            bestNormal = s.perp().normalized();
            if (dot(bestNormal, r) > 0.0) bestNormal = bestNormal * -1.0;
            found = true;
        }
    }
    if (!found) return false;
    t = best;
    normal = bestNormal;
    return true;
}

real triangleArea(const Vec2& a, const Vec2& b, const Vec2& c) {
    return 0.5 * cross(b - a, c - a);
}

bool pointInTriangle(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c) {
    const real d1 = cross(b - a, p - a);
    const real d2 = cross(c - b, p - b);
    const real d3 = cross(a - c, p - c);
    const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}

} // namespace

// ================================================================ swept tests
bool sweptCircleSegment(const Vec2& c0, const Vec2& c1, real r,
                        const Vec2& a, const Vec2& b, real& t, Vec2& normal) {
    const Vec2 motion = c1 - c0;
    const Vec2 seg = b - a;
    const real segLenSq = seg.lengthSq();
    if (segLenSq < 1e-12) {
        // вырождение: окружность против точки
        return rayCircle(c0, c1, a, r, t, normal);
    }

    // 1. Корпус отрезка: смещённые линии на расстояние r.
    const Vec2 n = seg.perp().normalized();
    const real side = dot(c0 - a, n);
    const Vec2 offsetNormal = (side >= 0.0) ? n : n * -1.0;
    const real denom = dot(motion, offsetNormal);
    real bestT = 2.0;
    Vec2 bestNormal;
    bool found = false;

    if (denom < -1e-12) {
        const real dist = std::fabs(side) - r;
        const real tt = dist / -denom;
        if (tt >= 0.0 && tt <= 1.0) {
            const Vec2 hit = c0 + motion * tt;
            const real proj = dot(hit - a, seg) / segLenSq;
            if (proj >= 0.0 && proj <= 1.0) {
                bestT = tt;
                bestNormal = offsetNormal;
                found = true;
            }
        }
    }

    // 2. Скруглённые концы.
    for (const Vec2& endpoint : {a, b}) {
        real tt = 0.0;
        Vec2 nn;
        if (rayCircle(c0, c1, endpoint, r, tt, nn) && tt < bestT) {
            bestT = tt;
            bestNormal = nn;
            found = true;
        }
    }

    if (!found) return false;
    t = bestT;
    normal = bestNormal;
    return true;
}

bool sweptCircleAABB(const Vec2& c0, const Vec2& c1, real r, const AABB& box, real& t, Vec2& normal) {
    const AABB expanded = box.expanded(r);
    const Vec2 d = c1 - c0;

    real tmin = 0.0, tmax = 1.0;
    for (int axis = 0; axis < 2; ++axis) {
        const real origin = (axis == 0) ? c0.x : c0.y;
        const real delta = (axis == 0) ? d.x : d.y;
        const real lo = (axis == 0) ? expanded.min.x : expanded.min.y;
        const real hi = (axis == 0) ? expanded.max.x : expanded.max.y;

        if (std::fabs(delta) < 1e-12) {
            if (origin < lo || origin > hi) return false;
            continue;
        }
        real t1 = (lo - origin) / delta;
        real t2 = (hi - origin) / delta;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }

    t = tmin;
    const Vec2 hit = c0 + d * tmin;
    const Vec2 c = box.center();
    const Vec2 e = box.extents();
    const Vec2 local = hit - c;
    normal = (std::fabs(local.x) / std::max((real)1e-9, e.x) > std::fabs(local.y) / std::max((real)1e-9, e.y))
                 ? Vec2(local.x > 0.0 ? 1.0 : -1.0, 0.0)
                 : Vec2(0.0, local.y > 0.0 ? 1.0 : -1.0);
    return true;
}

SweepResult conservativeAdvancement(const RigidBody& a, const RigidBody& b, real dt,
                                    int maxIterations, real tolerance) {
    SweepResult result;

    // Быстрый отсев по раздвинутым AABB.
    const Vec2 motion = (a.velocity - b.velocity) * dt;
    AABB boxA = a.aabb.expanded(motion.length());
    if (!boxA.overlaps(b.aabb)) return result;

    // Основной поиск: бинарный TOI ядра (GJK внутри) — консервативное продвижение.
    RigidBody& ma = const_cast<RigidBody&>(a);
    RigidBody& mb = const_cast<RigidBody&>(b);
    const real toi = timeOfImpact(ma, mb, dt, std::max(4, maxIterations));
    result.iterations = maxIterations;

    if (toi >= 1.0 - tolerance) return result;

    result.hit = true;
    result.t = clampr(toi, 0.0, 1.0);
    result.body = &mb;

    Manifold mf;
    if (collide(mf, ma, mb, nullptr) && mf.count > 0) {
        result.point = mf.points[0].point;
        result.normal = mf.normal;
    } else {
        result.normal = (b.position - a.position).normalized();
        result.point = a.position + result.normal * a.boundingRadius;
    }
    return result;
}

void CcdSystem::update(real dt) {
    if (!m_world || dt <= 0.0) return;
    m_resolved = 0;

    for (RigidBody* b : m_world->bodies()) {
        if (!b->isDynamic() || b->sleeping) continue;

        const Vec2 motion = b->position - b->prevPosition;
        const real travel = motion.length();
        const real size = std::max((real)1e-3, b->boundingRadius);
        if (travel < size * 0.9 && travel < speedThreshold * dt) continue;

        // Проверяем траекторию против тел по пути.
        AABB sweepBox;
        sweepBox.reset();
        sweepBox.add(b->prevPosition);
        sweepBox.add(b->position);
        sweepBox = sweepBox.expanded(size);

        RayHit best;
        best.fraction = 2.0;
        for (RigidBody* other : m_world->queryAABB(sweepBox)) {
            if (other == b) continue;
            if (other->isDynamic() && !other->sleeping) continue;   // обрабатываем против статики/спящих

            RayHit hit;
            if (!rayCastShape(*other, b->prevPosition, b->position, hit)) continue;
            if (hit.fraction < best.fraction) best = hit;
        }

        if (best.fraction <= 1.0 && best.body) {
            // Откат к моменту касания с небольшим зазором.
            const real safe = std::max((real)0.0, best.fraction - 0.02);
            b->position = b->prevPosition + motion * safe + best.normal * (size * 0.05);
            const real vn = dot(b->velocity, best.normal);
            if (vn < 0.0) {
                const real restitution = b->material.restitution * 0.5;
                b->velocity -= best.normal * (vn * (1.0 + restitution));
            }
            b->updateVertices();
            b->updateAABB();
            ++m_resolved;
        }
    }
}

// ================================================================ raycasts
bool rayCastShape(const RigidBody& b, const Vec2& p1, const Vec2& p2, RayHit& out) {
    real t = 0.0;
    Vec2 normal;
    bool hit = false;

    if (b.shape.type == ShapeType::Circle) {
        hit = rayCircle(p1, p2, b.position, b.shape.radius, t, normal);
    } else if (b.shape.type == ShapeType::Polygon) {
        hit = rayPolygon(p1, p2, b.worldVertices, t, normal);
    } else { // capsule
        Vec2 a, c;
        b.shape.capsuleSegment(b.transform(), a, c);
        // Луч против капсулы == swept окружность нулевого радиуса против отрезка радиуса R.
        hit = sweptCircleSegment(p1, p2, b.shape.radius, a, c, t, normal);
    }
    if (!hit) return false;

    out.body = const_cast<RigidBody*>(&b);
    out.fraction = t;
    out.point = p1 + (p2 - p1) * t;
    out.normal = normal;
    return true;
}

int rayCastAll(World& w, const Vec2& p1, const Vec2& p2, std::vector<RayHit>& out,
               const FilterRegistry* filters, uint32_t mask) {
    out.clear();

    AABB box;
    box.reset();
    box.add(p1);
    box.add(p2);

    for (RigidBody* b : w.queryAABB(box.expanded(0.05))) {
        if (filters) {
            const Filter f = filters->get(b->id);
            if ((f.category & mask) == 0) continue;
        }
        RayHit hit;
        if (rayCastShape(*b, p1, p2, hit)) out.push_back(hit);
    }
    std::sort(out.begin(), out.end(), [](const RayHit& a, const RayHit& b) { return a.fraction < b.fraction; });
    return (int)out.size();
}

bool rayCastClosest(World& w, const Vec2& p1, const Vec2& p2, RayHit& out,
                    const FilterRegistry* filters, uint32_t mask) {
    std::vector<RayHit> hits;
    if (rayCastAll(w, p1, p2, hits, filters, mask) == 0) return false;
    out = hits.front();
    return true;
}

// ================================================================ concave decomposition
std::vector<std::vector<Vec2>> decomposeConcave(const std::vector<Vec2>& polygon) {
    std::vector<std::vector<Vec2>> result;
    if (polygon.size() < 3) return result;

    std::vector<Vec2> poly = polygon;
    // Гарантируем обход против часовой стрелки.
    real area = 0.0;
    for (size_t i = 0; i < poly.size(); ++i)
        area += cross(poly[i], poly[(i + 1) % poly.size()]);
    if (area < 0.0) std::reverse(poly.begin(), poly.end());

    // Ear clipping -> треугольники.
    std::vector<Vec2> work = poly;
    std::vector<std::vector<Vec2>> triangles;
    int guard = 0;

    while (work.size() > 3 && guard++ < 10000) {
        bool clipped = false;
        for (size_t i = 0; i < work.size(); ++i) {
            const Vec2& prev = work[(i + work.size() - 1) % work.size()];
            const Vec2& cur = work[i];
            const Vec2& next = work[(i + 1) % work.size()];
            if (triangleArea(prev, cur, next) <= 1e-12) continue;

            bool ear = true;
            for (size_t j = 0; j < work.size(); ++j) {
                if (j == i || j == (i + work.size() - 1) % work.size() || j == (i + 1) % work.size()) continue;
                if (pointInTriangle(work[j], prev, cur, next)) { ear = false; break; }
            }
            if (!ear) continue;

            triangles.push_back({prev, cur, next});
            work.erase(work.begin() + (long)i);
            clipped = true;
            break;
        }
        if (!clipped) break;
    }
    if (work.size() == 3) triangles.push_back(work);

    // Слияние соседних треугольников, если объединение остаётся выпуклым (Hertel-Mehlhorn).
    std::vector<std::vector<Vec2>> pieces = triangles;
    bool merged = true;
    while (merged) {
        merged = false;
        for (size_t i = 0; i < pieces.size() && !merged; ++i) {
            for (size_t j = i + 1; j < pieces.size() && !merged; ++j) {
                std::vector<Vec2> combined = pieces[i];
                combined.insert(combined.end(), pieces[j].begin(), pieces[j].end());
                const std::vector<Vec2> hull = convexHull(combined);
                if (hull.size() < 3) continue;

                real hullArea = 0.0;
                for (size_t k = 0; k < hull.size(); ++k)
                    hullArea += cross(hull[k], hull[(k + 1) % hull.size()]);
                hullArea = std::fabs(hullArea) * 0.5;

                real sumArea = 0.0;
                for (const std::vector<Vec2>& piece : {pieces[i], pieces[j]}) {
                    real a = 0.0;
                    for (size_t k = 0; k < piece.size(); ++k)
                        a += cross(piece[k], piece[(k + 1) % piece.size()]);
                    sumArea += std::fabs(a) * 0.5;
                }
                if (hullArea > sumArea * 1.02) continue;   // объединение захватило лишнее

                pieces[i] = hull;
                pieces.erase(pieces.begin() + (long)j);
                merged = true;
            }
        }
    }
    return pieces;
}

std::vector<Shape> shapesFromConcave(const std::vector<Vec2>& polygon) {
    std::vector<Shape> shapes;
    for (const std::vector<Vec2>& piece : decomposeConcave(polygon)) {
        if (piece.size() < 3) continue;
        Shape s = Shape::polygon(convexHull(piece));
        if (s.vertices.size() >= 3) shapes.push_back(s);
    }
    return shapes;
}

// ================================================================ curves
Vec2 bezierPoint(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, real t) {
    const real u = 1.0 - t;
    return p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) + p3 * (t * t * t);
}

std::vector<Vec2> sampleBezier(const std::vector<Vec2>& controlPoints, int samples) {
    std::vector<Vec2> out;
    if (controlPoints.size() < 2) return out;
    const int n = std::max(2, samples);

    // Общий случай — алгоритм де Кастелью.
    for (int i = 0; i < n; ++i) {
        const real t = (real)i / (real)(n - 1);
        std::vector<Vec2> tmp = controlPoints;
        for (size_t k = 1; k < controlPoints.size(); ++k)
            for (size_t j = 0; j + k < controlPoints.size(); ++j)
                tmp[j] = tmp[j] * (1.0 - t) + tmp[j + 1] * t;
        out.push_back(tmp[0]);
    }
    return out;
}

} // namespace phys2d
