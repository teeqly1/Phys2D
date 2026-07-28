#include "phys2d/Shape.h"
#include <algorithm>

namespace phys2d {

// ---------------------------------------------------------------- утилиты
Vec2 closestPointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b, real* tOut) {
    const Vec2 ab = b - a;
    const real len2 = ab.lengthSq();
    real t = 0.0;
    if (len2 > EPSILON) t = clampr(dot(p - a, ab) / len2, 0.0, 1.0);
    if (tOut) *tOut = t;
    return a + ab * t;
}

real segmentSegmentDistance(const Vec2& p1, const Vec2& q1,
                            const Vec2& p2, const Vec2& q2,
                            Vec2& c1, Vec2& c2) {
    const Vec2 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const real a = d1.lengthSq(), e = d2.lengthSq(), f = dot(d2, r);
    real s = 0.0, t = 0.0;

    if (a <= EPSILON && e <= EPSILON) { c1 = p1; c2 = p2; return (c1 - c2).length(); }
    if (a <= EPSILON) {
        s = 0.0;
        t = clampr(f / e, 0.0, 1.0);
    } else {
        const real c = dot(d1, r);
        if (e <= EPSILON) {
            t = 0.0;
            s = clampr(-c / a, 0.0, 1.0);
        } else {
            const real b = dot(d1, d2);
            const real denom = a * e - b * b;
            s = (denom > EPSILON) ? clampr((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0)      { t = 0.0; s = clampr(-c / a, 0.0, 1.0); }
            else if (t > 1.0) { t = 1.0; s = clampr((b - c) / a, 0.0, 1.0); }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return (c1 - c2).length();
}

// Монотонная цепь Андреа — выпуклая оболочка (CCW).
std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    if (pts.size() < 3) return pts;
    std::sort(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b) {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    });
    pts.erase(std::unique(pts.begin(), pts.end(),
                          [](const Vec2& a, const Vec2& b) {
                              return (a - b).lengthSq() < 1e-16;
                          }), pts.end());
    if (pts.size() < 3) return pts;

    const size_t n = pts.size();
    std::vector<Vec2> hull(2 * n + 1);
    size_t k = 0;
    for (size_t i = 0; i < n; ++i) {
        while (k >= 2 && cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0.0) --k;
        hull[k++] = pts[i];
    }
    for (size_t i = n - 1, t = k + 1; i-- > 0;) {
        while (k >= t && cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0.0) --k;
        hull[k++] = pts[i];
    }
    hull.resize(k > 0 ? k - 1 : 0);
    return hull;
}

// ---------------------------------------------------------------- фабрики
Shape Shape::circle(real r) {
    Shape s;
    s.type = ShapeType::Circle;
    s.radius = std::max(r, 1e-6);
    s.vertices = { Vec2(0, 0) };
    s.updateDerived();
    return s;
}

Shape Shape::box(real w, real h) {
    Shape s;
    s.type = ShapeType::Polygon;
    s.isBox = true;
    s.width = w; s.height = h;
    s.radius = 0.0;
    const real hw = w * 0.5, hh = h * 0.5;
    s.vertices = { Vec2(-hw, -hh), Vec2(hw, -hh), Vec2(hw, hh), Vec2(-hw, hh) };
    s.updateDerived();
    return s;
}

Shape Shape::polygon(std::vector<Vec2> pts) {
    Shape s;
    s.type = ShapeType::Polygon;
    s.radius = 0.0;
    s.vertices = convexHull(std::move(pts));
    if (s.vertices.size() < 3) s.vertices = { Vec2(-0.5, -0.5), Vec2(0.5, -0.5), Vec2(0.0, 0.5) };
    s.updateDerived();
    return s;
}

Shape Shape::capsule(real r, real len) {
    Shape s;
    s.type = ShapeType::Capsule;
    s.radius = std::max(r, 1e-6);
    s.halfLength = std::max(len * 0.5, 0.0);
    s.vertices = { Vec2(-s.halfLength, 0.0), Vec2(s.halfLength, 0.0) };
    s.updateDerived();
    return s;
}

Shape Shape::regularPolygon(int n, real r, real phase) {
    n = std::max(3, n);
    std::vector<Vec2> pts;
    pts.reserve((size_t)n);
    for (int i = 0; i < n; ++i) {
        const real a = phase + 2.0 * PI * (real)i / (real)n;
        pts.push_back(Vec2::fromAngle(a, r));
    }
    return polygon(std::move(pts));
}

// Шестерня: выпуклое ядро строится по вершинам зубьев (выпуклая аппроксимация).
Shape Shape::gear(int teeth, real innerR, real outerR) {
    teeth = std::max(3, teeth);
    std::vector<Vec2> pts;
    pts.reserve((size_t)teeth * 2);
    for (int i = 0; i < teeth; ++i) {
        const real a0 = 2.0 * PI * (real)i / (real)teeth;
        const real a1 = a0 + PI / (real)teeth * 0.45;
        pts.push_back(Vec2::fromAngle(a0, outerR));
        pts.push_back(Vec2::fromAngle(a1, innerR));
    }
    Shape s = polygon(std::move(pts));
    return s;
}

// ---------------------------------------------------------------- derived
void Shape::updateDerived() {
    normals.clear();

    if (type == ShapeType::Circle) {
        if (vertices.empty()) vertices = { Vec2(0, 0) };
        centroid = vertices[0];
        boundingRadius = centroid.length() + radius;
        return;
    }
    if (type == ShapeType::Capsule) {
        if (vertices.size() != 2)
            vertices = { Vec2(-halfLength, 0.0), Vec2(halfLength, 0.0) };
        centroid = (vertices[0] + vertices[1]) * 0.5;
        boundingRadius = std::max(vertices[0].length(), vertices[1].length()) + radius;
        return;
    }

    // Polygon: гарантируем CCW
    real signedArea = 0.0;
    const size_t n = vertices.size();
    for (size_t i = 0; i < n; ++i)
        signedArea += cross(vertices[i], vertices[(i + 1) % n]);
    if (signedArea < 0.0) std::reverse(vertices.begin(), vertices.end());

    normals.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const Vec2 edge = vertices[(i + 1) % n] - vertices[i];
        normals[i] = edge.rperp().normalized();     // внешняя нормаль для CCW
    }

    // Центроид по интегрированию площади
    Vec2 c(0, 0);
    real area2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Vec2& p0 = vertices[i];
        const Vec2& p1 = vertices[(i + 1) % n];
        const real crs = cross(p0, p1);
        area2 += crs;
        c += (p0 + p1) * crs;
    }
    centroid = (std::fabs(area2) > EPSILON) ? c / (3.0 * area2) : Vec2(0, 0);

    boundingRadius = 0.0;
    for (const Vec2& v : vertices)
        boundingRadius = std::max(boundingRadius, v.length());
    boundingRadius += radius;
}

// ---------------------------------------------------------------- масса / инерция
MassData Shape::computeMass(real density) const {
    MassData md;
    density = std::max(density, 1e-9);

    if (type == ShapeType::Circle) {
        md.area    = PI * radius * radius;
        md.mass    = density * md.area;
        md.center  = centroid;
        md.inertia = 0.5 * md.mass * radius * radius;               // I = m r^2 / 2
        return md;
    }

    if (type == ShapeType::Capsule) {
        const real hl = halfLength, r = radius;
        const real rectArea = 4.0 * hl * r;
        const real circArea = PI * r * r;
        const real mRect = density * rectArea;
        const real mCirc = density * circArea;
        md.area   = rectArea + circArea;
        md.mass   = mRect + mCirc;
        md.center = centroid;
        const real iRect = mRect * (sqr(2.0 * hl) + sqr(2.0 * r)) / 12.0;
        const real iCaps = mCirc * (0.5 * r * r + hl * hl);         // теорема Штейнера
        md.inertia = iRect + iCaps;
        return md;
    }

    // Polygon: интегрирование по треугольникам от начала координат.
    const size_t n = vertices.size();
    real area = 0.0, I = 0.0;
    Vec2 c(0, 0);
    for (size_t i = 0; i < n; ++i) {
        const Vec2& p0 = vertices[i];
        const Vec2& p1 = vertices[(i + 1) % n];
        const real d = cross(p0, p1);
        const real triArea = 0.5 * d;
        area += triArea;
        c += (p0 + p1) * (triArea / 3.0);
        const real intx2 = p0.x * p0.x + p1.x * p0.x + p1.x * p1.x;
        const real inty2 = p0.y * p0.y + p1.y * p0.y + p1.y * p1.y;
        I += (0.25 / 3.0) * (intx2 + inty2) * d;
    }
    area = std::fabs(area);
    md.area   = area;
    md.mass   = density * area;
    md.center = (area > EPSILON) ? c / area : Vec2(0, 0);
    I = density * std::fabs(I);
    // К центру масс (обратная теорема Штейнера), затем в локальной системе тела.
    md.inertia = std::max(I - md.mass * md.center.lengthSq(), 1e-9);
    return md;
}

// ---------------------------------------------------------------- AABB / support
AABB Shape::computeAABB(const Transform& xf, real margin) const {
    AABB bb;
    if (type == ShapeType::Circle) {
        const Vec2 c = xf.apply(centroid);
        bb.min = c - Vec2(radius, radius);
        bb.max = c + Vec2(radius, radius);
    } else if (type == ShapeType::Capsule) {
        Vec2 a, b;
        capsuleSegment(xf, a, b);
        bb.add(a); bb.add(b);
        bb.min -= Vec2(radius, radius);
        bb.max += Vec2(radius, radius);
    } else {
        for (const Vec2& v : vertices) bb.add(xf.apply(v));
    }
    if (margin > 0.0) return bb.expanded(margin);
    return bb;
}

void Shape::capsuleSegment(const Transform& xf, Vec2& a, Vec2& b) const {
    a = xf.apply(Vec2(-halfLength, 0.0));
    b = xf.apply(Vec2( halfLength, 0.0));
}

Vec2 Shape::supportLocal(const Vec2& dirLocal) const {
    if (type == ShapeType::Circle) return centroid;
    real best = -BIG;
    Vec2 bestV = vertices.empty() ? Vec2(0, 0) : vertices[0];
    for (const Vec2& v : vertices) {
        const real d = dot(v, dirLocal);
        if (d > best) { best = d; bestV = v; }
    }
    return bestV;
}

Vec2 Shape::supportWorld(const Transform& xf, const Vec2& dirWorld) const {
    return xf.apply(supportLocal(xf.invApplyR(dirWorld)));
}

bool Shape::containsPoint(const Transform& xf, const Vec2& world) const {
    const Vec2 local = xf.invApply(world);
    if (type == ShapeType::Circle)
        return (local - centroid).lengthSq() <= radius * radius;
    if (type == ShapeType::Capsule) {
        const Vec2 cp = closestPointOnSegment(local, vertices[0], vertices[1]);
        return (local - cp).lengthSq() <= radius * radius;
    }
    const size_t n = vertices.size();
    for (size_t i = 0; i < n; ++i)
        if (dot(normals[i], local - vertices[i]) > 0.0) return false;
    return true;
}

} // namespace phys2d
