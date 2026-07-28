// phys2d - narrow phase: GJK, EPA, SAT, manifolds, time of impact.
#include "phys2d/Collision.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace phys2d {

// ============================================================ helpers
namespace {

struct SimplexVertex {
    Vec2 w;      // support point on the Minkowski difference
    Vec2 sa, sb; // witness points on A and B
};

inline Vec2 supportMinkowski(const Shape& sa, const Transform& xa,
                             const Shape& sb, const Transform& xb,
                             const Vec2& dir, Vec2& pa, Vec2& pb) {
    pa = sa.supportWorld(xa, dir);
    pb = sb.supportWorld(xb, -dir);
    return pa - pb;
}

// Closest point of the segment [a,b] to the origin; returns barycentric u for a.
inline Vec2 closestToOrigin(const Vec2& a, const Vec2& b, real& u) {
    const Vec2 ab = b - a;
    const real len2 = ab.lengthSq();
    if (len2 < EPSILON) { u = 1.0; return a; }
    real t = clampr(-dot(a, ab) / len2, 0.0, 1.0);
    u = 1.0 - t;
    return a + ab * t;
}

real shapeRadius(const Shape& s) {
    return (s.type == ShapeType::Polygon) ? 0.0 : s.radius;
}

} // namespace

// ============================================================ GJK + EPA
GjkEpaResult gjkEpa(const Shape& sa, const Transform& xa,
                    const Shape& sb, const Transform& xb) {
    GjkEpaResult res;

    std::array<SimplexVertex, 3> simplex;
    int count = 0;

    Vec2 dir = xb.p - xa.p;
    if (dir.lengthSq() < EPSILON) dir = Vec2(1.0, 0.0);

    SimplexVertex v;
    v.w = supportMinkowski(sa, xa, sb, xb, dir, v.sa, v.sb);
    simplex[count++] = v;
    dir = -v.w;

    const int kMaxIter = 32;
    bool intersecting = false;

    for (int iter = 0; iter < kMaxIter; ++iter) {
        res.gjkIterations = iter + 1;
        if (dir.lengthSq() < EPSILON) { intersecting = true; break; }

        SimplexVertex nv;
        nv.w = supportMinkowski(sa, xa, sb, xb, dir, nv.sa, nv.sb);
        if (dot(nv.w, dir) < 0.0) break;                 // separated: no intersection of cores

        simplex[count++] = nv;

        if (count == 2) {
            real u;
            const Vec2 p = closestToOrigin(simplex[0].w, simplex[1].w, u);
            if (p.lengthSq() < EPSILON) { intersecting = true; break; }
            dir = -p;
        } else {
            // triangle case
            const Vec2 a = simplex[2].w, b = simplex[1].w, c = simplex[0].w;
            const Vec2 ao = -a;
            const Vec2 ab = b - a, ac = c - a;
            const real abPerp = cross(ab, ao);
            const real acPerp = cross(ao, ac);

            const Vec2 abN = Vec2(-ab.y, ab.x) * ((cross(ab, ac) > 0.0) ? -1.0 : 1.0);
            const Vec2 acN = Vec2(-ac.y, ac.x) * ((cross(ac, ab) > 0.0) ? -1.0 : 1.0);

            if (dot(abN, ao) > 0.0) {
                simplex[0] = simplex[1];
                simplex[1] = simplex[2];
                count = 2;
                dir = abN;
            } else if (dot(acN, ao) > 0.0) {
                simplex[1] = simplex[2];
                count = 2;
                dir = acN;
            } else {
                intersecting = true;
                (void)abPerp; (void)acPerp;
                break;
            }
        }
    }

    const real ra = shapeRadius(sa), rb = shapeRadius(sb);

    if (!intersecting) {
        // Closest distance between the cores using the final simplex.
        Vec2 pa, pb;
        if (count >= 2) {
            real u;
            const Vec2 p = closestToOrigin(simplex[count - 2].w, simplex[count - 1].w, u);
            pa = simplex[count - 2].sa * u + simplex[count - 1].sa * (1.0 - u);
            pb = simplex[count - 2].sb * u + simplex[count - 1].sb * (1.0 - u);
            res.distance = p.length();
            res.normal = (res.distance > EPSILON) ? p / res.distance : Vec2(0.0, 1.0);
        } else {
            pa = simplex[0].sa;
            pb = simplex[0].sb;
            res.distance = simplex[0].w.length();
            res.normal = (res.distance > EPSILON) ? simplex[0].w / res.distance : Vec2(0.0, 1.0);
        }
        res.normal = -res.normal;                          // from A to B
        res.witnessA = pa + res.normal * ra;
        res.witnessB = pb - res.normal * rb;
        res.distance -= (ra + rb);
        if (res.distance <= 0.0) {
            res.hit = true;
            res.depth = -res.distance;
            res.distance = 0.0;
        }
        return res;
    }

    // ---------------------------------------------------------- EPA
    std::vector<SimplexVertex> poly;
    poly.reserve(32);
    for (int i = 0; i < count; ++i) poly.push_back(simplex[i]);
    while (poly.size() < 3) poly.push_back(poly.back());

    // ensure CCW winding
    if (cross(poly[1].w - poly[0].w, poly[2].w - poly[0].w) < 0.0)
        std::swap(poly[1], poly[2]);

    const int kMaxEpa = 32;
    for (int iter = 0; iter < kMaxEpa; ++iter) {
        res.epaIterations = iter + 1;

        // find the closest edge to the origin
        size_t bestIdx = 0;
        real bestDist = BIG;
        Vec2 bestNormal(0.0, 1.0);
        for (size_t i = 0; i < poly.size(); ++i) {
            const size_t j = (i + 1) % poly.size();
            const Vec2 e = poly[j].w - poly[i].w;
            Vec2 n(e.y, -e.x);
            const real len = n.normalize();
            if (len < EPSILON) continue;
            const real d = dot(n, poly[i].w);
            if (d < bestDist) { bestDist = d; bestNormal = n; bestIdx = i; }
        }

        SimplexVertex nv;
        nv.w = supportMinkowski(sa, xa, sb, xb, bestNormal, nv.sa, nv.sb);
        const real d = dot(nv.w, bestNormal);

        if (d - bestDist < 1e-7 || poly.size() >= 30) {
            // converged: build the witness points on the closest edge
            const size_t j = (bestIdx + 1) % poly.size();
            real u;
            closestToOrigin(poly[bestIdx].w, poly[j].w, u);
            const Vec2 pa = poly[bestIdx].sa * u + poly[j].sa * (1.0 - u);
            const Vec2 pb = poly[bestIdx].sb * u + poly[j].sb * (1.0 - u);

            res.hit = true;
            res.normal = bestNormal;                      // from A to B
            res.depth = bestDist + ra + rb;
            res.distance = 0.0;
            res.witnessA = pa + bestNormal * ra;
            res.witnessB = pb - bestNormal * rb;
            return res;
        }
        poly.insert(poly.begin() + (long)bestIdx + 1, nv);
    }

    res.hit = true;
    res.depth = ra + rb;
    res.normal = Vec2(0.0, 1.0);
    res.witnessA = xa.p;
    res.witnessB = xb.p;
    return res;
}

// ============================================================ SAT axis cache
void SatAxisCache::store(uint64_t key, int axisIndex, bool fromB) {
    m_cache[key] = Entry{axisIndex, fromB};
}

bool SatAxisCache::lookup(uint64_t key, int& axisIndex, bool& fromB) const {
    auto it = m_cache.find(key);
    if (it == m_cache.end()) return false;
    axisIndex = it->second.axis;
    fromB = it->second.fromB;
    return true;
}

// ============================================================ detectors
namespace {

void finishManifold(Manifold& m, RigidBody& a, RigidBody& b) {
    m.bodyA = &a;
    m.bodyB = &b;
    m.key = makePairKey(a.id, b.id);
    m.restitution = std::max(a.material.restitution, b.material.restitution);
    m.staticFriction = std::sqrt(a.material.staticFriction * b.material.staticFriction);
    m.dynamicFriction = std::sqrt(a.material.dynamicFriction * b.material.dynamicFriction);
    m.friction = m.dynamicFriction;
    m.touching = m.count > 0;
}

// SAT: maximum separation of B's vertices from A's faces.
real maxSeparation(int& bestIndex, const RigidBody& a, const RigidBody& b) {
    const Transform xa = a.transform();
    real best = -BIG;
    bestIndex = 0;
    for (size_t i = 0; i < a.shape.normals.size(); ++i) {
        const Vec2 n = xa.applyR(a.shape.normals[i]);
        const Vec2 v = xa.apply(a.shape.vertices[i]);
        const Vec2 support = b.shape.supportWorld(b.transform(), -n);
        const real s = dot(n, support - v);
        if (s > best) { best = s; bestIndex = (int)i; }
    }
    return best;
}

void incidentEdge(Vec2 out[2], const RigidBody& ref, int refIndex, const RigidBody& inc) {
    const Vec2 refNormal = ref.transform().applyR(ref.shape.normals[(size_t)refIndex]);
    const Transform xi = inc.transform();

    size_t best = 0;
    real minDot = BIG;
    for (size_t i = 0; i < inc.shape.normals.size(); ++i) {
        const real d = dot(refNormal, xi.applyR(inc.shape.normals[i]));
        if (d < minDot) { minDot = d; best = i; }
    }
    const size_t n = inc.shape.vertices.size();
    out[0] = xi.apply(inc.shape.vertices[best]);
    out[1] = xi.apply(inc.shape.vertices[(best + 1) % n]);
}

int clipSegment(Vec2 out[2], const Vec2 in[2], const Vec2& normal, real offset) {
    int numOut = 0;
    const real d0 = dot(normal, in[0]) - offset;
    const real d1 = dot(normal, in[1]) - offset;
    if (d0 <= 0.0) out[numOut++] = in[0];
    if (d1 <= 0.0) out[numOut++] = in[1];
    if (d0 * d1 < 0.0 && numOut < 2) {
        const real t = d0 / (d0 - d1);
        out[numOut++] = in[0] + (in[1] - in[0]) * t;
    }
    return numOut;
}

} // namespace

// ---------------------------------------------------------------- circle/circle
bool collideCircleCircle(Manifold& m, RigidBody& a, RigidBody& b) {
    m.count = 0;
    const Vec2 d = b.position - a.position;
    const real r = a.shape.radius + b.shape.radius;
    const real dist2 = d.lengthSq();
    if (dist2 > r * r) { m.touching = false; return false; }

    const real dist = std::sqrt(dist2);
    m.normal = (dist > EPSILON) ? d / dist : Vec2(0.0, 1.0);
    m.count = 1;
    m.points[0] = ContactPoint{};
    m.points[0].penetration = r - dist;
    m.points[0].point = a.position + m.normal * (a.shape.radius - 0.5 * m.points[0].penetration);
    m.points[0].featureId = 0;
    finishManifold(m, a, b);
    return true;
}

// ---------------------------------------------------------------- circle/polygon (GJK + EPA)
bool collideCirclePolygon(Manifold& m, RigidBody& circle, RigidBody& poly) {
    m.count = 0;
    const GjkEpaResult r = gjkEpa(circle.shape, circle.transform(), poly.shape, poly.transform());
    if (!r.hit) { m.touching = false; return false; }

    m.normal = r.normal;
    m.count = 1;
    m.points[0] = ContactPoint{};
    m.points[0].penetration = r.depth;
    m.points[0].point = (r.witnessA + r.witnessB) * 0.5;
    m.points[0].featureId = 1;
    finishManifold(m, circle, poly);
    return true;
}

// ---------------------------------------------------------------- polygon/polygon (SAT)
bool collidePolygonPolygon(Manifold& m, RigidBody& a, RigidBody& b, SatAxisCache* cache) {
    m.count = 0;
    const uint64_t key = makePairKey(a.id, b.id);

    // Cached axis early-out: if the cached axis still separates, we are done.
    if (cache) {
        int axis = 0;
        bool fromB = false;
        if (cache->lookup(key, axis, fromB)) {
            const RigidBody& ref = fromB ? b : a;
            const RigidBody& inc = fromB ? a : b;
            if (axis >= 0 && axis < (int)ref.shape.normals.size()) {
                const Transform xr = ref.transform();
                const Vec2 n = xr.applyR(ref.shape.normals[(size_t)axis]);
                const Vec2 v = xr.apply(ref.shape.vertices[(size_t)axis]);
                if (dot(n, inc.shape.supportWorld(inc.transform(), -n) - v) > 0.0) {
                    m.touching = false;
                    return false;
                }
            }
        }
    }

    int edgeA = 0, edgeB = 0;
    const real sepA = maxSeparation(edgeA, a, b);
    if (sepA > 0.0) {
        if (cache) cache->store(key, edgeA, false);
        m.touching = false;
        return false;
    }
    const real sepB = maxSeparation(edgeB, b, a);
    if (sepB > 0.0) {
        if (cache) cache->store(key, edgeB, true);
        m.touching = false;
        return false;
    }

    const bool flip = sepB > sepA + 1e-9;
    RigidBody& ref = flip ? b : a;
    RigidBody& inc = flip ? a : b;
    const int refIndex = flip ? edgeB : edgeA;

    const Transform xr = ref.transform();
    const size_t nv = ref.shape.vertices.size();
    const Vec2 v1 = xr.apply(ref.shape.vertices[(size_t)refIndex]);
    const Vec2 v2 = xr.apply(ref.shape.vertices[((size_t)refIndex + 1) % nv]);
    const Vec2 refNormal = xr.applyR(ref.shape.normals[(size_t)refIndex]);
    const Vec2 tangent = (v2 - v1).normalized();

    Vec2 incident[2];
    incidentEdge(incident, ref, refIndex, inc);

    Vec2 clipped1[2], clipped2[2];
    if (clipSegment(clipped1, incident, -tangent, -dot(tangent, v1)) < 2) { m.touching = false; return false; }
    if (clipSegment(clipped2, clipped1, tangent, dot(tangent, v2)) < 2)   { m.touching = false; return false; }

    const real frontOffset = dot(refNormal, v1);
    m.normal = flip ? -refNormal : refNormal;
    m.satAxis = refIndex;
    m.satAxisDir = m.normal;

    int count = 0;
    for (int i = 0; i < 2; ++i) {
        const real separation = dot(refNormal, clipped2[i]) - frontOffset;
        if (separation > 0.0) continue;
        ContactPoint cp;
        cp.point = clipped2[i];
        cp.penetration = -separation;
        cp.featureId = (uint32_t)((refIndex << 2) | i | (flip ? 0x8000u : 0u));
        m.points[count++] = cp;
        if (count == MAX_MANIFOLD_POINTS) break;
    }
    m.count = count;
    if (count == 0) { m.touching = false; return false; }

    finishManifold(m, a, b);
    if (flip) {
        // manifold normal must always point from A to B
        m.normal = -refNormal;
    }
    return true;
}

// ---------------------------------------------------------------- circle/capsule
bool collideCircleCapsule(Manifold& m, RigidBody& circle, RigidBody& capsule) {
    m.count = 0;
    Vec2 p, q;
    capsule.shape.capsuleSegment(capsule.transform(), p, q);
    const Vec2 closest = closestPointOnSegment(circle.position, p, q);

    const Vec2 d = closest - circle.position;
    const real r = circle.shape.radius + capsule.shape.radius;
    const real dist2 = d.lengthSq();
    if (dist2 > r * r) { m.touching = false; return false; }

    const real dist = std::sqrt(dist2);
    m.normal = (dist > EPSILON) ? d / dist : Vec2(0.0, 1.0);
    m.count = 1;
    m.points[0] = ContactPoint{};
    m.points[0].penetration = r - dist;
    m.points[0].point = circle.position + m.normal * (circle.shape.radius - 0.5 * m.points[0].penetration);
    m.points[0].featureId = 2;
    finishManifold(m, circle, capsule);
    return true;
}

// ---------------------------------------------------------------- capsule/capsule
bool collideCapsuleCapsule(Manifold& m, RigidBody& a, RigidBody& b) {
    m.count = 0;
    Vec2 a1, a2, b1, b2, c1, c2;
    a.shape.capsuleSegment(a.transform(), a1, a2);
    b.shape.capsuleSegment(b.transform(), b1, b2);

    const real dist = segmentSegmentDistance(a1, a2, b1, b2, c1, c2);
    const real r = a.shape.radius + b.shape.radius;
    if (dist > r) { m.touching = false; return false; }

    Vec2 n = c2 - c1;
    const real len = n.normalize();
    if (len < EPSILON) n = (b.position - a.position).normalized();
    if (n.lengthSq() < EPSILON) n = Vec2(0.0, 1.0);

    m.normal = n;
    m.points[0] = ContactPoint{};
    m.points[0].penetration = r - dist;
    m.points[0].point = (c1 + c2) * 0.5;
    m.points[0].featureId = 3;
    m.count = 1;

    // Nearly parallel capsules produce a two-point manifold (stable stacking).
    const Vec2 da = (a2 - a1).normalized();
    const Vec2 db = (b2 - b1).normalized();
    if (std::fabs(cross(da, db)) < 0.05 && (a2 - a1).lengthSq() > EPSILON) {
        const Vec2 alt = closestPointOnSegment(b2, a1, a2);
        const Vec2 diff = b2 - alt;
        if (diff.length() < r + 1e-6 && (alt - c1).lengthSq() > 1e-6) {
            ContactPoint cp;
            cp.point = (alt + b2) * 0.5;
            cp.penetration = r - diff.length();
            cp.featureId = 4;
            if (cp.penetration > 0.0) m.points[m.count++] = cp;
        }
    }

    finishManifold(m, a, b);
    return true;
}

// ---------------------------------------------------------------- polygon/capsule
bool collidePolygonCapsule(Manifold& m, RigidBody& poly, RigidBody& capsule) {
    m.count = 0;
    const GjkEpaResult r = gjkEpa(poly.shape, poly.transform(), capsule.shape, capsule.transform());
    if (!r.hit) { m.touching = false; return false; }

    m.normal = r.normal;
    m.points[0] = ContactPoint{};
    m.points[0].penetration = r.depth;
    m.points[0].point = (r.witnessA + r.witnessB) * 0.5;
    m.points[0].featureId = 5;
    m.count = 1;

    // Second contact point when the capsule axis lies flat on a polygon face.
    Vec2 p, q;
    capsule.shape.capsuleSegment(capsule.transform(), p, q);
    const Vec2 axis = (q - p).normalized();
    if (std::fabs(dot(axis, r.normal)) < 0.15) {
        const Vec2 other = (distanceSq(p, m.points[0].point) > distanceSq(q, m.points[0].point)) ? p : q;
        const Vec2 onPoly = poly.shape.supportWorld(poly.transform(), r.normal);
        const real pen = capsule.shape.radius - dot(other - onPoly, r.normal);
        if (pen > 0.0) {
            ContactPoint cp;
            cp.point = other - r.normal * capsule.shape.radius;
            cp.penetration = std::min(pen, r.depth);
            cp.featureId = 6;
            m.points[m.count++] = cp;
        }
    }

    finishManifold(m, poly, capsule);
    return true;
}

// ---------------------------------------------------------------- dispatcher
bool collide(Manifold& m, RigidBody& a, RigidBody& b, SatAxisCache* cache) {
    const ShapeType ta = a.shape.type;
    const ShapeType tb = b.shape.type;

    bool hit = false;
    bool swapped = false;

    if (ta == ShapeType::Circle && tb == ShapeType::Circle) {
        hit = collideCircleCircle(m, a, b);
    } else if (ta == ShapeType::Circle && tb == ShapeType::Polygon) {
        hit = collideCirclePolygon(m, a, b);
    } else if (ta == ShapeType::Polygon && tb == ShapeType::Circle) {
        hit = collideCirclePolygon(m, b, a);
        swapped = true;
    } else if (ta == ShapeType::Polygon && tb == ShapeType::Polygon) {
        hit = collidePolygonPolygon(m, a, b, cache);
    } else if (ta == ShapeType::Circle && tb == ShapeType::Capsule) {
        hit = collideCircleCapsule(m, a, b);
    } else if (ta == ShapeType::Capsule && tb == ShapeType::Circle) {
        hit = collideCircleCapsule(m, b, a);
        swapped = true;
    } else if (ta == ShapeType::Polygon && tb == ShapeType::Capsule) {
        hit = collidePolygonCapsule(m, a, b);
    } else if (ta == ShapeType::Capsule && tb == ShapeType::Polygon) {
        hit = collidePolygonCapsule(m, b, a);
        swapped = true;
    } else {
        hit = collideCapsuleCapsule(m, a, b);
    }

    if (!hit) { m.touching = false; return false; }

    if (swapped) {
        m.normal = -m.normal;
        m.satAxisDir = -m.satAxisDir;
    }
    m.bodyA = &a;
    m.bodyB = &b;
    m.key = makePairKey(a.id, b.id);
    m.touching = m.count > 0;
    return m.touching;
}

// ---------------------------------------------------------------- time of impact
real timeOfImpact(const RigidBody& a, const RigidBody& b, real dt, int iterations) {
    // Conservative binary search of the first contact instant within [0, dt].
    auto overlapAt = [&](real t) {
        Transform xa(a.position + a.velocity * t, a.angle + a.angularVelocity * t);
        Transform xb(b.position + b.velocity * t, b.angle + b.angularVelocity * t);
        const GjkEpaResult r = gjkEpa(a.shape, xa, b.shape, xb);
        return r.hit;
    };

    if (overlapAt(0.0)) return 0.0;
    if (!overlapAt(dt)) return 1.0;

    real lo = 0.0, hi = dt;
    for (int i = 0; i < iterations; ++i) {
        const real mid = 0.5 * (lo + hi);
        if (overlapAt(mid)) hi = mid; else lo = mid;
    }
    return (dt > EPSILON) ? clampr(hi / dt, 0.0, 1.0) : 1.0;
}

} // namespace phys2d
