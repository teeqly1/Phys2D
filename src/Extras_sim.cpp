// phys2d v2 - fracture, soft bodies, ragdolls, SPH particles, force fields, thermal, audio.
#include "phys2d/Extras.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <numeric>
#include <random>

namespace phys2d {
namespace {

inline uint32_t hashU32(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352dU;
    x ^= x >> 15; x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

inline real fade(real t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

inline real gradDot(uint32_t h, real x, real y) {
    switch (h & 7u) {
        case 0: return  x + y;
        case 1: return  x - y;
        case 2: return -x + y;
        case 3: return -x - y;
        case 4: return  x;
        case 5: return -x;
        case 6: return  y;
        default: return -y;
    }
}

inline uint64_t cellKey(int gx, int gy) {
    return ((uint64_t)(uint32_t)gx << 32) | (uint32_t)gy;
}

real polygonArea(const std::vector<Vec2>& p) {
    real a = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        const Vec2& v0 = p[i];
        const Vec2& v1 = p[(i + 1) % p.size()];
        a += cross(v0, v1);
    }
    return a * 0.5;
}

// Отсечение выпуклого многоугольника полуплоскостью dot(n, p) <= d (алгоритм Сазерленда-Ходжмена).
std::vector<Vec2> clipHalfPlane(const std::vector<Vec2>& poly, const Vec2& n, real d) {
    std::vector<Vec2> out;
    if (poly.empty()) return out;
    out.reserve(poly.size() + 2);

    for (size_t i = 0; i < poly.size(); ++i) {
        const Vec2& cur = poly[i];
        const Vec2& nxt = poly[(i + 1) % poly.size()];
        const real dc = dot(n, cur) - d;
        const real dn = dot(n, nxt) - d;

        if (dc <= 0.0) out.push_back(cur);
        if ((dc < 0.0 && dn > 0.0) || (dc > 0.0 && dn < 0.0)) {
            const real t = dc / (dc - dn);
            out.push_back(cur + (nxt - cur) * t);
        }
    }
    return out;
}

std::vector<Vec2> shapeOutline(const Shape& s, int circleSegments = 16) {
    std::vector<Vec2> pts;
    if (s.type == ShapeType::Polygon) {
        pts = s.vertices;
    } else if (s.type == ShapeType::Circle) {
        for (int i = 0; i < circleSegments; ++i) {
            const real a = 2.0 * PI * (real)i / (real)circleSegments;
            pts.push_back(Vec2(std::cos(a), std::sin(a)) * s.radius);
        }
    } else { // capsule
        const real hl = s.halfLength, r = s.radius;
        const int half = std::max(4, circleSegments / 2);
        for (int i = 0; i <= half; ++i) {
            const real a = -PI * 0.5 + PI * (real)i / (real)half;
            pts.push_back(Vec2(hl + std::cos(a) * r, std::sin(a) * r));
        }
        for (int i = 0; i <= half; ++i) {
            const real a = PI * 0.5 + PI * (real)i / (real)half;
            pts.push_back(Vec2(-hl + std::cos(a) * r, std::sin(a) * r));
        }
    }
    return pts;
}

} // namespace

// ================================================================ noise
real perlinNoise(real x, real y, uint32_t seed) {
    // Защита: при больших координатах (int)floor() переполняется — шум давал мусор/inf.
    if (!std::isfinite(x) || !std::isfinite(y)) return 0.0;
    const real kWrap = 1.0e6;
    x = std::fmod(x, kWrap);
    y = std::fmod(y, kWrap);

    const int xi = (int)std::floor(x), yi = (int)std::floor(y);
    const real xf = x - (real)xi, yf = y - (real)yi;
    const real u = fade(xf), v = fade(yf);

    auto h = [&](int i, int j) {
        return hashU32((uint32_t)(i * 374761393) ^ (uint32_t)(j * 668265263) ^ seed);
    };

    const real n00 = gradDot(h(xi, yi), xf, yf);
    const real n10 = gradDot(h(xi + 1, yi), xf - 1.0, yf);
    const real n01 = gradDot(h(xi, yi + 1), xf, yf - 1.0);
    const real n11 = gradDot(h(xi + 1, yi + 1), xf - 1.0, yf - 1.0);

    const real a = n00 + u * (n10 - n00);
    const real b = n01 + u * (n11 - n01);
    return a + v * (b - a);
}

real fractalNoise(real x, real y, int octaves, real persistence, uint32_t seed) {
    real total = 0.0, amplitude = 1.0, frequency = 1.0, norm = 0.0;
    for (int i = 0; i < std::max(1, octaves); ++i) {
        total += perlinNoise(x * frequency, y * frequency, seed + (uint32_t)i * 7919u) * amplitude;
        norm += amplitude;
        amplitude *= persistence;
        frequency *= 2.0;
    }
    return (norm > 0.0) ? total / norm : 0.0;
}

// ================================================================ fracture
std::vector<Shape> voronoiFracture(const Shape& shape, const std::vector<Vec2>& sites) {
    std::vector<Shape> out;
    if (sites.size() < 2) return out;

    const std::vector<Vec2> base = shapeOutline(shape, 20);
    if (base.size() < 3) return out;

    for (size_t i = 0; i < sites.size(); ++i) {
        std::vector<Vec2> cell = base;
        for (size_t j = 0; j < sites.size() && !cell.empty(); ++j) {
            if (i == j) continue;
            const Vec2 d = sites[j] - sites[i];
            const real len = d.length();
            if (len < 1e-9) continue;
            const Vec2 n = d / len;
            const real offset = dot(n, (sites[i] + sites[j]) * 0.5);
            cell = clipHalfPlane(cell, n, offset);
        }
        if (cell.size() < 3) continue;
        if (std::fabs(polygonArea(cell)) < 1e-6) continue;

        Shape piece = Shape::polygon(convexHull(cell));
        if (piece.vertices.size() >= 3) out.push_back(piece);
    }
    return out;
}

std::vector<Shape> radialFracture(const Shape& shape, const Vec2& impactLocal, int pieces, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    const real r = std::max((real)0.05, shape.boundingRadius);
    std::vector<Vec2> sites;
    sites.reserve((size_t)std::max(2, pieces));
    sites.push_back(impactLocal);

    for (int i = 1; i < std::max(2, pieces); ++i) {
        const real angle = 2.0 * PI * ((real)i / (real)pieces + uni(rng) * 0.12);
        const real dist = r * (0.25 + 0.75 * std::sqrt(uni(rng)));
        sites.push_back(impactLocal + Vec2::fromAngle(angle, dist));
    }
    return voronoiFracture(shape, sites);
}

void FractureSystem::makeBreakable(BodyId id, real threshold) {
    m_breakable[id] = Entry{threshold > 0.0 ? threshold : impulseThreshold, 0};
}

std::vector<BodyId> FractureSystem::fracture(BodyId id, const Vec2& impactWorld, int piecesCount) {
    std::vector<BodyId> result;
    if (!m_world) return result;

    RigidBody* b = m_world->body(id);
    if (!b || !b->isDynamic()) return result;

    const auto entryIt = m_breakable.find(id);
    const int generation = (entryIt == m_breakable.end()) ? 0 : entryIt->second.generation;
    if (generation >= maxGeneration) return result;

    const Transform xf = b->transform();
    const Vec2 impactLocal = xf.invApply(impactWorld);
    const Shape shape = b->shape;
    const Vec2 position = b->position;
    const Vec2 velocity = b->velocity;
    const real angle = b->angle;
    const real omega = b->angularVelocity;
    const Material material = b->material;
    const real gravityScale = b->gravityScale;

    std::vector<Shape> fragments = radialFracture(shape, impactLocal, std::max(2, piecesCount),
                                                  hashU32(id * 2654435761u));
    if (fragments.size() < 2) return result;
    if ((int)fragments.size() > maxFragmentsPerStep) fragments.resize((size_t)maxFragmentsPerStep);

    m_world->destroyBody(id);
    m_breakable.erase(id);

    std::mt19937 rng(hashU32(id));
    std::uniform_real_distribution<double> jitter(-0.35, 0.35);

    for (Shape& frag : fragments) {
        const MassData md = frag.computeMass(material.density);
        if (md.area < minFragmentArea) continue;

        // Центрируем осколок вокруг его центра масс.
        const Vec2 localCenter = md.center;
        for (Vec2& v : frag.vertices) v -= localCenter;
        frag.updateDerived();

        BodyDef def;
        def.type = BodyType::Dynamic;
        def.shape = frag;
        def.material = material;
        def.position = position + localCenter.rotated(angle);
        def.angle = angle;
        def.gravityScale = gravityScale;
        def.name = "fragment";

        const Vec2 outward = (def.position - position);
        def.velocity = velocity + outward * 1.5 + Vec2(jitter(rng), jitter(rng));
        def.angularVelocity = omega + (real)jitter(rng) * 4.0;

        const BodyId fid = m_world->createBody(def);
        result.push_back(fid);
        m_breakable[fid] = Entry{impulseThreshold * 1.6, generation + 1};
    }

    ++m_total;
    if (m_cb) {
        FractureEvent e;
        e.original = id;
        e.fragments = result;
        e.impactPoint = impactWorld;
        m_cb(e);
    }
    return result;
}

void FractureSystem::update(real) {
    if (!m_world || m_breakable.empty()) return;

    struct Pending { BodyId id; Vec2 point; real impulse; };
    std::vector<Pending> pending;

    for (const Manifold& mf : m_world->manifolds()) {
        if (!mf.touching || mf.count == 0) continue;
        real maxImpulse = 0.0;
        Vec2 point = mf.points[0].point;
        for (int i = 0; i < mf.count; ++i) {
            if (mf.points[i].normalImpulse > maxImpulse) {
                maxImpulse = mf.points[i].normalImpulse;
                point = mf.points[i].point;
            }
        }
        if (maxImpulse <= 0.0) continue;

        RigidBody* candidates[2] = {mf.bodyA, mf.bodyB};
        for (RigidBody* b : candidates) {
            if (!b || !b->isDynamic()) continue;
            const auto it = m_breakable.find(b->id);
            if (it == m_breakable.end()) continue;
            if (maxImpulse < it->second.threshold) continue;
            pending.push_back(Pending{b->id, point, maxImpulse});
        }
    }

    for (const Pending& p : pending) {
        const std::vector<BodyId> frags = fracture(p.id, p.point, pieces);
        if (!frags.empty() && m_cb) {
            // событие уже вызвано внутри fracture(); здесь лишь учёт импульса
            (void)p.impulse;
        }
    }
}

// ================================================================ soft bodies
SoftBody SoftBody::grid(const Vec2& origin, real w, real h, int nx, int ny, real totalMass) {
    SoftBody sb;
    sb.name = "soft_grid";
    const int cx = std::max(2, nx), cy = std::max(2, ny);
    const real dx = w / (real)(cx - 1), dy = h / (real)(cy - 1);
    const real nodeMass = totalMass / (real)(cx * cy);

    for (int j = 0; j < cy; ++j) {
        for (int i = 0; i < cx; ++i) {
            SoftNode n;
            n.position = origin + Vec2(dx * i, dy * j);
            n.previous = n.position;
            n.mass = nodeMass;
            n.invMass = (nodeMass > 0.0) ? 1.0 / nodeMass : 0.0;
            sb.nodes.push_back(n);
        }
    }
    auto idx = [cx](int i, int j) { return j * cx + i; };

    for (int j = 0; j < cy; ++j) {
        for (int i = 0; i < cx; ++i) {
            if (i + 1 < cx) sb.links.push_back(SoftLink{idx(i, j), idx(i + 1, j), dx, 900.0, 6.0, 0.0, false});
            if (j + 1 < cy) sb.links.push_back(SoftLink{idx(i, j), idx(i, j + 1), dy, 900.0, 6.0, 0.0, false});
            if (i + 1 < cx && j + 1 < cy) {
                const real diag = std::sqrt(dx * dx + dy * dy);
                sb.links.push_back(SoftLink{idx(i, j), idx(i + 1, j + 1), diag, 600.0, 5.0, 0.0, false});
                sb.links.push_back(SoftLink{idx(i + 1, j), idx(i, j + 1), diag, 600.0, 5.0, 0.0, false});
                sb.triangles.push_back(SoftTriangle{idx(i, j), idx(i + 1, j), idx(i, j + 1), 0.0, Mat22(), false});
                sb.triangles.push_back(SoftTriangle{idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1), 0.0, Mat22(), false});
            }
        }
    }
    sb.rebuildRest();
    return sb;
}

SoftBody SoftBody::rope(const Vec2& a, const Vec2& b, int segments, real totalMass) {
    SoftBody sb;
    sb.name = "soft_rope";
    sb.model = SoftModel::MassSpring;
    const int n = std::max(2, segments + 1);
    const real nodeMass = totalMass / (real)n;
    for (int i = 0; i < n; ++i) {
        SoftNode node;
        node.position = Vec2::lerp(a, b, (real)i / (real)(n - 1));
        node.previous = node.position;
        node.mass = nodeMass;
        node.invMass = 1.0 / nodeMass;
        sb.nodes.push_back(node);
    }
    for (int i = 0; i + 1 < n; ++i)
        sb.links.push_back(SoftLink{i, i + 1, 0.0, 2000.0, 8.0, 0.0, false});
    sb.rebuildRest();
    return sb;
}

SoftBody SoftBody::fromPolygon(const std::vector<Vec2>& poly, real spacing, real totalMass) {
    SoftBody sb;
    sb.name = "soft_poly";
    if (poly.size() < 3) return sb;

    AABB box;
    box.reset();
    for (const Vec2& p : poly) box.add(p);

    auto inside = [&poly](const Vec2& p) {
        bool in = false;
        for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
            if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
                (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
                in = !in;
        }
        return in;
    };

    const real step = std::max((real)1e-3, spacing);
    std::vector<std::vector<int>> gridIndex;
    const int nx = (int)std::ceil((box.max.x - box.min.x) / step) + 1;
    const int ny = (int)std::ceil((box.max.y - box.min.y) / step) + 1;
    gridIndex.assign((size_t)ny, std::vector<int>((size_t)nx, -1));

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const Vec2 p = box.min + Vec2(step * i, step * j);
            if (!inside(p)) continue;
            SoftNode n;
            n.position = p;
            n.previous = p;
            sb.nodes.push_back(n);
            gridIndex[(size_t)j][(size_t)i] = (int)sb.nodes.size() - 1;
        }
    }
    if (sb.nodes.empty()) return sb;

    const real nodeMass = totalMass / (real)sb.nodes.size();
    for (SoftNode& n : sb.nodes) {
        n.mass = nodeMass;
        n.invMass = 1.0 / nodeMass;
    }

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const int a = gridIndex[(size_t)j][(size_t)i];
            if (a < 0) continue;
            const int right = (i + 1 < nx) ? gridIndex[(size_t)j][(size_t)i + 1] : -1;
            const int up = (j + 1 < ny) ? gridIndex[(size_t)j + 1][(size_t)i] : -1;
            const int diag = (i + 1 < nx && j + 1 < ny) ? gridIndex[(size_t)j + 1][(size_t)i + 1] : -1;
            if (right >= 0) sb.links.push_back(SoftLink{a, right, step, 900.0, 6.0, 0.0, false});
            if (up >= 0) sb.links.push_back(SoftLink{a, up, step, 900.0, 6.0, 0.0, false});
            if (right >= 0 && up >= 0 && diag >= 0) {
                sb.triangles.push_back(SoftTriangle{a, right, up, 0.0, Mat22(), false});
                sb.triangles.push_back(SoftTriangle{right, diag, up, 0.0, Mat22(), false});
            }
        }
    }
    sb.rebuildRest();
    return sb;
}

void SoftBody::rebuildRest() {
    for (SoftLink& l : links) {
        if (l.a < 0 || l.b < 0 || (size_t)l.a >= nodes.size() || (size_t)l.b >= nodes.size()) continue;
        const real len = (nodes[(size_t)l.b].position - nodes[(size_t)l.a].position).length();
        if (l.restLength <= 0.0) l.restLength = len;
    }
    for (SoftTriangle& t : triangles) {
        const Vec2& p0 = nodes[(size_t)t.a].position;
        const Vec2& p1 = nodes[(size_t)t.b].position;
        const Vec2& p2 = nodes[(size_t)t.c].position;
        const Vec2 e1 = p1 - p0, e2 = p2 - p0;
        t.restArea = 0.5 * std::fabs(cross(e1, e2));
        Mat22 rest;
        rest.m00 = e1.x; rest.m01 = e2.x;
        rest.m10 = e1.y; rest.m11 = e2.y;
        t.invRest = rest.inverse();
    }
}

Vec2 SoftBody::centroid() const {
    if (nodes.empty()) return Vec2();
    Vec2 sum;
    for (const SoftNode& n : nodes) sum += n.position;
    return sum / (real)nodes.size();
}

real SoftBody::area() const {
    real a = 0.0;
    for (const SoftTriangle& t : triangles) {
        if (t.broken) continue;
        const Vec2 e1 = nodes[(size_t)t.b].position - nodes[(size_t)t.a].position;
        const Vec2 e2 = nodes[(size_t)t.c].position - nodes[(size_t)t.a].position;
        a += 0.5 * std::fabs(cross(e1, e2));
    }
    return a;
}

AABB SoftBody::bounds() const {
    AABB box;
    box.reset();
    for (const SoftNode& n : nodes) box.add(n.position);
    return box;
}

int SoftBody::componentCount() const {
    if (nodes.empty()) return 0;
    std::vector<int> parent(nodes.size());
    std::iota(parent.begin(), parent.end(), 0);
    std::function<int(int)> find = [&](int x) {
        while (parent[(size_t)x] != x) { parent[(size_t)x] = parent[(size_t)parent[(size_t)x]]; x = parent[(size_t)x]; }
        return x;
    };
    for (const SoftLink& l : links) {
        if (l.broken) continue;
        const int ra = find(l.a), rb = find(l.b);
        if (ra != rb) parent[(size_t)ra] = rb;
    }
    std::unordered_set<int> roots;
    for (size_t i = 0; i < nodes.size(); ++i) roots.insert(find((int)i));
    return (int)roots.size();
}

SoftBody* SoftBodySystem::add(SoftBody body) {
    m_bodies.push_back(std::make_unique<SoftBody>(std::move(body)));
    return m_bodies.back().get();
}

void SoftBodySystem::clear() { m_bodies.clear(); }

void SoftBodySystem::integrate(SoftBody& sb, real dt) {
    for (SoftNode& n : sb.nodes) {
        if (n.pinned || n.invMass <= 0.0) { n.velocity = Vec2(); n.force = Vec2(); continue; }
        n.force += gravity * n.mass;
        const Vec2 accel = n.force * n.invMass;
        n.velocity += accel * dt;
        n.velocity *= std::max(0.0, 1.0 - sb.damping * dt * 0.02);
        n.previous = n.position;
        n.position += n.velocity * dt;
        n.force = Vec2();
    }
}

void SoftBodySystem::solveLinks(SoftBody& sb, real dt) {
    const int iterations = std::max(1, solverIterations);
    for (int it = 0; it < iterations; ++it) {
        for (SoftLink& l : sb.links) {
            if (l.broken) continue;
            SoftNode& a = sb.nodes[(size_t)l.a];
            SoftNode& b = sb.nodes[(size_t)l.b];
            Vec2 d = b.position - a.position;
            const real len = d.length();
            if (len < 1e-9) continue;

            const real strain = (len - l.restLength) / std::max((real)1e-9, l.restLength);
            const real tear = (l.breakStrain > 0.0) ? l.breakStrain : sb.tearStrain;
            if (tear > 0.0 && std::fabs(strain) > tear) {
                l.broken = true;
                ++m_tears;
                continue;
            }

            const Vec2 n = d / len;
            const real invSum = a.invMass + b.invMass;
            if (invSum <= 0.0) continue;

            const real stiffness = clampr(l.stiffness * dt * dt, 0.0, 1.0);
            const real correction = (len - l.restLength) * (stiffness > 0.0 ? std::max(stiffness, 0.35) : 0.35);
            const Vec2 delta = n * (correction / invSum);

            if (!a.pinned) a.position += delta * a.invMass;
            if (!b.pinned) b.position -= delta * b.invMass;

            // вязкое демпфирование по относительной скорости
            const real relVel = dot(b.velocity - a.velocity, n);
            const Vec2 damp = n * (relVel * l.damping * dt / invSum);
            if (!a.pinned) a.velocity += damp * a.invMass;
            if (!b.pinned) b.velocity -= damp * b.invMass;
        }
    }

    // Внутреннее давление (надувные тела).
    if (sb.pressure > 0.0 && !sb.triangles.empty()) {
        const Vec2 c = sb.centroid();
        for (SoftNode& n : sb.nodes) {
            if (n.pinned) continue;
            Vec2 d = n.position - c;
            const real len = d.length();
            if (len < 1e-9) continue;
            n.velocity += (d / len) * (sb.pressure * dt * n.invMass);
        }
    }
}

// Co-rotational FEM на треугольной сетке: деформация -> узловые силы.
void SoftBodySystem::solveFem(SoftBody& sb, real dt) {
    const real E = sb.youngModulus;
    const real nu = clampr(sb.poissonRatio, 0.0, 0.49);
    const real lambda = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
    const real mu = E / (2.0 * (1.0 + nu));

    for (SoftTriangle& t : sb.triangles) {
        if (t.broken) continue;
        SoftNode& n0 = sb.nodes[(size_t)t.a];
        SoftNode& n1 = sb.nodes[(size_t)t.b];
        SoftNode& n2 = sb.nodes[(size_t)t.c];

        const Vec2 e1 = n1.position - n0.position;
        const Vec2 e2 = n2.position - n0.position;

        Mat22 D;
        D.m00 = e1.x; D.m01 = e2.x;
        D.m10 = e1.y; D.m11 = e2.y;

        Mat22 F = D * t.invRest;                    // градиент деформации

        // Полярное разложение -> вращение R (co-rotational).
        const real angle = std::atan2(F.m10 + (-F.m01), F.m00 + F.m11);
        const Mat22 R(angle);
        const Mat22 Rt = R.transposed();
        Mat22 S = Rt * F;

        // Деформация Коши (малые деформации в повёрнутом базисе).
        const real exx = S.m00 - 1.0;
        const real eyy = S.m11 - 1.0;
        const real exy = 0.5 * (S.m01 + S.m10);
        const real trace = exx + eyy;

        Mat22 stress;
        stress.m00 = 2.0 * mu * exx + lambda * trace;
        stress.m11 = 2.0 * mu * eyy + lambda * trace;
        stress.m01 = 2.0 * mu * exy;
        stress.m10 = stress.m01;

        // Разрыв по превышению деформации.
        if (sb.tearStrain > 0.0 && std::sqrt(exx * exx + eyy * eyy + 2.0 * exy * exy) > sb.tearStrain * 3.0) {
            t.broken = true;
            ++m_tears;
            continue;
        }

        const Mat22 P = R * stress;
        const real area = std::max((real)1e-9, t.restArea);

        // Узловые силы через градиенты базисных функций.
        const Mat22 invRestT = t.invRest.transposed();
        const Mat22 H = P * invRestT;
        const Vec2 f1(-H.m00 * area, -H.m10 * area);
        const Vec2 f2(-H.m01 * area, -H.m11 * area);
        const Vec2 f0 = (f1 + f2) * -1.0;

        const real scale = 1.0;
        n0.force += f0 * scale;
        n1.force += f1 * scale;
        n2.force += f2 * scale;
    }
    (void)dt;
}

void SoftBodySystem::collideRigid(SoftBody& sb, real dt) {
    if (!m_world || !coupleWithRigid) return;

    for (SoftNode& n : sb.nodes) {
        if (n.pinned) continue;
        if (!n.position.isFinite() || !n.velocity.isFinite()) continue;
        RigidBody* hit = m_world->queryPoint(n.position);
        if (!hit) continue;
        if (!hit->position.isFinite() || !hit->velocity.isFinite()) continue;

        // Выталкивание по направлению от центра тела к узлу.
        Vec2 dir = n.position - hit->position;
        real len = dir.length();
        if (len < 1e-9) { dir = Vec2(0.0, 1.0); len = 1e-9; }
        else dir /= len;

        // Глубина оценивается по описанной окружности (достаточно для связки).
        const real depth = std::max((real)0.0, hit->boundingRadius - len) + sb.nodeRadius;
        n.position += dir * depth;

        const Vec2 bodyVel = hit->velocityAtPoint(n.position);
        const Vec2 rel = n.velocity - bodyVel;
        const real vn = dot(rel, dir);
        if (vn < 0.0) {
            // Ограничиваем импульс массой лёгкого из двух тел: иначе узел разгоняет
            // тонкое звено (m ~ 0.1 кг) до расходимости в цепочке шарниров.
            const real coupleMass = hit->isDynamic()
                                        ? std::min(n.mass, hit->mass)
                                        : n.mass;
            const Vec2 impulse = dir * (-vn * (1.0 + sb.restitution) * coupleMass);
            if (impulse.isFinite()) {
                n.velocity += impulse * n.invMass;
                if (hit->isDynamic()) hit->applyImpulseAtPoint(impulse * -1.0, n.position);
            }

            // трение
            const Vec2 tangent = dir.perp();
            const real vt = dot(n.velocity - bodyVel, tangent);
            const Vec2 friction = tangent * (-vt * sb.friction * coupleMass);
            if (friction.isFinite()) {
                n.velocity += friction * n.invMass;
                if (hit->isDynamic()) hit->applyImpulseAtPoint(friction * -1.0, n.position);
            }
        }
    }
    (void)dt;
}

void SoftBodySystem::update(real dt) {
    if (dt <= 0.0) return;
    m_tears = 0;

    for (std::unique_ptr<SoftBody>& ptr : m_bodies) {
        SoftBody& sb = *ptr;
        if (sb.nodes.empty()) continue;

        if (sb.model == SoftModel::FEM || sb.model == SoftModel::Hybrid) solveFem(sb, dt);
        integrate(sb, dt);
        if (sb.model == SoftModel::MassSpring || sb.model == SoftModel::Hybrid) solveLinks(sb, dt);
        collideRigid(sb, dt);

        // Скорость из смещения (согласование после позиционных правок).
        for (SoftNode& n : sb.nodes) {
            if (n.pinned) { n.velocity = Vec2(); continue; }
            n.velocity = (n.position - n.previous) / dt;
        }
    }
}

// ================================================================ ragdoll
Ragdoll createRagdoll(World& w, const RagdollConfig& cfg) {
    Ragdoll r;
    const real s = cfg.scale;
    const Vec2 o = cfg.position;

    auto makePart = [&](const Vec2& offset, real width, real height, const char* nm) {
        BodyDef def;
        def.type = BodyType::Dynamic;
        def.shape = Shape::box(width * s, height * s);
        def.position = o + offset * s;
        def.material.density = cfg.density;
        def.material.restitution = 0.05;
        def.material.staticFriction = 0.7;
        def.material.dynamicFriction = 0.6;
        def.name = nm;
        const BodyId id = w.createBody(def);
        r.parts.push_back(id);
        return id;
    };

    r.torso    = makePart(Vec2(0.0, 0.0), 0.42, 0.62, "torso");
    r.pelvis   = makePart(Vec2(0.0, -0.46), 0.38, 0.26, "pelvis");
    r.head     = makePart(Vec2(0.0, 0.52), 0.28, 0.30, "head");
    r.armL     = makePart(Vec2(-0.36, 0.18), 0.16, 0.40, "arm_l");
    r.armR     = makePart(Vec2(0.36, 0.18), 0.16, 0.40, "arm_r");
    r.forearmL = makePart(Vec2(-0.36, -0.22), 0.14, 0.38, "forearm_l");
    r.forearmR = makePart(Vec2(0.36, -0.22), 0.14, 0.38, "forearm_r");
    r.thighL   = makePart(Vec2(-0.14, -0.86), 0.18, 0.46, "thigh_l");
    r.thighR   = makePart(Vec2(0.14, -0.86), 0.18, 0.46, "thigh_r");
    r.shinL    = makePart(Vec2(-0.14, -1.32), 0.15, 0.44, "shin_l");
    r.shinR    = makePart(Vec2(0.14, -1.32), 0.15, 0.44, "shin_r");

    struct JointSpec { BodyId a, b; Vec2 anchor; real lo, hi; };
    const JointSpec specs[] = {
        {r.torso,  r.head,     Vec2(0.0, 0.34) * s,   -0.6, 0.6},
        {r.torso,  r.pelvis,   Vec2(0.0, -0.32) * s,  -0.4, 0.4},
        {r.torso,  r.armL,     Vec2(-0.30, 0.30) * s, -2.2, 1.2},
        {r.torso,  r.armR,     Vec2(0.30, 0.30) * s,  -1.2, 2.2},
        {r.armL,   r.forearmL, Vec2(-0.36, -0.02) * s,-2.4, 0.0},
        {r.armR,   r.forearmR, Vec2(0.36, -0.02) * s,  0.0, 2.4},
        {r.pelvis, r.thighL,   Vec2(-0.14, -0.58) * s,-1.4, 1.0},
        {r.pelvis, r.thighR,   Vec2(0.14, -0.58) * s, -1.0, 1.4},
        {r.thighL, r.shinL,    Vec2(-0.14, -1.08) * s, 0.0, 2.0},
        {r.thighR, r.shinR,    Vec2(0.14, -1.08) * s,  0.0, 2.0},
    };

    for (const JointSpec& js : specs) {
        RigidBody* ba = w.body(js.a);
        RigidBody* bb = w.body(js.b);
        if (!ba || !bb) continue;
        const Vec2 world = o + js.anchor;
        Constraint* pivot = w.createRevolute(js.a, js.b,
                                             ba->transform().invApply(world),
                                             bb->transform().invApply(world));
        if (pivot) r.joints.push_back(pivot);
        Constraint* limit = w.createAngular(js.a, js.b, js.lo, js.hi);
        if (limit) r.joints.push_back(limit);
    }
    return r;
}

// ================================================================ SPH particles
void ParticleSystem::emit(const Vec2& p, const Vec2& v) {
    FluidParticle fp;
    fp.position = p;
    fp.velocity = v;
    fp.mass = particleMass;
    m_particles.push_back(fp);
}

void ParticleSystem::emitBlock(const Vec2& origin, real w, real h, real spacing) {
    const real step = std::max((real)1e-3, spacing);
    // Calibrate particle mass so a lattice with this spacing equals restDensity.
    {
        const real hh = smoothingRadius;
        const real poly6 = 4.0 / (PI * std::pow(hh, 8.0));
        real sum = 0.0;
        const int R = (int)std::ceil(hh / step) + 1;
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                const real r2 = (dx * step) * (dx * step) + (dy * step) * (dy * step);
                if (r2 < hh * hh) { const real d = hh * hh - r2; sum += poly6 * d * d * d; }
            }
        if (sum > 1e-9) particleMass = restDensity / sum;
    }
    for (real y = 0.0; y <= h; y += step)
        for (real x = 0.0; x <= w; x += step)
            emit(origin + Vec2(x, y), Vec2());
}

void ParticleSystem::clear() {
    m_particles.clear();
    m_grid.clear();
}

void ParticleSystem::buildGrid() {
    m_grid.clear();
    const real inv = 1.0 / smoothingRadius;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        if (!m_particles[i].active) continue;
        const int gx = (int)std::floor(m_particles[i].position.x * inv);
        const int gy = (int)std::floor(m_particles[i].position.y * inv);
        m_grid[cellKey(gx, gy)].push_back((int)i);
    }
}

void ParticleSystem::computeDensity() {
    const real h = smoothingRadius;
    const real h2 = h * h;
    const real poly6 = 4.0 / (PI * std::pow(h, 8.0));   // 2D poly6
    const real inv = 1.0 / h;

    for (FluidParticle& p : m_particles) {
        if (!p.active) continue;
        real density = 0.0;
        const int gx = (int)std::floor(p.position.x * inv);
        const int gy = (int)std::floor(p.position.y * inv);
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const auto it = m_grid.find(cellKey(gx + dx, gy + dy));
                if (it == m_grid.end()) continue;
                for (int j : it->second) {
                    const Vec2 d = m_particles[(size_t)j].position - p.position;
                    const real r2 = d.lengthSq();
                    if (r2 < h2) {
                        const real diff = h2 - r2;
                        density += m_particles[(size_t)j].mass * poly6 * diff * diff * diff;
                    }
                }
            }
        }
        p.density = std::max(density, restDensity * 0.05);
        p.pressure = stiffness * (p.density - restDensity);
        if (p.pressure < 0.0) p.pressure *= 0.25;   // мягкое растяжение
    }
}

void ParticleSystem::computeForces() {
    const real h = smoothingRadius;
    const real spikyGrad = -30.0 / (PI * std::pow(h, 5.0));
    const real viscLap = 40.0 / (PI * std::pow(h, 5.0));
    const real inv = 1.0 / h;

    for (size_t i = 0; i < m_particles.size(); ++i) {
        FluidParticle& p = m_particles[i];
        if (!p.active) continue;
        Vec2 pressureForce, viscousForce, cohesion;
        real neighbours = 0.0;

        const int gx = (int)std::floor(p.position.x * inv);
        const int gy = (int)std::floor(p.position.y * inv);
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const auto it = m_grid.find(cellKey(gx + dx, gy + dy));
                if (it == m_grid.end()) continue;
                for (int jj : it->second) {
                    const size_t j = (size_t)jj;
                    if (j == i) continue;
                    FluidParticle& q = m_particles[j];
                    const Vec2 d = q.position - p.position;
                    const real r = d.length();
                    if (r >= h || r < 1e-9) continue;

                    const Vec2 dir = d / r;
                    const real diff = h - r;

                    pressureForce += dir * (-q.mass * (p.pressure + q.pressure) /
                                            (2.0 * q.density) * spikyGrad * diff * diff);
                    viscousForce += (q.velocity - p.velocity) * (viscosity * q.mass / q.density * viscLap * diff);
                    cohesion += dir * (surfaceTension * diff / h);
                    neighbours += 1.0;
                }
            }
        }
        p.force = pressureForce + viscousForce + cohesion + gravity * p.density;
        if (neighbours < 4.0) p.force -= cohesion;   // граница — без ложного притяжения
    }
}

void ParticleSystem::integrate(real dt) {
    const bool hasDomain = domain.min.x < domain.max.x;

    for (FluidParticle& p : m_particles) {
        if (!p.active) continue;
        const Vec2 accel = p.force / std::max((real)1e-6, p.density);
        p.velocity += accel * dt;
        p.position += p.velocity * dt;

        if (hasDomain) {
            if (p.position.x < domain.min.x) { p.position.x = domain.min.x; p.velocity.x *= -boundaryDamping; }
            if (p.position.x > domain.max.x) { p.position.x = domain.max.x; p.velocity.x *= -boundaryDamping; }
            if (p.position.y < domain.min.y) { p.position.y = domain.min.y; p.velocity.y *= -boundaryDamping; }
            if (p.position.y > domain.max.y) { p.position.y = domain.max.y; p.velocity.y *= -boundaryDamping; }
        }

        // Связь с твёрдыми телами: выталкивание и отдача импульса.
        if (coupleWithRigid && m_world) {
            RigidBody* hit = m_world->queryPoint(p.position);
            if (hit) {
                Vec2 dir = p.position - hit->position;
                real len = dir.length();
                if (len < 1e-9) { dir = Vec2(0.0, 1.0); len = 1e-9; } else dir /= len;
                p.position += dir * std::max((real)0.0, hit->boundingRadius - len);
                const Vec2 rel = p.velocity - hit->velocityAtPoint(p.position);
                const real vn = dot(rel, dir);
                if (vn < 0.0) {
                    const Vec2 impulse = dir * (-vn * p.mass * 1.2);
                    p.velocity += impulse / p.mass;
                    if (hit->isDynamic()) hit->applyImpulseAtPoint(impulse * -1.0, p.position);
                }
            }
        }
    }
}

void ParticleSystem::update(real dt) {
    if (dt <= 0.0 || m_particles.empty()) return;
    // ---------------------------------------------------------------- PBF step
    // Position based fluid (Macklin & Muller): density constraints instead of
    // explicit pressure forces. Stable at dt = 1/60 with 1-2 substeps.
    {
        const size_t n = m_particles.size();
        const real h = smoothingRadius;
        const real h2 = h * h;
        const real poly6 = 4.0 / (PI * std::pow(h, 8.0));
        const real gradCoef = -30.0 / (PI * std::pow(h, 5.0));
        const real rho0 = std::max((real)1e-6, restDensity);
        const real invRho0 = 1.0 / rho0;
        const real eps = 120.0;
        const int  iterations = 4;
        const real maxSpeed = 26.0;
        const real pad = h * 0.25;
        const bool hasDomain = domain.min.x < domain.max.x;

        std::vector<Vec2> prev(n), pred(n), delta(n);
        std::vector<real> lambda(n, 0.0), dens(n, 0.0);
        std::vector<int>  live;
        live.reserve(n);

        for (size_t i = 0; i < n; ++i) {
            FluidParticle& p = m_particles[i];
            prev[i] = p.position;
            pred[i] = p.position;
            if (!p.active) continue;
            if (!p.position.isFinite() || !p.velocity.isFinite()) {
                p.position = Vec2((domain.min.x + domain.max.x) * 0.5, (domain.min.y + domain.max.y) * 0.5);
                p.velocity = Vec2();
                prev[i] = pred[i] = p.position;
                continue;
            }
            p.velocity += gravity * dt;
            const real sp = p.velocity.length();
            if (sp > maxSpeed) p.velocity = p.velocity * (maxSpeed / sp);
            pred[i] = p.position + p.velocity * dt;
            if (hasDomain) {
                pred[i].x = clampr(pred[i].x, domain.min.x + pad, domain.max.x - pad);
                pred[i].y = clampr(pred[i].y, domain.min.y + pad, domain.max.y - pad);
            }
            live.push_back((int)i);
        }
        if (live.empty()) return;

        std::unordered_map<uint64_t, std::vector<int> > grid;
        const real invH = 1.0 / h;

        for (int iter = 0; iter < iterations; ++iter) {
            grid.clear();
            for (size_t k = 0; k < live.size(); ++k) {
                const size_t i = (size_t)live[k];
                const uint64_t key = ((uint64_t)(uint32_t)(int)std::floor(pred[i].x * invH) << 32) ^
                                     (uint64_t)(uint32_t)(int)std::floor(pred[i].y * invH);
                grid[key].push_back((int)i);
            }
            for (size_t k = 0; k < live.size(); ++k) {
                const size_t i = (size_t)live[k];
                const int gx = (int)std::floor(pred[i].x * invH);
                const int gy = (int)std::floor(pred[i].y * invH);
                real rho = 0.0, sumSq = 0.0;
                Vec2 gradSelf;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const uint64_t key = ((uint64_t)(uint32_t)(gx + dx) << 32) ^ (uint64_t)(uint32_t)(gy + dy);
                        std::unordered_map<uint64_t, std::vector<int> >::const_iterator it = grid.find(key);
                        if (it == grid.end()) continue;
                        for (size_t t = 0; t < it->second.size(); ++t) {
                            const size_t j = (size_t)it->second[t];
                            const Vec2 d = pred[i] - pred[j];
                            const real r2 = d.lengthSq();
                            if (r2 >= h2) continue;
                            const real diff = h2 - r2;
                            rho += particleMass * poly6 * diff * diff * diff;
                            if (j == i) continue;
                            const real r = std::sqrt(r2);
                            if (r < 1e-9) continue;
                            const Vec2 gw = (d / r) * (gradCoef * (h - r) * (h - r) * particleMass * invRho0);
                            gradSelf += gw;
                            sumSq += gw.lengthSq();
                        }
                    }
                dens[i] = rho;
                sumSq += gradSelf.lengthSq();
                const real C = clampr(rho / rho0 - 1.0, -0.25, 4.0);
                lambda[i] = -C / (sumSq + eps);
            }
            for (size_t k = 0; k < live.size(); ++k) {
                const size_t i = (size_t)live[k];
                const int gx = (int)std::floor(pred[i].x * invH);
                const int gy = (int)std::floor(pred[i].y * invH);
                Vec2 dp;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const uint64_t key = ((uint64_t)(uint32_t)(gx + dx) << 32) ^ (uint64_t)(uint32_t)(gy + dy);
                        std::unordered_map<uint64_t, std::vector<int> >::const_iterator it = grid.find(key);
                        if (it == grid.end()) continue;
                        for (size_t t = 0; t < it->second.size(); ++t) {
                            const size_t j = (size_t)it->second[t];
                            if (j == i) continue;
                            const Vec2 d = pred[i] - pred[j];
                            const real r2 = d.lengthSq();
                            if (r2 >= h2 || r2 < 1e-12) continue;
                            const real r = std::sqrt(r2);
                            const Vec2 gw = (d / r) * (gradCoef * (h - r) * (h - r) * particleMass);
                            dp += gw * ((lambda[i] + lambda[j]) * invRho0);
                        }
                    }
                const real dpLen = dp.length();
                const real limit = h * 0.30;
                if (dpLen > limit) dp = dp * (limit / dpLen);
                delta[i] = dp;
            }
            for (size_t k = 0; k < live.size(); ++k) {
                const size_t i = (size_t)live[k];
                pred[i] += delta[i];
                if (hasDomain) {
                    pred[i].x = clampr(pred[i].x, domain.min.x + pad, domain.max.x - pad);
                    pred[i].y = clampr(pred[i].y, domain.min.y + pad, domain.max.y - pad);
                }
            }
        }

        const real invDt = 1.0 / dt;
        for (size_t k = 0; k < live.size(); ++k) {
            const size_t i = (size_t)live[k];
            m_particles[i].velocity = (pred[i] - prev[i]) * invDt;
            m_particles[i].position = pred[i];
            m_particles[i].density = dens[i] > 0.0 ? dens[i] : rho0;
            m_particles[i].pressure = std::max((real)0.0, stiffness * (m_particles[i].density - rho0));
        }

        // XSPH viscosity: neighbouring water moves together instead of jittering.
        const real cvisc = clampr(viscosity * 0.02, 0.0, 0.55);
        if (cvisc > 0.0) {
            std::vector<Vec2> dv(n);
            for (size_t k = 0; k < live.size(); ++k) {
                const size_t i = (size_t)live[k];
                const int gx = (int)std::floor(pred[i].x * invH);
                const int gy = (int)std::floor(pred[i].y * invH);
                Vec2 acc;
                real wsum = 0.0;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const uint64_t key = ((uint64_t)(uint32_t)(gx + dx) << 32) ^ (uint64_t)(uint32_t)(gy + dy);
                        std::unordered_map<uint64_t, std::vector<int> >::const_iterator it = grid.find(key);
                        if (it == grid.end()) continue;
                        for (size_t t = 0; t < it->second.size(); ++t) {
                            const size_t j = (size_t)it->second[t];
                            if (j == i) continue;
                            const Vec2 d = pred[i] - pred[j];
                            const real r2 = d.lengthSq();
                            if (r2 >= h2) continue;
                            const real diff = h2 - r2;
                            const real w = poly6 * diff * diff * diff;
                            acc += (m_particles[j].velocity - m_particles[i].velocity) * w;
                            wsum += w;
                        }
                    }
                dv[i] = wsum > 1e-12 ? acc * (cvisc / wsum) : Vec2();
            }
            for (size_t k = 0; k < live.size(); ++k) m_particles[(size_t)live[k]].velocity += dv[(size_t)live[k]];
        }
        // Rigid bodies: push out along real shape geometry and trade impulses.
        if (coupleWithRigid && m_world) {
            for (size_t k = 0; k < live.size(); ++k) {
                FluidParticle& p = m_particles[(size_t)live[k]];
                RigidBody* hit = m_world->queryPoint(p.position);
                if (!hit) continue;
                Vec2 dir(0.0, 1.0);
                real depth = 0.0;
                bool inside = false;
                const std::vector<Vec2>& wv = hit->worldVertices;
                if (!wv.empty()) {
                    real best = -1e30;
                    Vec2 bestN(0.0, 1.0);
                    for (size_t e = 0; e < wv.size(); ++e) {
                        const Vec2 a = wv[e];
                        const Vec2 c = wv[(e + 1) % wv.size()];
                        const Vec2 ed = c - a;
                        const real elen = ed.length();
                        if (elen < 1e-12) continue;
                        Vec2 nrm(ed.y / elen, -ed.x / elen);
                        if (dot(nrm, (a + c) * 0.5 - hit->position) < 0.0) nrm = nrm * -1.0;
                        const real d = dot(p.position - a, nrm);
                        if (d > best) { best = d; bestN = nrm; }
                    }
                    if (best < 0.0) { inside = true; dir = bestN; depth = -best; }
                } else if (hit->shape.halfLength > 0.0) {
                    Vec2 a, c;
                    hit->shape.capsuleSegment(hit->transform(), a, c);
                    real tt = 0.0;
                    const Vec2 cp = closestPointOnSegment(p.position, a, c, &tt);
                    const Vec2 d = p.position - cp;
                    const real len = d.length();
                    if (len < hit->shape.radius) {
                        inside = true;
                        dir = len > 1e-9 ? d / len : Vec2(0.0, 1.0);
                        depth = hit->shape.radius - len;
                    }
                } else {
                    const Vec2 d = p.position - hit->position;
                    const real len = d.length();
                    if (len < hit->shape.radius) {
                        inside = true;
                        dir = len > 1e-9 ? d / len : Vec2(0.0, 1.0);
                        depth = hit->shape.radius - len;
                    }
                }
                if (!inside) continue;
                p.position += dir * std::min(depth, h);
                const Vec2 rel = p.velocity - hit->velocityAtPoint(p.position);
                const real vn = dot(rel, dir);
                if (vn < 0.0) {
                    const Vec2 impulse = dir * (-vn * p.mass * (1.0 + clampr(boundaryDamping, 0.0, 1.0)));
                    p.velocity += impulse / p.mass;
                    p.velocity -= (rel - dir * vn) * 0.25;
                    if (hit->isDynamic()) hit->applyImpulseAtPoint(impulse * -1.0, p.position);
                }
            }
        }
    }

    // Волны на поверхности: одномерное волновое уравнение по верхним частицам.
    const size_t columns = 128;
    if (m_waveHeight.size() != columns) {
        m_waveHeight.assign(columns, 0.0);
        m_waveVelocity.assign(columns, 0.0);
    }
    const bool hasDomain = domain.min.x < domain.max.x;
    if (hasDomain) {
        const real width = domain.max.x - domain.min.x;
        std::vector<real> target(columns, 0.0);
        std::vector<int> counts(columns, 0);
        for (const FluidParticle& p : m_particles) {
            const int c = (int)clampr((p.position.x - domain.min.x) / width * (real)(columns - 1), 0.0, (real)(columns - 1));
            target[(size_t)c] = std::max(target[(size_t)c], p.position.y);
            counts[(size_t)c]++;
        }
        const real c2 = 0.25;
        for (size_t i = 1; i + 1 < columns; ++i) {
            const real laplace = m_waveHeight[i - 1] + m_waveHeight[i + 1] - 2.0 * m_waveHeight[i];
            m_waveVelocity[i] += (c2 * laplace + (counts[i] ? (target[i] - m_waveHeight[i]) * 0.5 : -m_waveHeight[i] * 0.5)) * dt;
            m_waveVelocity[i] *= 0.98;
        }
        for (size_t i = 0; i < columns; ++i) m_waveHeight[i] += m_waveVelocity[i] * dt;
    }
}

real ParticleSystem::wetnessAt(const Vec2& p, real radius) const {
    if (m_particles.empty() || radius <= 0.0) return 0.0;
    const real r2 = radius * radius;
    int hits = 0;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        if (!m_particles[i].active) continue;
        if ((m_particles[i].position - p).lengthSq() < r2) { if (++hits >= 24) break; }
    }
    return clampr((real)hits / 10.0, 0.0, 1.0);
}

real ParticleSystem::surfaceHeight(real x) const {
    if (m_waveHeight.empty() || domain.min.x >= domain.max.x) return 0.0;
    const real width = domain.max.x - domain.min.x;
    const real f = clampr((x - domain.min.x) / width, 0.0, 1.0) * (real)(m_waveHeight.size() - 1);
    const size_t i = (size_t)f;
    const size_t j = std::min(i + 1, m_waveHeight.size() - 1);
    const real t = f - (real)i;
    return m_waveHeight[i] * (1.0 - t) + m_waveHeight[j] * t;
}

// ================================================================ body physics registry
BodyPhysics& BodyPhysicsRegistry::ref(BodyId id) { return m_map[id]; }

const BodyPhysics* BodyPhysicsRegistry::find(BodyId id) const {
    const auto it = m_map.find(id);
    return (it == m_map.end()) ? nullptr : &it->second;
}

void BodyPhysicsRegistry::remove(BodyId id) { m_map.erase(id); }

// ================================================================ fields
namespace {

// Квадродерево Барнса-Хата для гравитации N тел.
struct BarnesHutNode {
    AABB box;
    Vec2 centerOfMass;
    real mass = 0.0;
    int  children[4] = {-1, -1, -1, -1};
    int  bodyIndex = -1;
    bool leaf = true;
};

class BarnesHutTree {
public:
    void build(const std::vector<RigidBody*>& bodies, const AABB& bounds) {
        m_nodes.clear();
        m_bodies = &bodies;
        BarnesHutNode root;
        root.box = bounds;
        m_nodes.push_back(root);
        for (size_t i = 0; i < bodies.size(); ++i) {
            if (bodies[i]->mass <= 0.0) continue;
            insert(0, (int)i, 0);
        }
        computeMass(0);
    }

    Vec2 force(const RigidBody& b, real G, real theta) const {
        return forceRecursive(0, b, G, theta);
    }

    size_t nodeCount() const { return m_nodes.size(); }

private:
    int quadrant(const BarnesHutNode& n, const Vec2& p) const {
        const Vec2 c = n.box.center();
        return (p.x >= c.x ? 1 : 0) | (p.y >= c.y ? 2 : 0);
    }

    AABB childBox(const BarnesHutNode& n, int q) const {
        const Vec2 c = n.box.center();
        AABB box;
        box.min.x = (q & 1) ? c.x : n.box.min.x;
        box.max.x = (q & 1) ? n.box.max.x : c.x;
        box.min.y = (q & 2) ? c.y : n.box.min.y;
        box.max.y = (q & 2) ? n.box.max.y : c.y;
        return box;
    }

    void insert(int nodeIndex, int bodyIndex, int depth) {
        if (depth > 24) return;
        BarnesHutNode& node = m_nodes[(size_t)nodeIndex];
        const Vec2 p = (*m_bodies)[(size_t)bodyIndex]->position;

        if (node.leaf && node.bodyIndex < 0) {
            node.bodyIndex = bodyIndex;
            return;
        }
        if (node.leaf) {
            const int existing = node.bodyIndex;
            node.bodyIndex = -1;
            node.leaf = false;
            for (int q = 0; q < 4; ++q) {
                BarnesHutNode child;
                child.box = childBox(node, q);
                m_nodes.push_back(child);
                m_nodes[(size_t)nodeIndex].children[q] = (int)m_nodes.size() - 1;
            }
            const Vec2 ep = (*m_bodies)[(size_t)existing]->position;
            insert(m_nodes[(size_t)nodeIndex].children[quadrant(m_nodes[(size_t)nodeIndex], ep)], existing, depth + 1);
        }
        const int q = quadrant(m_nodes[(size_t)nodeIndex], p);
        insert(m_nodes[(size_t)nodeIndex].children[q], bodyIndex, depth + 1);
    }

    void computeMass(int nodeIndex) {
        BarnesHutNode& node = m_nodes[(size_t)nodeIndex];
        if (node.leaf) {
            if (node.bodyIndex >= 0) {
                const RigidBody* b = (*m_bodies)[(size_t)node.bodyIndex];
                node.mass = b->mass;
                node.centerOfMass = b->position;
            }
            return;
        }
        node.mass = 0.0;
        Vec2 weighted;
        for (int q = 0; q < 4; ++q) {
            const int c = node.children[q];
            if (c < 0) continue;
            computeMass(c);
            const BarnesHutNode& child = m_nodes[(size_t)c];
            node.mass += child.mass;
            weighted += child.centerOfMass * child.mass;
        }
        if (node.mass > 0.0) node.centerOfMass = weighted / node.mass;
    }

    Vec2 forceRecursive(int nodeIndex, const RigidBody& b, real G, real theta) const {
        const BarnesHutNode& node = m_nodes[(size_t)nodeIndex];
        if (node.mass <= 0.0) return Vec2();

        Vec2 d = node.centerOfMass - b.position;
        const real distSq = d.lengthSq() + 1e-6;
        const real dist = std::sqrt(distSq);
        const real size = node.box.max.x - node.box.min.x;

        if (node.leaf || (size / dist) < theta) {
            if (dist < 1e-4) return Vec2();
            const real magnitude = G * b.mass * node.mass / distSq;
            return (d / dist) * magnitude;
        }
        Vec2 total;
        for (int q = 0; q < 4; ++q) {
            const int c = node.children[q];
            if (c >= 0) total += forceRecursive(c, b, G, theta);
        }
        return total;
    }

    std::vector<BarnesHutNode>      m_nodes;
    const std::vector<RigidBody*>*  m_bodies = nullptr;
};

} // namespace

void FieldSystem::applyNBody() {
    std::vector<RigidBody*>& bodies = m_world->bodies();
    if (bodies.size() < 2) return;

    AABB bounds;
    bounds.reset();
    for (RigidBody* b : bodies) bounds.add(b->position);
    const Vec2 extent = bounds.extents();
    const real side = std::max((real)1.0, std::max(extent.x, extent.y) * 2.2);
    const Vec2 c = bounds.center();
    bounds.min = c - Vec2(side, side) * 0.5;
    bounds.max = c + Vec2(side, side) * 0.5;

    BarnesHutTree tree;
    tree.build(bodies, bounds);
    m_nodes = (int)tree.nodeCount();

    for (RigidBody* b : bodies) {
        if (!b->isDynamic()) continue;
        addForce(*b, tree.force(*b, gravitationalConstant, nbodyTheta));
    }
}

void FieldSystem::applyCharges() {
    if (!registry) return;
    std::vector<RigidBody*>& bodies = m_world->bodies();

    for (size_t i = 0; i < bodies.size(); ++i) {
        const BodyPhysics* pi = registry->find(bodies[i]->id);
        if (!pi) continue;
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            const BodyPhysics* pj = registry->find(bodies[j]->id);
            if (!pj) continue;

            Vec2 d = bodies[j]->position - bodies[i]->position;
            const real distSq = d.lengthSq() + 1e-6;
            const real dist = std::sqrt(distSq);
            const Vec2 dir = d / dist;

            if (flags & FIELD_ELECTROSTATIC) {
                const real qi = pi->charge + pi->accumulatedCharge;
                const real qj = pj->charge + pj->accumulatedCharge;
                if (qi != 0.0 && qj != 0.0) {
                    const Vec2 f = dir * (coulombConstant * qi * qj / distSq);
                    addForce(*bodies[i], f * -1.0);
                    addForce(*bodies[j], f);
                }
            }
            if (flags & FIELD_MAGNETIC) {
                const real mi = pi->magneticMoment, mj = pj->magneticMoment;
                if (mi != 0.0 && mj != 0.0) {
                    // Диполь-дипольное приближение: F ~ 3*mu0*m1*m2 / (2*pi*r^4)
                    const real r4 = distSq * distSq;
                    const Vec2 f = dir * (3.0 * magneticConstant * mi * mj / std::max((real)1e-6, r4));
                    addForce(*bodies[i], f);
                    addForce(*bodies[j], f * -1.0);
                }
            }
        }
    }
}

void FieldSystem::applyWind(real t) {
    for (RigidBody* b : m_world->bodies()) {
        if (!b->isDynamic()) continue;
        const real nx = fractalNoise(b->position.x * windScale, b->position.y * windScale + t * 0.3, 4, 0.5, 11);
        const real ny = fractalNoise(b->position.x * windScale + 31.7, b->position.y * windScale - t * 0.25, 4, 0.5, 29);
        const Vec2 gust = Vec2(nx, ny) * windTurbulence;
        const Vec2 flow = windBase + gust;
        if (!b->position.isFinite() || !b->velocity.isFinite()) continue;
        const Vec2 rel = flow - b->velocity;
        const real area = b->boundingRadius * 2.0;
        Vec2 drag = rel * (rel.length() * 0.5 * 1.204 * area * 0.4);
        // Ограничение ускорения: лёгкие звенья цепей иначе раскачиваются до расходимости.
        const real maxForce = maxWindAcceleration * b->mass;
        const real fmag = drag.length();
        if (fmag > maxForce && fmag > 0.0) drag *= maxForce / fmag;
        if (drag.isFinite()) addForce(*b, drag);
    }
}

void FieldSystem::applyFrame() {
    if (frameOmega == 0.0) return;
    for (RigidBody* b : m_world->bodies()) {
        if (!b->isDynamic()) continue;
        const Vec2 r = b->position - frameCenter;
        if (flags & FIELD_CENTRIFUGAL) addForce(*b, r * (b->mass * frameOmega * frameOmega));
        if (flags & FIELD_CORIOLIS) {
            // F = -2 m (omega x v), в 2D: omega x v = (-omega*vy, omega*vx)
            const Vec2 coriolis(2.0 * b->mass * frameOmega * b->velocity.y,
                                -2.0 * b->mass * frameOmega * b->velocity.x);
            addForce(*b, coriolis);
        }
    }
}

void FieldSystem::applyTidal() {
    for (RigidBody* b : m_world->bodies()) {
        if (!b->isDynamic()) continue;
        for (const PointSource& s : tidalSources) {
            Vec2 d = s.position - b->position;
            const real dist = std::max((real)1e-3, d.length());
            const real r = std::max((real)1e-3, b->boundingRadius);
            // Приливная сила ~ 2 G M m r / d^3, растягивает вдоль оси к источнику.
            const real magnitude = 2.0 * gravitationalConstant * s.strength * b->mass * r / (dist * dist * dist);
            addForce(*b, (d / dist) * magnitude);
            b->applyTorque(magnitude * r * 0.05);
        }
    }
}

void FieldSystem::applyLight() {
    for (RigidBody* b : m_world->bodies()) {
        if (!b->isDynamic()) continue;
        for (const PointSource& s : lightSources) {
            Vec2 d = b->position - s.position;
            const real dist = std::max((real)1e-3, d.length());
            const real area = b->boundingRadius * 2.0;
            // P = L / (4 pi d^2 c) * A
            const real pressure = s.strength / (4.0 * PI * dist * dist * 2.998e8) * area;
            addForce(*b, (d / dist) * pressure);
        }
    }
}

void FieldSystem::update(real dt) {
    if (!m_world || flags == 0) return;
    m_time += dt;

    if (flags & FIELD_NBODY) applyNBody();
    if (flags & (FIELD_ELECTROSTATIC | FIELD_MAGNETIC)) applyCharges();
    if (flags & FIELD_WIND) applyWind(m_time);
    if (flags & (FIELD_CORIOLIS | FIELD_CENTRIFUGAL)) applyFrame();
    if (flags & FIELD_TIDAL) applyTidal();
    if (flags & FIELD_LIGHT) applyLight();
}

// ================================================================ thermal
void ThermalSystem::update(real dt) {
    if (!m_world || !registry || dt <= 0.0) return;
    m_molten = 0;

    // 1. Контактная теплопроводность и нагрев трением.
    for (const Manifold& mf : m_world->manifolds()) {
        if (!mf.touching || mf.count == 0 || !mf.bodyA || !mf.bodyB) continue;
        BodyPhysics& pa = registry->ref(mf.bodyA->id);
        BodyPhysics& pb = registry->ref(mf.bodyB->id);

        const real k = contactCoupling * 2.0 * pa.conductivity * pb.conductivity /
                       std::max((real)1e-6, pa.conductivity + pb.conductivity);
        const real deltaT = pb.temperature - pa.temperature;
        const real flow = k * deltaT * dt * 0.001;

        pa.temperature += flow / std::max((real)1e-6, pa.heatCapacity * std::max((real)1e-6, mf.bodyA->mass)) * 1000.0;
        pb.temperature -= flow / std::max((real)1e-6, pb.heatCapacity * std::max((real)1e-6, mf.bodyB->mass)) * 1000.0;

        // Трение -> тепло; пьезоэффект -> заряд.
        real tangential = 0.0, normalImp = 0.0;
        for (int i = 0; i < mf.count; ++i) {
            tangential += std::fabs(mf.points[i].tangentImpulse);
            normalImp += std::fabs(mf.points[i].normalImpulse);
        }
        const Vec2 rel = mf.bodyB->velocity - mf.bodyA->velocity;
        const real heat = frictionHeating * tangential * rel.length();
        pa.temperature += heat / std::max((real)1e-6, pa.heatCapacity) * 50.0;
        pb.temperature += heat / std::max((real)1e-6, pb.heatCapacity) * 50.0;

        if (pa.piezoCoefficient != 0.0) pa.accumulatedCharge += pa.piezoCoefficient * normalImp / dt * 1e-6;
        if (pb.piezoCoefficient != 0.0) pb.accumulatedCharge += pb.piezoCoefficient * normalImp / dt * 1e-6;
    }

    // 2. Обмен со средой, расширение, фазовые переходы.
    for (auto& kv : registry->all()) {
        RigidBody* b = m_world->body(kv.first);
        if (!b) continue;
        BodyPhysics& p = kv.second;

        const real toAmbient = (ambientTemperature - p.temperature) * ambientCoupling * dt;
        const real radiative = -p.emissivity * 5.67e-8 *
                               (std::pow(p.temperature, 4.0) - std::pow(ambientTemperature, 4.0)) * dt * 1e-3;
        p.temperature += toAmbient + clampr(radiative, -50.0, 50.0);
        p.temperature = clampr(p.temperature, 1.0, 6000.0);
        p.accumulatedCharge *= std::max(0.0, 1.0 - dt * 0.2);

        if (thermalExpansion) {
            const real targetScale = 1.0 + p.expansion * (p.temperature - 293.0);
            if (std::fabs(targetScale - p.baseScale) > 1e-4 && b->shape.type == ShapeType::Circle) {
                const real factor = targetScale / std::max((real)1e-6, p.baseScale);
                b->shape.radius *= factor;
                b->shape.updateDerived();
                b->computeMassFromShape();
                b->updateVertices();
                b->updateAABB();
                p.baseScale = targetScale;
            }
        }

        if (temperatureDependentCoefficients) {
            const real t = clampr((p.temperature - 200.0) / 1200.0, 0.0, 1.0);
            b->material.restitution = clampr(b->material.restitution * (1.0 - 0.5 * t) + 0.02, 0.0, 0.99);
            b->material.staticFriction = clampr(b->material.staticFriction * (1.0 - 0.4 * t) + 0.02, 0.01, 2.0);
            b->material.dynamicFriction = clampr(b->material.dynamicFriction * (1.0 - 0.4 * t) + 0.02, 0.01, 2.0);
        }

        if (phaseTransitions) {
            const bool moltenNow = p.temperature >= p.meltingPoint;
            if (moltenNow != p.molten) {
                p.molten = moltenNow;
                if (moltenNow) {
                    b->material.restitution = 0.02;
                    b->material.staticFriction = 0.05;
                    b->material.dynamicFriction = 0.03;
                    b->material.softness = 0.9;
                }
            }
            if (p.molten) ++m_molten;
        }
    }
}

// ================================================================ audio
void AudioSystem::addEvent(const AudioEvent& e) { m_events.push_back(e); }

void AudioSystem::clear() {
    m_events.clear();
    m_buffer.clear();
}

void AudioSystem::update(real dt) {
    if (!m_world) return;
    m_time += dt;

    // Акустические волны при ударах: частота по размеру тела, амплитуда по импульсу.
    for (const Manifold& mf : m_world->manifolds()) {
        if (!mf.touching || mf.count == 0) continue;
        real impulse = 0.0;
        for (int i = 0; i < mf.count; ++i) impulse += mf.points[i].normalImpulse;
        if (impulse < impulseThreshold) continue;

        AudioEvent e;
        e.time = m_time;
        e.position = mf.points[0].point;
        const real size = std::max((real)0.05, 0.5 * (mf.bodyA->boundingRadius + mf.bodyB->boundingRadius));
        e.frequency = clampr(320.0 / size, 60.0, 4000.0);
        e.amplitude = clampr(impulse * 0.05, 0.0, 1.0);
        e.duration = clampr(0.05 + impulse * 0.01, 0.02, 0.6);

        // Эффект Доплера относительно слушателя.
        Vec2 toListener = listener - e.position;
        const real dist = std::max((real)1e-3, toListener.length());
        const Vec2 dir = toListener / dist;
        const Vec2 sourceVel = mf.bodyA->velocity;
        const real vs = dot(sourceVel, dir);
        const real vl = dot(listenerVelocity, dir);
        e.doppler = clampr((speedOfSound + vl) / std::max((real)1.0, speedOfSound - vs), 0.5, 2.0);
        e.frequency *= e.doppler;
        e.amplitude *= masterGain / (1.0 + dist * 0.15);

        m_events.push_back(e);
        if (m_events.size() > 4096) m_events.erase(m_events.begin());
    }

    // Синтез аудиобуфера текущего кадра.
    const size_t frames = (size_t)std::max(1.0, (double)sampleRate * dt);
    const size_t base = m_buffer.size();
    m_buffer.resize(base + frames, 0.0f);

    for (const AudioEvent& e : m_events) {
        const real age = m_time - e.time;
        if (age < 0.0 || age > e.duration) continue;
        for (size_t i = 0; i < frames; ++i) {
            const real t = age + (real)i / (real)sampleRate;
            if (t > e.duration) break;
            const real envelope = std::exp(-4.0 * t / e.duration);
            const real sample = std::sin(2.0 * PI * e.frequency * t) * e.amplitude * envelope;
            m_buffer[base + i] += (float)sample;
        }
    }
    for (size_t i = base; i < m_buffer.size(); ++i)
        m_buffer[i] = (float)clampr(m_buffer[i], -1.0, 1.0);

    if (m_buffer.size() > (size_t)sampleRate * 30) {
        m_buffer.erase(m_buffer.begin(), m_buffer.begin() + (long)(m_buffer.size() - (size_t)sampleRate * 30));
    }
}

bool AudioSystem::writeWav(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f) { PHYS2D_ERROR(ErrorCode::IoFailure, path); return false; }

    const uint32_t dataBytes = (uint32_t)(m_buffer.size() * 2);
    const uint32_t riffSize = 36 + dataBytes;
    const uint16_t channels = 1, bits = 16;
    const uint32_t rate = (uint32_t)sampleRate;
    const uint32_t byteRate = rate * channels * bits / 8;
    const uint16_t blockAlign = (uint16_t)(channels * bits / 8);
    const uint16_t format = 1;
    const uint32_t fmtSize = 16;

    auto w32 = [&](uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); };
    auto w16 = [&](uint16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); };

    f.write("RIFF", 4); w32(riffSize); f.write("WAVE", 4);
    f.write("fmt ", 4); w32(fmtSize); w16(format); w16(channels);
    w32(rate); w32(byteRate); w16(blockAlign); w16(bits);
    f.write("data", 4); w32(dataBytes);

    for (float s : m_buffer) {
        const int16_t v = (int16_t)std::lround(clampr((real)s, -1.0, 1.0) * 32767.0);
        f.write(reinterpret_cast<const char*>(&v), 2);
    }
    return (bool)f;
}

} // namespace phys2d
