// phys2d v2 - NURBS, terrain, BSP, BVH, OBJ import, engine export, inertia, integrators.
#include "phys2d/Extras.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace phys2d {

// ================================================================ NURBS
std::vector<Vec2> sampleNurbs(const std::vector<Vec2>& controlPoints, const std::vector<real>& weights,
                              int degree, int samples) {
    std::vector<Vec2> out;
    const int n = (int)controlPoints.size();
    if (n < 2) return out;

    const int p = std::max(1, std::min(degree, n - 1));
    const int knotCount = n + p + 1;
    std::vector<real> knots((size_t)knotCount);
    for (int i = 0; i < knotCount; ++i) {
        if (i <= p) knots[(size_t)i] = 0.0;
        else if (i >= n) knots[(size_t)i] = 1.0;
        else knots[(size_t)i] = (real)(i - p) / (real)(n - p);
    }

    std::vector<real> w = weights;
    w.resize((size_t)n, 1.0);

    // Рекурсия Кокса-де Бура для базисных функций.
    std::function<real(int, int, real)> basis = [&](int i, int k, real t) -> real {
        if (k == 0) {
            const bool last = (i + 1 == n) && (t >= 1.0 - 1e-12);
            return ((t >= knots[(size_t)i] && t < knots[(size_t)i + 1]) || last) ? 1.0 : 0.0;
        }
        real left = 0.0, right = 0.0;
        const real d1 = knots[(size_t)(i + k)] - knots[(size_t)i];
        const real d2 = knots[(size_t)(i + k + 1)] - knots[(size_t)(i + 1)];
        if (d1 > 1e-12) left = (t - knots[(size_t)i]) / d1 * basis(i, k - 1, t);
        if (d2 > 1e-12) right = (knots[(size_t)(i + k + 1)] - t) / d2 * basis(i + 1, k - 1, t);
        return left + right;
    };

    const int count = std::max(2, samples);
    for (int s = 0; s < count; ++s) {
        const real t = (real)s / (real)(count - 1);
        Vec2 numerator;
        real denominator = 0.0;
        for (int i = 0; i < n; ++i) {
            const real b = basis(i, p, t) * w[(size_t)i];
            numerator += controlPoints[(size_t)i] * b;
            denominator += b;
        }
        out.push_back(denominator > 1e-12 ? numerator / denominator : controlPoints.back());
    }
    return out;
}

// ================================================================ terrain
Terrain buildTerrain(World& w, const std::vector<real>& heights, real x0, real dx, real thickness) {
    Terrain terrain;
    if (heights.size() < 2) return terrain;

    for (size_t i = 0; i < heights.size(); ++i)
        terrain.points.push_back(Vec2(x0 + dx * (real)i, heights[i]));

    for (size_t i = 0; i + 1 < terrain.points.size(); ++i) {
        const Vec2 a = terrain.points[i], b = terrain.points[i + 1];
        const Vec2 d = b - a;
        const real len = std::max((real)1e-3, d.length());

        BodyDef def;
        def.type = BodyType::Static;
        def.shape = Shape::box(len, thickness);
        def.position = (a + b) * 0.5 - Vec2(0.0, thickness * 0.5);
        def.angle = std::atan2(d.y, d.x);
        def.material.staticFriction = 0.85;
        def.material.dynamicFriction = 0.7;
        def.material.restitution = 0.1;
        def.name = "terrain";
        terrain.segments.push_back(w.createBody(def));
    }
    return terrain;
}

Terrain buildNoiseTerrain(World& w, real x0, real x1, real dx, real amplitude, uint32_t seed) {
    std::vector<real> heights;
    const real step = std::max((real)1e-3, dx);
    for (real x = x0; x <= x1; x += step)
        heights.push_back(fractalNoise(x * 0.08, 0.0, 5, 0.55, seed) * amplitude);
    return buildTerrain(w, heights, x0, step);
}

// ================================================================ BSP
void BspTree::build(const std::vector<std::pair<Vec2, Vec2>>& segments) {
    m_nodes.clear();
    if (segments.empty()) return;
    buildRecursive(segments, 0);
}

int BspTree::buildRecursive(std::vector<std::pair<Vec2, Vec2>> segs, int depth) {
    if (segs.empty() || depth > 24) {
        Node leaf;
        leaf.leaf = true;
        leaf.solid = segs.empty() ? false : true;
        m_nodes.push_back(leaf);
        return (int)m_nodes.size() - 1;
    }

    const std::pair<Vec2, Vec2> splitter = segs.front();
    Node node;
    node.point = splitter.first;
    node.normal = (splitter.second - splitter.first).perp().normalized();
    node.leaf = false;
    m_nodes.push_back(node);
    const int self = (int)m_nodes.size() - 1;

    std::vector<std::pair<Vec2, Vec2>> front, back;
    for (size_t i = 1; i < segs.size(); ++i) {
        const real da = dot(segs[i].first - node.point, node.normal);
        const real db = dot(segs[i].second - node.point, node.normal);
        if (da >= 0.0 && db >= 0.0) front.push_back(segs[i]);
        else if (da < 0.0 && db < 0.0) back.push_back(segs[i]);
        else { front.push_back(segs[i]); back.push_back(segs[i]); }
    }

    const int f = buildRecursive(front, depth + 1);
    const int b = buildRecursive(back, depth + 1);
    m_nodes[(size_t)self].front = f;
    m_nodes[(size_t)self].back = b;
    return self;
}

bool BspTree::pointSolid(const Vec2& p) const {
    if (m_nodes.empty()) return false;
    int index = 0;
    int guard = 0;
    while (index >= 0 && index < (int)m_nodes.size() && guard++ < 64) {
        const Node& n = m_nodes[(size_t)index];
        if (n.leaf) return n.solid;
        index = (dot(p - n.point, n.normal) >= 0.0) ? n.front : n.back;
    }
    return false;
}

int BspTree::depth() const {
    if (m_nodes.empty()) return 0;
    std::function<int(int)> rec = [&](int i) -> int {
        if (i < 0 || i >= (int)m_nodes.size()) return 0;
        const Node& n = m_nodes[(size_t)i];
        if (n.leaf) return 1;
        return 1 + std::max(rec(n.front), rec(n.back));
    };
    return rec(0);
}

// ================================================================ BVH (shaft-оптимизация)
void Bvh::build(const std::vector<RigidBody*>& bodies) {
    m_nodes.clear();
    m_bodies = bodies;
    m_shaftCost = 0.0;
    if (bodies.empty()) return;

    std::vector<int> indices(bodies.size());
    for (size_t i = 0; i < indices.size(); ++i) indices[i] = (int)i;
    build(indices, 0, (int)indices.size());
}

int Bvh::build(std::vector<int>& indices, int begin, int end) {
    Node node;
    node.box.reset();
    for (int i = begin; i < end; ++i) node.box.combine(m_bodies[(size_t)indices[(size_t)i]]->fatAABB);

    if (end - begin <= 1) {
        node.body = indices[(size_t)begin];
        m_nodes.push_back(node);
        return (int)m_nodes.size() - 1;
    }

    // Разбиение по длинной оси; shaft-оценка стоимости по SAH.
    const Vec2 extent = node.box.extents();
    const bool splitX = extent.x >= extent.y;
    std::sort(indices.begin() + begin, indices.begin() + end, [&](int a, int b) {
        const Vec2 ca = m_bodies[(size_t)a]->fatAABB.center();
        const Vec2 cb = m_bodies[(size_t)b]->fatAABB.center();
        return splitX ? (ca.x < cb.x) : (ca.y < cb.y);
    });

    const int mid = (begin + end) / 2;
    m_nodes.push_back(node);
    const int self = (int)m_nodes.size() - 1;

    const int left = build(indices, begin, mid);
    const int right = build(indices, mid, end);
    m_nodes[(size_t)self].left = left;
    m_nodes[(size_t)self].right = right;

    // Стоимость «шахты»: площадь пересечения поддеревьев относительно родителя.
    const real parent = std::max((real)1e-9, m_nodes[(size_t)self].box.area());
    const real childSum = m_nodes[(size_t)left].box.area() + m_nodes[(size_t)right].box.area();
    m_shaftCost += childSum / parent;
    return self;
}

void Bvh::query(const AABB& box, std::vector<int>& out) const {
    if (m_nodes.empty()) return;
    std::vector<int> stack{0};
    while (!stack.empty()) {
        const int i = stack.back();
        stack.pop_back();
        const Node& n = m_nodes[(size_t)i];
        if (!n.box.overlaps(box)) continue;
        if (n.body >= 0) { out.push_back(n.body); continue; }
        if (n.left >= 0) stack.push_back(n.left);
        if (n.right >= 0) stack.push_back(n.right);
    }
}

int Bvh::raycast(const Vec2& p1, const Vec2& p2, std::vector<int>& out) const {
    AABB box;
    box.reset();
    box.add(p1);
    box.add(p2);
    query(box, out);
    return (int)out.size();
}

int Bvh::height() const {
    if (m_nodes.empty()) return 0;
    std::function<int(int)> rec = [&](int i) -> int {
        if (i < 0) return 0;
        const Node& n = m_nodes[(size_t)i];
        if (n.body >= 0) return 1;
        return 1 + std::max(rec(n.left), rec(n.right));
    };
    return rec(0);
}

// ================================================================ OBJ import
bool importObj(const std::string& path, ObjMesh& out) {
    std::ifstream f(path);
    if (!f) { PHYS2D_ERROR(ErrorCode::IoFailure, path); return false; }

    out.vertices.clear();
    out.faces.clear();
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() < 2) continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v") {
            double x = 0.0, y = 0.0, z = 0.0;
            ss >> x >> y >> z;
            out.vertices.push_back(Vec2((real)x, (real)y));   // проекция на XY
        } else if (tag == "f") {
            std::vector<int> face;
            std::string token;
            while (ss >> token) {
                const size_t slash = token.find('/');
                if (slash != std::string::npos) token = token.substr(0, slash);
                if (token.empty()) continue;
                int idx = std::atoi(token.c_str());
                if (idx < 0) idx = (int)out.vertices.size() + idx + 1;
                if (idx >= 1) face.push_back(idx - 1);
            }
            if (face.size() >= 3) out.faces.push_back(face);
        }
    }
    return !out.vertices.empty();
}

std::vector<Shape> shapesFromObj(const ObjMesh& mesh, real scale) {
    std::vector<Shape> shapes;
    for (const std::vector<int>& face : mesh.faces) {
        std::vector<Vec2> poly;
        for (int idx : face) {
            if (idx < 0 || (size_t)idx >= mesh.vertices.size()) continue;
            poly.push_back(mesh.vertices[(size_t)idx] * scale);
        }
        if (poly.size() < 3) continue;

        for (const Shape& s : shapesFromConcave(poly)) shapes.push_back(s);
    }
    return shapes;
}

// ================================================================ export
namespace {

const char* shapeTypeName(ShapeType t) {
    switch (t) {
        case ShapeType::Circle: return "circle";
        case ShapeType::Capsule: return "capsule";
        default: return "polygon";
    }
}

const char* bodyTypeName(BodyType t) {
    switch (t) {
        case BodyType::Static: return "static";
        case BodyType::Kinematic: return "kinematic";
        default: return "dynamic";
    }
}

} // namespace

bool exportBox2D(const World& world, const std::string& path) {
    std::ofstream f(path);
    if (!f) { PHYS2D_ERROR(ErrorCode::IoFailure, path); return false; }

    World& w = const_cast<World&>(world);
    f << "{\n  \"gravity\": { \"x\": " << w.gravity().x << ", \"y\": " << w.gravity().y << " },\n";
    f << "  \"allowSleep\": true,\n  \"positionIterations\": " << w.config().solver.positionIterations << ",\n";
    f << "  \"velocityIterations\": " << w.config().solver.velocityIterations << ",\n  \"body\": [\n";

    const std::vector<RigidBody*>& bodies = w.bodies();
    for (size_t i = 0; i < bodies.size(); ++i) {
        const RigidBody* b = bodies[i];
        f << "    {\n      \"name\": \"" << (b->name.empty() ? "body" : b->name) << "\",\n";
        f << "      \"type\": " << (b->isStatic() ? 0 : (b->isDynamic() ? 2 : 1)) << ",\n";
        f << "      \"angle\": " << b->angle << ",\n";
        f << "      \"angularVelocity\": " << b->angularVelocity << ",\n";
        f << "      \"position\": { \"x\": " << b->position.x << ", \"y\": " << b->position.y << " },\n";
        f << "      \"linearVelocity\": { \"x\": " << b->velocity.x << ", \"y\": " << b->velocity.y << " },\n";
        f << "      \"fixture\": [\n        {\n";
        f << "          \"density\": " << b->material.density << ",\n";
        f << "          \"friction\": " << b->material.dynamicFriction << ",\n";
        f << "          \"restitution\": " << b->material.restitution << ",\n";

        if (b->shape.type == ShapeType::Circle) {
            f << "          \"circle\": { \"radius\": " << b->shape.radius
              << ", \"center\": { \"x\": 0, \"y\": 0 } }\n";
        } else {
            f << "          \"polygon\": { \"vertices\": { \"x\": [";
            const std::vector<Vec2>& verts = b->shape.vertices;
            for (size_t v = 0; v < verts.size(); ++v) f << (v ? ", " : "") << verts[v].x;
            f << "], \"y\": [";
            for (size_t v = 0; v < verts.size(); ++v) f << (v ? ", " : "") << verts[v].y;
            f << "] } }\n";
        }
        f << "        }\n      ]\n    }" << (i + 1 < bodies.size() ? "," : "") << "\n";
    }
    f << "  ]\n}\n";
    return (bool)f;
}

bool exportBullet(const World& world, const std::string& path) {
    std::ofstream f(path);
    if (!f) { PHYS2D_ERROR(ErrorCode::IoFailure, path); return false; }

    World& w = const_cast<World&>(world);
    f << "# phys2d -> bullet text dump v1\n";
    f << "gravity " << w.gravity().x << " " << w.gravity().y << " 0\n";

    for (const RigidBody* b : w.bodies()) {
        f << "rigidbody " << b->id << " " << bodyTypeName(b->type) << "\n";
        f << "  mass " << b->mass << "\n";
        f << "  inertia " << b->inertia << "\n";
        f << "  origin " << b->position.x << " " << b->position.y << " 0\n";
        f << "  rotation 0 0 " << std::sin(b->angle * 0.5) << " " << std::cos(b->angle * 0.5) << "\n";
        f << "  linvel " << b->velocity.x << " " << b->velocity.y << " 0\n";
        f << "  angvel 0 0 " << b->angularVelocity << "\n";
        f << "  friction " << b->material.dynamicFriction << "\n";
        f << "  restitution " << b->material.restitution << "\n";
        f << "  shape " << shapeTypeName(b->shape.type);

        if (b->shape.type == ShapeType::Circle) {
            f << " " << b->shape.radius << "\n";
        } else if (b->shape.type == ShapeType::Capsule) {
            f << " " << b->shape.radius << " " << b->shape.halfLength * 2.0 << "\n";
        } else {
            f << " " << b->shape.vertices.size();
            for (const Vec2& v : b->shape.vertices) f << " " << v.x << " " << v.y << " 0";
            f << "\n";
        }
    }
    return (bool)f;
}

// ================================================================ тензоры высших порядков
HigherOrderInertia computeHigherOrderInertia(const Shape& s) {
    HigherOrderInertia h;

    if (s.type == ShapeType::Circle) {
        const real r = s.radius;
        h.m0 = PI * r * r;
        h.mxx = h.myy = PI * r * r * r * r * 0.25;
        h.m4 = PI * std::pow(r, 6.0) / 8.0;
        return h;
    }

    const std::vector<Vec2>& v = s.vertices;
    if (v.size() < 3) return h;

    // Интегрирование моментов по треугольникам от начала координат.
    for (size_t i = 0; i < v.size(); ++i) {
        const Vec2& p0 = v[i];
        const Vec2& p1 = v[(i + 1) % v.size()];
        const real d = cross(p0, p1);
        const real area = d * 0.5;

        h.m0 += area;
        h.mx += area * (p0.x + p1.x) / 3.0;
        h.my += area * (p0.y + p1.y) / 3.0;

        h.mxx += d * (p0.x * p0.x + p0.x * p1.x + p1.x * p1.x) / 12.0;
        h.myy += d * (p0.y * p0.y + p0.y * p1.y + p1.y * p1.y) / 12.0;
        h.mxy += d * (2.0 * p0.x * p0.y + p0.x * p1.y + p1.x * p0.y + 2.0 * p1.x * p1.y) / 24.0;

        h.mxxx += d * (std::pow(p0.x, 3.0) + std::pow(p0.x, 2.0) * p1.x +
                       p0.x * std::pow(p1.x, 2.0) + std::pow(p1.x, 3.0)) / 20.0;
        h.myyy += d * (std::pow(p0.y, 3.0) + std::pow(p0.y, 2.0) * p1.y +
                       p0.y * std::pow(p1.y, 2.0) + std::pow(p1.y, 3.0)) / 20.0;
        h.mxxy += d * (p0.x * p0.x * (3.0 * p0.y + p1.y) + 2.0 * p0.x * p1.x * (p0.y + p1.y) +
                       p1.x * p1.x * (p0.y + 3.0 * p1.y)) / 60.0;
        h.mxyy += d * (p0.y * p0.y * (3.0 * p0.x + p1.x) + 2.0 * p0.y * p1.y * (p0.x + p1.x) +
                       p1.y * p1.y * (p0.x + 3.0 * p1.x)) / 60.0;
    }
    h.m4 = h.mxx * h.mxx + h.myy * h.myy + 2.0 * h.mxy * h.mxy;
    return h;
}

// ================================================================ пересчёт массы
void rebuildMass(RigidBody& b, real density) {
    if (density > 0.0) b.material.density = density;
    b.shape.updateDerived();
    b.computeMassFromShape();
    b.updateVertices();
    b.updateAABB();
}

void setShape(RigidBody& b, const Shape& s, real density) {
    b.shape = s;
    b.shape.updateDerived();
    b.localVertices = b.shape.vertices;
    b.boundingRadius = b.shape.boundingRadius;
    rebuildMass(b, density);
}

// ================================================================ интеграторы
namespace {

struct WorldState {
    std::vector<BodyId> ids;
    std::vector<Vec2>   positions, velocities;
    std::vector<real>   angles, omegas;
};

WorldState captureState(World& w) {
    WorldState s;
    for (RigidBody* b : w.bodies()) {
        s.ids.push_back(b->id);
        s.positions.push_back(b->position);
        s.velocities.push_back(b->velocity);
        s.angles.push_back(b->angle);
        s.omegas.push_back(b->angularVelocity);
    }
    return s;
}

void restoreState(World& w, const WorldState& s) {
    for (size_t i = 0; i < s.ids.size(); ++i) {
        RigidBody* b = w.body(s.ids[i]);
        if (!b) continue;
        b->position = s.positions[i];
        b->prevPosition = s.positions[i];
        b->velocity = s.velocities[i];
        b->angle = s.angles[i];
        b->prevAngle = s.angles[i];
        b->angularVelocity = s.omegas[i];
        b->updateVertices();
        b->updateAABB();
    }
}

real stateDifference(World& w, const WorldState& s) {
    real err = 0.0;
    for (size_t i = 0; i < s.ids.size(); ++i) {
        const RigidBody* b = w.body(s.ids[i]);
        if (!b) continue;
        err = std::max(err, (b->position - s.positions[i]).length());
        err = std::max(err, std::fabs(b->angle - s.angles[i]));
    }
    return err;
}

} // namespace

void predictorCorrectorStep(World& w, real dt, int correctorPasses) {
    if (dt <= 0.0) return;

    const WorldState before = captureState(w);
    w.step(dt);

    // Корректор: трапецеидальное усреднение скоростей до/после шага.
    for (int pass = 0; pass < std::max(0, correctorPasses); ++pass) {
        for (size_t i = 0; i < before.ids.size(); ++i) {
            RigidBody* b = w.body(before.ids[i]);
            if (!b || !b->isDynamic()) continue;
            const Vec2 avg = (before.velocities[i] + b->velocity) * 0.5;
            const Vec2 target = before.positions[i] + avg * dt;
            b->position += (target - b->position) * 0.5;

            const real avgOmega = (before.omegas[i] + b->angularVelocity) * 0.5;
            const real targetAngle = before.angles[i] + avgOmega * dt;
            b->angle += (targetAngle - b->angle) * 0.5;

            b->updateVertices();
            b->updateAABB();
        }
    }
}

real adaptiveStep(World& w, real dt, real tolerance, int maxHalvings) {
    if (dt <= 0.0) return dt;

    const WorldState start = captureState(w);

    // Грубое решение: один шаг dt.
    w.step(dt);
    const WorldState coarse = captureState(w);

    real h = dt;
    for (int level = 0; level < std::max(0, maxHalvings); ++level) {
        restoreState(w, start);
        h *= 0.5;
        const int steps = 1 << (level + 1);
        for (int i = 0; i < steps; ++i) w.step(h);

        const real error = stateDifference(w, coarse);
        if (error <= tolerance) return h;
    }
    return h;
}

} // namespace phys2d
