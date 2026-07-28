#include "phys2d/phys2d_c_api.h"
#include "phys2d/World.h"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <new>
#include <string>

using namespace phys2d;

struct p2World {
    World world;
    p2CollisionCallback cbBegin = nullptr, cbPersist = nullptr, cbEnd = nullptr;
    void* userBegin = nullptr; void* userPersist = nullptr; void* userEnd = nullptr;
    explicit p2World(const WorldConfig& cfg) : world(cfg) {}
};

static World* W(p2WorldHandle h) { return h ? &h->world : nullptr; }

static p2CollisionEvent toC(const CollisionEvent& ev) {
    p2CollisionEvent out{};
    out.a = ev.a ? ev.a->id : 0u;
    out.b = ev.b ? ev.b->id : 0u;
    out.normalX = ev.normal.x; out.normalY = ev.normal.y;
    out.pointX = ev.point.x;   out.pointY = ev.point.y;
    out.penetration = ev.penetration;
    out.normalImpulse = ev.normalImpulse;
    out.timeOfImpact = ev.timeOfImpact;
    out.phase = (int)ev.phase;
    return out;
}

extern "C" {

// ---------------------------------------------------------------- мир
p2WorldConfig p2DefaultConfig(void) {
    const WorldConfig d;
    p2WorldConfig c{};
    c.gravityX = d.gravity.x;
    c.gravityY = d.gravity.y;
    c.integrator = (int)d.integrator;
    c.velocityIterations = d.solver.velocityIterations;
    c.positionIterations = d.solver.positionIterations;
    c.substeps = d.substeps;
    c.threadCount = d.threadCount;
    c.fixedTimeStep = d.fixedTimeStep;
    c.gridCellSize = d.gridCellSize;
    c.modules = d.modules;
    c.arenaBytes = d.arenaBytes;
    return c;
}

p2WorldHandle p2CreateWorld(const p2WorldConfig* cfg) {
    WorldConfig w;
    if (cfg) {
        w.gravity = Vec2(cfg->gravityX, cfg->gravityY);
        w.integrator = (Integrator)cfg->integrator;
        if (cfg->velocityIterations > 0) w.solver.velocityIterations = cfg->velocityIterations;
        if (cfg->positionIterations > 0) w.solver.positionIterations = cfg->positionIterations;
        if (cfg->substeps > 0)           w.substeps = cfg->substeps;
        w.threadCount = cfg->threadCount;
        if (cfg->fixedTimeStep > 0.0)    w.fixedTimeStep = cfg->fixedTimeStep;
        if (cfg->gridCellSize > 0.0)     w.gridCellSize = cfg->gridCellSize;
        if (cfg->modules != 0u)          w.modules = cfg->modules;
        if (cfg->arenaBytes != 0u)       w.arenaBytes = cfg->arenaBytes;
    }
    return new (std::nothrow) p2World(w);
}

void p2DestroyWorld(p2WorldHandle w) { delete w; }
void p2Step(p2WorldHandle w, double dt) { if (W(w)) W(w)->step(dt); }
void p2SetGravity(p2WorldHandle w, double x, double y) { if (W(w)) W(w)->setGravity(Vec2(x, y)); }
void p2SetIntegrator(p2WorldHandle w, int integrator) {
    if (W(w)) W(w)->setIntegrator((Integrator)integrator);
}
void p2EnableModule(p2WorldHandle w, uint32_t flag, int enabled) {
    if (W(w)) W(w)->enableModule(flag, enabled != 0);
}
uint32_t p2Modules(p2WorldHandle w) { return W(w) ? W(w)->modules() : 0u; }
void p2Clear(p2WorldHandle w) { if (W(w)) W(w)->clear(); }

// ---------------------------------------------------------------- тела
static BodyDef makeDef(int type, double x, double y, double angle, double density) {
    BodyDef def;
    def.type = (BodyType)type;
    def.position = Vec2(x, y);
    def.angle = angle;
    def.material.density = (density > 0.0) ? density : 1.0;
    return def;
}

p2BodyId p2CreateCircle(p2WorldHandle w, int type, double x, double y, double radius, double density) {
    if (!W(w)) return 0u;
    BodyDef def = makeDef(type, x, y, 0.0, density);
    def.shape = Shape::circle(radius);
    return W(w)->createBody(def);
}

p2BodyId p2CreateBox(p2WorldHandle w, int type, double x, double y, double width, double height,
                     double angle, double density) {
    if (!W(w)) return 0u;
    BodyDef def = makeDef(type, x, y, angle, density);
    def.shape = Shape::box(width, height);
    return W(w)->createBody(def);
}

p2BodyId p2CreatePolygon(p2WorldHandle w, int type, double x, double y,
                         const p2Vec2* verts, int count, double density) {
    if (!W(w) || !verts || count < 3) return 0u;
    std::vector<Vec2> v;
    v.reserve((size_t)count);
    for (int i = 0; i < count; ++i) v.emplace_back(verts[i].x, verts[i].y);
    BodyDef def = makeDef(type, x, y, 0.0, density);
    def.shape = Shape::polygon(v);
    return W(w)->createBody(def);
}

p2BodyId p2CreateCapsule(p2WorldHandle w, int type, double x, double y, double radius, double length,
                         double angle, double density) {
    if (!W(w)) return 0u;
    BodyDef def = makeDef(type, x, y, angle, density);
    def.shape = Shape::capsule(radius, length);
    return W(w)->createBody(def);
}

void p2DestroyBody(p2WorldHandle w, p2BodyId id) { if (W(w)) W(w)->destroyBody(id); }

int p2GetTransform(p2WorldHandle w, p2BodyId id, double* x, double* y, double* angle) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (!b) return 0;
    if (x) *x = b->position.x;
    if (y) *y = b->position.y;
    if (angle) *angle = b->angle;
    return 1;
}

int p2SetTransform(p2WorldHandle w, p2BodyId id, double x, double y, double angle) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (!b) return 0;
    b->position = Vec2(x, y);
    b->prevPosition = b->position;
    b->angle = angle;
    b->prevAngle = angle;
    b->updateVertices();
    b->updateAABB(W(w)->config().aabbMargin);
    b->wake();
    return 1;
}

int p2GetVelocity(p2WorldHandle w, p2BodyId id, double* vx, double* vy, double* omega) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (!b) return 0;
    if (vx) *vx = b->velocity.x;
    if (vy) *vy = b->velocity.y;
    if (omega) *omega = b->angularVelocity;
    return 1;
}

int p2SetVelocity(p2WorldHandle w, p2BodyId id, double vx, double vy, double omega) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (!b) return 0;
    b->velocity = Vec2(vx, vy);
    b->angularVelocity = omega;
    b->wake();
    return 1;
}

int p2ApplyForce(p2WorldHandle w, p2BodyId id, double fx, double fy) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (!b) return 0;
    b->applyForce(Vec2(fx, fy));
    return 1;
}

int p2ApplyImpulseAtPoint(p2WorldHandle w, p2BodyId id, double ix, double iy, double px, double py) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (!b) return 0;
    b->applyImpulseAtPoint(Vec2(ix, iy), Vec2(px, py));
    return 1;
}

int p2SetMaterial(p2WorldHandle w, p2BodyId id, double restitution,
                  double staticFriction, double dynamicFriction) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (!b) return 0;
    b->material.restitution = restitution;
    b->material.staticFriction = staticFriction;
    b->material.dynamicFriction = dynamicFriction;
    return 1;
}

int p2GetAABB(p2WorldHandle w, p2BodyId id, p2AABB* out) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (!b || !out) return 0;
    out->minX = b->aabb.min.x; out->minY = b->aabb.min.y;
    out->maxX = b->aabb.max.x; out->maxY = b->aabb.max.y;
    return 1;
}

double p2GetMass(p2WorldHandle w, p2BodyId id) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    return b ? b->mass : 0.0;
}
double p2GetInertia(p2WorldHandle w, p2BodyId id) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    return b ? b->inertia : 0.0;
}
int p2IsSleeping(p2WorldHandle w, p2BodyId id) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    return (b && b->sleeping) ? 1 : 0;
}
void p2Wake(p2WorldHandle w, p2BodyId id) {
    RigidBody* b = W(w) ? W(w)->body(id) : nullptr;
    if (b) b->wake();
}

// ---------------------------------------------------------------- ограничения
p2ConstraintHandle p2CreateDistance(p2WorldHandle w, p2BodyId a, p2BodyId b,
                                    double lax, double lay, double lbx, double lby, double length) {
    if (!W(w)) return nullptr;
    return (p2ConstraintHandle)W(w)->createDistance(a, b, Vec2(lax, lay), Vec2(lbx, lby), length);
}
p2ConstraintHandle p2CreateRevolute(p2WorldHandle w, p2BodyId a, p2BodyId b,
                                    double lax, double lay, double lbx, double lby) {
    if (!W(w)) return nullptr;
    return (p2ConstraintHandle)W(w)->createRevolute(a, b, Vec2(lax, lay), Vec2(lbx, lby));
}
p2ConstraintHandle p2CreateSpring(p2WorldHandle w, p2BodyId a, p2BodyId b,
                                  double lax, double lay, double lbx, double lby,
                                  double length, double stiffness, double damping) {
    if (!W(w)) return nullptr;
    return (p2ConstraintHandle)W(w)->createSpring(a, b, Vec2(lax, lay), Vec2(lbx, lby),
                                                  length, stiffness, damping);
}
p2ConstraintHandle p2CreateRope(p2WorldHandle w, p2BodyId a, p2BodyId b,
                                double lax, double lay, double lbx, double lby, double maxLength) {
    if (!W(w)) return nullptr;
    return (p2ConstraintHandle)W(w)->createRope(a, b, Vec2(lax, lay), Vec2(lbx, lby), maxLength);
}
p2ConstraintHandle p2CreateAngular(p2WorldHandle w, p2BodyId a, p2BodyId b,
                                   double minAngle, double maxAngle) {
    if (!W(w)) return nullptr;
    return (p2ConstraintHandle)W(w)->createAngular(a, b, minAngle, maxAngle);
}
void p2DestroyConstraint(p2WorldHandle w, p2ConstraintHandle c) {
    if (W(w) && c) W(w)->destroyConstraint((Constraint*)c);
}

// ---------------------------------------------------------------- события
void p2SetContactCallback(p2WorldHandle w, int phase, p2CollisionCallback cb, void* user) {
    if (!W(w)) return;
    p2World* h = w;
    if (phase == 0) {
        h->cbBegin = cb; h->userBegin = user;
        W(w)->setBeginContactCallback([h](const CollisionEvent& ev) {
            if (h->cbBegin) { p2CollisionEvent e = toC(ev); h->cbBegin(&e, h->userBegin); }
        });
    } else if (phase == 1) {
        h->cbPersist = cb; h->userPersist = user;
        W(w)->setPersistContactCallback([h](const CollisionEvent& ev) {
            if (h->cbPersist) { p2CollisionEvent e = toC(ev); h->cbPersist(&e, h->userPersist); }
        });
    } else {
        h->cbEnd = cb; h->userEnd = user;
        W(w)->setEndContactCallback([h](const CollisionEvent& ev) {
            if (h->cbEnd) { p2CollisionEvent e = toC(ev); h->cbEnd(&e, h->userEnd); }
        });
    }
}

// ---------------------------------------------------------------- статистика
size_t p2BodyCount(p2WorldHandle w)   { return W(w) ? W(w)->bodyCount() : 0u; }
size_t p2ContactCount(p2WorldHandle w){ return W(w) ? W(w)->contactCount() : 0u; }
size_t p2ArenaBytes(p2WorldHandle w)  { return W(w) ? W(w)->arenaBytes() : 0u; }

double p2StageMilliseconds(p2WorldHandle w, int stage) {
    if (!W(w) || stage < 0 || stage >= (int)Stage::Count) return 0.0;
    return W(w)->profile().ms[stage];
}
double p2TotalEnergy(p2WorldHandle w) { return W(w) ? W(w)->totalEnergy() : 0.0; }

int p2ProfileString(p2WorldHandle w, char* buffer, int bufferSize) {
    if (!W(w) || !buffer || bufferSize <= 0) return 0;
    const std::string s = W(w)->profile().toString();
    const int n = (int)std::min<size_t>(s.size(), (size_t)bufferSize - 1);
    std::memcpy(buffer, s.data(), (size_t)n);
    buffer[n] = '\0';
    return n;
}

// ---------------------------------------------------------------- сериализация
int p2SaveToFile(p2WorldHandle w, const char* path) {
    if (!W(w) || !path) return 0;
    return W(w)->saveToFile(path) ? 1 : 0;
}
int p2LoadFromFile(p2WorldHandle w, const char* path) {
    if (!W(w) || !path) return 0;
    return W(w)->loadFromFile(path, nullptr) ? 1 : 0;
}
char* p2ToJson(p2WorldHandle w, int pretty) {
    if (!W(w)) return nullptr;
    const std::string s = W(w)->toJson(pretty != 0);
    char* out = (char*)std::malloc(s.size() + 1);
    if (!out) return nullptr;
    std::memcpy(out, s.c_str(), s.size() + 1);
    return out;
}
int p2FromJson(p2WorldHandle w, const char* json) {
    if (!W(w) || !json) return 0;
    return W(w)->fromJson(json, nullptr) ? 1 : 0;
}
void p2FreeString(char* s) { std::free(s); }

// ---------------------------------------------------------------- дебаг
void p2SetDebugLayers(p2WorldHandle w, uint32_t layers) {
    if (W(w)) W(w)->debugBuffer().layers = layers;
}

int p2BuildDebugSvg(p2WorldHandle w, const char* path, double viewSize) {
    if (!W(w) || !path) return 0;
    W(w)->buildDebugGeometry();
    AABB view;
    const real half = (viewSize > 0.0 ? viewSize : 100.0) * 0.5;
    view.min = Vec2(-half, -half);
    view.max = Vec2( half,  half);
    const std::string svg = debugBufferToSvg(W(w)->debugBuffer(), view, 1400);
    std::ofstream f(path, std::ios::binary);
    if (!f) return 0;
    f << svg;
    return f.good() ? 1 : 0;
}

} // extern "C"
