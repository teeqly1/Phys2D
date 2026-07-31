#include "phys2d/Nova.h"
#include <algorithm>
#include <cmath>

namespace phys2d {
namespace nova {

static inline real cl(real v, real a, real b){ return v < a ? a : (v > b ? b : v); }
static inline real vlen(const Vec2& v){ return std::sqrt((double)(v.x * v.x + v.y * v.y)); }
struct SRng {
    uint32_t s;
    explicit SRng(uint32_t seed) : s(seed ? seed : 7u) {}
    uint32_t next(){ s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    real uni(){ return (real)(next() % 1000000u) / 1000000.0; }
    real range(real a, real b){ return a + (b - a) * uni(); }
};

// ============================================================ 7. CLOTH/HAIR
static void addLink(ClothPiece& p, int a, int b, real stiffness, bool shear)
{
    ClothLink l;
    l.a = a; l.b = b; l.stiffness = stiffness; l.shear = shear;
    l.rest = vlen(p.nodes[(size_t)a].p - p.nodes[(size_t)b].p);
    p.links.push_back(l);
}
static int makeGrid(std::vector<ClothPiece>& pieces, const Vec2& topLeft, int cols, int rows,
                    real spacing, real stiffness, bool pinTopRow, bool pinFirstColumn, uint32_t color, real tear)
{
    ClothPiece p;
    p.cols = cols; p.rows = rows; p.spacing = spacing; p.color = color; p.tearFactor = tear;
    for(int y = 0; y < rows; ++y){
        for(int x = 0; x < cols; ++x){
            ClothNode n;
            n.p = topLeft + Vec2((real)x * spacing, -(real)y * spacing);
            n.prev = n.p;
            n.pinned = (pinTopRow && y == 0) || (pinFirstColumn && x == 0);
            n.invMass = n.pinned ? 0.0 : 1.0;
            p.nodes.push_back(n);
        }
    }
    for(int y = 0; y < rows; ++y)
        for(int x = 0; x < cols; ++x){
            int i = y * cols + x;
            if(x + 1 < cols) addLink(p, i, i + 1, stiffness, false);
            if(y + 1 < rows) addLink(p, i, i + cols, stiffness, false);
            if(x + 1 < cols && y + 1 < rows) addLink(p, i, i + cols + 1, stiffness * 0.5, true);
            if(x > 0 && y + 1 < rows) addLink(p, i, i + cols - 1, stiffness * 0.5, true);
        }
    pieces.push_back(p);
    return (int)pieces.size() - 1;
}
int ClothSystem::createFlag(const Vec2& topLeft, int cols, int rows, real spacing, real stiffness)
{ return makeGrid(pieces_, topLeft, cols, rows, spacing, stiffness, false, true, 0xD04A5A, 3.2); }
int ClothSystem::createCurtain(const Vec2& topLeft, int cols, int rows, real spacing)
{ return makeGrid(pieces_, topLeft, cols, rows, spacing, 1.0, true, false, 0x4A6AC8, 0.0); }
int ClothSystem::createHair(const Vec2& root, int nodes, real length, real stiffness)
{
    ClothPiece p;
    p.cols = 1; p.rows = nodes; p.hair = true; p.color = 0x503020;
    p.spacing = length / (real)std::max(1, nodes - 1);
    for(int i = 0; i < nodes; ++i){
        ClothNode n;
        n.p = root + Vec2(0, -(real)i * p.spacing);
        n.prev = n.p;
        n.pinned = (i == 0);
        n.invMass = n.pinned ? 0.0 : 1.0;
        p.nodes.push_back(n);
    }
    for(int i = 0; i + 1 < nodes; ++i) addLink(p, i, i + 1, stiffness, false);
    for(int i = 0; i + 2 < nodes; ++i) addLink(p, i, i + 2, 0.35, true);
    pieces_.push_back(p);
    return (int)pieces_.size() - 1;
}
int ClothSystem::createRopeStrand(const Vec2& a, const Vec2& b, int nodes, real stiffness)
{
    ClothPiece p;
    p.cols = 1; p.rows = nodes; p.color = 0xA08050;
    Vec2 d = b - a;
    p.spacing = vlen(d) / (real)std::max(1, nodes - 1);
    for(int i = 0; i < nodes; ++i){
        ClothNode n;
        real t = (real)i / (real)std::max(1, nodes - 1);
        n.p = a + d * t;
        n.prev = n.p;
        n.pinned = (i == 0 || i == nodes - 1);
        n.invMass = n.pinned ? 0.0 : 1.0;
        p.nodes.push_back(n);
    }
    for(int i = 0; i + 1 < nodes; ++i) addLink(p, i, i + 1, stiffness, false);
    pieces_.push_back(p);
    return (int)pieces_.size() - 1;
}
void ClothSystem::pinToBody(int piece, int node, BodyId body)
{ pins_.push_back(std::make_pair(std::make_pair(piece, node), body)); }
void ClothSystem::setWind(const Vec2& wind, real turbulence){ wind_ = wind; turbulence_ = turbulence; }
size_t ClothSystem::nodeCount() const
{
    size_t n = 0;
    for(size_t i = 0; i < pieces_.size(); ++i) n += pieces_[i].nodes.size();
    return n;
}
void ClothSystem::solveBodies(ClothPiece& piece)
{
    if(!world_) return;
    uint32_t limit = (uint32_t)world_->bodyCount() + 64u;
    for(uint32_t id = 0; id <= limit; ++id){
        RigidBody* b = world_->body((BodyId)id);
        if(!b) continue;
        if(b->boundingRadius > 4.0) continue; // huge static slabs are not round
        real rad = b->boundingRadius + piece.thickness;
        for(size_t i = 0; i < piece.nodes.size(); ++i){
            ClothNode& n = piece.nodes[i];
            if(n.pinned || !n.alive) continue;
            Vec2 d = n.p - b->position;
            real dist = vlen(d);
            if(dist > rad || dist < 1e-9) continue;
            Vec2 nrm = d * (1.0 / dist);
            real push = rad - dist;
            n.p = n.p + nrm * push;
            if(b->type == BodyType::Dynamic) b->applyImpulse(nrm * (-push * 1.5));
        }
    }
}
void ClothSystem::update(real dt, int iterations)
{
    time_ += dt;
    for(size_t pi = 0; pi < pieces_.size(); ++pi){
        ClothPiece& p = pieces_[pi];
        real drag = 1.0 - p.damping;
        for(size_t i = 0; i < p.nodes.size(); ++i){
            ClothNode& n = p.nodes[i];
            if(n.pinned || !n.alive) continue;
            Vec2 acc = gravity;
            Vec2 w = wind_;
            if(p.hair) w = w * 0.35;
            real turb = turbulence_ * std::sin(time_ * 3.1 + (real)i * 0.7);
            acc = acc + w + Vec2(turb, turb * 0.35);
            Vec2 cur = n.p;
            n.p = n.p + (n.p - n.prev) * drag + acc * (dt * dt);
            n.prev = cur;
        }
        for(int it = 0; it < iterations; ++it){
            for(size_t li = 0; li < p.links.size(); ++li){
                ClothLink& l = p.links[li];
                if(!l.alive) continue;
                ClothNode& a = p.nodes[(size_t)l.a];
                ClothNode& b = p.nodes[(size_t)l.b];
                if(!a.alive || !b.alive) continue;
                Vec2 d = b.p - a.p;
                real dist = vlen(d);
                if(dist < 1e-9) continue;
                if(p.tearFactor > 0 && dist > l.rest * p.tearFactor){ l.alive = false; ++torn_; continue; }
                real diff = (dist - l.rest) / dist * l.stiffness;
                real wsum = a.invMass + b.invMass;
                if(wsum < 1e-9) continue;
                Vec2 corr = d * diff;
                a.p = a.p + corr * (a.invMass / wsum);
                b.p = b.p - corr * (b.invMass / wsum);
            }
            solveBodies(p);
        }
    }
    if(world_){
        for(size_t i = 0; i < pins_.size(); ++i){
            int pi = pins_[i].first.first, ni = pins_[i].first.second;
            RigidBody* b = world_->body(pins_[i].second);
            if(!b || pi < 0 || pi >= (int)pieces_.size()) continue;
            ClothPiece& p = pieces_[(size_t)pi];
            if(ni < 0 || ni >= (int)p.nodes.size()) continue;
            p.nodes[(size_t)ni].p = b->position;
            p.nodes[(size_t)ni].prev = b->position;
        }
    }
}
void ClothSystem::applyImpulse(const Vec2& at, real radius, const Vec2& impulse)
{
    for(size_t pi = 0; pi < pieces_.size(); ++pi){
        ClothPiece& p = pieces_[pi];
        for(size_t i = 0; i < p.nodes.size(); ++i){
            ClothNode& n = p.nodes[i];
            if(n.pinned) continue;
            real d = vlen(n.p - at);
            if(d > radius) continue;
            real k = 1.0 - d / radius;
            n.prev = n.prev - impulse * k;
        }
    }
}

// ================================================================ 8. GRAINS
static real grainDensity(GrainKind k)
{
    switch(k){
        case GrainKind::Sand: return 1600;
        case GrainKind::Gravel: return 1900;
        case GrainKind::Snow: return 250;
        case GrainKind::Spark: return 400;
        case GrainKind::Smoke: return 0.6;
        case GrainKind::Ember: return 400;
        case GrainKind::Rain: return 1000;
        case GrainKind::Hail: return 900;
        default: return 800;
    }
}
static uint32_t grainColor(GrainKind k)
{
    switch(k){
        case GrainKind::Sand: return 0xC2A050;
        case GrainKind::Gravel: return 0x8C8880;
        case GrainKind::Snow: return 0xF2F6FA;
        case GrainKind::Spark: return 0xFFD060;
        case GrainKind::Smoke: return 0x606068;
        case GrainKind::Ember: return 0xFF6020;
        case GrainKind::Rain: return 0x6FA8D8;
        case GrainKind::Hail: return 0xD8E8F0;
        default: return 0xA0A0A0;
    }
}
static real grainMaxLife(GrainKind k)
{
    if(k == GrainKind::Spark) return 0.9;
    if(k == GrainKind::Smoke) return 3.0;
    if(k == GrainKind::Ember) return 2.2;
    return 1e30;
}
static Grain makeGrain(const Vec2& p, const Vec2& v, real radius, GrainKind k)
{
    Grain g;
    g.p = p; g.v = v; g.radius = radius; g.kind = k;
    g.mass = grainDensity(k) * 3.14159265358979 * radius * radius;
    g.color = grainColor(k);
    g.maxLife = grainMaxLife(k);
    return g;
}
void GrainSystem::emitPile(const Vec2& center, real radius, real height, real grainRadius, GrainKind kind)
{
    real step = grainRadius * 2.02;
    for(real y = 0; y < height; y += step){
        real halfW = radius * (1.0 - y / (height + 1e-9)) + grainRadius;
        for(real x = -halfW; x <= halfW; x += step){
            if(grains_.size() >= maxCount) return;
            real jitter = ((real)(rng_ = rng_ * 1664525u + 1013904223u) / 4294967296.0 - 0.5) * grainRadius * 0.3;
            grains_.push_back(makeGrain(center + Vec2(x + jitter, y + grainRadius), Vec2(0, 0), grainRadius, kind));
        }
    }
}
void GrainSystem::emitBurst(const Vec2& at, int count, real speed, GrainKind kind)
{
    SRng rng(rng_ ^ 0x9E3779B9u);
    for(int i = 0; i < count; ++i){
        if(grains_.size() >= maxCount) return;
        real a = rng.range(0, 6.2831853);
        real s = speed * rng.range(0.35, 1.0);
        Vec2 v(std::cos(a) * s, std::sin(a) * s);
        real r = kind == GrainKind::Smoke ? 0.10 : 0.04;
        grains_.push_back(makeGrain(at, v, r, kind));
    }
    rng_ = rng.s;
}
void GrainSystem::emitJet(const Vec2& at, const Vec2& dir, int count, real speed, real spread, GrainKind kind)
{
    SRng rng(rng_ ^ 0x85EBCA6Bu);
    real base = std::atan2((double)dir.y, (double)dir.x);
    for(int i = 0; i < count; ++i){
        if(grains_.size() >= maxCount) return;
        real a = base + rng.range(-spread, spread);
        real s = speed * rng.range(0.6, 1.2);
        grains_.push_back(makeGrain(at, Vec2(std::cos(a) * s, std::sin(a) * s), 0.05, kind));
    }
    rng_ = rng.s;
}
void GrainSystem::collideGrains(real dt)
{
    cells_.clear();
    contacts_ = 0;
    real inv = 1.0 / cellSize_;
    for(size_t i = 0; i < grains_.size(); ++i){
        int cx = (int)std::floor((double)(grains_[i].p.x * inv));
        int cy = (int)std::floor((double)(grains_[i].p.y * inv));
        uint64_t key = ((uint64_t)(uint32_t)(cx + 100000) << 32) | (uint32_t)(cy + 100000);
        cells_[key].push_back((int)i);
    }
    for(size_t i = 0; i < grains_.size(); ++i){
        Grain& a = grains_[i];
        if(a.kind == GrainKind::Smoke) continue;
        int cx = (int)std::floor((double)(a.p.x * inv));
        int cy = (int)std::floor((double)(a.p.y * inv));
        for(int dy = -1; dy <= 1; ++dy)
            for(int dx = -1; dx <= 1; ++dx){
                uint64_t key = ((uint64_t)(uint32_t)(cx + dx + 100000) << 32) | (uint32_t)(cy + dy + 100000);
                std::unordered_map<uint64_t, std::vector<int> >::iterator it = cells_.find(key);
                if(it == cells_.end()) continue;
                for(size_t k = 0; k < it->second.size(); ++k){
                    int j = it->second[k];
                    if((size_t)j <= i) continue;
                    Grain& b = grains_[(size_t)j];
                    if(b.kind == GrainKind::Smoke) continue;
                    Vec2 d = b.p - a.p;
                    real dist = vlen(d);
                    real rad = a.radius + b.radius;
                    if(dist >= rad || dist < 1e-9) continue;
                    ++contacts_;
                    Vec2 n = d * (1.0 / dist);
                    real pen = rad - dist;
                    Vec2 rel = b.v - a.v;
                    real vn = rel.x * n.x + rel.y * n.y;
                    real fn = stiffness * std::pow((double)pen, 1.5) - damping * vn * std::sqrt((double)pen);
                    if(fn < 0) fn = 0;
                    Vec2 imp = n * (fn * dt);
                    real ima = 1.0 / a.mass, imb = 1.0 / b.mass;
                    a.v = a.v - imp * ima;
                    b.v = b.v + imp * imb;
                    Vec2 tang(-n.y, n.x);
                    real vt = rel.x * tang.x + rel.y * tang.y;
                    Vec2 ft = tang * (-cl(vt, -1.0, 1.0) * fn * friction * dt);
                    a.v = a.v - ft * ima;
                    b.v = b.v + ft * imb;
                    real corr = pen * 0.25;
                    a.p = a.p - n * corr;
                    b.p = b.p + n * corr;
                }
            }
    }
}
void GrainSystem::collideBodies(real dt)
{
    if(!world_) return;
    uint32_t limit = (uint32_t)world_->bodyCount() + 64u;
    for(uint32_t id = 0; id <= limit; ++id){
        RigidBody* b = world_->body((BodyId)id);
        if(!b || b->boundingRadius > 6.0) continue;
        for(size_t i = 0; i < grains_.size(); ++i){
            Grain& g = grains_[i];
            if(g.kind == GrainKind::Smoke) continue;
            Vec2 d = g.p - b->position;
            real dist = vlen(d);
            real rad = b->boundingRadius + g.radius;
            if(dist >= rad || dist < 1e-9) continue;
            Vec2 n = d * (1.0 / dist);
            g.p = g.p + n * (rad - dist);
            real vn = g.v.x * n.x + g.v.y * n.y;
            if(vn < 0) g.v = g.v - n * (vn * 1.4);
            if(b->type == BodyType::Dynamic) b->applyImpulse(n * (-g.mass * 0.5 * (vn < 0 ? -vn : 0.0) * dt * 60.0));
        }
    }
}
void GrainSystem::update(real dt)
{
    for(size_t i = 0; i < grains_.size(); ++i){
        Grain& g = grains_[i];
        Vec2 acc = gravity;
        if(g.kind == GrainKind::Smoke) acc = Vec2(gravity.x * 0.05, 2.6);
        else if(g.kind == GrainKind::Ember) acc = Vec2(gravity.x * 0.2, gravity.y * 0.25 + 1.2);
        else if(g.kind == GrainKind::Snow) acc = gravity * 0.22;
        g.v = g.v + acc * dt;
        g.v = g.v - g.v * (airDrag * dt);
        g.p = g.p + g.v * dt;
        g.life += dt;
        if(g.p.y - g.radius < bounds[1]){
            g.p.y = bounds[1] + g.radius;
            if(g.v.y < 0) g.v.y = -g.v.y * 0.15;
            g.v.x *= 0.86;
            g.resting = vlen(g.v) < 0.05;
        }
    }
    collideGrains(dt);
    collideBodies(dt);
    size_t w = 0;
    for(size_t i = 0; i < grains_.size(); ++i){
        Grain& g = grains_[i];
        bool dead = g.life > g.maxLife || g.p.x < bounds[0] || g.p.x > bounds[2] || g.p.y > bounds[3];
        if(!dead) grains_[w++] = g;
    }
    grains_.resize(w);
}
void GrainSystem::applyExplosion(const Vec2& at, real radius, real energy)
{
    for(size_t i = 0; i < grains_.size(); ++i){
        Grain& g = grains_[i];
        Vec2 d = g.p - at;
        real dist = vlen(d);
        if(dist > radius || dist < 1e-6) continue;
        real k = 1.0 - dist / radius;
        g.v = g.v + d * (1.0 / dist) * (energy * k / g.mass * 0.001);
        g.heat += k;
        g.resting = false;
    }
}
real GrainSystem::packedFraction() const
{
    if(grains_.empty()) return 0;
    int rest = 0;
    for(size_t i = 0; i < grains_.size(); ++i) if(grains_[i].resting) ++rest;
    return (real)rest / (real)grains_.size();
}

void WeatherSystem::configure(Kind k, real intensity, const Vec2& wind)
{ kind_ = k; intensity_ = intensity; wind_ = wind; }
void WeatherSystem::update(real dt, GrainSystem& grains)
{
    if(kind_ == Kind::Clear || intensity_ <= 0) return;
    real rate = 0;
    GrainKind gk = GrainKind::Rain;
    if(kind_ == Kind::Rain){ rate = 900; gk = GrainKind::Rain; }
    else if(kind_ == Kind::Snowfall){ rate = 380; gk = GrainKind::Snow; }
    else if(kind_ == Kind::Hail){ rate = 160; gk = GrainKind::Hail; }
    else { rate = 700; gk = GrainKind::Sand; }
    accum_ += rate * intensity_ * dt;
    int n = (int)accum_;
    accum_ -= (real)n;
    SRng rng(rng_);
    for(int i = 0; i < n; ++i){
        real x = rng.range(min_.x, max_.x);
        Vec2 p = kind_ == Kind::Sandstorm ? Vec2(min_.x, rng.range(min_.y, max_.y)) : Vec2(x, max_.y);
        Vec2 v = kind_ == Kind::Sandstorm ? wind_ + Vec2(12, 0) : wind_ + Vec2(0, -6);
        grains.emitJet(p, v, 1, vlen(v) + 0.01, 0.05, gk);
        ++spawned_;
    }
    rng_ = rng.s;
}

// ============================================================== 9. VEHICLES
Vec2 WheeledVehicle::strutAxis() const
{
    const RigidBody* c = world_->body(chassis_);
    real a = c ? c->angle : 0.0;
    return Vec2(std::sin((double)a), -std::cos((double)a));
}
Vec2 WheeledVehicle::strutTop(const WheelState& ws) const
{
    const RigidBody* c = world_->body(chassis_);
    if(!c) return ws.localAnchor;
    real ca = std::cos((double)c->angle), sa = std::sin((double)c->angle);
    Vec2 r(ws.localAnchor.x * ca - ws.localAnchor.y * sa, ws.localAnchor.x * sa + ws.localAnchor.y * ca);
    return c->position + r;
}
void WheeledVehicle::create(World& w, const Config& cfg, const std::vector<WheelSetup>& setups)
{
    world_ = &w;
    cfg_ = cfg;
    const MaterialLibrary& lib = MaterialLibrary::instance();
    BodyDef bd;
    bd.type = BodyType::Dynamic;
    bd.shape = Shape::box(cfg.halfWidth * 2.0, cfg.halfHeight * 2.0);
    bd.material = lib.physics(cfg.materialIndex);
    bd.material.density = cfg.mass / (cfg.halfWidth * 2.0 * cfg.halfHeight * 2.0);
    bd.material.staticFriction = 0.45;
    bd.material.dynamicFriction = 0.35;
    bd.material.restitution = 0.05;
    bd.position = cfg.position;
    bd.allowSleep = false;
    bd.name = "chassis";
    chassis_ = w.createBody(bd);

    real downT = 0;
    for(size_t i = 0; i < setups.size(); ++i){
        const WheelSetup& s = setups[i];
        WheelState ws;
        ws.setup = s;
        ws.localAnchor = s.anchor;
        downT = s.travel * 0.60;
        real clearance = cfg.halfHeight + s.radius + 0.05 - s.anchor.y;
        ws.restLen = cfg.halfHeight * 0.35 + downT + 0.06;
        if(ws.restLen < clearance) ws.restLen = clearance;
        ws.minLen = ws.restLen - downT;
        ws.maxLen = ws.restLen + s.travel * 0.40;
        ws.len = ws.restLen;
        BodyDef wd;
        wd.type = BodyType::Dynamic;
        wd.shape = Shape::circle(s.radius);
        wd.material.density = 190;
        wd.material.staticFriction = 0.30;
        wd.material.dynamicFriction = 0.20;
        wd.material.rollingFriction = 0.0;
        wd.material.restitution = 0.05;
        wd.position = cfg.position + s.anchor + Vec2(0, -ws.restLen);
        wd.fixedRotation = true;
        wd.allowSleep = false;
        wd.name = "wheel";
        ws.body = w.createBody(wd);
        wheels_.push_back(ws);
    }
    if(downT < 0.05) downT = 0.05;
    springK_ = 2.6 * (cfg.mass * 9.81 * 0.5) / std::max((real)0.05, cfg.ridePercent * downT);
    springC_ = cfg.damperRatio * 2.0 * std::sqrt((double)(springK_ * cfg.mass * 0.5));
}
void WheeledVehicle::destroy()
{
    if(!world_) return;
    for(size_t i = 0; i < wheels_.size(); ++i) world_->destroyBody(wheels_[i].body);
    world_->destroyBody(chassis_);
    wheels_.clear();
    chassis_ = INVALID_BODY;
}
void WheeledVehicle::update(real dt, real throttle, real brake, real lean)
{
    if(!world_) return;
    RigidBody* c = world_->body(chassis_);
    if(!c) return;
    c->wake();
    Vec2 axis = strutAxis();
    Vec2 fwd(std::cos((double)c->angle), std::sin((double)c->angle));
    real vfwd = c->velocity.x * fwd.x + c->velocity.y * fwd.y;
    real fade = cl(1.0 - std::fabs((double)(vfwd * 3.6)) / std::max((real)1.0, cfg_.topSpeed), 0.0, 1.0);
    real share = wheels_.empty() ? 1.0 : 1.0 / (real)wheels_.size();
    int grounded = 0;
    for(size_t i = 0; i < wheels_.size(); ++i){
        WheelState& ws = wheels_[i];
        RigidBody* wb = world_->body(ws.body);
        if(!wb) continue;
        wb->wake();
        Vec2 top = strutTop(ws);
        Vec2 d = wb->position - top;
        ws.len = d.x * axis.x + d.y * axis.y;
        ws.len = cl(ws.len, ws.minLen - 0.05, ws.maxLen);
        ws.compression = cl((ws.restLen - ws.len) / std::max((real)1e-4, ws.restLen - ws.minLen), -1.0, 1.0);
        Vec2 relV = wb->velocity - c->velocity;
        real vRel = relV.x * axis.x + relV.y * axis.y;
        real force = springK_ * (ws.restLen - ws.len) * ws.setup.stiffnessScale - springC_ * vRel * ws.setup.damperScale;
        if(ws.len < ws.minLen + 0.04) force += springK_ * 6.0 * (ws.minLen + 0.04 - ws.len);
        force = cl(force, -cfg_.mass * 30.0, cfg_.mass * 140.0);
        ws.load = force;
        wb->applyForce(axis * force);
        c->applyForceAtPoint(axis * (-force), top);
        ws.grounded = (force > cfg_.mass * 0.5) && (ws.len > ws.restLen * 0.15);
        if(ws.grounded) ++grounded;
        real wvf = wb->velocity.x * fwd.x + wb->velocity.y * fwd.y;
        ws.slip = wvf - vfwd;
        if(ws.setup.driven && throttle != 0.0){
            real tr = cfg_.mass * cfg_.driveAccel * (0.45 + 0.55 * fade) * share * cfg_.grip * 3.0 * throttle;
            c->applyForceAtPoint(fwd * tr, wb->position);
        }
        if(ws.setup.brakes && brake > 0.0){
            real br = -cl(vfwd, -1.0, 1.0) * cfg_.mass * cfg_.brakeAccel * share * brake;
            c->applyForceAtPoint(fwd * br, wb->position);
        }
        if(throttle == 0.0 && brake == 0.0)
            c->applyForceAtPoint(fwd * (-cl(vfwd, -1.0, 1.0) * cfg_.mass * 0.35 * share), wb->position);
        ws.spin += (wvf / std::max((real)0.05, ws.setup.radius)) * dt;
    }
    if(lean != 0.0) c->applyTorque(cfg_.mass * (grounded ? 3.0 : 9.0) * lean);
    if(cfg_.downforce > 0) c->applyForce(Vec2(0, -cfg_.downforce * vfwd * vfwd));
    odo_ += std::fabs((double)vfwd) * dt;
    real kmh = std::fabs((double)vfwd) * 3.6;
    if(kmh > topKmh_) topKmh_ = kmh;
}
void WheeledVehicle::postStep()
{
    if(!world_) return;
    RigidBody* c = world_->body(chassis_);
    if(!c) return;
    Vec2 axis = strutAxis();
    Vec2 perp(-axis.y, axis.x);
    for(size_t i = 0; i < wheels_.size(); ++i){
        WheelState& ws = wheels_[i];
        RigidBody* wb = world_->body(ws.body);
        if(!wb) continue;
        Vec2 top = strutTop(ws);
        Vec2 d = wb->position - top;
        real along = cl(d.x * axis.x + d.y * axis.y, ws.minLen, ws.maxLen);
        wb->position = top + axis * along;
        ws.len = along;
        real vperp = (wb->velocity.x - c->velocity.x) * perp.x + (wb->velocity.y - c->velocity.y) * perp.y;
        wb->velocity = wb->velocity - perp * vperp;
    }
}
real WheeledVehicle::speedKmh() const { return topKmh_; }
real WheeledVehicle::suspensionTravelUsed() const
{
    real used = 0;
    for(size_t i = 0; i < wheels_.size(); ++i){
        const WheelState& ws = wheels_[i];
        real span = std::max((real)1e-4, ws.maxLen - ws.minLen);
        used = std::max(used, cl((ws.restLen - ws.len) / span, 0.0, 1.0));
    }
    return used;
}
bool WheeledVehicle::airborne() const
{
    for(size_t i = 0; i < wheels_.size(); ++i) if(wheels_[i].grounded) return false;
    return true;
}

void TrackedVehicle::create(World& w, const Config& cfg)
{
    world_ = &w;
    cfg_ = cfg;
    const MaterialLibrary& lib = MaterialLibrary::instance();
    BodyDef hd;
    hd.type = BodyType::Dynamic;
    hd.shape = Shape::box(cfg.halfLength * 2.0, cfg.halfHeight * 2.0);
    hd.material = lib.physics(lib.indexOf("steel"));
    hd.material.density = cfg.mass / (cfg.halfLength * 2.0 * cfg.halfHeight * 2.0);
    hd.material.staticFriction = cfg.trackGrip;
    hd.material.dynamicFriction = cfg.trackGrip * 0.8;
    hd.position = cfg.position;
    hd.allowSleep = false;
    hd.name = "hull";
    hull_ = w.createBody(hd);

    BodyDef td;
    td.type = BodyType::Dynamic;
    td.shape = Shape::box(2.2, 0.6);
    td.material = lib.physics(lib.indexOf("steel"));
    td.material.density = cfg.turretMass / (2.2 * 0.6);
    td.position = cfg.position + Vec2(0, cfg.halfHeight + 0.35);
    td.allowSleep = false;
    td.name = "turret";
    turret_ = w.createBody(td);
    w.createRevolute(hull_, turret_, Vec2(0, cfg.halfHeight + 0.3), Vec2(0, -0.3));

    for(int i = 0; i < cfg.roadWheels; ++i){
        real t = cfg.roadWheels > 1 ? (real)i / (real)(cfg.roadWheels - 1) : 0.5;
        real x = -cfg.halfLength * 0.85 + t * cfg.halfLength * 1.7;
        real ay = -(cfg.halfHeight + cfg.wheelRadius + 0.05);
        BodyDef wd;
        wd.type = BodyType::Dynamic;
        wd.shape = Shape::circle(cfg.wheelRadius);
        wd.material.density = 400;
        wd.material.staticFriction = cfg.trackGrip;
        wd.material.dynamicFriction = cfg.trackGrip * 0.85;
        wd.material.rollingFriction = 0.0;
        wd.position = cfg.position + Vec2(x, ay);
        wd.allowSleep = false;
        wd.name = "roadwheel";
        BodyId id = w.createBody(wd);
        w.createRevolute(hull_, id, Vec2(x, ay), Vec2(0, 0));
        wheels_.push_back(id);
    }
}
void TrackedVehicle::update(real dt, real leftThrottle, real rightThrottle)
{
    if(!world_) return;
    RigidBody* h = world_->body(hull_);
    if(!h) return;
    h->wake();
    Vec2 fwd(std::cos((double)h->angle), std::sin((double)h->angle));
    real vfwd = h->velocity.x * fwd.x + h->velocity.y * fwd.y;
    trackSpeed_ = vfwd;
    real drive = (leftThrottle + rightThrottle) * 0.5;
    real thrust = cfg_.drivePower / std::max((real)1.5, std::fabs((double)vfwd)) * drive;
    thrust = cl(thrust, -cfg_.mass * 12.0, cfg_.mass * 12.0);
    contact_ = 0;
    for(size_t i = 0; i < wheels_.size(); ++i){
        RigidBody* wb = world_->body(wheels_[i]);
        if(!wb) continue;
        wb->wake();
        contact_ += cfg_.wheelRadius * 2.0;
    }
    h->applyForce(fwd * thrust);
    h->applyTorque((leftThrottle - rightThrottle) * cfg_.mass * 0.6);
    (void)dt;
}
void TrackedVehicle::postStep() {}
real TrackedVehicle::groundPressure() const
{
    real area = std::max((real)0.5, contact_ * 0.6);
    return cfg_.mass * 9.81 / area / 1000.0;
}

void Aircraft::create(World& w, const Config& cfg, const std::vector<Wing>& wings)
{
    world_ = &w;
    cfg_ = cfg;
    wings_ = wings;
    BodyDef bd;
    bd.type = BodyType::Dynamic;
    bd.shape = Shape::box(cfg.halfWidth * 2.0, cfg.halfHeight * 2.0);
    bd.material.density = cfg.mass / (cfg.halfWidth * 2.0 * cfg.halfHeight * 2.0);
    bd.material.restitution = 0.05;
    bd.position = cfg.position;
    bd.allowSleep = false;
    bd.name = "aircraft";
    body_ = w.createBody(bd);
}
void Aircraft::update(real dt, real throttle, real pitchInput)
{
    if(!world_) return;
    RigidBody* b = world_->body(body_);
    if(!b) return;
    b->wake();
    Vec2 fwd(std::cos((double)b->angle), std::sin((double)b->angle));
    Vec2 up(-fwd.y, fwd.x);
    Vec2 vel = b->velocity - wind_;
    airspeed_ = vlen(vel);
    if(cfg_.rotorcraft){
        b->applyForce(up * (cfg_.rotorThrust * throttle));
        b->applyTorque(-b->angularVelocity * cfg_.mass * 1.2 + pitchInput * cfg_.mass * 2.0);
        lift_ = cfg_.rotorThrust * throttle;
        drag_ = 0.5 * cfg_.airDensity * airspeed_ * airspeed_ * 1.2;
        b->applyForce(vel * (airspeed_ > 1e-6 ? -drag_ / airspeed_ : 0.0));
        aoa_ = 0;
        stalled_ = false;
        (void)dt;
        return;
    }
    b->applyForce(fwd * (cfg_.thrust * throttle));
    real qd = 0.5 * cfg_.airDensity * airspeed_ * airspeed_;
    real totalLift = 0, totalDrag = 0;
    stalled_ = false;
    if(airspeed_ > 1e-3){
        Vec2 vdir = vel * (1.0 / airspeed_);
        real cosA = vdir.x * fwd.x + vdir.y * fwd.y;
        real sinA = vdir.x * up.x + vdir.y * up.y;
        aoa_ = -std::atan2((double)sinA, (double)cosA);
        for(size_t i = 0; i < wings_.size(); ++i){
            const Wing& wg = wings_[i];
            real a = aoa_ + wg.controlGain * pitchInput;
            real cL = wg.liftSlope * a;
            if(std::fabs((double)a) > wg.stallAngle){
                real over = std::fabs((double)a) - wg.stallAngle;
                cL *= std::max((real)0.15, 1.0 - 2.2 * over);
                stalled_ = true;
            }
            real L = qd * wg.area * cL;
            real D = qd * wg.area * (wg.dragBase + 0.06 * cL * cL);
            totalLift += L;
            totalDrag += D;
            real ca = std::cos((double)b->angle), sa = std::sin((double)b->angle);
            Vec2 r(wg.offset.x * ca - wg.offset.y * sa, wg.offset.x * sa + wg.offset.y * ca);
            b->applyForceAtPoint(up * L - vdir * D, b->position + r);
        }
    }
    lift_ = totalLift;
    drag_ = totalDrag;
    b->applyTorque(-b->angularVelocity * cfg_.mass * 0.9);
    (void)dt;
}

// ================================================================ 10. OCEAN
void Ocean::makeSea(int waveCount, real windSpeed, uint32_t seed)
{
    waves_.clear();
    SRng rng(seed);
    real g = 9.81;
    real peakLen = std::max((real)2.0, 6.2831853 * windSpeed * windSpeed / (g * 0.769));
    for(int i = 0; i < waveCount; ++i){
        Wave w;
        real spread = rng.range(0.45, 1.6);
        w.length = peakLen * spread;
        real k = 6.2831853 / w.length;
        w.speed = std::sqrt((double)(g / k));
        w.amplitude = 0.0081 * windSpeed * windSpeed / g * spread;
        w.phase = rng.range(0, 6.2831853);
        w.steepness = cl(0.55 / (1.0 + 0.35 * (real)i), 0.1, 0.9);
        waves_.push_back(w);
    }
}
void Ocean::setShore(real shoreX, real slope, real deepDepth){ shoreX_ = shoreX; slope_ = slope; deepDepth_ = deepDepth; }
real Ocean::depth(real x) const
{
    if(shoreX_ > 1e29) return deepDepth_;
    real d = deepDepth_ - (x - (shoreX_ - deepDepth_ / std::max((real)1e-4, slope_))) * slope_;
    return cl(d, 0.2, deepDepth_);
}
real Ocean::height(real x, real t) const
{
    real h = level_;
    real shoal = std::pow((double)cl(deepDepth_ / std::max((real)0.2, depth(x)), 1.0, 25.0), 0.25);
    for(size_t i = 0; i < waves_.size(); ++i){
        const Wave& w = waves_[i];
        real k = 6.2831853 / w.length;
        h += w.amplitude * shoal * std::cos((double)(k * x - k * w.speed * t + w.phase));
    }
    return h;
}
Vec2 Ocean::surfacePoint(real x, real t) const
{
    real px = x;
    for(size_t i = 0; i < waves_.size(); ++i){
        const Wave& w = waves_[i];
        real k = 6.2831853 / w.length;
        px -= w.steepness * w.amplitude * std::sin((double)(k * x - k * w.speed * t + w.phase));
    }
    return Vec2(px, height(x, t));
}
Vec2 Ocean::orbitalVelocity(const Vec2& at, real t) const
{
    Vec2 v(0, 0);
    for(size_t i = 0; i < waves_.size(); ++i){
        const Wave& w = waves_[i];
        real k = 6.2831853 / w.length;
        real ph = k * at.x - k * w.speed * t + w.phase;
        real decay = std::exp((double)(-k * std::max((real)0.0, level_ - at.y)));
        v.x += w.speed * k * w.amplitude * decay * std::cos((double)ph);
        v.y += w.speed * k * w.amplitude * decay * std::sin((double)ph);
    }
    return v;
}
real Ocean::breakingIntensity(real x, real t) const
{
    real d = depth(x);
    real h = 2.0 * (height(x, t) - level_);
    real ratio = std::fabs((double)h) / std::max((real)0.2, d);
    return cl((ratio - 0.55) / 0.45, 0.0, 1.0);
}
void Ocean::applyBuoyancy(real t, real dt)
{
    if(!world_) return;
    floating_ = 0;
    uint32_t limit = (uint32_t)world_->bodyCount() + 64u;
    for(uint32_t id = 0; id <= limit; ++id){
        RigidBody* b = world_->body((BodyId)id);
        if(!b || b->type != BodyType::Dynamic) continue;
        real r = b->boundingRadius;
        if(r > 12.0) continue;
        real stripW = r * 1.4 * 2.0 / 8.0;
        bool wet = false;
        for(int s = 0; s < 8; ++s){
            real sx = b->position.x - r * 1.4 + stripW * ((real)s + 0.5);
            real surf = height(sx, t);
            real bottom = b->position.y - r;
            if(bottom >= surf) continue;
            wet = true;
            real sub = std::min(surf - bottom, r * 2.0);
            real area = stripW * sub;
            real F = fluidDensity * 9.81 * area;
            b->applyForceAtPoint(Vec2(0, F), Vec2(sx, bottom + sub * 0.5));
            Vec2 vrel = b->velocity - orbitalVelocity(Vec2(sx, surf), t);
            real sp = vlen(vrel);
            if(sp > 1e-6){
                real D = 0.5 * fluidDensity * dragCoefficient * area * sp;
                b->applyForce(vrel * (-D / sp));
            }
        }
        if(wet){
            ++floating_;
            b->applyTorque(-b->angularVelocity * angularDrag * fluidDensity * 0.02);
            b->wake();
        }
    }
}

} // namespace nova
} // namespace phys2d
