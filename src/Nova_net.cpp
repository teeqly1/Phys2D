#include "phys2d/Nova.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace phys2d {
namespace nova {

static inline real nlen(const Vec2& v){ return std::sqrt((double)(v.x * v.x + v.y * v.y)); }
static inline real ncl(real v, real a, real b){ return v < a ? a : (v > b ? b : v); }

const NetBodyState* NetSnapshot::find(uint32_t id) const
{
    for(size_t i = 0; i < bodies.size(); ++i) if(bodies[i].id == id) return &bodies[i];
    return nullptr;
}

// ------------------------------------------------------- binary helpers
static void putU32(std::vector<uint8_t>& out, uint32_t v)
{
    out.push_back((uint8_t)(v & 0xFF));
    out.push_back((uint8_t)((v >> 8) & 0xFF));
    out.push_back((uint8_t)((v >> 16) & 0xFF));
    out.push_back((uint8_t)((v >> 24) & 0xFF));
}
static bool getU32(const std::vector<uint8_t>& in, size_t& p, uint32_t& v)
{
    if(p + 4 > in.size()) return false;
    v = (uint32_t)in[p] | ((uint32_t)in[p+1] << 8) | ((uint32_t)in[p+2] << 16) | ((uint32_t)in[p+3] << 24);
    p += 4;
    return true;
}
static void putVar(std::vector<uint8_t>& out, long long value)
{
    uint64_t zz = (uint64_t)((value << 1) ^ (value >> 63));
    while(zz >= 0x80){ out.push_back((uint8_t)(zz | 0x80)); zz >>= 7; }
    out.push_back((uint8_t)zz);
}
static bool getVar(const std::vector<uint8_t>& in, size_t& p, long long& value)
{
    uint64_t zz = 0;
    int shift = 0;
    while(true){
        if(p >= in.size() || shift > 63) return false;
        uint8_t b = in[p++];
        zz |= (uint64_t)(b & 0x7F) << shift;
        if(!(b & 0x80)) break;
        shift += 7;
    }
    value = (long long)((zz >> 1) ^ (~(zz & 1) + 1));
    return true;
}
static inline long long q(real v, real step){ return (long long)std::llround((double)(v / step)); }

size_t SnapshotCodec::rawSize(const NetSnapshot& s)
{ return 8 + s.bodies.size() * sizeof(NetBodyState); }

std::vector<uint8_t> SnapshotCodec::encode(const NetSnapshot& cur, const NetSnapshot* base)
{
    std::vector<uint8_t> out;
    out.reserve(cur.bodies.size() * 8 + 16);
    putU32(out, cur.tick);
    size_t countPos = out.size();
    putU32(out, 0);
    uint32_t written = 0;
    for(size_t i = 0; i < cur.bodies.size(); ++i){
        const NetBodyState& b = cur.bodies[i];
        const NetBodyState* old = base ? base->find(b.id) : nullptr;
        long long dx = q(b.x, posQuantum()), dy = q(b.y, posQuantum());
        long long da = q(b.angle, angQuantum());
        long long dvx = q(b.vx, velQuantum()), dvy = q(b.vy, velQuantum());
        long long dw = q(b.w, velQuantum());
        if(old){
            long long ox = q(old->x, posQuantum()), oy = q(old->y, posQuantum());
            long long oa = q(old->angle, angQuantum());
            long long ovx = q(old->vx, velQuantum()), ovy = q(old->vy, velQuantum());
            long long ow = q(old->w, velQuantum());
            if(dx == ox && dy == oy && da == oa && dvx == ovx && dvy == ovy && dw == ow) continue;
            dx -= ox; dy -= oy; da -= oa; dvx -= ovx; dvy -= ovy; dw -= ow;
        }
        putVar(out, (long long)b.id);
        putVar(out, dx); putVar(out, dy); putVar(out, da);
        putVar(out, dvx); putVar(out, dvy); putVar(out, dw);
        ++written;
    }
    out[countPos]     = (uint8_t)(written & 0xFF);
    out[countPos + 1] = (uint8_t)((written >> 8) & 0xFF);
    out[countPos + 2] = (uint8_t)((written >> 16) & 0xFF);
    out[countPos + 3] = (uint8_t)((written >> 24) & 0xFF);
    return out;
}

bool SnapshotCodec::decode(const std::vector<uint8_t>& data, const NetSnapshot* base, NetSnapshot& outSnap)
{
    size_t p = 0;
    uint32_t tick = 0, count = 0;
    if(!getU32(data, p, tick)) return false;
    if(!getU32(data, p, count)) return false;
    outSnap.tick = tick;
    outSnap.bodies.clear();
    if(base) outSnap.bodies = base->bodies;
    for(uint32_t i = 0; i < count; ++i){
        long long id = 0, dx = 0, dy = 0, da = 0, dvx = 0, dvy = 0, dw = 0;
        if(!getVar(data, p, id)) return false;
        if(!getVar(data, p, dx) || !getVar(data, p, dy) || !getVar(data, p, da)) return false;
        if(!getVar(data, p, dvx) || !getVar(data, p, dvy) || !getVar(data, p, dw)) return false;
        NetBodyState st;
        st.id = (uint32_t)id;
        const NetBodyState* old = base ? base->find(st.id) : nullptr;
        long long bx = old ? q(old->x, posQuantum()) : 0;
        long long by = old ? q(old->y, posQuantum()) : 0;
        long long ba = old ? q(old->angle, angQuantum()) : 0;
        long long bvx = old ? q(old->vx, velQuantum()) : 0;
        long long bvy = old ? q(old->vy, velQuantum()) : 0;
        long long bw = old ? q(old->w, velQuantum()) : 0;
        if(!old){ bx = 0; by = 0; ba = 0; bvx = 0; bvy = 0; bw = 0; }
        st.x = (float)((bx + dx) * posQuantum());
        st.y = (float)((by + dy) * posQuantum());
        st.angle = (float)((ba + da) * angQuantum());
        st.vx = (float)((bvx + dvx) * velQuantum());
        st.vy = (float)((bvy + dvy) * velQuantum());
        st.w = (float)((bw + dw) * velQuantum());
        bool replaced = false;
        for(size_t k = 0; k < outSnap.bodies.size(); ++k)
            if(outSnap.bodies[k].id == st.id){ outSnap.bodies[k] = st; replaced = true; break; }
        if(!replaced) outSnap.bodies.push_back(st);
    }
    return true;
}

static NetSnapshot captureWorld(World* w, uint32_t tick)
{
    NetSnapshot s;
    s.tick = tick;
    if(!w) return s;
    uint32_t limit = (uint32_t)w->bodyCount() + 64u;
    for(uint32_t id = 0; id <= limit; ++id){
        const RigidBody* b = w->body((BodyId)id);
        if(!b || b->type == BodyType::Static) continue;
        NetBodyState st;
        st.id = id;
        st.x = (float)b->position.x;
        st.y = (float)b->position.y;
        st.angle = (float)b->angle;
        st.vx = (float)b->velocity.x;
        st.vy = (float)b->velocity.y;
        st.w = (float)b->angularVelocity;
        s.bodies.push_back(st);
    }
    return s;
}
static void applySnapshotToWorld(World* w, const NetSnapshot& s, real blend)
{
    if(!w) return;
    for(size_t i = 0; i < s.bodies.size(); ++i){
        const NetBodyState& st = s.bodies[i];
        RigidBody* b = w->body((BodyId)st.id);
        if(!b) continue;
        Vec2 target((real)st.x, (real)st.y);
        b->position = b->position + (target - b->position) * blend;
        b->angle += ((real)st.angle - b->angle) * blend;
        b->velocity = Vec2((real)st.vx, (real)st.vy);
        b->angularVelocity = (real)st.w;
        b->wake();
    }
}

// -------------------------------------------------------- server / client
void ServerAuthority::step(real dt)
{
    if(!world_) return;
    for(size_t i = 0; i < pending_.size(); ++i)
        if(handler_) handler_(*world_, pending_[i]);
    pending_.clear();
    world_->step(dt);
    ++tick_;
    snaps_.push_back(captureWorld(world_, tick_));
    while((int)snaps_.size() > history_) snaps_.erase(snaps_.begin());
}
NetSnapshot ServerAuthority::capture() const
{
    if(!snaps_.empty()) return snaps_.back();
    return captureWorld(world_, tick_);
}
bool ServerAuthority::rewind(uint32_t tick)
{
    if(!world_) return false;
    for(size_t i = 0; i < snaps_.size(); ++i){
        if(snaps_[i].tick != tick) continue;
        if(!savedValid_){ saved_ = captureWorld(world_, tick_); savedValid_ = true; }
        applySnapshotToWorld(world_, snaps_[i], 1.0);
        return true;
    }
    return false;
}
void ServerAuthority::restore()
{
    if(!world_ || !savedValid_) return;
    applySnapshotToWorld(world_, saved_, 1.0);
    savedValid_ = false;
}
RigidBody* ServerAuthority::rewindQueryPoint(uint32_t tick, const Vec2& at)
{
    if(!world_) return nullptr;
    bool ok = rewind(tick);
    RigidBody* hit = nullptr;
    uint32_t limit = (uint32_t)world_->bodyCount() + 64u;
    for(uint32_t id = 0; id <= limit && !hit; ++id){
        RigidBody* b = world_->body((BodyId)id);
        if(!b || b->type == BodyType::Static) continue;
        if(nlen(b->position - at) <= b->boundingRadius) hit = b;
    }
    if(ok) restore();
    return hit;
}

void ClientPredictor::predict(real dt)
{
    if(!world_) return;
    dt_ = dt;
    ++tick_;
    NetInput in;
    in.tick = tick_;
    if(!inputs_.empty()) in = inputs_.back();
    if(handler_) handler_(*world_, in);
    world_->step(dt);
}
void ClientPredictor::applySnapshot(const NetSnapshot& s, uint32_t ackTick)
{
    if(!world_) return;
    real err = 0;
    int n = 0;
    for(size_t i = 0; i < s.bodies.size(); ++i){
        const RigidBody* b = world_->body((BodyId)s.bodies[i].id);
        if(!b) continue;
        err += nlen(b->position - Vec2((real)s.bodies[i].x, (real)s.bodies[i].y));
        ++n;
    }
    error_ = n ? err / (real)n : 0.0;
    applySnapshotToWorld(world_, s, error_ > 0.5 ? 1.0 : smoothing_);
    std::vector<NetInput> keep;
    for(size_t i = 0; i < inputs_.size(); ++i)
        if(inputs_[i].tick > ackTick) keep.push_back(inputs_[i]);
    inputs_.swap(keep);
    for(size_t i = 0; i < inputs_.size(); ++i){
        if(handler_) handler_(*world_, inputs_[i]);
        world_->step(dt_);
    }
}

void InterpolationBuffer::push(const NetSnapshot& s)
{
    snaps_.push_back(s);
    while(snaps_.size() > 64) snaps_.erase(snaps_.begin());
}
bool InterpolationBuffer::sample(real now, NetSnapshot& out) const
{
    if(snaps_.empty()) return false;
    real target = now - delay_;
    const NetSnapshot* a = nullptr;
    const NetSnapshot* b = nullptr;
    for(size_t i = 0; i < snaps_.size(); ++i){
        if(snaps_[i].time <= target) a = &snaps_[i];
        else { b = &snaps_[i]; break; }
    }
    if(!a){ out = snaps_.front(); return true; }
    if(!b){ out = *a; return true; }
    real span = (real)(b->time - a->time);
    real t = span > 1e-9 ? ncl((target - (real)a->time) / span, 0.0, 1.0) : 0.0;
    out.tick = b->tick;
    out.time = target;
    out.bodies.clear();
    for(size_t i = 0; i < a->bodies.size(); ++i){
        const NetBodyState& sa = a->bodies[i];
        const NetBodyState* sb = b->find(sa.id);
        NetBodyState st = sa;
        if(sb){
            st.x = (float)(sa.x + (sb->x - sa.x) * t);
            st.y = (float)(sa.y + (sb->y - sa.y) * t);
            st.angle = (float)(sa.angle + (sb->angle - sa.angle) * t);
            st.vx = (float)(sa.vx + (sb->vx - sa.vx) * t);
            st.vy = (float)(sa.vy + (sb->vy - sa.vy) * t);
            st.w = (float)(sa.w + (sb->w - sa.w) * t);
        }
        out.bodies.push_back(st);
    }
    return true;
}

// ------------------------------------------------------------- acoustics
static bool segIntersect(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec2& p4)
{
    real d1 = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    real d2 = (p2.x - p1.x) * (p4.y - p1.y) - (p2.y - p1.y) * (p4.x - p1.x);
    real d3 = (p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x);
    real d4 = (p4.x - p3.x) * (p2.y - p3.y) - (p4.y - p3.y) * (p2.x - p3.x);
    return (d1 * d2 < 0) && (d3 * d4 < 0);
}
void Acoustics::addWall(const Vec2& a, const Vec2& b, int material)
{
    Wall w;
    w.a = a; w.b = b; w.material = material;
    walls_.push_back(w);
}
void Acoustics::setRoom(real volume, real surface){ volume_ = volume; surface_ = surface; }
real Acoustics::reverbTime60() const
{
    const MaterialLibrary& lib = MaterialLibrary::instance();
    real absorb = surface_ * 0.15;
    for(size_t i = 0; i < walls_.size(); ++i){
        real len = nlen(walls_[i].b - walls_[i].a);
        absorb += len * 3.0 * (real)lib.byIndex(walls_[i].material).sound.absorption;
    }
    if(absorb < 1e-4) absorb = 1e-4;
    return 0.161 * volume_ / absorb;
}
real Acoustics::dopplerFor(const Vec2& sourcePos, const Vec2& sourceVel) const
{
    Vec2 d = listener_ - sourcePos;
    real dist = nlen(d);
    if(dist < 1e-6) return 1.0;
    Vec2 dir = d * (1.0 / dist);
    real vs = sourceVel.x * dir.x + sourceVel.y * dir.y;
    real vl = listenerVel_.x * dir.x + listenerVel_.y * dir.y;
    real f = (speed_ - vl) / std::max((real)1.0, speed_ - vs);
    return ncl(f, 0.25, 4.0);
}
Voice Acoustics::impact(const Vec2& at, real impulse, int matA, int matB)
{
    const MaterialLibrary& lib = MaterialLibrary::instance();
    Voice v;
    v.position = at;
    v.frequency = lib.impactHz(matA, matB, impulse);
    real dist = std::max((real)0.5, nlen(listener_ - at));
    real occl = 1.0;
    for(size_t i = 0; i < walls_.size(); ++i)
        if(segIntersect(at, listener_, walls_[i].a, walls_[i].b))
            occl *= 1.0 - 0.55 * (real)lib.byIndex(walls_[i].material).sound.absorption;
    v.gain = ncl(std::log10(1.0 + std::fabs((double)impulse)) * 0.6 * occl / (dist * dist) * 8.0, 0.0, 1.0);
    v.delay = dist / speed_;
    v.pan = ncl((at.x - listener_.x) / 20.0, -1.0, 1.0);
    v.pitch = dopplerFor(at, Vec2(0, 0));
    v.reverbTail = reverbTime60();
    v.materialA = matA;
    v.materialB = matB;
    voices_.push_back(v);
    return v;
}
Voice Acoustics::footstep(const Vec2& at, int surfaceMaterial)
{
    const MaterialLibrary& lib = MaterialLibrary::instance();
    Voice v;
    v.position = at;
    v.frequency = (real)lib.byIndex(surfaceMaterial).sound.footstepHz;
    real dist = std::max((real)0.5, nlen(listener_ - at));
    v.gain = ncl(0.6 / (dist * 0.35 + 1.0), 0.0, 1.0);
    v.delay = dist / speed_;
    v.pan = ncl((at.x - listener_.x) / 20.0, -1.0, 1.0);
    v.reverbTail = reverbTime60() * 0.5;
    v.materialA = surfaceMaterial;
    v.materialB = surfaceMaterial;
    voices_.push_back(v);
    return v;
}
void Acoustics::update(real dt)
{
    size_t w = 0;
    for(size_t i = 0; i < voices_.size(); ++i){
        Voice v = voices_[i];
        real tail = v.reverbTail + 0.05;
        v.gain *= std::exp((double)(-dt * 6.9078 / tail));
        if(v.gain > 0.001) voices_[w++] = v;
    }
    voices_.resize(w);
}
real Acoustics::masterLevel() const
{
    real s = 0;
    for(size_t i = 0; i < voices_.size(); ++i) s += voices_[i].gain;
    return ncl(s, 0.0, 4.0);
}

// ----------------------------------------------------------------- suite
void NovaSuite::attachAll(World& w)
{
    depth.attach(w);
    server.attach(w);
    client.attach(w);
    cloth.attach(w);
    grains.attach(w);
    ocean.attach(w);
    audio.attach(w);
    power.attach(w);
    editor.attach(w);
    lua.bindWorld(w);
}
void NovaSuite::update(real dt)
{
    cloth.update(dt, 6);
    weather.update(dt, grains);
    grains.update(dt);
    audio.update(dt);
    power.update(dt);
}
std::string NovaSuite::report() const
{
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "materials %d | grains %d | cloth nodes %d | waves %d | voices %d | rt60 %.2f s | simd %s | power %.1f W",
        MaterialLibrary::instance().count(),
        (int)grains.count(),
        (int)cloth.nodeCount(),
        ocean.waveCount(),
        (int)audio.voices().size(),
        (double)audio.reverbTime60(),
        simdBackend(),
        (double)power.estimatedPowerW());
    return std::string(buf);
}

} // namespace nova
} // namespace phys2d
