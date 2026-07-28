// phys2d - detailed blood / gore simulation layer (implementation).
#include "phys2d/Blood.h"

#include <algorithm>
#include <cmath>

namespace phys2d {

namespace {
constexpr real kTau = 6.28318530717958647692;

inline real clamp01(real v) { return (v < 0.0) ? 0.0 : (v > 1.0 ? 1.0 : v); }

inline Vec2 safeNormal(const Vec2& v, const Vec2& fallback) {
    const real len = v.length();
    if (!std::isfinite(len) || len < 1e-9) return fallback;
    return v / len;
}
} // namespace

// ------------------------------------------------------------------ rng
real BloodSystem::rnd() {
    m_rng = m_rng * 1664525u + 1013904223u;
    return (real)((m_rng >> 8) & 0xFFFFFFu) / (real)0x1000000u;
}
real BloodSystem::rnd(real lo, real hi) { return lo + (hi - lo) * rnd(); }

// ------------------------------------------------------------------ colour
void BloodSystem::colorOf(uint8_t oxygen, real wetness, uint8_t& r, uint8_t& g, uint8_t& b) {
    const real ox = (real)oxygen / 255.0;
    const real w  = clamp01(wetness);
    // fresh: arterial 196,28,32  venous 118,16,28 ; dried: 74,26,22 -> 52,24,22
    const real fr = 118.0 + 78.0 * ox;
    const real fg =  16.0 + 12.0 * ox;
    const real fb =  28.0 +  4.0 * ox;
    const real dr =  52.0 + 22.0 * ox;
    const real dg =  24.0 +  2.0 * ox;
    const real db =  22.0 +  1.0 * ox;
    r = (uint8_t)std::lround(dr + (fr - dr) * w);
    g = (uint8_t)std::lround(dg + (fg - dg) * w);
    b = (uint8_t)std::lround(db + (fb - db) * w);
}

// ------------------------------------------------------------------ emission
void BloodSystem::emit(const Vec2& p, const Vec2& v, real volumeMl, BloodKind kind, uint8_t oxygen) {
    if (m_drops.size() >= maxDroplets) return;
    if (!p.isFinite() || !v.isFinite()) return;
    BloodDroplet d;
    d.position = p;
    d.previous = p;
    d.velocity = v;
    d.volume   = std::max((real)0.02, volumeMl);
    d.radius   = dropletRadius * std::cbrt(d.volume / std::max((real)1e-3, dropletVolume));
    d.kind     = kind;
    d.oxygen   = oxygen;
    d.spin     = rnd(-6.0, 6.0);
    switch (kind) {
        case BloodKind::Mist:  d.life = rnd(0.5, 1.4);  d.radius *= 0.45; break;
        case BloodKind::Gush:  d.life = rnd(9.0, 16.0); d.radius *= 1.7;  break;
        case BloodKind::Clot:  d.life = rnd(12.0, 22.0); d.radius *= 1.3; break;
        default:               d.life = rnd(8.0, 15.0); break;
    }
    // Emission can happen from inside the droplet loop (cast-off spatter), so
    // never touch m_drops while it is being iterated - that would invalidate
    // the references held by the caller.
    if (m_iterating) m_pending.push_back(d);
    else             m_drops.push_back(d);
    ++m_stats.dropletsSpawned;
}

void BloodSystem::spray(const Vec2& origin, const Vec2& dir, real speed, real coneRad,
                        int count, real volumeMl, uint8_t oxygen) {
    const Vec2 d0 = safeNormal(dir, Vec2(0.0, 1.0));
    const real per = (count > 0) ? volumeMl / (real)count : volumeMl;
    for (int i = 0; i < count; ++i) {
        const real a  = rnd(-coneRad, coneRad);
        const Vec2 dv = d0.rotated(a);
        const real sp = speed * rnd(0.55, 1.25);
        BloodKind kind = BloodKind::Droplet;
        if (enableMist && rnd() < 0.28) kind = BloodKind::Mist;
        else if (rnd() < 0.12)          kind = BloodKind::Gush;
        emit(origin + dv * 0.05, dv * sp, per * rnd(0.5, 1.8), kind, oxygen);
    }
}

void BloodSystem::splash(const Vec2& center, real speed, int count, real volumeMl, uint8_t oxygen) {
    const real per = (count > 0) ? volumeMl / (real)count : volumeMl;
    for (int i = 0; i < count; ++i) {
        const real a = rnd(0.0, kTau);
        const Vec2 dv(std::cos(a), std::sin(a));
        emit(center, dv * (speed * rnd(0.2, 1.0)), per * rnd(0.4, 2.0),
             (rnd() < 0.3 && enableMist) ? BloodKind::Mist : BloodKind::Droplet, oxygen);
    }
}

// ------------------------------------------------------------------ wounds
Wound* BloodSystem::addWound(BodyId body, const Vec2& worldPoint, const Vec2& worldDir,
                             real severity, bool arterial) {
    if (!m_world) return nullptr;
    RigidBody* b = m_world->body(body);
    if (!b) return nullptr;

    const Vec2 localHit = b->transform().invApply(worldPoint);

    // A cluster of hits in the same spot is one deeper wound, not fifty.
    for (Wound& ex : m_wounds) {
        if (ex.id != body) continue;
        if ((ex.local - localHit).length() > woundMergeDist) continue;
        ex.severity  = clamp01(ex.severity + 0.35 * clamp01(severity));
        ex.clot      = ex.clot * 0.45;
        ex.reservoir = std::max(ex.reservoir, 260.0 + 900.0 * ex.severity);
        ex.arterial  = ex.arterial || arterial;
        spray(worldPoint, safeNormal(worldDir, Vec2(0.0, 1.0)),
              3.0 + 7.0 * ex.severity, 0.6, (int)(4 + 14 * ex.severity),
              2.0 + 10.0 * ex.severity, arterial ? 235 : 190);
        stainBody(body, 0.12 * ex.severity);
        return &ex;
    }

    // Budget: recycle the most clotted / emptiest wound instead of growing.
    if (m_wounds.size() >= maxWounds) {
        size_t worst = 0;
        real   score = -1.0;
        for (size_t i = 0; i < m_wounds.size(); ++i) {
            const real sc = m_wounds[i].clot * 100.0 - m_wounds[i].reservoir * 0.01 + m_wounds[i].age;
            if (sc > score) { score = sc; worst = i; }
        }
        m_wounds.erase(m_wounds.begin() + (long)worst);
    }

    Wound w;
    w.id        = body;
    w.local     = localHit;
    w.localDir  = safeNormal(worldDir, Vec2(0.0, 1.0)).rotated(-b->angle);
    w.severity  = clamp01(severity);
    w.arterial  = arterial;
    w.reservoir = 320.0 + 1400.0 * w.severity;
    m_wounds.push_back(w);

    // initial burst
    const Vec2 dir = safeNormal(worldDir, Vec2(0.0, 1.0));
    spray(worldPoint, dir, 3.5 + 9.0 * w.severity, 0.55,
          (int)(8 + 40 * w.severity), 4.0 + 26.0 * w.severity, arterial ? 235 : 190);
    stainBody(body, 0.25 * w.severity);
    return &m_wounds.back();
}

Wound* BloodSystem::sever(BodyId body, const Vec2& worldPoint, const Vec2& worldDir, real severity) {
    Wound* w = addWound(body, worldPoint, worldDir, severity, true);
    if (!w) return nullptr;
    w->severed   = true;
    w->reservoir = 900.0 + 1800.0 * clamp01(severity);
    const Vec2 dir = safeNormal(worldDir, Vec2(0.0, 1.0));
    spray(worldPoint, dir, 11.0, 0.85, 90, 70.0, 240);
    splash(worldPoint, 5.0, 40, 30.0, 210);
    return w;
}

void BloodSystem::healWound(BodyId body) {
    m_wounds.erase(std::remove_if(m_wounds.begin(), m_wounds.end(),
                                  [body](const Wound& w) { return w.id == body; }),
                   m_wounds.end());
}

void BloodSystem::stainBody(BodyId body, real amount) {
    if (!stainBodies || body == INVALID_BODY) return;
    BloodStain& s = m_stains[body];
    s.wetness  = clamp01(s.wetness + amount);
    s.coverage += amount;
}

real BloodSystem::wetnessOf(BodyId body) const {
    auto it = m_stains.find(body);
    return (it == m_stains.end()) ? 0.0 : it->second.wetness;
}

void BloodSystem::forgetBody(BodyId body) {
    healWound(body);
    m_stains.erase(body);
    // decals attached to a dead body fall to world space so the gore stays
    for (BloodDecal& d : m_decals) {
        if (d.body == body) d.body = INVALID_BODY;
    }
}

void BloodSystem::clear() {
    m_drops.clear();
    m_pending.clear();
    m_iterating = false;
    m_decals.clear();
    m_pools.clear();
    m_wounds.clear();
    m_stains.clear();
    m_stats = BloodStats{};
}

// ------------------------------------------------------------------ splats
void BloodSystem::makeDecal(BodyId body, const Vec2& worldPoint, const Vec2& normal,
                            const Vec2& incoming, real volumeMl, uint8_t oxygen, real speed) {
    if (m_decals.size() >= maxDecals) {
        // recycle the oldest, driest mark
        size_t victim = 0;
        real   worst  = -1.0;
        for (size_t i = 0; i < m_decals.size(); i += 7) {
            if (m_decals[i].age > worst) { worst = m_decals[i].age; victim = i; }
        }
        m_decals.erase(m_decals.begin() + (long)victim);
    }

    BloodDecal d;
    d.body   = body;
    d.volume = volumeMl;
    d.oxygen = oxygen;

    RigidBody* b = (body != INVALID_BODY && m_world) ? m_world->body(body) : nullptr;
    const Transform xf = b ? b->transform() : Transform(Vec2(), 0.0);
    d.anchor = b ? xf.invApply(worldPoint) : worldPoint;

    // Base radius from volume, elongation from impact speed and angle.
    const real base    = 0.055 * std::cbrt(std::max((real)0.05, volumeMl)) * 2.6;
    const Vec2 dirW    = safeNormal(incoming, Vec2(0.0, -1.0));
    const real grazing = clamp01(1.0 - std::fabs(dot(dirW, safeNormal(normal, Vec2(0.0, 1.0)))));
    const real stretch = 1.0 + grazing * (0.9 + 0.16 * std::min(speed, (real)26.0));

    // Local frame: x along the travel direction, y across it.
    Vec2 ax = dirW;
    if (b) ax = ax.rotated(-b->angle);
    const Vec2 ay = ax.perp();

    const int lobes = std::max(5, splatterLobes + (int)(speed * 0.25));
    d.outline.reserve((size_t)lobes);
    for (int i = 0; i < lobes; ++i) {
        const real t   = kTau * (real)i / (real)lobes;
        const real jag = 0.62 + rnd(0.0, 0.85) + 0.35 * std::sin(t * 3.0 + rnd(0.0, 2.0));
        const real rx  = base * stretch * jag * (0.55 + 0.75 * std::max((real)0.0, std::cos(t)));
        const real ry  = base * jag;
        d.outline.push_back(ax * (rx * std::cos(t)) + ay * (ry * std::sin(t)));
    }
    // Directional tail: fast grazing hits throw a spike forward.
    if (grazing > 0.45 && speed > 4.0) {
        d.outline.insert(d.outline.begin(), ax * (base * stretch * (1.6 + rnd(0.0, 1.4))));
    }

    m_decals.push_back(std::move(d));
    stainBody(body, 0.05 + 0.02 * std::min(volumeMl, (real)6.0));
    ++m_stats.impactsThisStep;

    // Secondary cast-off spatter from violent impacts.
    if (enableCastOff && speed > castOffSpeed && volumeMl > 0.15) {
        const Vec2 n = safeNormal(normal, Vec2(0.0, 1.0));
        const int  n2 = (int)std::min((real)9.0, 2.0 + speed * 0.22);
        for (int i = 0; i < n2; ++i) {
            const Vec2 dv = safeNormal(n.rotated(rnd(-1.1, 1.1)) + dirW * 0.4, n);
            emit(worldPoint + n * 0.02, dv * (speed * rnd(0.10, 0.32)),
                 volumeMl * rnd(0.03, 0.12),
                 (enableMist && rnd() < 0.5) ? BloodKind::Mist : BloodKind::Droplet, oxygen);
        }
    }
}

void BloodSystem::addToPool(const Vec2& worldPoint, real volumeMl, uint8_t oxygen) {
    if (!enablePools) return;
    for (BloodPool& p : m_pools) {
        if (std::fabs(p.center.x - worldPoint.x) < p.halfWidth + poolMergeDist &&
            std::fabs(p.center.y - worldPoint.y) < 0.45) {
            const real total = p.volume + volumeMl;
            p.center.x = (p.center.x * p.volume + worldPoint.x * volumeMl) / std::max(total, (real)1e-6);
            p.volume   = total;
            p.halfWidth = poolSpread * std::sqrt(p.volume) * 3.2;
            p.depth     = std::min((real)0.09, 0.004 + p.volume * 0.0007);
            p.wetness   = std::min((real)1.0, p.wetness + 0.25);
            p.age       = std::max((real)0.0, p.age - 0.5);
            return;
        }
    }
    if (m_pools.size() >= maxPools) return;
    BloodPool p;
    p.center    = worldPoint;
    p.volume    = volumeMl;
    p.halfWidth = poolSpread * std::sqrt(std::max(volumeMl, (real)0.05)) * 3.2;
    p.oxygen    = oxygen;
    m_pools.push_back(p);
}

// ------------------------------------------------------------------ impacts
void BloodSystem::resolveImpact(BloodDroplet& d, const RayHit& hit, real dt) {
    const Vec2 n     = safeNormal(hit.normal, Vec2(0.0, 1.0));
    const Vec2 dir   = safeNormal(d.velocity, Vec2(0.0, -1.0));
    const real speed = d.velocity.length();
    const BodyId id  = hit.body ? hit.body->id : INVALID_BODY;

    // Relative speed against a moving surface matters for smearing.
    real rel = speed;
    if (hit.body) rel = (d.velocity - hit.body->velocityAtPoint(hit.point)).length();

    const bool clot   = (d.kind == BloodKind::Clot);
    const real stickP = clot ? (stickiness * 0.35) : stickiness;
    const bool sticks = (rel < 1.2) || (rnd() < stickP);

    if (sticks) {
        makeDecal(id, hit.point + n * 0.005, n, dir, d.volume, d.oxygen, rel);
        // Blood landing on an upward facing surface starts a puddle.
        if (n.y > 0.55) addToPool(hit.point, d.volume, d.oxygen);
        if (hit.body && hit.body->isDynamic()) {
            // tiny momentum transfer keeps big gushes physical
            hit.body->applyImpulseAtPoint(d.velocity * (d.volume * 1.0e-4), hit.point);
        }
        d.active = false;
        return;
    }

    // Bounce, losing volume to a smaller mark on the way.
    const real lost = d.volume * rnd(0.25, 0.6);
    makeDecal(id, hit.point + n * 0.005, n, dir, lost, d.oxygen, rel);
    d.volume  = std::max((real)0.02, d.volume - lost);
    d.radius  = dropletRadius * std::cbrt(d.volume / std::max((real)1e-3, dropletVolume));
    const Vec2 vn = n * dot(d.velocity, n);
    const Vec2 vt = d.velocity - vn;
    d.velocity = vt * 0.72 - vn * bounceRestitution;
    d.position = hit.point + n * (d.radius + 0.01);
    d.previous = d.position;
    if (d.volume < 0.05) d.active = false;
    (void)dt;
}

void BloodSystem::integrateDroplets(real dt) {
    if (!m_world) return;

    m_iterating = true;
    for (size_t i = 0; i < m_drops.size(); ++i) {
        BloodDroplet& d = m_drops[i];
        if (!d.active) continue;
        d.age += dt;
        if (d.age > d.life) { d.active = false; continue; }

        const real drag = (d.kind == BloodKind::Mist) ? mistDrag : airDrag;
        const real sp   = d.velocity.length();
        Vec2 acc = gravity;
        if (d.kind == BloodKind::Mist) acc = gravity * 0.35;
        if (sp > 1e-6) acc -= d.velocity * (drag * sp * d.radius * 12.0);

        d.previous = d.position;
        d.velocity += acc * dt;
        d.position += d.velocity * dt;

        if (!d.position.isFinite() || !d.velocity.isFinite()) { d.active = false; continue; }

        // Mist just evaporates, it does not paint the world.
        if (d.kind == BloodKind::Mist && d.age > d.life * 0.8) continue;

        RayHit hit;
        if (rayCastClosest(*m_world, d.previous, d.position, hit) && hit.body) {
            resolveImpact(d, hit, dt);
            continue;
        }
        // Optional flat floor fallback (a world without a ground body).
        if (d.position.y < groundLevel) {
            const Vec2 p(d.position.x, groundLevel);
            makeDecal(INVALID_BODY, p, Vec2(0.0, 1.0), safeNormal(d.velocity, Vec2(0.0, -1.0)),
                      d.volume, d.oxygen, d.velocity.length());
            addToPool(p, d.volume, d.oxygen);
            d.active = false;
        }
    }

    m_iterating = false;
    if (!m_pending.empty()) {
        m_drops.insert(m_drops.end(), m_pending.begin(), m_pending.end());
        m_pending.clear();
    }

    m_drops.erase(std::remove_if(m_drops.begin(), m_drops.end(),
                                 [](const BloodDroplet& d) { return !d.active; }),
                  m_drops.end());
}

// ------------------------------------------------------------------ bleeding
void BloodSystem::bleedWounds(real dt) {
    if (!m_world) return;

    // Heartbeat: arterial wounds squirt in bursts, venous ones just ooze.
    m_pulse += dt * pulseHz * kTau;
    if (m_pulse > kTau) m_pulse -= kTau;
    const real beat = 0.5 + 0.5 * std::sin(m_pulse);
    const real jet  = 1.0 - pulseStrength + pulseStrength * std::pow(beat, 3.0);

    for (Wound& w : m_wounds) {
        RigidBody* b = m_world->body(w.id);
        if (!b) { w.reservoir = 0.0; continue; }

        w.age  += dt;
        w.clot  = clamp01(w.clot + clotRate * dt * (w.severed ? 0.25 : 1.0));
        const real open = (1.0 - w.clot) * w.severity;
        if (open <= 0.01 || w.reservoir <= 0.0) continue;

        const Transform xf = b->transform();
        const Vec2 origin  = xf.apply(w.local);
        const Vec2 dirW    = w.localDir.rotated(b->angle);
        if (!origin.isFinite()) continue;

        // Flow rate in ml/s, modulated by the heartbeat for arterial wounds.
        real flow = (w.severed ? 260.0 : 90.0) * open;
        if (w.arterial) flow *= jet;
        else            flow *= 0.55 + 0.45 * beat;

        w.emitAcc += flow * dt;
        const real per = dropletVolume * 1.6;
        int burst = (int)(w.emitAcc / per);
        if (burst > 60) burst = 60;
        if (burst <= 0) continue;
        w.emitAcc -= per * burst;

        const real avail = std::min(w.reservoir, per * burst);
        w.reservoir -= avail;

        const real speed = (w.arterial ? (2.5 + 12.0 * open * jet) : (0.6 + 2.4 * open));
        const real cone  = w.severed ? 0.55 : (0.22 + 0.4 * (1.0 - open));
        const Vec2 carry = b->velocityAtPoint(origin);

        for (int i = 0; i < burst; ++i) {
            const Vec2 dv = dirW.rotated(rnd(-cone, cone));
            BloodKind kind = BloodKind::Droplet;
            if (w.severed && rnd() < 0.22)      kind = BloodKind::Gush;
            else if (enableMist && rnd() < 0.14) kind = BloodKind::Mist;
            emit(origin + dv * 0.04, carry + dv * (speed * rnd(0.6, 1.35)),
                 per * rnd(0.6, 1.5), kind, w.arterial ? 235 : 185);
        }
        stainBody(w.id, 0.9 * dt * open);
        m_stats.spilledMl += avail;
    }

    m_wounds.erase(std::remove_if(m_wounds.begin(), m_wounds.end(),
                                  [](const Wound& w) {
                                      return w.reservoir <= 0.0 && w.clot >= 0.999;
                                  }),
                   m_wounds.end());
}

// ------------------------------------------------------------------ drying
void BloodSystem::ageMarks(real dt) {
    for (BloodDecal& d : m_decals) {
        d.age += dt;
        d.wetness = clamp01(1.0 - d.age / std::max(dryTime, (real)0.1));
        d.soak    = clamp01(d.soak + dt * 0.03);
    }
    for (BloodPool& p : m_pools) {
        p.age += dt;
        p.wetness = clamp01(1.0 - p.age / std::max(poolDryTime, (real)0.1));
        // A drying puddle slowly shrinks and darkens.
        if (p.wetness < 0.35) p.halfWidth *= (1.0 - 0.05 * dt);
    }
}

// ------------------------------------------------- stains, smears and drips
void BloodSystem::updateStains(real dt) {
    if (!m_world || !stainBodies) return;

    for (auto it = m_stains.begin(); it != m_stains.end();) {
        BodyId id = it->first;
        BloodStain& s = it->second;
        RigidBody* b = m_world->body(id);
        if (!b) { it = m_stains.erase(it); continue; }

        // Wet bodies sliding along a surface smear blood behind them.
        const real sp = b->velocity.length();
        if (s.wetness > 0.12 && sp > 0.8 && rnd() < clamp01(smearRate * dt * sp)) {
            const Vec2 down(0.0, -1.0);
            const Vec2 p = b->position + down * (b->boundingRadius * 0.85);
            makeDecal(INVALID_BODY, p, Vec2(0.0, 1.0),
                      safeNormal(b->velocity, Vec2(1.0, 0.0)),
                      0.35 * s.wetness, 175, sp);
            s.wetness = clamp01(s.wetness - 0.02);
        }

        // Soaked bodies drip.
        s.dripTimer -= dt;
        if (s.wetness > 0.3 && s.dripTimer <= 0.0) {
            s.dripTimer = rnd(0.08, 0.5) / std::max((real)0.05, dripRate * s.wetness);
            const Vec2 p = b->position + Vec2(rnd(-1.0, 1.0) * b->boundingRadius * 0.6,
                                              -b->boundingRadius * 0.9);
            emit(p, b->velocity * 0.4 + Vec2(0.0, -0.4), dropletVolume * rnd(0.6, 1.4),
                 BloodKind::Droplet, 180);
            s.wetness = clamp01(s.wetness - 0.012);
        }

        s.wetness = clamp01(s.wetness - dt * 0.012);
        ++it;
    }
}

void BloodSystem::budget() {
    m_wounds.erase(std::remove_if(m_wounds.begin(), m_wounds.end(),
                                  [this](const Wound& w) {
                                      if (w.reservoir <= 0.0) return true;
                                      if (w.clot >= 0.999) return true;
                                      return m_world && !m_world->body(w.id);
                                  }),
                   m_wounds.end());

    if (m_drops.size() > maxDroplets) {
        m_drops.erase(m_drops.begin(), m_drops.begin() + (long)(m_drops.size() - maxDroplets));
    }
    if (m_pools.size() > maxPools) {
        std::sort(m_pools.begin(), m_pools.end(),
                  [](const BloodPool& a, const BloodPool& b) { return a.volume > b.volume; });
        m_pools.resize(maxPools);
    }
}

// ------------------------------------------------------------------ update
void BloodSystem::update(real dt) {
    if (!enabled || !m_world || dt <= 0.0) return;

    m_stats.impactsThisStep = 0;
    m_stats.dropletsSpawned = 0;

    bleedWounds(dt);
    integrateDroplets(dt);
    updateStains(dt);
    ageMarks(dt);
    budget();

    real air = 0.0, ink = 0.0, pooled = 0.0;
    for (const BloodDroplet& d : m_drops) air += d.volume;
    for (const BloodDecal& d : m_decals) ink += d.volume;
    for (const BloodPool& p : m_pools)   pooled += p.volume;
    m_stats.droplets   = m_drops.size();
    m_stats.decals     = m_decals.size();
    m_stats.pools      = m_pools.size();
    m_stats.wounds     = m_wounds.size();
    m_stats.airborneMl = air;
    m_stats.decalMl    = ink;
    m_stats.pooledMl   = pooled;
}

} // namespace phys2d
