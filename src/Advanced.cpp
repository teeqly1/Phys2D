// phys2d - Advanced physics pack (block 1) implementation.
#include "phys2d/Advanced.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>

namespace phys2d {
namespace adv {

// ============================================================ RNG
uint64_t Rng::next() {
    m_s ^= m_s << 13; m_s ^= m_s >> 7; m_s ^= m_s << 17;
    return m_s;
}
real Rng::uniform(real a, real b) {
    const real u = (real)(next() >> 11) / (real)(1ull << 53);
    return a + (b - a) * u;
}
real Rng::gauss(real mu, real sigma) {
    // Box-Muller: every measurement can carry a Gaussian error term.
    const real u1 = std::max((real)1e-12, uniform(0.0, 1.0));
    const real u2 = uniform(0.0, 1.0);
    return mu + sigma * std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
}

// ============================================================ WIND
void WindSystem::setDirectionDegrees(real deg) {
    const real r = deg * 3.14159265358979323846 / 180.0;
    profile.direction = Vec2(std::cos(r), std::sin(r));
}

AeroProfile& WindSystem::aero(BodyId id) {
    auto it = m_aero.find(id);
    if (it != m_aero.end()) return it->second;
    return m_aero[id] = AeroProfile{};
}

Vec2 WindSystem::windVelocityAt(const Vec2& p) const {
    real s = profile.strength;

    // Gusts: slow sinusoidal modulation of the mean wind.
    s *= 1.0 + profile.gustAmplitude *
         std::sin(6.283185307179586 * m_time / std::max((real)0.05, profile.gustPeriod));

    // Boundary layer: wind grows with height above the ground.
    if (profile.shearHeight > 0.0) {
        const real h = std::max((real)0.0, p.y);
        s *= std::log(1.0 + h / profile.shearHeight) / std::log(1.0 + 10.0 / profile.shearHeight);
    }

    Vec2 v = profile.direction.normalized() * s;

    // Turbulence from fractal noise in space and time.
    if (profile.turbulence > 0.0) {
        const real nx = fractalNoise(p.x * 0.15, p.y * 0.15 + m_time * 0.4, 3, 0.55, 7u);
        const real ny = fractalNoise(p.x * 0.15 + 31.7, p.y * 0.15 - m_time * 0.4, 3, 0.55, 11u);
        v += Vec2(nx, ny) * (profile.turbulence * s);
    }
    return v;
}

real WindSystem::projectedArea(const RigidBody& b) const {
    auto it = m_aero.find(b.id);
    const AeroProfile ap = (it != m_aero.end()) ? it->second : AeroProfile{};
    if (ap.area > 0.0 && !ap.sail) return ap.area;

    const Vec2 wdir = profile.direction.normalized();
    real area;

    if (b.shape.type == ShapeType::Circle) {
        area = 2.0 * b.shape.radius;                       // diameter x 1 m depth
    } else if (b.shape.type == ShapeType::Capsule) {
        const Vec2 axis(std::cos(b.angle), std::sin(b.angle));
        const real perp = std::fabs(cross(axis, wdir));
        area = 2.0 * b.shape.radius + 2.0 * b.shape.halfLength * perp;
    } else if (b.worldVertices.size() >= 2) {
        // Real silhouette: spread of the vertices along the axis normal to
        // the wind, recomputed as the body tumbles.
        const Vec2 perp(-wdir.y, wdir.x);
        real lo = 1e30, hi = -1e30;
        for (const Vec2& v : b.worldVertices) {
            const real d = dot(v - b.position, perp);
            lo = std::min(lo, d); hi = std::max(hi, d);
        }
        area = std::max((real)0.0, hi - lo);
    } else {
        area = 2.0 * b.boundingRadius;
    }

    if (ap.area > 0.0) area = std::min(area, ap.area);
    return area;
}

Vec2 WindSystem::dragForceOn(const RigidBody& b) const {
    if (!b.isDynamic()) return Vec2();
    auto it = m_aero.find(b.id);
    const AeroProfile ap = (it != m_aero.end()) ? it->second : AeroProfile{};

    const Vec2 wind = windVelocityAt(b.position);
    const Vec2 vrel = wind - b.velocity;
    const real speed = vrel.length();
    if (speed < 1e-9) return Vec2();

    const real A = projectedArea(b);
    // Textbook drag: F = 1/2 * rho * Cd * A * v^2 along the relative flow.
    const real f = 0.5 * profile.airDensity * ap.dragCoefficient * A * speed * speed;
    Vec2 force = vrel * (f / speed);

    // Lift acts across the flow, which is what makes cloth flutter upward.
    if (ap.liftCoefficient != 0.0) {
        const Vec2 n(-vrel.y / speed, vrel.x / speed);
        force += n * (0.5 * profile.airDensity * ap.liftCoefficient * A * speed * speed);
    }
    return force * strengthScale;
}

WindReport WindSystem::report(const RigidBody& b) const {
    WindReport r;
    r.body      = b.id;
    r.windSpeed = windVelocityAt(b.position).length();
    r.area      = projectedArea(b);
    const Vec2 f = dragForceOn(b);
    r.force     = f.length();
    r.mass      = b.mass;
    // The whole "scarf vs 20 kg crate" answer lives in this one division.
    r.acceleration = (b.mass > 1e-9) ? r.force / b.mass : 0.0;

    auto it = m_deform.find(b.id);
    r.deformation = (it != m_deform.end()) ? it->second.bend + it->second.set : 0.0;

    auto ait = m_aero.find(b.id);
    const AeroProfile ap = (ait != m_aero.end()) ? ait->second : AeroProfile{};
    const real k = 0.5 * profile.airDensity * ap.dragCoefficient * std::max(r.area, (real)1e-6);
    r.terminalSpeed = (k > 0.0) ? std::sqrt(b.mass * 9.81 / k) : 0.0;
    return r;
}

void WindSystem::update(real dt) {
    if (!m_world) return;
    m_time += dt;

    for (RigidBody* b : m_world->bodies()) {
        if (!b->isDynamic()) continue;
        const Vec2 f = dragForceOn(*b);
        if (f.lengthSq() < 1e-18) continue;

        // Applied at the aerodynamic centre, offset from the centre of mass,
        // so wind also produces torque and things tumble.
        const Vec2 wdir = profile.direction.normalized();
        const Vec2 perp(-wdir.y, wdir.x);
        b->applyForceAtPoint(f, b->position + perp * (0.25 * b->boundingRadius));
    }

    applyToSoftBodies(dt);
    if (enableDeformation) deformBodies(dt);
}

void WindSystem::applyToSoftBodies(real dt) {
    (void)dt;
    if (!softBodies) return;
    for (size_t i = 0; i < softBodies->count(); ++i) {
        SoftBody* sb = softBodies->at(i);
        if (!sb) continue;
        for (SoftNode& n : sb->nodes) {
            if (n.pinned) continue;
            const Vec2 vrel = windVelocityAt(n.position) - n.velocity;
            const real sp = vrel.length();
            if (sp < 1e-9) continue;
            const real A = 2.0 * sb->nodeRadius;
            n.force += vrel * (0.5 * profile.airDensity * 1.28 * A * sp * strengthScale);
        }
    }
}

void WindSystem::deformBodies(real dt) {
    // Dynamic pressure q = 1/2 rho v^2 bends a flexible part in proportion to
    // its flexibility, with elastic spring-back and a permanent plastic set.
    for (RigidBody* b : m_world->bodies()) {
        auto ait = m_aero.find(b->id);
        if (ait == m_aero.end()) continue;
        const AeroProfile& ap = ait->second;
        if (ap.flexibility <= 0.0) continue;
        if (b->shape.type != ShapeType::Polygon || b->localVertices.size() < 3) continue;

        DeformState& ds = m_deform[b->id];
        if (ds.rest.size() != b->localVertices.size()) ds.rest = b->localVertices;

        const Vec2 vrel = windVelocityAt(b->position) - b->velocity;
        const real q = 0.5 * profile.airDensity * vrel.lengthSq();   // Pa

        // Saturating response: a breeze ripples a flag, a storm folds it flat.
        const real target = maxDeformation * ap.flexibility *
                            (1.0 - std::exp(-q * deformationGain / 400.0));
        ds.bend += (target - ds.bend) * std::min((real)1.0, deformationRecovery * dt);

        if (ds.bend > 0.85 * maxDeformation * ap.flexibility)
            ds.set = std::min(maxDeformation * 0.5, ds.set + 0.05 * dt);

        if (q > ap.tearPressure) ++m_torn;

        // Rebuild the outline: displace vertices downwind, scaled by their
        // distance from the upwind (mounted) edge.
        const real ca = std::cos(-b->angle), sa = std::sin(-b->angle);
        const Vec2 wlocal(ca * profile.direction.x - sa * profile.direction.y,
                          sa * profile.direction.x + ca * profile.direction.y);
        real lo = 1e30, hi = -1e30;
        for (const Vec2& v : ds.rest) {
            const real d = dot(v, wlocal);
            lo = std::min(lo, d); hi = std::max(hi, d);
        }
        const real span = std::max((real)1e-6, hi - lo);
        const real amp = (ds.bend + ds.set) * b->boundingRadius;

        for (size_t i = 0; i < ds.rest.size(); ++i) {
            const real t = (dot(ds.rest[i], wlocal) - lo) / span;   // 0 upwind .. 1 downwind
            b->localVertices[i] = ds.rest[i] + wlocal * (amp * t * t);
        }
    }
}

// ============================================================ ELECTRICITY IN WATER
void ElectricSystem::buildGraph() {
    m_pos.clear(); m_links.clear();
    if (!fluid) return;

    const std::vector<FluidParticle>& ps = fluid->particles();
    m_pos.reserve(ps.size());
    for (const FluidParticle& p : ps)
        if (p.active) m_pos.push_back(p.position);

    const size_t n = m_pos.size();
    if (m_voltage.size() != n) m_voltage.assign(n, 0.0);
    if (m_temp.size() != n)    m_temp.assign(n, 293.0);
    m_fixed.assign(n, 0);
    if (n == 0) return;

    // Spatial hash keeps connectivity O(n): a 20 000 particle puddle still
    // solves in real time.
    const real cell = std::max((real)1e-3, linkRadius);
    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(n * 2);
    auto key = [cell](const Vec2& p) {
        const int64_t gx = (int64_t)std::floor(p.x / cell);
        const int64_t gy = (int64_t)std::floor(p.y / cell);
        return (uint64_t)((gx & 0xFFFFFFFF) | ((gy & 0xFFFFFFFF) << 32));
    };
    for (size_t i = 0; i < n; ++i) grid[key(m_pos[i])].push_back((int)i);

    real rho = waterResistivity;
    if (superconducting) rho = std::max((real)1e-9, rho * 1e-9);

    const real r2 = linkRadius * linkRadius;
    for (size_t i = 0; i < n; ++i) {
        const int64_t gx = (int64_t)std::floor(m_pos[i].x / cell);
        const int64_t gy = (int64_t)std::floor(m_pos[i].y / cell);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy) {
                const uint64_t k = (uint64_t)(((gx + dx) & 0xFFFFFFFF) | (((gy + dy) & 0xFFFFFFFF) << 32));
                auto it = grid.find(k);
                if (it == grid.end()) continue;
                for (int j : it->second) {
                    if ((size_t)j <= i) continue;
                    const real d2 = distanceSq(m_pos[i], m_pos[(size_t)j]);
                    if (d2 > r2) continue;
                    const real d = std::max((real)1e-4, std::sqrt(d2));
                    // Ohm for a slab of water: G = A / (rho * L)
                    m_links.push_back(Link{(int)i, j, crossSection / (rho * d)});
                }
            }
    }

    // Electrodes pin the voltage of the particles they touch (Dirichlet).
    for (const Electrode& e : m_electrodes) {
        const real er2 = e.radius * e.radius;
        int pinned = 0;
        for (size_t i = 0; i < n; ++i) {
            if (distanceSq(m_pos[i], e.position) > er2) continue;
            m_voltage[i] = e.ground ? 0.0 : e.voltage;
            m_fixed[i] = 1;
            ++pinned;
        }
        // The puddle settles and spreads, so an electrode hovering just above
        // the surface would energise nothing. A real dipped electrode always
        // makes contact somewhere: fall back to the nearest particle.
        if (pinned == 0) {
            size_t best = 0; real bd = 1e30;
            for (size_t i = 0; i < n; ++i) {
                const real d = distanceSq(m_pos[i], e.position);
                if (d < bd) { bd = d; best = i; }
            }
            m_voltage[best] = e.ground ? 0.0 : e.voltage;
            m_fixed[best] = 1;
        }
    }
}

void ElectricSystem::solveField() {
    const size_t n = m_pos.size();
    if (n == 0 || m_links.empty()) { m_current = m_power = 0.0; return; }

    // Gauss-Seidel relaxation of div(sigma grad V) = 0, warm-started from the
    // previous frame so a settled puddle converges in a couple of iterations.
    std::vector<real> num(n, 0.0), den(n, 0.0);
    for (int it = 0; it < relaxIterations; ++it) {
        std::fill(num.begin(), num.end(), 0.0);
        std::fill(den.begin(), den.end(), 0.0);
        for (const Link& l : m_links) {
            num[(size_t)l.a] += l.g * m_voltage[(size_t)l.b];
            den[(size_t)l.a] += l.g;
            num[(size_t)l.b] += l.g * m_voltage[(size_t)l.a];
            den[(size_t)l.b] += l.g;
        }
        for (size_t i = 0; i < n; ++i) {
            if (m_fixed[i] || den[i] < 1e-30) continue;
            m_voltage[i] += 0.85 * (num[i] / den[i] - m_voltage[i]);
        }
    }

    m_power = 0.0;
    real inflow = 0.0;
    for (const Link& l : m_links) {
        const real dv = m_voltage[(size_t)l.a] - m_voltage[(size_t)l.b];
        const real i  = l.g * dv;          // I = G * dV
        m_power += i * dv;                 // P = G * dV^2
        // Current leaving an electrode. Links are stored once, so both
        // orientations must be tested or half the current is lost.
        const bool fa = m_fixed[(size_t)l.a] != 0, fb = m_fixed[(size_t)l.b] != 0;
        if (fa != fb) inflow += std::fabs(i);
    }
    m_current = inflow;

    if (getenv("PHYS2D_ELEC_DEBUG")) {
        int nf = 0; for (char c : m_fixed) nf += (c != 0);
        fprintf(stderr, "[elec] nodes=%zu links=%zu fixed=%d I=%.6f P=%.3f\n",
                m_pos.size(), m_links.size(), nf, m_current, m_power);
    }
}

void ElectricSystem::couple(real dt) {
    const size_t n = m_pos.size();
    m_shocks.clear();

    // Joule heating of the water itself.
    if (jouleHeating > 0.0 && n) {
        std::vector<real> heat(n, 0.0);
        for (const Link& l : m_links) {
            const real dv = m_voltage[(size_t)l.a] - m_voltage[(size_t)l.b];
            const real p  = l.g * dv * dv * 0.5;
            heat[(size_t)l.a] += p; heat[(size_t)l.b] += p;
        }
        const real pm = fluid ? std::max((real)1e-6, fluid->particleMass) : 1.0;
        for (size_t i = 0; i < n; ++i)
            m_temp[i] += jouleHeating * heat[i] * dt / (pm * 4186.0);   // c_water
    }

    if (!m_world) return;

    for (RigidBody* b : m_world->bodies()) {
        // Distance to the body AABB, not to its centre: a 120 m ground slab
        // has a huge bounding radius and would "touch" every puddle.
        real best = 0.0; Vec2 bestPt = b->position; bool touching = false;
        const AABB& bb = b->aabb;
        for (size_t i = 0; i < n; ++i) {
            const Vec2& q = m_pos[i];
            const real cx = clampr(q.x, bb.min.x, bb.max.x);
            const real cy = clampr(q.y, bb.min.y, bb.max.y);
            if (sqr(q.x - cx) + sqr(q.y - cy) > linkRadius * linkRadius) continue;
            touching = true;
            if (std::fabs(m_voltage[i]) > std::fabs(best)) { best = m_voltage[i]; bestPt = q; }
        }
        if (!touching || std::fabs(best) < 1e-6) continue;

        real R = bodyResistance;
        // Piezoresistive effect: load changes the resistance.
        if (registry) {
            const BodyPhysics* bp = registry->find(b->id);
            if (bp) R = std::max((real)1.0, R * (1.0 - 0.15 * clampr(bp->charge * 1e3, -1.0, 1.0)));
        }
        ShockEvent s;
        s.body = b->id; s.voltage = best; s.point = bestPt;
        s.current = best / R;              // I = U / R
        s.power   = best * s.current;      // P = U * I
        m_shocks.push_back(s);

        if (registry) {
            BodyPhysics& bp = registry->ref(b->id);
            bp.temperature += s.power * dt /
                std::max((real)1.0, bp.heatCapacity * std::max((real)0.01, b->mass));
        }

        // Electrokinetic drift of charged bodies along the local field.
        if (electrokinetic > 0.0 && b->isDynamic() && n > 1) {
            size_t i0 = 0, i1 = 0; real d0 = 1e30, d1 = 1e30;
            for (size_t i = 0; i < n; ++i) {
                const real d = distanceSq(m_pos[i], b->position);
                if (d < d0) { d1 = d0; i1 = i0; d0 = d; i0 = i; }
                else if (d < d1) { d1 = d; i1 = i; }
            }
            const Vec2 seg = m_pos[i1] - m_pos[i0];
            const real len = seg.length();
            if (len > 1e-6) {
                const Vec2 E = seg * ((m_voltage[i0] - m_voltage[i1]) / (len * len));  // V/m
                b->applyForceAtPoint(E * (chargeOf(b->id) + electrokinetic * b->mass), b->position);
            }
        }
    }

    for (ArcEvent& a : m_arcs) a.life -= dt;
    m_arcs.erase(std::remove_if(m_arcs.begin(), m_arcs.end(),
                                [](const ArcEvent& a) { return a.life <= 0.0; }),
                 m_arcs.end());
}

void ElectricSystem::update(real dt) {
    buildGraph();
    solveField();
    couple(dt);
}

real ElectricSystem::voltageAt(const Vec2& p) const {
    real best = 0.0, bestD = 1e30;
    for (size_t i = 0; i < m_pos.size(); ++i) {
        const real d = distanceSq(m_pos[i], p);
        if (d < bestD) { bestD = d; best = m_voltage[i]; }
    }
    return (bestD < linkRadius * linkRadius * 9.0) ? best : 0.0;
}

void ElectricSystem::notifyContactBreak(const Vec2& p, const Vec2& q, real voltage) {
    // Breaking a live contact strikes an arc.
    if (std::fabs(voltage) < arcVoltage) return;
    ArcEvent a;
    a.from = p; a.to = q; a.voltage = voltage;
    a.energy = 0.5 * 1e-3 * sqr(voltage / std::max((real)1.0, bodyResistance));
    m_arcs.push_back(a);
}

void ElectricSystem::addFrictionWork(BodyId a, BodyId b, real joules) {
    // Triboelectric series: rubbing separates charge, one side goes positive.
    const real q = triboRate * joules;
    m_charge[a] += q;
    m_charge[b] -= q;
}

real ElectricSystem::chargeOf(BodyId id) const {
    auto it = m_charge.find(id);
    return (it == m_charge.end()) ? 0.0 : it->second;
}

// ============================================================ MATERIAL LAB
void MaterialLab::registerBody(BodyId id, const MaterialProfile& p) {
    m_profile[id] = p;
    m_state[id] = MaterialState{};
}
MaterialProfile& MaterialLab::profile(BodyId id) {
    auto it = m_profile.find(id);
    if (it != m_profile.end()) return it->second;
    return m_profile[id] = MaterialProfile{};
}
MaterialState& MaterialLab::state(BodyId id) {
    auto it = m_state.find(id);
    if (it != m_state.end()) return it->second;
    return m_state[id] = MaterialState{};
}
const MaterialState* MaterialLab::find(BodyId id) const {
    auto it = m_state.find(id);
    return (it == m_state.end()) ? nullptr : &it->second;
}

real MaterialLab::effectiveStrength(BodyId id, real strainRate) const {
    auto pi = m_profile.find(id);
    if (pi == m_profile.end()) return 0.0;
    const MaterialProfile& p = pi->second;
    auto si = m_state.find(id);
    const real scale = (si == m_state.end()) ? 1.0 : si->second.strengthScale;
    // Impact toughness: strength rises with strain rate, which is why a fast
    // bullet shatters what a slow press only bends.
    return p.ultimateStress * scale *
           (1.0 + p.toughnessRateExp * std::log10(1.0 + std::fabs(strainRate)));
}

real MaterialLab::viscoelasticStress(BodyId id, real strain, real strainRate) const {
    auto pi = m_profile.find(id);
    if (pi == m_profile.end()) return 0.0;
    const MaterialProfile& p = pi->second;
    // Kelvin-Voigt sigma = E*e + eta*de/dt, plus a cubic term for nonlinear
    // elasticity (rubber stiffens, foam softens).
    return p.youngModulus * strain
         + p.cubicStiffness * p.youngModulus * strain * strain * strain
         + p.kelvinViscosity * strainRate;
}

real MaterialLab::shapeMemoryRecovery(BodyId id) const {
    auto pi = m_profile.find(id);
    auto si = m_state.find(id);
    if (pi == m_profile.end() || si == m_state.end()) return 0.0;
    if (!pi->second.shapeMemory) return 0.0;
    // Nitinol walks its plastic strain back once it is above austenite finish.
    return clampr((si->second.temperature - pi->second.austeniteTemp) / 30.0, 0.0, 1.0);
}

void MaterialLab::applyStress(BodyId id, real sigma, real dt) {
    auto pi = m_profile.find(id);
    if (pi == m_profile.end()) return;
    MaterialProfile& p = pi->second;
    MaterialState& s = state(id);

    s.lastStress = s.stress;
    s.stress = sigma;
    s.elasticStrain = sigma / std::max((real)1.0, p.youngModulus);

    const real strength = std::max((real)1.0, p.ultimateStress * s.strengthScale);
    const real yield    = std::max((real)1.0, p.yieldStress   * s.strengthScale);

    // Plasticity: irreversible strain past the yield point, plus cyclic
    // softening (every excursion weakens the material).
    if (std::fabs(sigma) > yield) {
        const real excess = (std::fabs(sigma) - yield) / std::max((real)1.0, p.youngModulus);
        s.plasticStrain += excess * dt * 10.0 * (sigma > 0 ? 1.0 : -1.0);
        s.strengthScale = std::max((real)0.05, s.strengthScale - p.cyclicSoftening * excess * 50.0);
    }

    // Creep: Norton power law, slow flow under a constant load.
    if (p.creepRate > 0.0 && sigma != 0.0) {
        const real ratio = std::fabs(sigma) / yield;
        s.creepStrain += p.creepRate * std::pow(ratio, p.creepExponent) * dt * (sigma > 0 ? 1.0 : -1.0);
    }

    // Stress relaxation: held strain sheds stress exponentially.
    if (p.relaxationTime > 0.0) s.stress *= std::exp(-dt / p.relaxationTime);

    // Fatigue: Basquin curve accumulated with Miner's rule over load cycles.
    const bool reversal = (s.stress - s.lastStress) * (s.lastStress - s.peakStress) < 0.0;
    s.peakStress = std::max(s.peakStress, std::fabs(sigma));
    if (reversal) {
        s.cycles += 1.0;
        if (s.peakStress > p.fatigueLimit) {
            const real Nf = std::pow(p.fatigueLimit / s.peakStress, p.fatigueExponent) * 1.0e6;
            s.fatigue += 1.0 / std::max((real)1.0, Nf);
        }
        s.peakStress = 0.0;
    }

    if (std::fabs(sigma) > strength)
        s.damage += (std::fabs(sigma) / strength - 1.0) * dt * 4.0;

    if (!s.failed && (s.damage + s.fatigue) >= 1.0) {
        s.failed = true;
        m_failed.push_back(id);
    }
}

void MaterialLab::onImpact(BodyId id, real impulse, real speed, const Vec2& point) {
    (void)point;
    auto pi = m_profile.find(id);
    if (pi == m_profile.end()) return;
    MaterialState& s = state(id);
    ++s.impacts;

    const real strainRate = speed / 0.01;          // over a 10 mm zone
    const real strength = std::max((real)1.0, effectiveStrength(id, strainRate));
    const real sigma = impulse / (1.0e-3 * 2.0e-3);  // small patch, 1 ms contact

    // Cumulative damage: the tenth tap breaks what the first nine bruised.
    s.damage += std::pow(std::max((real)0.0, sigma / strength), 1.5);
    s.peakStress = std::max(s.peakStress, sigma);

    if (!s.failed && (s.damage + s.fatigue) >= 1.0) {
        s.failed = true;
        m_failed.push_back(id);
    }
}

void MaterialLab::addFrictionWork(BodyId id, real joules) {
    auto pi = m_profile.find(id);
    if (pi == m_profile.end()) return;
    state(id).ablated += pi->second.ablationRate * joules;   // ablation
}

void MaterialLab::irradiate(BodyId id, real gray) {
    auto pi = m_profile.find(id);
    if (pi == m_profile.end()) return;
    MaterialState& s = state(id);
    s.dose += gray;
    s.temperature += gray * 1e-3;                                   // heating
    s.strengthScale = std::max((real)0.05,
        s.strengthScale - pi->second.radiationSoftening * gray);    // embrittlement
}

void MaterialLab::update(real dt) {
    m_failed.clear();

    for (auto& kv : m_state) {
        const BodyId id = kv.first;
        MaterialState& s = kv.second;
        auto pi = m_profile.find(id);
        if (pi == m_profile.end()) continue;
        MaterialProfile& p = pi->second;

        if (aggressiveMedium > 0.0 && p.corrosionRate > 0.0) {
            s.corroded += p.corrosionRate * aggressiveMedium * dt;
            s.strengthScale = std::max((real)0.05, s.strengthScale - s.corroded * 0.5 * dt);
        }

        // Parabolic oxide film growth: the film then protects the metal.
        if (p.oxidationRate > 0.0) {
            const real k = p.oxidationRate * std::exp(-3000.0 / std::max((real)50.0, s.temperature));
            s.oxide = std::sqrt(std::max((real)0.0, s.oxide * s.oxide + 2.0 * k * dt));
        }

        if (radiationField > 0.0) irradiate(id, radiationField * dt);

        const real rec = shapeMemoryRecovery(id);
        if (rec > 0.0) s.plasticStrain *= std::max((real)0.0, 1.0 - rec * dt * 2.0);

        if (p.relaxationTime > 0.0) s.stress *= std::exp(-dt / p.relaxationTime);
    }
}

// ============================================================ FRAGMENTATION
ShatterResult ImpactFragmentation::shatter(BodyId id, const Vec2& impactPoint, const Vec2& impactDir) {
    ShatterResult res;
    if (!m_world) return res;
    RigidBody* b = m_world->body(id);
    if (!b || !b->isDynamic()) return res;

    // Conserved quantities of the parent, measured before the split.
    const real m0 = b->mass;
    const Vec2 v0 = b->velocity;
    const Vec2 x0 = b->position;
    const real w0 = b->angularVelocity;
    const real I0 = b->inertia;
    const Vec2 P0 = v0 * m0;
    const real L0 = I0 * w0;
    const real R  = b->boundingRadius;

    bool brittle = true;
    if (lab && lab->tracks(id)) brittle = lab->profile(id).brittleness >= 0.5;
    res.brittle = brittle;

    // Brittle -> many radial wedges. Ductile -> a few stretched threads.
    const int n = brittle ? std::max(3, maxFragments) : std::max(2, maxFragments / 3);
    const real rFrag = R / std::sqrt((real)n);
    if (rFrag < minFragmentRadius) return res;

    struct Piece { Vec2 pos, vel; real mass, inertia, w, radius; };
    std::vector<Piece> pieces;
    pieces.reserve((size_t)n);

    const Vec2 dir = (impactDir.lengthSq() > 1e-12) ? impactDir.normalized() : Vec2(1, 0);
    real massSum = 0.0;

    for (int i = 0; i < n; ++i) {
        real ang, dist;
        if (brittle) {
            ang  = 6.283185307179586 * (real)i / (real)n + rng.uniform(-0.15, 0.15);
            dist = R * 0.55 * rng.uniform(0.6, 1.0);
        } else {
            ang  = std::atan2(dir.y, dir.x) + rng.uniform(-0.35, 0.35);
            dist = R * (0.3 + 0.9 * (real)i / (real)n);
        }
        Piece pc;
        pc.radius  = rFrag * rng.uniform(0.75, 1.15);
        pc.pos     = x0 + Vec2(std::cos(ang), std::sin(ang)) * dist;
        pc.mass    = m0 / (real)n;
        pc.inertia = 0.5 * pc.mass * pc.radius * pc.radius;
        pc.w       = w0 + rng.uniform(-2.0, 2.0);
        pc.vel     = v0 + (pc.pos - impactPoint).normalized() * (spreadSpeed * rng.uniform(0.4, 1.4))
                        + cross(w0, pc.pos - x0);
        massSum += pc.mass;
        pieces.push_back(pc);
    }

    // Exact linear momentum: subtract the mass-weighted mean of the scatter.
    Vec2 P;
    for (const Piece& p : pieces) P += p.vel * p.mass;
    const Vec2 correction = (P0 - P) * (1.0 / massSum);
    for (Piece& p : pieces) p.vel += correction;

    // Exact angular momentum: uniform spin correction.
    real L = 0.0, Isum = 0.0;
    for (const Piece& p : pieces) {
        L += p.inertia * p.w + p.mass * cross(p.pos - x0, p.vel);
        Isum += p.inertia;
    }
    const real dw = (L0 - L) / std::max((real)1e-12, Isum);
    for (Piece& p : pieces) p.w += dw;

    // Verify; the demo prints these residuals.
    Vec2 Pf; real Lf = 0.0, Mf = 0.0;
    for (const Piece& p : pieces) {
        Pf += p.vel * p.mass;
        Lf += p.inertia * p.w + p.mass * cross(p.pos - x0, p.vel);
        Mf += p.mass;
    }
    res.momentumResidual = (Pf - P0).length();
    res.angularResidual  = std::fabs(Lf - L0);
    res.massResidual     = std::fabs(Mf - m0);

    for (const Piece& p : pieces) {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.shape = brittle ? Shape::regularPolygon(3 + (int)rng.uniform(0, 3), p.radius)
                          : Shape::capsule(p.radius * 0.45, p.radius * 1.6);
        d.material = b->material;
        d.position = p.pos;
        d.velocity = p.vel;
        d.angularVelocity = p.w;
        d.name = "fragment";
        const BodyId fid = m_world->createBody(d);
        if (RigidBody* fb = m_world->body(fid)) {
            // Force the exact mass so the budget balances whatever the shape
            // integrator computes for the random outline.
            fb->setMassData(p.mass, p.inertia);
            fb->velocity = p.vel;
            fb->angularVelocity = p.w;
        }
        res.fragments.push_back(fid);
    }
    m_world->destroyBody(id);
    m_total += (int)res.fragments.size();
    m_history.push_back(res);
    if (m_history.size() > 256) m_history.erase(m_history.begin());
    return res;
}

void ImpactFragmentation::onCollision(const CollisionEvent& e) {
    if (!e.a || !e.b) return;
    const real ma = e.a->mass, mb = e.b->mass;
    const real mu = (e.a->isDynamic() && e.b->isDynamic())
                        ? (ma * mb) / std::max((real)1e-9, ma + mb)
                        : (e.a->isDynamic() ? ma : mb);
    if (0.5 * mu * sqr(e.relativeSpeed) < energyThreshold) return;

    for (RigidBody* body : {e.a, e.b}) {
        if (!body->isDynamic()) continue;
        if (lab) {
            lab->onImpact(body->id, e.normalImpulse, e.relativeSpeed, e.point);
            const MaterialState* st = lab->find(body->id);
            if (!st || !st->failed) continue;      // only shatter what failed
        }
        m_queue.push_back(body->id);
        m_queuePoint.push_back(e.point);
        m_queueDir.push_back(body == e.a ? e.normal : e.normal * -1.0);
    }
}

void ImpactFragmentation::update(real dt) {
    (void)dt;
    // Deferred: never mutate the body list from inside a contact callback.
    for (size_t i = 0; i < m_queue.size(); ++i)
        shatter(m_queue[i], m_queuePoint[i], m_queueDir[i]);
    m_queue.clear(); m_queuePoint.clear(); m_queueDir.clear();
}

// ============================================================ CONTACT CHEMISTRY
real ContactChemistry::wetness(BodyId id) const {
    auto it = m_wet.find(id);
    return (it == m_wet.end()) ? 0.0 : it->second;
}
real ContactChemistry::concentration(BodyId id) const {
    auto it = m_conc.find(id);
    return (it == m_conc.end()) ? 0.0 : it->second;
}

void ContactChemistry::onContact(const CollisionEvent& e) {
    if (!e.a || !e.b) return;
    Pair p;
    p.a = e.a->id; p.b = e.b->id;
    p.point = e.point; p.normal = e.normal;
    p.gap = -e.penetration;
    m_pairs.push_back(p);
}

void ContactChemistry::update(real dt) {
    if (!m_world) { m_pairs.clear(); return; }
    m_lastAdhesion = 0.0;
    m_bridges = 0;

    // A liquid bridge spans a gap and does not need a solver contact, so wet
    // beads pull on each other before they actually touch.
    if (enableCapillary) {
        std::vector<RigidBody*>& all = m_world->bodies();
        for (size_t i = 0; i < all.size(); ++i) {
            RigidBody* a = all[i];
            if (a->boundingRadius > capillaryMaxRadius) continue;
            for (size_t j = i + 1; j < all.size(); ++j) {
                RigidBody* b = all[j];
                if (b->boundingRadius > capillaryMaxRadius) continue;
                const Vec2 d = b->position - a->position;
                const real dist = d.length();
                if (dist < 1e-9) continue;
                const real gap = dist - (a->boundingRadius + b->boundingRadius);
                if (gap > capillaryRange) continue;
                if (std::max(wetness(a->id), wetness(b->id)) < 0.01) continue;

                bool known = false;
                for (const Pair& pr : m_pairs)
                    if ((pr.a == a->id && pr.b == b->id) || (pr.a == b->id && pr.b == a->id)) { known = true; break; }
                if (known) continue;

                Pair pr;
                pr.a = a->id; pr.b = b->id;
                pr.normal = d * (1.0 / dist);
                pr.point = a->position + d * 0.5;
                pr.gap = std::max((real)0.0, gap);
                m_pairs.push_back(pr);
            }
        }
    }

    for (const Pair& pr : m_pairs) {
        RigidBody* a = m_world->body(pr.a);
        RigidBody* b = m_world->body(pr.b);
        if (!a || !b) continue;

        real force = 0.0;
        const real contactArea = 0.5 * std::min(a->boundingRadius, b->boundingRadius);

        if (enableAdhesion && lab) {
            // Cohesion binds identical materials, adhesion binds different ones.
            const bool sameKind = lab->tracks(pr.a) && lab->tracks(pr.b) &&
                                  std::fabs(lab->profile(pr.a).youngModulus -
                                            lab->profile(pr.b).youngModulus) < 1e-6;
            const real ka = lab->tracks(pr.a) ? lab->profile(pr.a).adhesion : 0.0;
            const real kb = lab->tracks(pr.b) ? lab->profile(pr.b).adhesion : 0.0;
            const real ca = lab->tracks(pr.a) ? lab->profile(pr.a).cohesion : 0.0;
            const real cb = lab->tracks(pr.b) ? lab->profile(pr.b).cohesion : 0.0;
            force += (sameKind ? 0.5 * (ca + cb) : std::min(ka, kb)) * contactArea;
        }

        // Capillary bridge: F = 2*pi*R*gamma*cos(theta)
        if (enableCapillary && pr.gap < capillaryRange) {
            const real R = std::min(a->boundingRadius, b->boundingRadius);
            if (R <= capillaryMaxRadius) {
                const real w = std::max(wetness(pr.a), wetness(pr.b));
                if (w > 0.01) {
                    force += w * 2.0 * 3.14159265358979 * R * surfaceTension * std::cos(contactAngle);
                    ++m_bridges;
                }
            }
        }

        if (force > 0.0) {
            m_lastAdhesion = std::max(m_lastAdhesion, force);
            const Vec2 f = pr.normal * force;
            if (a->isDynamic()) a->applyForceAtPoint(f, pr.point);
            if (b->isDynamic()) b->applyForceAtPoint(f * -1.0, pr.point);
        }

        // Diffusion between touching bodies; osmosis when only one side is
        // semi-permeable.
        const real ca = concentration(pr.a), cb = concentration(pr.b);
        const bool pa = m_perm.count(pr.a) && m_perm.at(pr.a);
        const bool pb = m_perm.count(pr.b) && m_perm.at(pr.b);
        const real flux = ((pa != pb) ? osmosisRate : diffusionRate) * (ca - cb) * dt;
        m_conc[pr.a] = ca - flux;
        m_conc[pr.b] = cb + flux;
    }
    m_pairs.clear();
}

// ============================================================ presets
MaterialProfile materialSteel() {
    MaterialProfile p;
    p.youngModulus = 2.0e11; p.yieldStress = 2.5e8; p.ultimateStress = 4.0e8;
    p.fatigueLimit = 1.8e8; p.fatigueExponent = 3.2; p.creepRate = 2e-14;
    p.relaxationTime = 60.0; p.kelvinViscosity = 1e8; p.cubicStiffness = -2e-4;
    p.brittleness = 0.25; p.toughnessRateExp = 0.10;
    p.corrosionRate = 4e-9; p.oxidationRate = 1e-8; p.ablationRate = 1e-12;
    p.radiationSoftening = 2e-6; p.adhesion = 1.2e3; p.cohesion = 4.0e3;
    return p;
}
MaterialProfile materialGlass() {
    MaterialProfile p;
    p.youngModulus = 7.0e10; p.yieldStress = 5.0e7; p.ultimateStress = 5.2e7;
    p.fatigueLimit = 2.0e7; p.fatigueExponent = 8.0; p.creepRate = 1e-16;
    p.relaxationTime = 600.0; p.kelvinViscosity = 1e9; p.cubicStiffness = 0.0;
    p.brittleness = 1.0; p.toughnessRateExp = 0.02;
    p.corrosionRate = 1e-11; p.adhesion = 300.0; p.cohesion = 800.0;
    return p;
}
MaterialProfile materialConcrete() {
    MaterialProfile p;
    p.youngModulus = 3.0e10; p.yieldStress = 2.0e7; p.ultimateStress = 3.0e7;
    p.fatigueLimit = 8.0e6; p.fatigueExponent = 5.0; p.creepRate = 5e-12;
    p.relaxationTime = 4000.0; p.brittleness = 0.85; p.corrosionRate = 6e-10;
    p.adhesion = 900.0; p.cohesion = 2.0e3;
    return p;
}
MaterialProfile materialRubber() {
    MaterialProfile p;
    p.youngModulus = 5.0e6; p.yieldStress = 8.0e6; p.ultimateStress = 1.6e7;
    p.fatigueLimit = 3.0e6; p.fatigueExponent = 2.2; p.creepRate = 4e-10;
    p.relaxationTime = 3.0; p.kelvinViscosity = 2.0e5;
    p.cubicStiffness = 0.9;                  // strain stiffening
    p.brittleness = 0.0; p.toughnessRateExp = 0.25;
    p.adhesion = 6.0e3; p.cohesion = 9.0e3; p.ablationRate = 5e-11;
    return p;
}
MaterialProfile materialNitinol() {
    MaterialProfile p;
    p.youngModulus = 7.5e10; p.yieldStress = 3.0e8; p.ultimateStress = 9.0e8;
    p.fatigueLimit = 2.0e8; p.brittleness = 0.1;
    p.shapeMemory = true; p.austeniteTemp = 340.0;
    p.relaxationTime = 20.0; p.cubicStiffness = 0.4;
    return p;
}
MaterialProfile materialCloth() {
    MaterialProfile p;
    p.youngModulus = 8.0e7; p.yieldStress = 2.0e6; p.ultimateStress = 6.0e6;
    p.fatigueLimit = 6.0e5; p.brittleness = 0.0; p.relaxationTime = 1.2;
    p.kelvinViscosity = 3.0e4; p.adhesion = 200.0; p.cohesion = 400.0;
    p.ablationRate = 4e-10;
    return p;
}
MaterialProfile materialIce() {
    MaterialProfile p;
    p.youngModulus = 9.0e9; p.yieldStress = 1.0e6; p.ultimateStress = 2.0e6;
    p.fatigueLimit = 4.0e5; p.creepRate = 8e-9; p.creepExponent = 3.0;
    p.brittleness = 0.95; p.relaxationTime = 30.0;
    return p;
}
MaterialProfile materialWood() {
    MaterialProfile p;
    p.youngModulus = 1.1e10; p.yieldStress = 3.0e7; p.ultimateStress = 4.5e7;
    p.fatigueLimit = 1.2e7; p.brittleness = 0.6; p.creepRate = 3e-11;
    p.relaxationTime = 900.0; p.ablationRate = 2e-11;
    return p;
}

AeroProfile aeroScarf() {
    AeroProfile a;
    a.dragCoefficient = 1.28;    // flat plate normal to the flow
    a.area = -1.0;               // auto silhouette
    a.liftCoefficient = 0.35;
    a.flexibility = 1.0;
    a.tearPressure = 4.0e3;
    a.sail = true;
    return a;
}
AeroProfile aeroCrate() {
    AeroProfile a;
    a.dragCoefficient = 1.05;    // cube
    a.liftCoefficient = 0.0;
    a.flexibility = 0.0;         // rigid
    a.tearPressure = 1.0e9;
    a.sail = true;
    return a;
}
AeroProfile aeroSphere() {
    AeroProfile a;
    a.dragCoefficient = 0.47;
    a.flexibility = 0.0;
    a.sail = false;
    return a;
}
AeroProfile aeroFlag() {
    AeroProfile a;
    a.dragCoefficient = 1.2;
    a.liftCoefficient = 0.5;
    a.flexibility = 0.85;
    a.tearPressure = 8.0e3;
    return a;
}

} // namespace adv
} // namespace phys2d
