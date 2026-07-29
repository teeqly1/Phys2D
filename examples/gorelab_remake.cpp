// ============================================================================
//  GoreLab Remake - a People Playground style sandbox built on phys2d.
//
//  Two maps, a tool belt, and every major engine subsystem running at once:
//  rigid bodies, ragdolls with joints, SPH water, blood (wounds / decals /
//  pools), radial fracture, fire with heat, wind fields and electric current
//  conducted through the water.
//
//  Windows build (MSYS2 MINGW64):
//    g++ -std=c++17 -O2 -Iinclude examples/gorelab_remake.cpp src/*.cpp \\
//        -pthread -lgdi32 -luser32 -mwindows -o gorelab_remake.exe
//  Headless self test (any platform):
//    g++ -std=c++17 -O2 -DGORELAB_HEADLESS -Iinclude examples/gorelab_remake.cpp \\
//        src/*.cpp -pthread -o gorelab_test
// ============================================================================
#include "phys2d/World.h"
#include "phys2d/Extras.h"
#include "phys2d/Blood.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace phys2d;

namespace {

struct Xor32 {
    uint32_t s = 0x1BADB002u;
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    real unit() { return (real)(next() >> 8) / (real)0x00FFFFFF; }
    real range(real a, real b) { return a + (b - a) * unit(); }
};

enum class Stuff : uint8_t { Ground = 0, Wall, Crate, Wood, Metal, Glass, Shard, Flesh, Bone, Debris };

enum class Tool : uint8_t {
    Human = 0, Water, Crate, Glass, Metal, Pistol, Shotgun, Rifle, Fire, Grenade, Voltage, Erase
};

const char* toolName(Tool t) {
    switch (t) {
        case Tool::Human:   return "HUMAN (ragdoll)";
        case Tool::Water:   return "WATER";
        case Tool::Crate:   return "CRATE";
        case Tool::Glass:   return "GLASS PANE";
        case Tool::Metal:   return "STEEL BLOCK";
        case Tool::Pistol:  return "PISTOL";
        case Tool::Shotgun: return "SHOTGUN";
        case Tool::Rifle:   return "AUTO RIFLE";
        case Tool::Fire:    return "FIRE";
        case Tool::Grenade: return "FRAG GRENADE";
        case Tool::Voltage: return "220 V ELECTRODE";
        case Tool::Erase:   return "ERASE";
    }
    return "?";
}

enum class MapId : uint8_t { Box = 0, WaterBox = 1 };

struct Flame  { Vec2 pos, vel; real life = 0.0, maxLife = 1.0, size = 0.10; bool smoke = false; };
struct Spark  { Vec2 pos, vel; real life = 0.0, maxLife = 0.5; int r = 255, g = 200, b = 90; };
struct Tracer { Vec2 a, b; real life = 0.0; };
struct Muzzle { Vec2 pos, dir; real life = 0.0; };

struct HitResult {
    bool   hit = false;
    Vec2   point, normal;
    BodyId body = INVALID_BODY;
    real   distance = 0.0;
};

struct WeaponSpec {
    const char* name;
    int  pellets;
    real spread, speed, damage, impulse, cooldown, range;
    bool automatic;
};

const WeaponSpec WEAPON_PISTOL  = {"pistol",  1, 0.012, 380.0, 0.45,  9.0, 0.26,  60.0, false};
const WeaponSpec WEAPON_SHOTGUN = {"shotgun", 9, 0.150, 420.0, 0.34, 11.0, 0.80,  28.0, false};
const WeaponSpec WEAPON_RIFLE   = {"rifle",   1, 0.020, 780.0, 0.60, 14.0, 0.085, 90.0, true};

// ============================================================ water current
// Voltage in a puddle: every SPH particle is a node, neighbours within
// linkRadius form resistors (G = A / (rho * L)), electrodes pin the potential
// and the field is relaxed with Gauss-Seidel. Anything standing in energised
// water is shocked; dry ground stays safe.
struct Electrode { Vec2 position; real voltage = 220.0; real radius = 0.30; bool ground = false; };
struct ShockEvent { BodyId body = INVALID_BODY; real voltage = 0.0, current = 0.0, power = 0.0; Vec2 point; };

class WaterCurrent {
public:
    ParticleSystem* fluid = nullptr;
    World*          world = nullptr;
    real resistivity     = 20.0;    // ohm*m, tap water
    real linkRadius      = 0.9;     // must comfortably exceed particle spacing
    real crossSection    = 0.03;
    real bodyResistance  = 1200.0;
    int  relaxIterations = 40;
    size_t maxPinnedNodes = 10;     // a rod is a rod, not half the pool

    void addElectrode(const Electrode& e) { m_electrodes.push_back(e); }
    void clearElectrodes() {
        m_electrodes.clear(); m_shocks.clear();
        m_pos.clear(); m_volt.clear(); m_fixed.clear(); m_links.clear();
        m_current = m_power = 0.0;
    }
    const std::vector<Electrode>&  electrodes()   const { return m_electrodes; }
    const std::vector<ShockEvent>& shocks()       const { return m_shocks; }
    const std::vector<real>&       nodeVoltages() const { return m_volt; }
    size_t wetNodeCount()    const { return m_pos.size(); }
    real   totalCurrent()    const { return m_current; }
    real   dissipatedPower() const { return m_power; }

    void update(real dt) {
        m_shocks.clear();
        m_current = m_power = 0.0;
        if (!fluid || m_electrodes.empty()) { m_pos.clear(); m_volt.clear(); return; }
        buildGraph();
        if (m_pos.empty()) return;
        pinElectrodes();
        solveField();
        collectShocks(dt);
        if (std::getenv("GORELAB_ELEC_DEBUG")) {
            size_t hot = 0, cold = 0, bnd = 0;
            for (size_t i = 0; i < m_pos.size(); ++i)
                if (m_fixed[i]) { (m_volt[i] > 1.0 ? hot : cold)++; }
            for (const Link& L : m_links)
                if ((m_fixed[L.a] != 0) != (m_fixed[L.b] != 0)) ++bnd;
            std::fprintf(stderr, "[elec] nodes=%zu links=%zu hot=%zu cold=%zu bnd=%zu I=%.4f P=%.1f shocks=%zu\\n",
                         m_pos.size(), m_links.size(), hot, cold, bnd, m_current, m_power, m_shocks.size());
        }
    }

private:
    struct Link { int a = 0, b = 0; real g = 0.0; };
    static uint64_t key(int64_t gx, int64_t gy) {
        return ((uint64_t)(gx + 0x20000000) << 32) ^ (uint64_t)(gy + 0x20000000);
    }

    void buildGraph() {
        const std::vector<FluidParticle>& ps = fluid->particles();
        m_pos.clear(); m_links.clear();
        m_pos.reserve(ps.size());
        for (const FluidParticle& q : ps)
            if (q.active && std::isfinite(q.position.x) && std::isfinite(q.position.y))
                m_pos.push_back(q.position);
        m_volt.assign(m_pos.size(), 0.0);
        m_fixed.assign(m_pos.size(), 0);
        if (m_pos.empty()) return;

        const real cell = linkRadius;
        std::unordered_map<uint64_t, std::vector<int>> grid;
        grid.reserve(m_pos.size());
        for (int i = 0; i < (int)m_pos.size(); ++i)
            grid[key((int64_t)std::floor(m_pos[i].x / cell),
                     (int64_t)std::floor(m_pos[i].y / cell))].push_back(i);

        const real r2 = linkRadius * linkRadius;
        std::vector<int> degree(m_pos.size(), 0);
        const int maxDegree = 12;                 // dense clumps must not explode the graph
        for (int i = 0; i < (int)m_pos.size(); ++i) {
            if (degree[i] >= maxDegree) continue;
            const int64_t gx = (int64_t)std::floor(m_pos[i].x / cell);
            const int64_t gy = (int64_t)std::floor(m_pos[i].y / cell);
            for (int ox = -1; ox <= 1 && degree[i] < maxDegree; ++ox)
                for (int oy = -1; oy <= 1 && degree[i] < maxDegree; ++oy) {
                    auto it = grid.find(key(gx + ox, gy + oy));
                    if (it == grid.end()) continue;
                    for (int j : it->second) {
                        if (j <= i || degree[i] >= maxDegree || degree[j] >= maxDegree) continue;
                        const real d2 = distanceSq(m_pos[i], m_pos[j]);
                        if (d2 > r2 || d2 < 1e-9) continue;
                        Link L;
                        L.a = i; L.b = j;
                        L.g = crossSection / (resistivity * std::max((real)0.02, std::sqrt(d2)));
                        m_links.push_back(L);
                        ++degree[i]; ++degree[j];
                    }
                }
        }
    }

    // Only a handful of nodes per rod: pinning a whole clump would leave no
    // potential gradient and therefore no current at all.
    int pinNear(const Electrode& e, bool onlyFree) {
        std::vector<std::pair<real, int>> cand;
        int nearest = -1;
        real bestD = 1e30;
        for (int i = 0; i < (int)m_pos.size(); ++i) {
            if (onlyFree && m_fixed[i]) continue;
            const real d = distance(m_pos[i], e.position);
            if (d < bestD) { bestD = d; nearest = i; }
            if (d <= e.radius) cand.push_back(std::make_pair(d, i));
        }
        // An electrode held just above the surface still energises the pool.
        if (cand.empty() && nearest >= 0 && bestD < 3.0)
            cand.push_back(std::make_pair(bestD, nearest));
        std::sort(cand.begin(), cand.end());
        const size_t take = std::min<size_t>(maxPinnedNodes, cand.size());
        for (size_t k = 0; k < take; ++k) {
            m_volt[cand[k].second] = e.voltage;
            m_fixed[cand[k].second] = 1;
        }
        return (int)take;
    }

    // Live rods win, earth rods may only claim still-free nodes, so one
    // electrode can never overwrite another and short the pool out.
    void pinElectrodes() {
        for (const Electrode& e : m_electrodes)
            if (!e.ground) pinNear(e, false);

        Vec2 liveCenter;
        int liveCount = 0;
        for (size_t i = 0; i < m_pos.size(); ++i)
            if (m_fixed[i] && m_volt[i] > 1.0) { liveCenter += m_pos[i]; ++liveCount; }
        if (liveCount > 0) liveCenter *= (real)1.0 / (real)liveCount;

        for (const Electrode& e : m_electrodes) {
            if (!e.ground) continue;
            if (pinNear(e, true) > 0 || liveCount == 0) continue;
            // No water near the earth rod: earth the far end of the pool so the
            // current still has to cross the whole thing.
            std::vector<std::pair<real, int>> byDist;
            for (int i = 0; i < (int)m_pos.size(); ++i)
                if (!m_fixed[i]) byDist.push_back(std::make_pair(distanceSq(m_pos[i], liveCenter), i));
            std::sort(byDist.begin(), byDist.end(),
                      [](const std::pair<real, int>& a, const std::pair<real, int>& b) {
                          return a.first > b.first;
                      });
            const size_t take = std::min<size_t>(maxPinnedNodes, byDist.size());
            for (size_t k = 0; k < take; ++k) {
                m_volt[byDist[k].second] = e.voltage;
                m_fixed[byDist[k].second] = 1;
            }
        }
    }

    void solveField() {
        const size_t n = m_pos.size();
        std::vector<real> num(n, 0.0), den(n, 0.0);
        for (int it = 0; it < relaxIterations; ++it) {
            std::fill(num.begin(), num.end(), 0.0);
            std::fill(den.begin(), den.end(), 0.0);
            for (const Link& L : m_links) {
                num[L.a] += L.g * m_volt[L.b]; den[L.a] += L.g;
                num[L.b] += L.g * m_volt[L.a]; den[L.b] += L.g;
            }
            for (size_t i = 0; i < n; ++i) {
                if (m_fixed[i] || den[i] <= 0.0) continue;
                m_volt[i] += 0.9 * (num[i] / den[i] - m_volt[i]);
            }
        }
        for (const Link& L : m_links) {
            const real dv = m_volt[L.a] - m_volt[L.b];
            const bool fa = m_fixed[L.a] != 0, fb = m_fixed[L.b] != 0;
            if (fa != fb) {
                const bool srcHigher = fa ? (m_volt[L.a] > m_volt[L.b]) : (m_volt[L.b] > m_volt[L.a]);
                if (srcHigher) m_current += std::fabs(L.g * dv);
            }
            m_power += L.g * dv * dv;
        }
    }

    void collectShocks(real) {
        if (!world) return;
        for (RigidBody* b : world->bodies()) {
            if (!b->isDynamic()) continue;
            real best = 0.0;
            Vec2 where = b->position;
            for (size_t i = 0; i < m_pos.size(); ++i) {
                if (m_volt[i] <= 1.0 || m_volt[i] <= best) continue;
                const Vec2& p = m_pos[i];
                // Distance to the AABB: a big slab cannot cheat its way to a shock.
                const real dx = std::max(std::max(b->aabb.min.x - p.x, (real)0.0), p.x - b->aabb.max.x);
                const real dy = std::max(std::max(b->aabb.min.y - p.y, (real)0.0), p.y - b->aabb.max.y);
                if (dx * dx + dy * dy > 0.09) continue;
                best = m_volt[i];
                where = p;
            }
            if (best > 1.0) {
                ShockEvent s;
                s.body = b->id; s.voltage = best;
                s.current = best / bodyResistance;
                s.power = s.voltage * s.current;
                s.point = where;
                m_shocks.push_back(s);
            }
        }
    }

    std::vector<Vec2>       m_pos;
    std::vector<real>       m_volt;
    std::vector<uint8_t>    m_fixed;
    std::vector<Link>       m_links;
    std::vector<Electrode>  m_electrodes;
    std::vector<ShockEvent> m_shocks;
    real m_current = 0.0, m_power = 0.0;
};

} // namespace

// ============================================================ the game
class GoreLab {
public:
    World               world;
    ParticleSystem      fluid;
    BloodSystem         blood;
    BodyPhysicsRegistry registry;
    ThermalSystem       thermal;
    FieldSystem         field;
    FractureSystem      fracture;
    WaterCurrent        elec;

    MapId map  = MapId::Box;
    Tool  tool = Tool::Human;
    bool  paused = false, slowMotion = false, turbo = false, stepOnce = false;
    bool  showWater = true;
    real  windStrength = 0.0, simTime = 0.0;
    int   ragdollCount = 0, shotsFired = 0, kills = 0;

    std::unordered_map<BodyId, Stuff> stuffOf;
    std::unordered_set<BodyId>        fleshParts;
    std::unordered_map<BodyId, real>  burning, health, propDamage;
    std::vector<Flame>  flames;
    std::vector<Spark>  sparks;
    std::vector<Tracer> tracers;
    std::vector<Muzzle> muzzles;
    std::vector<BodyId> pendingShatter;
    std::vector<Vec2>   pendingShatterPt, pendingShatterDir;
    std::unordered_set<uint64_t> waterCells;

    BodyId dragBody = INVALID_BODY;
    Vec2   dragLocal, dragTarget, aimPoint;
    real   fireCooldown = 0.0;
    Xor32  rng;

    real arenaLeft = -16.0, arenaRight = 16.0, arenaFloor = 0.0, arenaTop = 18.0;

    GoreLab() { buildSystems(); loadMap(MapId::Box); }

    void buildSystems() {
        WorldConfig& cfg = world.config();
        cfg.gravity = Vec2(0.0, -9.81);
        cfg.solver.velocityIterations = 10;
        cfg.solver.positionIterations = 8;
        cfg.substeps = 3;
        world.enableModule(MOD_TOI, true);          // no bullets through walls
        world.enableModule(MOD_SLEEPING, true);

        fluid.attach(world);
        // Spacing must sit at ~0.8 of the smoothing radius or the pool explodes.
        fluid.smoothingRadius = 0.45;
        fluid.particleMass    = 1.4;
        fluid.restDensity     = 1000.0;
        fluid.stiffness       = 420.0;
        fluid.viscosity       = 22.0;
        fluid.surfaceTension  = 1.0;

        blood.attach(world);
        blood.enablePools = true;
        blood.maxDroplets = 7000;
        blood.maxDecals   = 3000;
        blood.maxWounds   = 180;

        thermal.attach(world);
        thermal.registry = &registry;
        thermal.frictionHeating = 0.3;

        field.attach(world);
        field.registry = &registry;
        field.flags = FIELD_WIND;
        field.windBase = Vec2(0.0, 0.0);
        field.windTurbulence = 2.5;
        field.maxWindAcceleration = 300.0;

        fracture.attach(world);
        fracture.pieces = 7;
        fracture.impulseThreshold = 14.0;
        fracture.maxFragmentsPerStep = 40;
        fracture.minFragmentArea = 0.004;

        elec.fluid = &fluid;
        elec.world = &world;

        world.setBeginContactCallback([this](const CollisionEvent& e) { onContact(e); });
        world.setPersistContactCallback([this](const CollisionEvent& e) { onPersist(e); });
    }

    // ------------------------------------------------------------ maps
    void loadMap(MapId id) {
        map = id;
        world.clear(); fluid.clear(); blood.clear(); elec.clearElectrodes();
        stuffOf.clear(); fleshParts.clear(); burning.clear(); health.clear();
        propDamage.clear(); flames.clear(); sparks.clear(); tracers.clear();
        muzzles.clear(); registry.all().clear(); waterCells.clear();
        pendingShatter.clear(); pendingShatterPt.clear(); pendingShatterDir.clear();
        ragdollCount = shotsFired = kills = 0;
        simTime = 0.0;
        dragBody = INVALID_BODY;
        blood.groundLevel = arenaFloor;

        makeWall(Vec2(0.0, arenaFloor - 0.5), 34.0, 1.0, Stuff::Ground);
        makeWall(Vec2(arenaLeft - 0.5, 9.0), 1.0, 18.0, Stuff::Wall);
        makeWall(Vec2(arenaRight + 0.5, 9.0), 1.0, 18.0, Stuff::Wall);
        makeWall(Vec2(0.0, arenaTop + 0.5), 34.0, 1.0, Stuff::Wall);   // lid
        // Kept clear of the static slabs: a particle inside one would be pushed
        // out by that slab's bounding radius and fly across the arena.
        fluid.domain = AABB(Vec2(arenaLeft + 0.45, arenaFloor + 0.35),
                            Vec2(arenaRight - 0.45, arenaTop - 1.2));

        if (id == MapId::WaterBox) {
            fluid.emitBlock(Vec2(-8.0, arenaFloor + 0.45), 16.0, 2.0, 0.32);
            makeWall(Vec2(-9.0, 6.6), 7.0, 0.4, Stuff::Wood);   // diving board
            makeCrate(Vec2(-11.0, 7.4), 0.9);
            makeCrate(Vec2(-10.0, 7.4), 0.9);
        } else {
            makeWall(Vec2(7.0, 3.0), 6.0, 0.4, Stuff::Wood);    // shelf
            for (int i = 0; i < 4; ++i) makeCrate(Vec2(6.0 + i * 1.05, 3.8), 0.9);
            makeGlassPane(Vec2(-6.0, 2.0), 0.16, 4.0);
            makeGlassPane(Vec2(-3.0, 2.0), 0.16, 4.0);
            makeMetalBlock(Vec2(2.5, 1.0), 0.9);
        }
        spawnRagdoll(Vec2(0.0, 5.0));
    }

    // ------------------------------------------------------------ builders
    BodyId tag(BodyId id, Stuff s) { if (id != INVALID_BODY) stuffOf[id] = s; return id; }
    Stuff  stuff(BodyId id) const {
        auto it = stuffOf.find(id);
        return (it == stuffOf.end()) ? Stuff::Debris : it->second;
    }

    BodyId makeWall(const Vec2& c, real w, real h, Stuff s) {
        BodyDef d;
        d.type = BodyType::Static;
        d.shape = Shape::box(w, h);
        d.position = c;
        d.material.staticFriction = 0.7;
        d.material.dynamicFriction = 0.55;
        d.material.restitution = 0.05;
        d.name = "wall";
        return tag(world.createBody(d), s);
    }

    BodyId makeCrate(const Vec2& c, real size) {
        BodyDef d;
        d.shape = Shape::box(size, size);
        d.position = c;
        d.material.density = 260.0;
        d.material.restitution = 0.18;
        d.material.staticFriction = 0.62;
        d.material.dynamicFriction = 0.48;
        d.name = "crate";
        const BodyId id = world.createBody(d);
        if (RigidBody* b = world.body(id)) b->computeMassFromShape();
        fracture.makeBreakable(id, 260.0);
        registry.ref(id).temperature = 293.0;
        return tag(id, Stuff::Crate);
    }

    BodyId makeMetalBlock(const Vec2& c, real size) {
        BodyDef d;
        d.shape = Shape::box(size, size * 0.7);
        d.position = c;
        d.material.density = 7800.0;
        d.material.restitution = 0.10;
        d.material.staticFriction = 0.55;
        d.name = "steel";
        const BodyId id = world.createBody(d);
        if (RigidBody* b = world.body(id)) b->computeMassFromShape();
        registry.ref(id).conductivity = 120.0;
        return tag(id, Stuff::Metal);
    }

    BodyId makeGlassPane(const Vec2& c, real w, real h) {
        BodyDef d;
        d.shape = Shape::box(w, h);
        d.position = c;
        d.material.density = 2500.0;
        d.material.restitution = 0.05;
        d.material.staticFriction = 0.4;
        d.name = "glass";
        const BodyId id = world.createBody(d);
        if (RigidBody* b = world.body(id)) b->computeMassFromShape();
        fracture.makeBreakable(id, 10.0);
        return tag(id, Stuff::Glass);
    }

    void spawnRagdoll(const Vec2& at) {
        RagdollConfig rc;
        rc.position = at;
        rc.scale = 1.0;
        rc.density = 1050.0;              // human tissue
        rc.jointFriction = 0.08;
        Ragdoll r = createRagdoll(world, rc);
        for (BodyId p : r.parts) {
            if (p == INVALID_BODY) continue;
            tag(p, (p == r.head) ? Stuff::Bone : Stuff::Flesh);
            fleshParts.insert(p);
            BodyPhysics& bp = registry.ref(p);
            bp.temperature = 310.0;       // 37 C
            bp.heatCapacity = 3500.0;
            bp.meltingPoint = 600.0;
            if (RigidBody* b = world.body(p)) {
                b->material.restitution = 0.05;
                b->material.staticFriction = 0.85;
                b->material.dynamicFriction = 0.7;
            }
        }
        if (r.torso != INVALID_BODY) health[r.torso] = 100.0;
        ++ragdollCount;
    }

    // ------------------------------------------------------------ queries
    HitResult rayCast(const Vec2& origin, const Vec2& dir, real maxDist,
                      BodyId ignore = INVALID_BODY) {
        HitResult res;
        const Vec2 d = dir.normalized();
        Vec2 from = origin + d * 0.02;
        const Vec2 to = origin + d * maxDist;
        for (int guard = 0; guard < 3; ++guard) {
            RayHit hit;
            if (!rayCastClosest(world, from, to, hit) || !hit.body) return res;
            if (hit.body->id == ignore) { from = hit.point + d * 0.08; continue; }
            res.hit = true;
            res.body = hit.body->id;
            res.point = hit.point;
            res.normal = hit.normal;
            res.distance = distance(origin, hit.point);
            return res;
        }
        return res;
    }

    // Water occupancy grid: "is this wet?" in O(1) instead of a full scan.
    static constexpr real WATER_CELL = 0.5;
    static uint64_t wcell(int64_t gx, int64_t gy) {
        return ((uint64_t)(gx + 0x20000000) << 32) ^ (uint64_t)(gy + 0x20000000);
    }
    void rebuildWaterGrid() {
        waterCells.clear();
        for (const FluidParticle& q : fluid.particles()) {
            if (!q.active) continue;
            if (!std::isfinite(q.position.x) || !std::isfinite(q.position.y)) continue;
            waterCells.insert(wcell((int64_t)std::floor(q.position.x / WATER_CELL),
                                    (int64_t)std::floor(q.position.y / WATER_CELL)));
        }
    }
    bool inWater(const Vec2& p, real radius) const {
        if (waterCells.empty()) return false;
        const int span = std::max(1, (int)std::ceil(radius / WATER_CELL));
        const int64_t gx = (int64_t)std::floor(p.x / WATER_CELL);
        const int64_t gy = (int64_t)std::floor(p.y / WATER_CELL);
        for (int ox = -span; ox <= span; ++ox)
            for (int oy = -span; oy <= span; ++oy)
                if (waterCells.count(wcell(gx + ox, gy + oy))) return true;
        return false;
    }

    // ------------------------------------------------------------ tools
    void useTool(const Vec2& at, const Vec2& aimFrom) {
        switch (tool) {
            case Tool::Human:   spawnRagdoll(at); break;
            case Tool::Water:   pourWater(at); break;
            case Tool::Crate:   makeCrate(at, 0.9); break;
            case Tool::Glass:   makeGlassPane(at, 0.16, 3.0); break;
            case Tool::Metal:   makeMetalBlock(at, 0.9); break;
            case Tool::Pistol:  shoot(aimFrom, at, WEAPON_PISTOL); break;
            case Tool::Shotgun: shoot(aimFrom, at, WEAPON_SHOTGUN); break;
            case Tool::Rifle:   shoot(aimFrom, at, WEAPON_RIFLE); break;
            case Tool::Fire:    igniteAt(at, 0.9); break;
            case Tool::Grenade: throwGrenade(at); break;
            case Tool::Voltage: placeElectrode(at); break;
            case Tool::Erase:   eraseAt(at); break;
        }
    }

    void pourWater(const Vec2& at) {
        if (fluid.count() > 3500) return;          // keep it real time
        for (int i = 0; i < 5; ++i)
            fluid.emit(at + Vec2(rng.range(-0.3, 0.3), rng.range(-0.3, 0.3)),
                       Vec2(rng.range(-0.4, 0.4), rng.range(-1.2, 0.0)));
    }

    // The rod snaps onto the water, the earth rod goes to the far end of the
    // same pool, so the current has to cross the whole puddle.
    void placeElectrode(const Vec2& at) {
        elec.clearElectrodes();
        Vec2 live = at, ground(arenaRight - 1.0, arenaFloor + 0.3);
        real best = 1e30;
        bool found = false;
        for (const FluidParticle& q : fluid.particles()) {
            if (!q.active) continue;
            const real d = distanceSq(q.position, at);
            if (d < best) { best = d; live = q.position; found = true; }
        }
        if (found) {
            real far = -1.0;
            for (const FluidParticle& q : fluid.particles()) {
                if (!q.active) continue;
                const real d = distanceSq(q.position, live);
                if (d > far) { far = d; ground = q.position; }
            }
        }
        elec.addElectrode(Electrode{live, 220.0, 0.30, false});
        elec.addElectrode(Electrode{ground, 0.0, 0.30, true});
    }

    void eraseAt(const Vec2& at) {
        if (RigidBody* b = world.queryPoint(at)) {
            if (b->isStatic()) return;
            removeBody(b->id);
        }
    }

    void removeBody(BodyId id) {
        blood.forgetBody(id);
        stuffOf.erase(id); fleshParts.erase(id); burning.erase(id);
        health.erase(id); propDamage.erase(id);
        registry.remove(id);
        if (dragBody == id) dragBody = INVALID_BODY;
        world.destroyBody(id);
    }

    void damageProp(BodyId id, real amount) {
        if (amount <= 0.0) return;
        const Stuff s = stuff(id);
        if (s == Stuff::Ground || s == Stuff::Wall) return;
        propDamage[id] += amount;
    }

    // ------------------------------------------------------------ shooting
    void shoot(const Vec2& from, const Vec2& target, const WeaponSpec& w) {
        if (fireCooldown > 0.0) return;
        fireCooldown = w.cooldown;
        ++shotsFired;
        const Vec2 base = (target - from).normalized();
        muzzles.push_back(Muzzle{from, base, 0.05});

        for (int p = 0; p < w.pellets; ++p) {
            const real a = std::atan2(base.y, base.x) + rng.range(-w.spread, w.spread);
            const Vec2 dir(std::cos(a), std::sin(a));
            Vec2 origin = from;
            real energy = 1.0;
            BodyId last = INVALID_BODY;
            // Penetration: each hit eats energy, the bullet keeps going on the rest.
            for (int pass = 0; pass < 4 && energy > 0.12; ++pass) {
                const HitResult h = rayCast(origin, dir, w.range, last);
                if (!h.hit) { tracers.push_back(Tracer{origin, origin + dir * w.range, 0.0}); break; }
                tracers.push_back(Tracer{origin, h.point, 0.0});
                energy = impactBullet(h, dir, w, energy);
                last = h.body;
                origin = h.point + dir * 0.06;
            }
        }
    }

    real impactBullet(const HitResult& h, const Vec2& dir, const WeaponSpec& w, real energy) {
        RigidBody* b = world.body(h.body);
        if (!b) return 0.0;
        const Stuff s = stuff(h.body);
        if (b->isDynamic()) b->applyImpulseAtPoint(dir * (w.impulse * energy), h.point);

        real loss = 0.35;
        switch (s) {
            case Stuff::Flesh:
            case Stuff::Bone: {
                loss = (s == Stuff::Bone) ? 0.75 : 0.40;
                const real sev = w.damage * energy;
                blood.addWound(h.body, h.point, dir * -1.0, sev * 0.7, sev > 0.4);
                blood.spray(h.point, dir * -1.0, 5.0 + 9.0 * sev, 0.55, (int)(6 + 18 * sev), 0.5, 235);
                blood.spray(h.point + dir * 0.05, dir, 7.0 + 14.0 * sev, 0.7, (int)(8 + 24 * sev), 0.7, 210);
                if (sev > 0.5) blood.sever(h.body, h.point, dir, sev);
                damageHuman(h.body, sev * 60.0);
                break;
            }
            case Stuff::Glass:
                loss = 0.18;
                queueShatter(h.body, h.point, dir);
                break;
            case Stuff::Crate:
            case Stuff::Wood:
                loss = 0.45;
                emitSparks(h.point, h.normal, 6, 180, 140, 90);
                damageProp(h.body, w.impulse * energy * 3.0);
                if (propDamage[h.body] > 55.0) queueShatter(h.body, h.point, dir);
                break;
            case Stuff::Metal:
                loss = 0.85;                           // steel eats the bullet
                emitSparks(h.point, h.normal, 16, 255, 220, 140);
                break;
            default:
                loss = 0.9;
                emitSparks(h.point, h.normal, 4, 200, 200, 200);
                break;
        }
        return energy * (1.0 - loss);
    }

    void damageHuman(BodyId part, real dmg) {
        RigidBody* hit = world.body(part);
        if (!hit) return;
        for (auto& kv : health) {
            if (kv.second <= 0.0) continue;
            RigidBody* torso = world.body(kv.first);
            if (!torso) continue;
            if (distance(torso->position, hit->position) > 2.2) continue;
            kv.second -= dmg;
            if (kv.second <= 0.0) { kv.second = 0.0; ++kills; }
            return;
        }
    }

    void queueShatter(BodyId id, const Vec2& point, const Vec2& dir) {
        for (BodyId q : pendingShatter) if (q == id) return;
        pendingShatter.push_back(id);
        pendingShatterPt.push_back(point);
        pendingShatterDir.push_back(dir);
    }

    void throwGrenade(const Vec2& at) {
        const real R = 5.0;
        for (RigidBody* b : world.bodies()) {
            if (!b->isDynamic()) continue;
            const Vec2 d = b->position - at;
            const real dist = std::max((real)0.25, d.length());
            if (dist > R) continue;
            const real falloff = 1.0 - dist / R;
            b->applyImpulseAtPoint(d.normalized() * (900.0 * falloff * falloff), b->position);
            if (fleshParts.count(b->id)) {
                blood.addWound(b->id, b->position, d.normalized(), falloff, true);
                blood.spray(b->position, d.normalized(), 12.0 * falloff, 1.2,
                            (int)(24 * falloff), 0.8, 230);
                damageHuman(b->id, 90.0 * falloff);
            }
            if (stuff(b->id) == Stuff::Glass && falloff > 0.25)
                queueShatter(b->id, b->position, d.normalized());
            else
                damageProp(b->id, 90.0 * falloff);
        }
        for (int i = 0; i < 90; ++i) {
            const real a = rng.range(0.0, 6.2831853);
            sparks.push_back(Spark{at, Vec2(std::cos(a), std::sin(a)) * rng.range(6.0, 26.0),
                                   0.0, rng.range(0.25, 0.7), 255, 210, 120});
        }
        for (int i = 0; i < 40; ++i) {
            const real a = rng.range(0.0, 6.2831853);
            flames.push_back(Flame{at, Vec2(std::cos(a), std::sin(a)) * rng.range(2.0, 9.0),
                                   0.0, rng.range(0.35, 0.8), rng.range(0.18, 0.4), false});
        }
        igniteAt(at, 1.0);
    }

    void emitSparks(const Vec2& p, const Vec2& n, int count, int r, int g, int b) {
        for (int i = 0; i < count; ++i) {
            const real a = std::atan2(n.y, n.x) + rng.range(-1.1, 1.1);
            sparks.push_back(Spark{p, Vec2(std::cos(a), std::sin(a)) * rng.range(2.5, 11.0),
                                   0.0, rng.range(0.12, 0.45), r, g, b});
        }
    }

    // ------------------------------------------------------------ fire
    void igniteAt(const Vec2& at, real strength) {
        for (RigidBody* b : world.bodies()) {
            if (distance(b->position, at) > 1.0) continue;
            const Stuff s = stuff(b->id);
            if (s == Stuff::Metal || s == Stuff::Glass || s == Stuff::Ground || s == Stuff::Wall) continue;
            if (inWater(b->position, 0.45)) continue;         // wet things do not light
            burning[b->id] = std::max(burning[b->id], strength);
        }
        for (int i = 0; i < 12; ++i)
            flames.push_back(Flame{at + Vec2(rng.range(-0.3, 0.3), rng.range(-0.2, 0.2)),
                                   Vec2(rng.range(-0.6, 0.6), rng.range(0.8, 2.4)),
                                   0.0, rng.range(0.4, 0.9), rng.range(0.12, 0.26), false});
    }

    void updateFire(real dt) {
        std::vector<BodyId> extinguished, spreadTo;
        for (auto& kv : burning) {
            const BodyId id = kv.first;
            RigidBody* b = world.body(id);
            if (!b) { extinguished.push_back(id); continue; }

            if (inWater(b->position, b->boundingRadius + 0.3)) {
                extinguished.push_back(id);
                for (int i = 0; i < 5; ++i)                   // steam
                    flames.push_back(Flame{b->position,
                                           Vec2(rng.range(-0.5, 0.5), rng.range(0.6, 1.8)),
                                           0.0, 0.5, 0.16, true});
                continue;
            }

            kv.second = std::min((real)1.0, kv.second + 0.25 * dt);
            const real heat = kv.second;
            registry.ref(id).temperature += 420.0 * heat * dt;

            if (rng.unit() < heat * 0.85)
                flames.push_back(Flame{b->position + Vec2(rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)) * b->boundingRadius,
                                       Vec2(rng.range(-0.7, 0.7) + windStrength * 0.25, rng.range(1.4, 3.4)),
                                       0.0, rng.range(0.35, 0.85), rng.range(0.10, 0.24), false});

            if (fleshParts.count(id)) {
                if (rng.unit() < heat * 6.0 * dt) {
                    blood.addWound(id, b->position + Vec2(rng.range(-0.1, 0.1), rng.range(-0.1, 0.1)),
                                   Vec2(rng.range(-1.0, 1.0), 1.0), 0.18 * heat, false);
                    damageHuman(id, 110.0 * heat * dt);
                }
            } else {
                damageProp(id, 26.0 * heat * dt);
                if (propDamage[id] > 55.0 && stuff(id) != Stuff::Shard)
                    queueShatter(id, b->position, Vec2(0.0, 1.0));
            }

            for (RigidBody* o : world.bodies()) {              // radiation spread
                if (o->id == id || burning.count(o->id)) continue;
                const Stuff s = stuff(o->id);
                if (s == Stuff::Metal || s == Stuff::Glass || s == Stuff::Ground || s == Stuff::Wall) continue;
                if (distance(o->position, b->position) > 0.95) continue;
                if (inWater(o->position, 0.4)) continue;
                if (burning.size() + spreadTo.size() < 40 && rng.unit() < 0.10 * dt)
                    spreadTo.push_back(o->id);
            }
        }
        for (BodyId id : spreadTo) burning[id] = 0.15;
        for (BodyId id : extinguished) burning.erase(id);
    }

    void updateFx(real dt) {
        for (Flame& f : flames) {
            f.life += dt;
            f.vel.y += (f.smoke ? 1.2 : 3.4) * dt;
            f.vel.x += windStrength * 0.35 * dt;
            f.vel *= 0.985;
            f.pos += f.vel * dt;
        }
        flames.erase(std::remove_if(flames.begin(), flames.end(),
                                    [](const Flame& f) { return f.life >= f.maxLife; }), flames.end());
        if (flames.size() > 2400) flames.erase(flames.begin(), flames.begin() + (flames.size() - 2400));

        for (Spark& s : sparks) {
            s.life += dt;
            s.vel.y -= 16.0 * dt;
            s.vel *= 0.97;
            s.pos += s.vel * dt;
        }
        sparks.erase(std::remove_if(sparks.begin(), sparks.end(),
                                    [](const Spark& s) { return s.life >= s.maxLife; }), sparks.end());
        for (Tracer& t : tracers) t.life += dt;
        tracers.erase(std::remove_if(tracers.begin(), tracers.end(),
                                     [](const Tracer& t) { return t.life > 0.05; }), tracers.end());
        for (Muzzle& m : muzzles) m.life -= dt;
        muzzles.erase(std::remove_if(muzzles.begin(), muzzles.end(),
                                     [](const Muzzle& m) { return m.life <= 0.0; }), muzzles.end());
    }

    // ------------------------------------------------------------ contacts
    void onContact(const CollisionEvent& e) {
        if (!e.a || !e.b) return;
        const real speed = e.relativeSpeed;
        if (speed < 1.5) return;
        RigidBody* pair[2] = {e.a, e.b};
        for (RigidBody* b : pair) {
            if (!b->isDynamic()) continue;
            const Stuff s = stuff(b->id);
            damageProp(b->id, e.normalImpulse * 0.6);
            if (s == Stuff::Glass && speed > 3.5) {
                queueShatter(b->id, e.point, e.normal);
            } else if ((s == Stuff::Flesh || s == Stuff::Bone) && speed > 9.0) {
                const real sev = std::min((real)0.9, (speed - 9.0) / 26.0);
                if (sev > 0.12) {
                    blood.addWound(b->id, e.point, e.normal, sev, sev > 0.55);
                    blood.spray(e.point, e.normal, 3.0 + 9.0 * sev, 0.9, (int)(5 + 20 * sev), 0.5, 215);
                    damageHuman(b->id, sev * 45.0);
                }
            } else if (s == Stuff::Metal && speed > 5.0) {
                emitSparks(e.point, e.normal, 5, 255, 210, 130);
            } else if ((s == Stuff::Crate || s == Stuff::Wood) && propDamage[b->id] > 90.0) {
                queueShatter(b->id, e.point, e.normal);
            }
        }
    }

    void onPersist(const CollisionEvent& e) {
        if (!e.a || !e.b) return;
        const real work = std::fabs(e.tangentImpulse) * e.relativeSpeed;
        if (work < 12.0) return;
        RigidBody* pair[2] = {e.a, e.b};
        for (RigidBody* b : pair) {
            if (!b->isDynamic()) continue;
            registry.ref(b->id).temperature += work * 0.02;
            const Stuff s = stuff(b->id);
            if ((s == Stuff::Crate || s == Stuff::Wood) && work > 90.0 && !inWater(b->position, 0.4))
                burning[b->id] = std::max(burning[b->id], (real)0.12);
        }
    }

    // ------------------------------------------------------------ drag
    void beginDrag(const Vec2& at) {
        RigidBody* b = world.queryPoint(at);
        if (!b || b->isStatic()) return;
        dragBody = b->id;
        const real ca = std::cos(-b->angle), sa = std::sin(-b->angle);
        const Vec2 d = at - b->position;
        dragLocal = Vec2(ca * d.x - sa * d.y, sa * d.x + ca * d.y);
        dragTarget = at;
    }
    void endDrag() { dragBody = INVALID_BODY; }

    void applyDrag() {
        if (dragBody == INVALID_BODY) return;
        RigidBody* b = world.body(dragBody);
        if (!b) { dragBody = INVALID_BODY; return; }
        const Vec2 grip = b->transform().apply(dragLocal);
        const Vec2 delta = dragTarget - grip;
        const Vec2 v = b->velocityAtPoint(grip);
        b->applyForceAtPoint(delta * (900.0 * b->mass) - v * (40.0 * b->mass), grip);
    }

    // ------------------------------------------------------------ step
    void processPending() {
        for (size_t i = 0; i < pendingShatter.size(); ++i) {
            const BodyId id = pendingShatter[i];
            if (!world.body(id)) continue;
            const bool wasFlesh = fleshParts.count(id) != 0;
            const int pieces = wasFlesh ? 4 : (stuff(id) == Stuff::Glass ? 9 : 6);
            const std::vector<BodyId> frags = fracture.fracture(id, pendingShatterPt[i], pieces);
            if (frags.empty()) continue;
            for (BodyId f : frags) {
                tag(f, wasFlesh ? Stuff::Flesh : Stuff::Shard);
                if (wasFlesh) fleshParts.insert(f);
            }
            blood.forgetBody(id);
            stuffOf.erase(id); fleshParts.erase(id); burning.erase(id);
            health.erase(id); propDamage.erase(id);
            emitSparks(pendingShatterPt[i], pendingShatterDir[i] * -1.0, 10, 220, 240, 255);
        }
        pendingShatter.clear(); pendingShatterPt.clear(); pendingShatterDir.clear();
    }

    // A single NaN would take the whole simulation (and the SPH wave array)
    // down with it, so both worlds are sanitised every step.
    void sanitizeBodies() {
        std::vector<BodyId> bad;
        for (RigidBody* b : world.bodies()) {
            if (!std::isfinite(b->position.x) || !std::isfinite(b->position.y) ||
                !std::isfinite(b->velocity.x) || !std::isfinite(b->velocity.y) ||
                !std::isfinite(b->angle) || !std::isfinite(b->angularVelocity)) {
                if (b->isDynamic()) bad.push_back(b->id);
                continue;
            }
            if (!b->isDynamic()) continue;
            // Bullets are ray casts, so nothing needs to travel at 250 m/s.
            const real v = b->velocity.length();
            if (v > 45.0) b->velocity *= 45.0 / v;
            if (b->angularVelocity >  90.0) b->angularVelocity =  90.0;
            if (b->angularVelocity < -90.0) b->angularVelocity = -90.0;
            if (b->position.y < arenaFloor - 30.0 || std::fabs(b->position.x) > 60.0)
                bad.push_back(b->id);
        }
        for (BodyId id : bad) removeBody(id);
    }

    // SPH can build up runaway pressure; anything moving like a bullet is
    // cooled back down so the pool stays a pool.
    void sanitizeFluid() {
        const real maxSpeed = 6.0;
        bool bad = false;
        for (const FluidParticle& q : fluid.particles()) {
            if (!q.active) continue;
            if (!std::isfinite(q.position.x) || !std::isfinite(q.position.y) ||
                !std::isfinite(q.velocity.x) || !std::isfinite(q.velocity.y) ||
                q.velocity.lengthSq() > maxSpeed * maxSpeed) { bad = true; break; }
        }
        if (!bad) return;
        std::vector<std::pair<Vec2, Vec2>> keep;
        keep.reserve(fluid.count());
        for (const FluidParticle& q : fluid.particles()) {
            if (!q.active) continue;
            if (!std::isfinite(q.position.x) || !std::isfinite(q.position.y)) continue;
            Vec2 v = q.velocity;
            if (!std::isfinite(v.x) || !std::isfinite(v.y)) v = Vec2();
            const real sp = v.length();
            if (sp > maxSpeed) v *= maxSpeed / sp;
            keep.push_back(std::make_pair(q.position, v));
        }
        fluid.clear();
        for (size_t i = 0; i < keep.size(); ++i) fluid.emit(keep[i].first, keep[i].second);
    }

    void update(real dt) {
        if (paused && !stepOnce) { updateFx(dt); return; }
        stepOnce = false;
        const real scale = slowMotion ? 0.22 : (turbo ? 2.2 : 1.0);
        const real h = std::min((real)0.05, dt * scale);

        fireCooldown -= h;
        field.windBase = Vec2(windStrength, 0.0);
        if (windStrength > 0.01 || windStrength < -0.01) field.update(h);

        applyDrag();
        sanitizeBodies();
        sanitizeFluid();
        fluid.update(h);
        rebuildWaterGrid();

        if (!elec.electrodes().empty()) { elec.update(h); shockBodies(h); }

        world.step(h);
        thermal.update(h);
        blood.update(h);
        updateFire(h);
        updateFx(h);
        processPending();
        simTime += h;
    }

    void shockBodies(real dt) {
        for (const ShockEvent& s : elec.shocks()) {
            RigidBody* b = world.body(s.body);
            if (!b || !b->isDynamic() || !fleshParts.count(s.body)) continue;
            // Electrocution: the muscles convulse, the skin burns.
            b->applyImpulseAtPoint(Vec2(rng.range(-1.0, 1.0), rng.range(-0.4, 1.2)) *
                                       (0.35 * std::fabs(s.current) * b->mass), b->position);
            registry.ref(s.body).temperature += std::fabs(s.power) * dt * 0.4;
            damageHuman(s.body, std::fabs(s.power) * dt * 2.5);
            if (rng.unit() < 0.05) {
                blood.addWound(s.body, b->position, Vec2(0.0, 1.0), 0.12, false);
                emitSparks(b->position, Vec2(0.0, 1.0), 3, 180, 220, 255);
            }
        }
    }

    std::string hud() const {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "%s | map %d | bodies %zu | water %zu | blood %zu/%zu | fire %zu | "
                      "wind %.0f m/s | humans %d | shots %d | kills %d%s",
                      toolName(tool), (int)map + 1, world.bodyCount(), fluid.count(),
                      blood.stats().droplets, blood.stats().decals, burning.size(),
                      windStrength, ragdollCount, shotsFired, kills,
                      paused ? " | PAUSED" : (slowMotion ? " | SLOW-MO" : (turbo ? " | TURBO" : "")));
        return std::string(buf);
    }
};

// ============================================================ RENDERER
#if defined(_WIN32) && !defined(GORELAB_HEADLESS)
#include <windows.h>

static int  g_width = 1440, g_height = 860;
static real g_ppm = 34.0;                    // pixels per metre
static Vec2 g_cam(0.0, 7.0);

struct BackBuffer {
    HBITMAP bmp = nullptr;
    HDC dc = nullptr;
    uint32_t* px = nullptr;
    int w = 0, h = 0;

    void resize(HDC ref, int nw, int nh) {
        if (bmp && nw == w && nh == h) return;
        if (bmp) { DeleteObject(bmp); bmp = nullptr; }
        if (dc) { DeleteDC(dc); dc = nullptr; }
        w = nw; h = nh;
        BITMAPINFO bi;
        ZeroMemory(&bi, sizeof(bi));
        bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bi.bmiHeader.biWidth = w;
        bi.bmiHeader.biHeight = -h;             // top-down
        bi.bmiHeader.biPlanes = 1;
        bi.bmiHeader.biBitCount = 32;
        bi.bmiHeader.biCompression = BI_RGB;
        void* bits = nullptr;
        bmp = CreateDIBSection(ref, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
        px = (uint32_t*)bits;
        dc = CreateCompatibleDC(ref);
        SelectObject(dc, bmp);
    }

    inline void blend(int x, int y, int r, int g, int b, real a) {
        if (x < 0 || y < 0 || x >= w || y >= h || a <= 0.0 || !px) return;
        if (a > 1.0) a = 1.0;
        uint32_t& d = px[y * w + x];
        const int dr = (d >> 16) & 0xFF, dg = (d >> 8) & 0xFF, db = d & 0xFF;
        const int nr = (int)(dr + (r - dr) * a);
        const int ng = (int)(dg + (g - dg) * a);
        const int nb = (int)(db + (b - db) * a);
        d = (uint32_t)((nr << 16) | (ng << 8) | nb);
    }
    void disc(int cx, int cy, real rad, int r, int g, int b, real a) {
        const int ri = (int)std::ceil(rad);
        for (int y = -ri; y <= ri; ++y)
            for (int x = -ri; x <= ri; ++x) {
                const real d = std::sqrt((real)(x * x + y * y));
                if (d > rad) continue;
                blend(cx + x, cy + y, r, g, b, a * (1.0 - 0.55 * d / std::max((real)1.0, rad)));
            }
    }
    void line(Vec2 a, Vec2 b, int r, int g, int bb, real alpha, real thick = 1.0);
};
static BackBuffer g_bb;

static inline POINT toScreen(const Vec2& w) {
    POINT p;
    p.x = (LONG)((w.x - g_cam.x) * g_ppm + g_width * 0.5);
    p.y = (LONG)(g_height * 0.5 - (w.y - g_cam.y) * g_ppm);
    return p;
}
static inline Vec2 toWorld(int sx, int sy) {
    return Vec2((sx - g_width * 0.5) / g_ppm + g_cam.x,
                (g_height * 0.5 - sy) / g_ppm + g_cam.y);
}
void BackBuffer::line(Vec2 a, Vec2 b, int r, int g, int bb2, real alpha, real thick) {
    const POINT pa = toScreen(a), pb = toScreen(b);
    const int steps = (int)std::max((real)1.0,
                        (real)std::max(std::abs(pb.x - pa.x), std::abs(pb.y - pa.y)));
    const int ti = (int)thick;
    for (int i = 0; i <= steps; ++i) {
        const real t = (real)i / steps;
        const int x = (int)(pa.x + (pb.x - pa.x) * t);
        const int y = (int)(pa.y + (pb.y - pa.y) * t);
        for (int oy = -ti; oy <= ti; ++oy)
            for (int ox = -ti; ox <= ti; ++ox) blend(x + ox, y + oy, r, g, bb2, alpha);
    }
}

// ---- basic procedural textures -------------------------------------------
static void boundsOf(const POINT* p, int n, int& minx, int& maxx, int& miny, int& maxy) {
    minx = maxx = (int)p[0].x; miny = maxy = (int)p[0].y;
    for (int i = 1; i < n; ++i) {
        minx = std::min(minx, (int)p[i].x); maxx = std::max(maxx, (int)p[i].x);
        miny = std::min(miny, (int)p[i].y); maxy = std::max(maxy, (int)p[i].y);
    }
}
static void textureWood(const POINT* pts, int n, real angle, real charAmount) {
    int minx, maxx, miny, maxy;
    boundsOf(pts, n, minx, maxx, miny, maxy);
    const real c = std::cos(angle), s = std::sin(angle);
    const int step = std::max(5, (maxy - miny) / 4);
    const real k = std::max((real)0.0, (real)1.0 - charAmount);
    for (int y = std::max(0, miny); y <= maxy; ++y)
        for (int x = std::max(0, minx); x <= maxx; ++x) {
            const int lx = (int)((x - minx) * c + (y - miny) * s);
            if (step <= 0 || (lx % step) != 0) continue;
            g_bb.blend(x, y, (int)(72 * k), (int)(48 * k), (int)(26 * k), 0.35);
        }
}
static void textureMetal(const POINT* pts, int n) {
    int minx, maxx, miny, maxy;
    boundsOf(pts, n, minx, maxx, miny, maxy);
    for (int y = miny + 3; y <= maxy - 3; y += 9)
        for (int x = minx + 3; x <= maxx - 3; x += 9) g_bb.disc(x, y, 1.6, 210, 214, 224, 0.5);
    for (int y = miny; y <= maxy; ++y) g_bb.blend(minx + (maxx - minx) / 4, y, 235, 240, 250, 0.25);
}
static void textureGlass(const POINT* pts, int n) {
    int minx, maxx, miny, maxy;
    boundsOf(pts, n, minx, maxx, miny, maxy);
    const int span = std::max(1, maxx - minx);
    for (int y = miny; y <= maxy; ++y) {
        const int x = minx + (int)((y - miny) * 0.35) % span;
        g_bb.blend(x, y, 255, 255, 255, 0.20);
        g_bb.blend(x + 1, y, 255, 255, 255, 0.12);
    }
}
static void bloodColor(uint8_t oxygen, real wetness, int& r, int& g, int& b) {
    uint8_t rr = 0, gg = 0, bb = 0;
    BloodSystem::colorOf(oxygen, wetness, rr, gg, bb);
    r = (int)rr; g = (int)gg; b = (int)bb;
}

static void stuffColor(Stuff s, real charAmount, real wet, int& r, int& g, int& b) {
    switch (s) {
        case Stuff::Ground: r = 62;  g = 58;  b = 52;  break;
        case Stuff::Wall:   r = 74;  g = 70;  b = 64;  break;
        case Stuff::Crate:  r = 158; g = 108; b = 56;  break;
        case Stuff::Wood:   r = 132; g = 92;  b = 48;  break;
        case Stuff::Metal:  r = 168; g = 174; b = 186; break;
        case Stuff::Glass:  r = 150; g = 198; b = 214; break;
        case Stuff::Shard:  r = 176; g = 214; b = 226; break;
        case Stuff::Flesh:  r = 214; g = 156; b = 142; break;
        case Stuff::Bone:   r = 226; g = 214; b = 196; break;
        default:            r = 130; g = 130; b = 130; break;
    }
    const real c = std::min((real)0.85, charAmount);
    r = (int)(r * (1.0 - c)); g = (int)(g * (1.0 - c)); b = (int)(b * (1.0 - c));
    if (wet > 0.0) {                                   // blood soaked
        const real w = std::min((real)0.8, wet);
        r = (int)(r + (120 - r) * w);
        g = (int)(g * (1.0 - w * 0.8));
        b = (int)(b * (1.0 - w * 0.8));
    }
}

static void renderScene(GoreLab& game) {
    for (int y = 0; y < g_bb.h; ++y) {                 // backdrop gradient
        const real t = (real)y / std::max(1, g_bb.h);
        const uint32_t c = ((uint32_t)(26 + 22 * t) << 16) | ((uint32_t)(28 + 24 * t) << 8) |
                           (uint32_t)(34 + 30 * t);
        for (int x = 0; x < g_bb.w; ++x) g_bb.px[y * g_bb.w + x] = c;
    }

    if (game.showWater) {                              // water, tinted by voltage
        const std::vector<real>& volts = game.elec.nodeVoltages();
        size_t idx = 0;
        for (const FluidParticle& p : game.fluid.particles()) {
            if (!p.active) continue;
            const POINT sp = toScreen(p.position);
            const real v = (idx < volts.size()) ? volts[idx] : 0.0;
            ++idx;
            const real e = std::min((real)1.0, std::fabs(v) / 220.0);
            g_bb.disc(sp.x, sp.y, g_ppm * 0.22, (int)(50 + 190 * e), (int)(120 + 110 * e),
                      (int)(210 + 40 * e), 0.5 + 0.4 * e);
        }
    }

    for (const BloodPool& pool : game.blood.pools()) { // pools on the floor
        const POINT sp = toScreen(pool.center);
        int r, g, b;
        bloodColor(pool.oxygen, pool.wetness, r, g, b);
        const int hw = (int)(pool.halfWidth * g_ppm);
        const int hh = std::max(2, (int)(pool.depth * g_ppm));
        for (int y = -hh; y <= hh; ++y)
            for (int x = -hw; x <= hw; ++x)
                g_bb.blend(sp.x + x, sp.y + y, r, g, b,
                           0.75 * (1.0 - (real)std::abs(x) / std::max(1, hw)));
    }

    for (RigidBody* body : game.world.bodies()) {      // bodies
        const Stuff s = game.stuff(body->id);
        real charAmount = 0.0;
        auto bit = game.burning.find(body->id);
        if (bit != game.burning.end()) charAmount = 0.5 * bit->second;
        auto dit = game.propDamage.find(body->id);
        if (dit != game.propDamage.end()) charAmount += std::min((real)0.4, dit->second / 200.0);
        int r, g, b;
        stuffColor(s, charAmount, game.blood.wetnessOf(body->id), r, g, b);

        if (body->shape.type == ShapeType::Circle) {
            const POINT c = toScreen(body->position);
            g_bb.disc(c.x, c.y, body->shape.radius * g_ppm, r, g, b, 0.95);
        } else if (body->shape.type == ShapeType::Capsule) {
            Vec2 a, e;
            body->shape.capsuleSegment(body->transform(), a, e);
            const POINT pa = toScreen(a), pb = toScreen(e);
            const int steps = std::max(1, (int)std::max(std::abs(pb.x - pa.x), std::abs(pb.y - pa.y)));
            for (int i = 0; i <= steps; ++i) {
                const real t = (real)i / steps;
                g_bb.disc((int)(pa.x + (pb.x - pa.x) * t), (int)(pa.y + (pb.y - pa.y) * t),
                          body->shape.radius * g_ppm, r, g, b, 0.95);
            }
        } else {
            const std::vector<Vec2>& vs = body->worldVertices;
            if (vs.size() < 3) continue;
            std::vector<POINT> pts(vs.size());
            for (size_t i = 0; i < vs.size(); ++i) pts[i] = toScreen(vs[i]);
            int minx, maxx, miny, maxy;
            boundsOf(pts.data(), (int)pts.size(), minx, maxx, miny, maxy);
            for (int y = std::max(0, miny); y <= std::min(g_bb.h - 1, maxy); ++y) {
                std::vector<int> xs;
                for (size_t i = 0; i < pts.size(); ++i) {
                    const POINT& p1 = pts[i];
                    const POINT& p2 = pts[(i + 1) % pts.size()];
                    if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y))
                        xs.push_back((int)(p1.x + (real)(y - p1.y) / (p2.y - p1.y) * (p2.x - p1.x)));
                }
                std::sort(xs.begin(), xs.end());
                for (size_t i = 0; i + 1 < xs.size(); i += 2)
                    for (int x = xs[i]; x <= xs[i + 1]; ++x) g_bb.blend(x, y, r, g, b, 0.95);
            }
            if (s == Stuff::Crate || s == Stuff::Wood || s == Stuff::Ground)
                textureWood(pts.data(), (int)pts.size(), body->angle, charAmount);
            else if (s == Stuff::Metal) textureMetal(pts.data(), (int)pts.size());
            else if (s == Stuff::Glass || s == Stuff::Shard) textureGlass(pts.data(), (int)pts.size());
            for (size_t i = 0; i < pts.size(); ++i)
                g_bb.line(vs[i], vs[(i + 1) % vs.size()], 12, 12, 16, 0.6);
        }
    }

    for (const BloodDecal& d : game.blood.decals()) {  // blood stuck to bodies
        RigidBody* host = game.world.body(d.body);
        if (!host) continue;
        const POINT sp = toScreen(host->transform().apply(d.anchor));
        int r, g, b;
        bloodColor(d.oxygen, d.wetness, r, g, b);
        g_bb.disc(sp.x, sp.y, std::max((real)1.5, d.volume * 0.6), r, g, b, 0.85);
    }
    for (const BloodDroplet& dr : game.blood.droplets()) {
        if (!dr.active) continue;
        const POINT sp = toScreen(dr.position);
        int r, g, b;
        bloodColor(dr.oxygen, 1.0, r, g, b);
        g_bb.disc(sp.x, sp.y, std::max((real)1.0, dr.radius * g_ppm), r, g, b, 0.9);
    }
    for (const Tracer& t : game.tracers) g_bb.line(t.a, t.b, 255, 238, 170, 0.85, 1.0);
    for (const Muzzle& m : game.muzzles) {
        const POINT sp = toScreen(m.pos + m.dir * 0.25);
        g_bb.disc(sp.x, sp.y, 9.0, 255, 220, 150, 0.9);
    }
    for (const Spark& sp : game.sparks) {
        const POINT p = toScreen(sp.pos);
        g_bb.disc(p.x, p.y, 1.8, sp.r, sp.g, sp.b, 1.0 - sp.life / sp.maxLife);
    }
    for (const Flame& f : game.flames) {
        const POINT p = toScreen(f.pos);
        const real t = f.life / f.maxLife, a = (1.0 - t) * 0.85;
        if (f.smoke) g_bb.disc(p.x, p.y, f.size * g_ppm * (1.0 + t * 2.0), 180, 180, 185, a * 0.5);
        else g_bb.disc(p.x, p.y, f.size * g_ppm * (1.0 + t), 255, (int)(200 - 140 * t),
                       (int)(70 - 60 * t), a);
    }
    for (const Electrode& e : game.elec.electrodes()) {
        const POINT p = toScreen(e.position);
        g_bb.disc(p.x, p.y, 6.0, e.ground ? 120 : 255, e.ground ? 200 : 240, 255, 0.9);
    }
    if (game.dragBody != INVALID_BODY)
        if (RigidBody* b = game.world.body(game.dragBody))
            g_bb.line(b->transform().apply(game.dragLocal), game.dragTarget, 240, 240, 120, 0.7);
}

static GoreLab* g_game = nullptr;
static bool g_lmb = false, g_lmbEdge = false;

static LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_DESTROY: PostQuitMessage(0); return 0;
        case WM_SIZE: g_width = LOWORD(lp); g_height = HIWORD(lp); return 0;
        case WM_MOUSEMOVE: {
            if (g_game) {
                g_game->aimPoint = toWorld((int)(short)LOWORD(lp), (int)(short)HIWORD(lp));
                g_game->dragTarget = g_game->aimPoint;
            }
            return 0;
        }
        case WM_LBUTTONDOWN: g_lmb = true; g_lmbEdge = true; return 0;
        case WM_LBUTTONUP:   g_lmb = false; return 0;
        case WM_RBUTTONDOWN: if (g_game) g_game->beginDrag(g_game->aimPoint); return 0;
        case WM_RBUTTONUP:   if (g_game) g_game->endDrag(); return 0;
        case WM_MOUSEWHEEL:
            g_ppm = std::max((real)8.0, std::min((real)120.0,
                    g_ppm * (GET_WHEEL_DELTA_WPARAM(wp) > 0 ? 1.1 : 0.9)));
            return 0;
        case WM_KEYDOWN: {
            if (!g_game) return 0;
            GoreLab& g = *g_game;
            switch (wp) {
                case '1': g.tool = Tool::Human;   break;
                case '2': g.tool = Tool::Water;   break;
                case '3': g.tool = Tool::Crate;   break;
                case '4': g.tool = Tool::Glass;   break;
                case '5': g.tool = Tool::Metal;   break;
                case '6': g.tool = Tool::Pistol;  break;
                case '7': g.tool = Tool::Shotgun; break;
                case '8': g.tool = Tool::Rifle;   break;
                case '9': g.tool = Tool::Fire;    break;
                case '0': g.tool = Tool::Grenade; break;
                case 'E': g.tool = Tool::Voltage; break;
                case 'X': g.tool = Tool::Erase;   break;
                case VK_F1: g.loadMap(MapId::Box); break;
                case VK_F2: g.loadMap(MapId::WaterBox); break;
                case 'P': g.paused = !g.paused; break;
                case VK_OEM_PERIOD: g.stepOnce = true; break;
                case 'L': g.slowMotion = !g.slowMotion; g.turbo = false; break;
                case 'T': g.turbo = !g.turbo; g.slowMotion = false; break;
                case VK_OEM_4: g.windStrength = std::max((real)-30.0, g.windStrength - 2.0); break;
                case VK_OEM_6: g.windStrength = std::min((real)30.0, g.windStrength + 2.0); break;
                case 'R': g.loadMap(g.map); break;
                case VK_ESCAPE: PostQuitMessage(0); break;
            }
            return 0;
        }
    }
    return DefWindowProc(hwnd, msg, wp, lp);
}

int WINAPI WinMain(HINSTANCE inst, HINSTANCE, LPSTR, int) {
    WNDCLASS wc;
    ZeroMemory(&wc, sizeof(wc));
    wc.lpfnWndProc = wndProc;
    wc.hInstance = inst;
    wc.hCursor = LoadCursor(nullptr, IDC_CROSS);
    wc.lpszClassName = "GoreLabRemake";
    RegisterClass(&wc);
    HWND hwnd = CreateWindow("GoreLabRemake", "GoreLab Remake - phys2d sandbox",
                             WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
                             g_width, g_height, nullptr, nullptr, inst, nullptr);
    GoreLab game;
    g_game = &game;

    LARGE_INTEGER freq, prev;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);

    MSG msg;
    bool running = true;
    while (running) {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) { running = false; break; }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        if (!running) break;

        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        real dt = (real)(now.QuadPart - prev.QuadPart) / (real)freq.QuadPart;
        prev = now;
        dt = std::max((real)(1.0 / 600.0), std::min((real)(1.0 / 20.0), dt));

        if (g_lmb) {
            const bool autoFire = (game.tool == Tool::Rifle || game.tool == Tool::Water ||
                                   game.tool == Tool::Fire);
            if (autoFire || g_lmbEdge) {
                const Vec2 muzzle(g_cam.x - 14.0, 3.0);   // guns fire from the left
                game.useTool(game.aimPoint, muzzle);
            }
            g_lmbEdge = false;
        }

        game.update(dt);

        HDC hdc = GetDC(hwnd);
        g_bb.resize(hdc, g_width, g_height);
        renderScene(game);
        SetBkMode(g_bb.dc, TRANSPARENT);
        SetTextColor(g_bb.dc, RGB(235, 235, 235));
        const std::string hud = game.hud();
        TextOutA(g_bb.dc, 12, 10, hud.c_str(), (int)hud.size());
        const char* l1 = "1 human  2 water  3 crate  4 glass  5 steel  6 pistol  7 shotgun  8 rifle  9 fire  0 grenade";
        const char* l2 = "E electrode  X erase  RMB drag  wheel zoom  F1/F2 map  P pause  . step  L slow-mo  T turbo  [ ] wind  R reset";
        TextOutA(g_bb.dc, 12, 28, l1, (int)std::strlen(l1));
        TextOutA(g_bb.dc, 12, 46, l2, (int)std::strlen(l2));
        BitBlt(hdc, 0, 0, g_width, g_height, g_bb.dc, 0, 0, SRCCOPY);
        ReleaseDC(hwnd, hdc);
    }
    return 0;
}

#else
// ============================================================ headless test
int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("GoreLab Remake - headless verification\n");
    std::printf("======================================\n\n");

    GoreLab game;
    std::printf("[map 1] plain box\n");
    std::printf("  bodies after load      : %zu\n", game.world.bodyCount());
    game.spawnRagdoll(Vec2(-2.0, 6.0));
    game.spawnRagdoll(Vec2(2.0, 6.0));
    for (int i = 0; i < 120; ++i) game.update(1.0 / 60.0);
    std::printf("  humans                 : %d (bodies %zu)\n", game.ragdollCount, game.world.bodyCount());

    const Vec2 muzzle(-14.0, 3.0);
    game.fireCooldown = 0.0;
    game.shoot(muzzle, Vec2(0.0, 3.2), WEAPON_PISTOL);
    for (int i = 0; i < 30; ++i) game.update(1.0 / 60.0);
    std::printf("  pistol                 : airborne blood %zu, wounds %zu\n",
                game.blood.stats().droplets, game.blood.stats().wounds);

    game.fireCooldown = 0.0;
    game.shoot(muzzle, Vec2(-2.0, 3.0), WEAPON_SHOTGUN);
    for (int i = 0; i < 40; ++i) game.update(1.0 / 60.0);
    std::printf("  shotgun (9 pellets)    : blood %zu drops / %zu decals / %zu pools\n",
                game.blood.stats().droplets, game.blood.stats().decals, game.blood.stats().pools);

    for (int shot = 0; shot < 90; ++shot) {
        game.fireCooldown = 0.0;
        game.shoot(muzzle, Vec2(2.0, 3.0 + 0.02 * shot), WEAPON_RIFLE);
        game.update(1.0 / 60.0);
    }
    std::printf("  rifle burst            : %d shots, kills %d\n", game.shotsFired, game.kills);

    const BodyId pane = game.makeGlassPane(Vec2(10.0, 2.0), 0.16, 3.0);
    size_t before = game.world.bodyCount();
    game.queueShatter(pane, Vec2(10.0, 2.4), Vec2(1.0, 0.0));
    game.update(1.0 / 60.0);
    std::printf("  glass shattered        : bodies %zu -> %zu (shards)\n", before, game.world.bodyCount());

    game.igniteAt(Vec2(6.5, 3.8), 1.0);
    for (int i = 0; i < 180; ++i) game.update(1.0 / 60.0);
    std::printf("  fire                   : burning %zu, flame particles %zu\n",
                game.burning.size(), game.flames.size());

    before = game.world.bodyCount();
    game.throwGrenade(Vec2(0.0, 2.0));
    for (int i = 0; i < 5; ++i) game.update(1.0 / 60.0);
    std::printf("  grenade                : bodies %zu -> %zu, airborne blood %zu\n",
                before, game.world.bodyCount(), game.blood.stats().droplets);
    for (int i = 0; i < 55; ++i) game.update(1.0 / 60.0);

    game.windStrength = 24.0;
    for (int i = 0; i < 120; ++i) game.update(1.0 / 60.0);
    std::printf("  wind 24 m/s            : stable, bodies %zu\n\n", game.world.bodyCount());

    game.loadMap(MapId::WaterBox);
    for (int i = 0; i < 120; ++i) game.update(1.0 / 60.0);
    {
        Vec2 lo(1e30, 1e30), hi(-1e30, -1e30);
        for (const FluidParticle& q : game.fluid.particles()) {
            if (!q.active) continue;
            lo.x = std::min(lo.x, q.position.x); lo.y = std::min(lo.y, q.position.y);
            hi.x = std::max(hi.x, q.position.x); hi.y = std::max(hi.y, q.position.y);
        }
        std::printf("[map 2] water box\n");
        std::printf("  water particles        : %zu, pool x[%.1f..%.1f] y[%.1f..%.1f]\n",
                    game.fluid.count(), lo.x, hi.x, lo.y, hi.y);
    }

    game.spawnRagdoll(Vec2(-6.0, 6.0));
    for (int i = 0; i < 150; ++i) game.update(1.0 / 60.0);
    std::printf("  human dropped in water : bodies %zu\n", game.world.bodyCount());

    game.placeElectrode(Vec2(-8.0, 0.8));
    for (int i = 0; i < 60; ++i) game.update(1.0 / 60.0);
    std::printf("  220 V in the water     : nodes %zu, current %.4f A, power %.1f W, shocked %zu\n",
                game.elec.wetNodeCount(), game.elec.totalCurrent(),
                game.elec.dissipatedPower(), game.elec.shocks().size());

    const BodyId dry = game.makeCrate(Vec2(0.0, 12.0), 0.9);
    game.update(1.0 / 60.0);
    bool dryShocked = false;
    for (const ShockEvent& s : game.elec.shocks()) if (s.body == dry) dryShocked = true;
    std::printf("  dry body above the pool: %s\n", dryShocked ? "SHOCKED (bug)" : "safe (correct)");

    const size_t burnBefore = game.burning.size();
    game.makeCrate(Vec2(-2.0, 6.0), 0.9);
    game.igniteAt(Vec2(-2.0, 6.0), 1.0);
    const size_t lit = game.burning.size();
    for (int i = 0; i < 240; ++i) game.update(1.0 / 60.0);
    std::printf("  burning crate in water : %zu -> %zu lit -> %zu after it fell in\n",
                burnBefore, lit, game.burning.size());

    std::printf("\nfinal: %s\n", game.hud().c_str());
    std::printf("\nall gameplay systems executed.\n");
    return 0;
}
#endif
