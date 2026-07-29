// ============================================================================
// GORELAB REMAKE 2 - People-Playground style sandbox stressing every phys2d
// subsystem: rigid bodies, joints, CCD, fracture, SPH water, ragdolls, blood,
// fire, wind, contact events, runtime statistics, software renderer + Win32.
// ============================================================================
#include "phys2d/World.h"
#include "phys2d/Extras.h"
#include "phys2d/Blood.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>
#include <unordered_map>

#if defined(_WIN32) && !defined(GORELAB_HEADLESS)
#  define GORELAB_WIN 1
#  include <windows.h>
#endif

using namespace phys2d;

static inline RigidBody* asBody(RigidBody* b) { return b; }
static inline RigidBody* asBody(RigidBody& b) { return &b; }
template <class T> static inline RigidBody* asBody(const std::unique_ptr<T>& p) { return p.get(); }

static uint32_t g_rng = 0x1234567u;
static inline real rnd01() { g_rng = g_rng * 1664525u + 1013904223u; return (real)((g_rng >> 8) & 0xFFFFFF) / (real)0x1000000; }
static inline real rnd(real a, real b) { return a + (b - a) * rnd01(); }
static inline real clampr2(real v, real lo, real hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Canvas {
    int w = 0, h = 0;
    std::vector<uint32_t> px;
    void resize(int W, int H) { w = W; h = H; px.assign((size_t)W * H, 0xFF000000u); }
    void clearGradient(int r0, int g0, int b0, int r1, int g1, int b1) {
        for (int y = 0; y < h; ++y) {
            real t = (real)y / (real)(h > 1 ? h - 1 : 1);
            uint32_t c = 0xFF000000u | ((uint32_t)(int)(r0 + (r1 - r0) * t) << 16)
                       | ((uint32_t)(int)(g0 + (g1 - g0) * t) << 8) | (uint32_t)(int)(b0 + (b1 - b0) * t);
            uint32_t* row = &px[(size_t)y * w];
            for (int x = 0; x < w; ++x) row[x] = c;
        }
    }
    inline void blend(int x, int y, int r, int g, int b, real a) {
        if (a <= 0.0 || x < 0 || y < 0 || x >= w || y >= h) return;
        if (a > 1.0) a = 1.0;
        uint32_t& d = px[(size_t)y * w + x];
        int dr = (int)((d >> 16) & 255), dg = (int)((d >> 8) & 255), db = (int)(d & 255);
        d = 0xFF000000u | ((uint32_t)(int)(dr + (r - dr) * a) << 16)
          | ((uint32_t)(int)(dg + (g - dg) * a) << 8) | (uint32_t)(int)(db + (b - db) * a);
    }
    inline void add(int x, int y, int r, int g, int b, real a) {
        if (a <= 0.0 || x < 0 || y < 0 || x >= w || y >= h) return;
        uint32_t& d = px[(size_t)y * w + x];
        int nr = std::min(255, (int)((d >> 16) & 255) + (int)(r * a));
        int ng = std::min(255, (int)((d >> 8) & 255) + (int)(g * a));
        int nb = std::min(255, (int)(d & 255) + (int)(b * a));
        d = 0xFF000000u | ((uint32_t)nr << 16) | ((uint32_t)ng << 8) | (uint32_t)nb;
    }
    void rect(int x0, int y0, int x1, int y1, int r, int g, int b, real a = 1.0) {
        if (x1 < x0) std::swap(x0, x1);
        if (y1 < y0) std::swap(y0, y1);
        for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) blend(x, y, r, g, b, a);
    }
    void disc(real cx, real cy, real rad, int r, int g, int b, real a = 1.0) {
        if (rad < 0.4) rad = 0.4;
        int x0 = (int)std::floor(cx - rad - 1), x1 = (int)std::ceil(cx + rad + 1);
        int y0 = (int)std::floor(cy - rad - 1), y1 = (int)std::ceil(cy + rad + 1);
        for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
            real dx = x + 0.5 - cx, dy = y + 0.5 - cy;
            real cov = clampr2(rad + 0.5 - std::sqrt(dx * dx + dy * dy), 0.0, 1.0);
            if (cov > 0.0) blend(x, y, r, g, b, a * cov);
        }
    }
    void softDisc(real cx, real cy, real rad, int r, int g, int b, real a, bool additive = false) {
        int x0 = (int)std::floor(cx - rad - 1), x1 = (int)std::ceil(cx + rad + 1);
        int y0 = (int)std::floor(cy - rad - 1), y1 = (int)std::ceil(cy + rad + 1);
        real inv = rad > 0.0 ? 1.0 / rad : 0.0;
        for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
            real dx = (x + 0.5 - cx) * inv, dy = (y + 0.5 - cy) * inv;
            real q = dx * dx + dy * dy;
            if (q >= 1.0) continue;
            real f = (1.0 - q) * (1.0 - q);
            if (additive) add(x, y, r, g, b, a * f); else blend(x, y, r, g, b, a * f);
        }
    }
    void line(real x0, real y0, real x1, real y1, real width, int r, int g, int b, real a = 1.0) {
        real dx = x1 - x0, dy = y1 - y0;
        real len = std::sqrt(dx * dx + dy * dy);
        int steps = (int)(len / std::max((real)0.5, width * 0.4)) + 1;
        for (int i = 0; i <= steps; ++i) {
            real t = (real)i / (real)steps;
            disc(x0 + dx * t, y0 + dy * t, width * 0.5, r, g, b, a);
        }
    }
    void poly(const std::vector<Vec2>& p, int r, int g, int b, real a = 1.0) {
        if (p.size() < 3) return;
        real ymin = p[0].y, ymax = p[0].y;
        for (size_t i = 0; i < p.size(); ++i) { ymin = std::min(ymin, p[i].y); ymax = std::max(ymax, p[i].y); }
        int y0 = std::max(0, (int)std::floor(ymin)), y1 = std::min(h - 1, (int)std::ceil(ymax));
        std::vector<real> xs;
        for (int y = y0; y <= y1; ++y) {
            real yc = y + 0.5;
            xs.clear();
            for (size_t i = 0; i < p.size(); ++i) {
                const Vec2& A = p[i];
                const Vec2& B = p[(i + 1) % p.size()];
                if ((A.y <= yc && B.y > yc) || (B.y <= yc && A.y > yc))
                    xs.push_back(A.x + (yc - A.y) / (B.y - A.y) * (B.x - A.x));
            }
            if (xs.size() < 2) continue;
            std::sort(xs.begin(), xs.end());
            for (size_t k = 0; k + 1 < xs.size(); k += 2) {
                int xa = std::max(0, (int)std::floor(xs[k])), xb = std::min(w - 1, (int)std::ceil(xs[k + 1]));
                for (int x = xa; x <= xb; ++x) {
                    real cov = std::min(x + 1.0 - xs[k], xs[k + 1] - x);
                    if (cov > 0.0) blend(x, y, r, g, b, a * std::min((real)1.0, cov));
                }
            }
        }
    }
    void outline(const std::vector<Vec2>& p, real width, int r, int g, int b, real a = 1.0) {
        for (size_t i = 0; i < p.size(); ++i) {
            const Vec2& A = p[i];
            const Vec2& B = p[(i + 1) % p.size()];
            line(A.x, A.y, B.x, B.y, width, r, g, b, a);
        }
    }
    void glyph(char c, int x, int y, int s, int r, int g, int b, real a);
    void text(const char* str, int x, int y, int s, int r, int g, int b, real a = 1.0) {
        int cx = x;
        for (const char* p = str; *p; ++p) { glyph(*p, cx, y, s, r, g, b, a); cx += 4 * s; }
    }
    int textW(const char* str, int s) const { return (int)std::strlen(str) * 4 * s; }
    void savePpm(const char* path) const {
        FILE* f = std::fopen(path, "wb");
        if (!f) return;
        std::fprintf(f, "P6\n%d %d\n255\n", w, h);
        for (size_t i = 0; i < px.size(); ++i) {
            unsigned char rgb[3] = { (unsigned char)((px[i] >> 16) & 255), (unsigned char)((px[i] >> 8) & 255), (unsigned char)(px[i] & 255) };
            std::fwrite(rgb, 1, 3, f);
        }
        std::fclose(f);
    }
};

#define GG(a, b, c, d, e) (uint16_t)(((a) << 12) | ((b) << 9) | ((c) << 6) | ((d) << 3) | (e))
struct Glyph { char c; uint16_t bits; };
static const Glyph FONT[] = {
    { '0', GG(7,5,5,5,7) }, { '1', GG(2,6,2,2,7) }, { '2', GG(7,1,7,4,7) }, { '3', GG(7,1,7,1,7) },
    { '4', GG(5,5,7,1,1) }, { '5', GG(7,4,7,1,7) }, { '6', GG(7,4,7,5,7) }, { '7', GG(7,1,1,1,1) },
    { '8', GG(7,5,7,5,7) }, { '9', GG(7,5,7,1,7) },
    { 'A', GG(7,5,7,5,5) }, { 'B', GG(7,5,6,5,7) }, { 'C', GG(7,4,4,4,7) }, { 'D', GG(6,5,5,5,6) },
    { 'E', GG(7,4,7,4,7) }, { 'F', GG(7,4,7,4,4) }, { 'G', GG(7,4,5,5,7) }, { 'H', GG(5,5,7,5,5) },
    { 'I', GG(7,2,2,2,7) }, { 'J', GG(1,1,1,5,7) }, { 'K', GG(5,5,6,5,5) }, { 'L', GG(4,4,4,4,7) },
    { 'M', GG(5,7,7,5,5) }, { 'N', GG(6,5,5,5,5) }, { 'O', GG(7,5,5,5,7) }, { 'P', GG(7,5,7,4,4) },
    { 'Q', GG(7,5,5,7,1) }, { 'R', GG(7,5,7,6,5) }, { 'S', GG(7,4,7,1,7) }, { 'T', GG(7,2,2,2,2) },
    { 'U', GG(5,5,5,5,7) }, { 'V', GG(5,5,5,5,2) }, { 'W', GG(5,5,7,7,5) }, { 'X', GG(5,5,2,5,5) },
    { 'Y', GG(5,5,2,2,2) }, { 'Z', GG(7,1,2,4,7) },
    { '.', GG(0,0,0,0,2) }, { ',', GG(0,0,0,2,4) }, { '-', GG(0,0,7,0,0) }, { '+', GG(0,2,7,2,0) },
    { ':', GG(0,2,0,2,0) }, { '/', GG(1,1,2,4,4) }, { '%', GG(5,1,2,4,5) }, { '!', GG(2,2,2,0,2) },
    { '(', GG(2,4,4,4,2) }, { ')', GG(2,1,1,1,2) }, { '<', GG(1,2,4,2,1) }, { '>', GG(4,2,1,2,4) },
    { '*', GG(5,2,7,2,5) }, { '=', GG(0,7,0,7,0) }, { '?', GG(7,1,3,0,2) }, { '_', GG(0,0,0,0,7) },
};
void Canvas::glyph(char c, int x, int y, int s, int r, int g, int b, real a) {
    if (c == ' ') return;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    uint16_t bits = 0;
    bool found = false;
    for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); ++i)
        if (FONT[i].c == c) { bits = FONT[i].bits; found = true; break; }
    if (!found) return;
    for (int row = 0; row < 5; ++row) {
        int rowBits = (bits >> (12 - row * 3)) & 7;
        for (int col = 0; col < 3; ++col)
            if (rowBits & (4 >> col))
                rect(x + col * s, y + row * s, x + col * s + s - 1, y + row * s + s - 1, r, g, b, a);
    }
}

struct Camera {
    Vec2 center{ 0.0, 8.0 };
    real ppm = 34.0;
    int w = 1280, h = 760;
    Vec2 toScreen(const Vec2& p) const { return Vec2(w * 0.5 + (p.x - center.x) * ppm, h * 0.5 - (p.y - center.y) * ppm); }
    Vec2 toWorld(real sx, real sy) const { return Vec2(center.x + (sx - w * 0.5) / ppm, center.y - (sy - h * 0.5) / ppm); }
};

struct Input {
    bool key[256];
    bool pressed[256];
    int mx = 0, my = 0, wheel = 0;
    bool lmb = false, rmb = false, lclick = false, rclick = false;
    Input() { std::memset(key, 0, sizeof(key)); std::memset(pressed, 0, sizeof(pressed)); }
    void newFrame() { std::memset(pressed, 0, sizeof(pressed)); lclick = rclick = false; wheel = 0; }
};

enum class Kind : uint8_t { Ground, Wood, Steel, Glass, Ice, Rubber, Stone, Bomb, Balloon, Human, Debris };

struct Meta {
    Kind kind = Kind::Wood;
    real hp = 100.0, maxHp = 100.0;
    real burn = 0.0, temp = 293.0;
    bool flammable = false, breakable = false, sharp = false;
    int  gen = 0, human = -1;
};
struct HumanInfo { Ragdoll rag; real hp = 100.0; bool dead = false; };
struct Ember { Vec2 p, v; real life = 0.0, max = 1.0, heat = 1100.0; };
struct Spark { Vec2 p, v; real life = 0.0, max = 0.4; int r = 255, g = 210, b = 120; };
struct Tracer { Vec2 a, b; real life = 0.0; };
struct Bomb { BodyId id = INVALID_BODY; real fuse = 1.6; };
struct HitEvent { BodyId a = INVALID_BODY, b = INVALID_BODY; Vec2 point, normal; real impulse = 0.0, speed = 0.0; };

static Material matOf(Kind k) {
    Material m;
    switch (k) {
        case Kind::Wood:    m.density = 550;  m.restitution = 0.22; m.staticFriction = 0.74; m.dynamicFriction = 0.60; break;
        case Kind::Steel:   m.density = 7850; m.restitution = 0.30; m.staticFriction = 0.58; m.dynamicFriction = 0.45; break;
        case Kind::Glass:   m.density = 2500; m.restitution = 0.14; m.staticFriction = 0.52; m.dynamicFriction = 0.42; break;
        case Kind::Ice:     m.density = 917;  m.restitution = 0.16; m.staticFriction = 0.09; m.dynamicFriction = 0.06; break;
        case Kind::Rubber:  m.density = 1100; m.restitution = 0.82; m.staticFriction = 0.98; m.dynamicFriction = 0.88; break;
        case Kind::Stone:   m.density = 2400; m.restitution = 0.18; m.staticFriction = 0.80; m.dynamicFriction = 0.68; break;
        case Kind::Bomb:    m.density = 1400; m.restitution = 0.24; m.staticFriction = 0.70; m.dynamicFriction = 0.55; break;
        case Kind::Balloon: m.density = 8;    m.restitution = 0.55; m.staticFriction = 0.40; m.dynamicFriction = 0.30; break;
        case Kind::Human:   m.density = 1050; m.restitution = 0.10; m.staticFriction = 0.95; m.dynamicFriction = 0.85; break;
        case Kind::Debris:  m.density = 2500; m.restitution = 0.20; m.staticFriction = 0.60; m.dynamicFriction = 0.48; break;
        default:            m.density = 2400; m.restitution = 0.20; m.staticFriction = 0.85; m.dynamicFriction = 0.72; break;
    }
    m.linearDrag = 0.04; m.quadraticDrag = 0.05; m.rollingFriction = 0.03; m.hysteresis = 0.18;
    return m;
}

static void colorOfKind(Kind k, real burn, int& r, int& g, int& b) {
    switch (k) {
        case Kind::Wood:    r = 168; g = 118; b = 62;  break;
        case Kind::Steel:   r = 150; g = 158; b = 172; break;
        case Kind::Glass:   r = 150; g = 210; b = 226; break;
        case Kind::Ice:     r = 178; g = 224; b = 240; break;
        case Kind::Rubber:  r = 58;  g = 58;  b = 66;  break;
        case Kind::Stone:   r = 122; g = 122; b = 118; break;
        case Kind::Bomb:    r = 178; g = 62;  b = 48;  break;
        case Kind::Balloon: r = 226; g = 88;  b = 150; break;
        case Kind::Human:   r = 214; g = 176; b = 148; break;
        case Kind::Debris:  r = 140; g = 186; b = 200; break;
        default:            r = 74;  g = 78;  b = 88;  break;
    }
    if (burn > 0.0) {
        real t = clampr2(burn, 0.0, 1.0);
        r = (int)(r * (1.0 - t) + 46 * t);
        g = (int)(g * (1.0 - t) + 34 * t);
        b = (int)(b * (1.0 - t) + 30 * t);
    }
}

struct WeaponSpec { const char* name; int pellets; real spread, speed, damage, impulse, cooldown; bool ignite, lobbed; };
static const WeaponSpec WEAPONS[] = {
    { "PISTOL",   1, 0.014, 380.0,  9.0, 0.26, 0.28, false, false },
    { "SHOTGUN",  9, 0.150, 420.0,  7.0, 0.80, 0.85, false, false },
    { "RIFLE",    1, 0.020, 780.0, 14.0, 0.09, 0.09, false, false },
    { "GRENADE",  1, 0.000,  24.0,  0.0, 0.00, 1.00, false, true  },
    { "FLAME",    5, 0.220,  13.0,  1.1, 0.02, 0.05, true,  false },
    { "GRAVGUN",  0, 0.000,   0.0,  0.0, 0.00, 0.00, false, false },
    { "SPAWNER",  0, 0.000,   0.0,  0.0, 0.00, 0.00, false, false },
};
static const int WEAPON_COUNT = (int)(sizeof(WEAPONS) / sizeof(WEAPONS[0]));

enum SpawnKind { SP_CRATE = 0, SP_STEEL, SP_GLASS, SP_ICE, SP_BALL, SP_STONE, SP_BOMB, SP_DOMINO,
                 SP_BALLOON, SP_HUMAN, SP_WATER, SP_PLANK, SP_WHEEL, SP_CHAIN, SP_COUNT };
static const char* SPAWN_NAMES[SP_COUNT] = { "CRATE", "STEEL BOX", "GLASS", "ICE BLOCK", "BALL", "BOULDER",
    "BOMB", "DOMINO", "BALLOON", "HUMAN", "WATER", "PLANK", "WHEEL", "CHAIN" };

struct Game {
    World world;
    FractureSystem frac;
    ParticleSystem water;
    BloodSystem    blood;
    CcdSystem      ccd;

    std::unordered_map<BodyId, Meta> meta;
    std::vector<HumanInfo> humans;
    std::vector<Ember>  embers;
    std::vector<Spark>  sparks;
    std::vector<Tracer> tracers;
    std::vector<Bomb>   bombs;
    std::vector<HitEvent> hits;
    std::vector<BodyId> doomed;
    struct Boom { Vec2 c; real power, radius; };
    std::vector<Boom> pendingBooms;

    int  map = 1;
    real wind = 0.0, gustAmp = 3.0, windTime = 0.0;
    real gravityMul = 1.0;
    int  weapon = 0, spawnSel = SP_CRATE;
    real cooldown = 0.0;
    int  shots = 0, kills = 0, cuts = 0, fractures = 0;
    real waterSpacing = 0.20;
    size_t waterCap = 6000;
    BodyId held = INVALID_BODY;
    real timeScale = 1.0;
    bool paused = false, showDebug = false, showBlood = true;
    real fps = 0.0, stepMs = 0.0, simTime = 0.0;

    Meta* metaOf(BodyId id) {
        std::unordered_map<BodyId, Meta>::iterator it = meta.find(id);
        return it == meta.end() ? nullptr : &it->second;
    }

    BodyId add(Kind k, const Shape& s, const Vec2& p, real angle = 0.0,
               BodyType type = BodyType::Dynamic, real hp = -1.0) {
        BodyDef d;
        d.type = type;
        d.shape = s;
        d.material = matOf(k);
        d.position = p;
        d.angle = angle;
        if (k == Kind::Balloon) d.gravityScale = -0.55;
        BodyId id = world.createBody(d);
        Meta m;
        m.kind = k;
        m.hp = m.maxHp = (hp > 0.0 ? hp : (k == Kind::Glass ? 22.0 : (k == Kind::Wood ? 55.0 : (k == Kind::Steel ? 260.0 : 80.0))));
        m.flammable = (k == Kind::Wood || k == Kind::Human || k == Kind::Balloon);
        m.breakable = (k == Kind::Glass || k == Kind::Ice || k == Kind::Stone);
        m.sharp = (k == Kind::Glass || k == Kind::Debris);
        meta[id] = m;
        if (m.breakable) frac.makeBreakable(id, k == Kind::Glass ? 14.0 : 34.0);
        return id;
    }

    void init() {
        WorldConfig& cfg = world.config();
        cfg.gravity = Vec2(0.0, -9.81);
        cfg.solver.velocityIterations = 14;
        cfg.solver.positionIterations = 10;
        cfg.substeps = 4;
        cfg.gridCellSize = 1.6;
        cfg.aabbMargin = 0.08;

        frac.attach(world);
        water.attach(world);
        blood.attach(world);
        ccd.attach(world);

        frac.pieces = 6;
        frac.impulseThreshold = 14.0;
        frac.maxFragmentsPerStep = 24;
        frac.minFragmentArea = 0.010;
        frac.maxGeneration = 2;
        frac.setCallback([this](const FractureEvent& e) { this->onFracture(e); });

        water.smoothingRadius = waterSpacing * 2.0;
        water.restDensity = 1000.0;
        water.particleMass = 1000.0 * waterSpacing * waterSpacing;
        water.stiffness = 400.0;
        water.viscosity = 22.0;
        water.surfaceTension = 0.3;
        water.boundaryDamping = 0.3;
        water.coupleWithRigid = true;
        water.domain = AABB{ Vec2(-16.4, 0.03), Vec2(16.4, 24.0) };

        blood.groundLevel = 0.0;
        blood.enablePools = true;
        blood.enableCastOff = true;
        blood.stainBodies = true;
        blood.maxDroplets = 9000;
        blood.maxDecals = 2600;
        blood.maxPools = 420;

        ccd.speedThreshold = 5.0;
        ccd.maxSubsteps = 8;

        world.setPersistContactCallback([this](const CollisionEvent& e) { this->onContact(e); });
        world.setBeginContactCallback([this](const CollisionEvent& e) { this->onContact(e); });
    }

    void forget(BodyId id) {
        blood.forgetBody(id);
        meta.erase(id);
        for (size_t k = 0; k < bombs.size(); ++k)
            if (bombs[k].id == id) { bombs[k] = bombs.back(); bombs.pop_back(); break; }
    }

    void onFracture(const FractureEvent& e) {
        ++fractures;
        Meta base;
        Meta* src = metaOf(e.original);
        if (src) base = *src;
        for (size_t i = 0; i < e.fragments.size(); ++i) {
            Meta m = base;
            m.gen = base.gen + 1;
            m.kind = (base.kind == Kind::Glass) ? Kind::Glass : Kind::Debris;
            m.hp = m.maxHp = std::max((real)4.0, base.maxHp * 0.45);
            m.sharp = true;
            m.human = -1;
            m.breakable = (m.gen < 2);
            meta[e.fragments[i]] = m;
            if (m.breakable)
                frac.makeBreakable(e.fragments[i], (base.kind == Kind::Glass ? 16.0 : 46.0) * (real)(m.gen + 1));
        }
        forget(e.original);
        for (int i = 0; i < 8; ++i) {
            Spark s;
            s.p = e.impactPoint;
            s.v = Vec2(rnd(-3.0, 3.0), rnd(0.0, 4.5));
            s.max = s.life = rnd(0.15, 0.45);
            s.r = 210; s.g = 240; s.b = 255;
            sparks.push_back(s);
        }
    }

    void onContact(const CollisionEvent& e) {
        if (!e.a || !e.b) return;
        if (e.normalImpulse < 0.4 && std::fabs(e.relativeSpeed) < 3.0) return;
        if (hits.size() >= 3000) return;
        HitEvent h;
        h.a = e.a->id; h.b = e.b->id;
        h.point = e.point; h.normal = e.normal;
        h.impulse = e.normalImpulse;
        h.speed = std::fabs(e.relativeSpeed);
        hits.push_back(h);
    }

    void wall(const Vec2& p, real w, real h) { add(Kind::Ground, Shape::box(w, h), p, 0.0, BodyType::Static, 1e9); }

    void spawnHuman(const Vec2& p) {
        RagdollConfig rc;
        rc.position = p;
        rc.scale = 1.0;
        rc.density = 1050.0;
        rc.jointFriction = 0.06;
        HumanInfo hi;
        hi.rag = createRagdoll(world, rc);
        int index = (int)humans.size();
        for (size_t i = 0; i < hi.rag.parts.size(); ++i) {
            BodyId id = hi.rag.parts[i];
            Meta m;
            m.kind = Kind::Human;
            m.hp = m.maxHp = 60.0;
            m.flammable = true;
            m.human = index;
            meta[id] = m;
            RigidBody* b = world.body(id);
            if (b) { b->material = matOf(Kind::Human); b->computeMassFromShape(); }
        }
        humans.push_back(hi);
    }

    void spawnChain(const Vec2& p, int links = 8) {
        BodyId prev = add(Kind::Steel, Shape::box(0.16, 0.16), p, 0.0, BodyType::Static, 1e9);
        Vec2 at = p;
        for (int i = 0; i < links; ++i) {
            at = at + Vec2(0.0, -0.34);
            BodyId seg = add(Kind::Steel, Shape::box(0.12, 0.30), at);
            world.createRevolute(prev, seg, Vec2(0.0, -0.16), Vec2(0.0, 0.16));
            prev = seg;
        }
        add(Kind::Steel, Shape::circle(0.34), at + Vec2(0.0, -0.5));
    }

    void spawnAt(int sel, const Vec2& p) {
        switch (sel) {
            case SP_CRATE:   add(Kind::Wood, Shape::box(0.85, 0.85), p); break;
            case SP_STEEL:   add(Kind::Steel, Shape::box(0.75, 0.75), p); break;
            case SP_GLASS:   add(Kind::Glass, Shape::box(1.30, 1.70), p); break;
            case SP_ICE:     add(Kind::Ice, Shape::box(1.10, 0.70), p); break;
            case SP_BALL:    add(Kind::Rubber, Shape::circle(0.40), p); break;
            case SP_STONE:   add(Kind::Stone, Shape::circle(0.62), p); break;
            case SP_BOMB:    add(Kind::Bomb, Shape::circle(0.34), p, 0.0, BodyType::Dynamic, 12.0); break;
            case SP_DOMINO:  add(Kind::Stone, Shape::box(0.16, 1.30), p); break;
            case SP_BALLOON: {
                BodyId anchor = add(Kind::Wood, Shape::box(0.4, 0.4), p);
                BodyId ball = add(Kind::Balloon, Shape::circle(0.46), p + Vec2(0.0, 2.2));
                world.createRope(anchor, ball, Vec2(0.0, 0.2), Vec2(0.0, -0.46), 2.2);
                break;
            }
            case SP_HUMAN:   spawnHuman(p); break;
            case SP_WATER:   if (water.count() + 400 < waterCap) water.emitBlock(p, 1.4, 1.4, waterSpacing); break;
            case SP_PLANK:   add(Kind::Wood, Shape::box(3.0, 0.18), p); break;
            case SP_WHEEL:   add(Kind::Steel, Shape::circle(0.55), p); break;
            case SP_CHAIN:   spawnChain(p); break;
            default: break;
        }
    }

    void buildMap(int m) {
        map = m;
        wall(Vec2(0.0, -0.6), 40.0, 1.2);
        wall(Vec2(-17.0, 9.0), 1.2, 22.0);
        wall(Vec2(17.0, 9.0), 1.2, 22.0);
        wall(Vec2(0.0, 21.0), 40.0, 1.0);

        BodyId pivot = add(Kind::Steel, Shape::circle(0.22), Vec2(-11.0, 0.9), 0.0, BodyType::Static, 1e9);
        BodyId plank = add(Kind::Wood, Shape::box(5.0, 0.22), Vec2(-11.0, 1.05));
        world.createRevolute(pivot, plank, Vec2(), Vec2());
        add(Kind::Steel, Shape::box(0.7, 0.7), Vec2(-9.2, 3.4));

        BodyId hub = add(Kind::Steel, Shape::circle(0.3), Vec2(9.5, 7.5), 0.0, BodyType::Kinematic, 1e9);
        RigidBody* hb = world.body(hub);
        if (hb) hb->angularVelocity = 1.6;
        BodyId vane = add(Kind::Wood, Shape::box(6.0, 0.24), Vec2(9.5, 7.5));
        world.createRevolute(hub, vane, Vec2(), Vec2());

        BodyId top = add(Kind::Steel, Shape::box(0.4, 0.4), Vec2(3.0, 13.0), 0.0, BodyType::Static, 1e9);
        BodyId ball = add(Kind::Steel, Shape::circle(0.7), Vec2(6.5, 10.0));
        world.createRope(top, ball, Vec2(), Vec2(), 5.2);

        for (int i = 0; i < 9; ++i) add(Kind::Stone, Shape::box(0.15, 1.25), Vec2(-5.5 + i * 0.62, 0.7));

        if (m == 1) {
            for (int row = 0; row < 6; ++row)
                for (int col = 0; col < 6 - row; ++col)
                    add(Kind::Wood, Shape::box(0.8, 0.8), Vec2(-1.6 + col * 0.86 + row * 0.43, 0.5 + row * 0.84));
            for (int i = 0; i < 4; ++i) add(Kind::Glass, Shape::box(1.4, 2.0), Vec2(11.0 + (i % 2) * 1.5, 1.1 + (i / 2) * 2.1));
            for (int i = 0; i < 6; ++i) add(Kind::Steel, Shape::box(0.6, 0.6), Vec2(13.5, 0.4 + i * 0.65));
            for (int i = 0; i < 5; ++i) add(Kind::Rubber, Shape::circle(0.35), Vec2(-14.0 + i * 0.9, 6.0));
            add(Kind::Bomb, Shape::circle(0.34), Vec2(-13.0, 0.5), 0.0, BodyType::Dynamic, 12.0);
            spawnChain(Vec2(-3.0, 12.0));
            for (int i = 0; i < 3; ++i) spawnHuman(Vec2(-8.0 + i * 5.0, 3.4));
        } else if (m == 2) {
            wall(Vec2(-6.0, 2.0), 0.6, 4.0);
            wall(Vec2(8.0, 2.0), 0.6, 4.0);
            water.emitBlock(Vec2(-5.4, 0.2), 13.2, 3.1, waterSpacing);
            for (int i = 0; i < 6; ++i) add(Kind::Wood, Shape::box(1.6, 0.35), Vec2(-4.0 + i * 2.0, 4.6));
            for (int i = 0; i < 4; ++i) add(Kind::Ice, Shape::box(0.9, 0.6), Vec2(-2.0 + i * 1.6, 6.4));
            for (int i = 0; i < 3; ++i) add(Kind::Steel, Shape::box(0.7, 0.7), Vec2(2.0 + i * 1.2, 9.0));
            add(Kind::Glass, Shape::box(1.4, 2.2), Vec2(-9.0, 1.2));
            add(Kind::Glass, Shape::box(1.4, 2.2), Vec2(12.0, 1.2));
            spawnAt(SP_BALLOON, Vec2(10.0, 1.0));
            for (int i = 0; i < 3; ++i) spawnHuman(Vec2(-11.0 + i * 1.6, 4.0));
            spawnHuman(Vec2(4.0, 8.0));
        } else {
            for (int lvl = 0; lvl < 9; ++lvl) {
                real y = 0.5 + lvl * 1.4;
                add(Kind::Wood, Shape::box(0.22, 1.2), Vec2(-1.2, y));
                add(Kind::Wood, Shape::box(0.22, 1.2), Vec2(1.2, y));
                add(Kind::Wood, Shape::box(3.0, 0.2), Vec2(0.0, y + 0.7));
                if (lvl % 3 == 2) add(Kind::Glass, Shape::box(2.2, 1.0), Vec2(0.0, y + 1.3));
            }
            BodyId prev = add(Kind::Wood, Shape::box(0.5, 0.2), Vec2(-14.0, 8.0), 0.0, BodyType::Static, 1e9);
            for (int i = 0; i < 12; ++i) {
                BodyId seg = add(Kind::Wood, Shape::box(0.8, 0.14), Vec2(-13.2 + i * 0.9, 8.0));
                world.createRevolute(prev, seg, Vec2(0.4, 0.0), Vec2(-0.4, 0.0));
                prev = seg;
            }
            BodyId anchor = add(Kind::Wood, Shape::box(0.5, 0.2), Vec2(-2.6, 8.0), 0.0, BodyType::Static, 1e9);
            world.createRevolute(prev, anchor, Vec2(0.4, 0.0), Vec2(-0.4, 0.0));
            for (int i = 0; i < 4; ++i) add(Kind::Bomb, Shape::circle(0.32), Vec2(5.0 + i * 0.8, 0.5), 0.0, BodyType::Dynamic, 12.0);
            for (int i = 0; i < 6; ++i) add(Kind::Stone, Shape::circle(0.4), Vec2(-15.5, 2.0 + i * 1.0));
            for (int i = 0; i < 4; ++i) spawnHuman(Vec2(-11.0 + i * 3.0, 9.6));
            spawnChain(Vec2(12.0, 14.0), 10);
        }
    }

    void killBody(BodyId id) {
        for (size_t i = 0; i < doomed.size(); ++i) if (doomed[i] == id) return;
        doomed.push_back(id);
    }

    void ignite(BodyId id, real amount) {
        Meta* m = metaOf(id);
        if (!m || !m->flammable) return;
        m->burn = std::min((real)1.0, m->burn + amount);
        m->temp += amount * 220.0;
    }

    void damage(BodyId id, real amount, const Vec2& point, const Vec2& dir, bool bullet) {
        Meta* m = metaOf(id);
        if (!m || m->kind == Kind::Ground) return;
        RigidBody* b = world.body(id);
        Vec2 nd = dir.lengthSq() > 1e-12 ? dir.normalized() : Vec2(0.0, 1.0);
        if (m->human >= 0 && m->human < (int)humans.size()) {
            HumanInfo& hi = humans[(size_t)m->human];
            hi.hp -= amount;
            real sev = clampr2(amount / 26.0, 0.12, 1.0);
            blood.addWound(id, point, nd, sev, bullet && sev > 0.45);
            blood.spray(point, nd * -1.0, 5.0 + 14.0 * sev, 0.55, (int)(4 + 12 * sev), 0.4 + 1.6 * sev, 228);
            blood.stainBody(id, 0.3 + 2.2 * sev);
            if (hi.hp <= 0.0 && !hi.dead) {
                hi.dead = true;
                ++kills;
                blood.sever(id, point, nd, 1.0);
                blood.splash(point, 9.0, 40, 2.4, 210);
                for (size_t k = 0; k < hi.rag.parts.size(); ++k) blood.stainBody(hi.rag.parts[k], 1.4);
            }
            return;
        }
        m->hp -= amount;
        if (m->breakable && amount > 11.0 && b) {
            std::vector<BodyId> pieces = frac.fracture(id, point, m->kind == Kind::Glass ? 6 : 5);
            if (!pieces.empty()) return;
        }
        if (m->kind == Kind::Bomb && m->hp <= 0.0) {
            Boom bo; bo.c = b ? b->position : point; bo.power = 26.0; bo.radius = 5.5;
            if (pendingBooms.size() < 64) pendingBooms.push_back(bo);
            killBody(id);
            return;
        }
        if (m->hp <= 0.0) killBody(id);
    }

    void explode(const Vec2& c, real power, real radius) {
        std::vector<BodyId> touched;
        std::vector<RigidBody*>& list = world.bodies();
        for (size_t i = 0; i < list.size(); ++i) {
            RigidBody* b = asBody(list[i]);
            if (!b || !b->isDynamic()) continue;
            Vec2 d = b->position - c;
            real r = d.length();
            if (r > radius || r < 1e-6) continue;
            real falloff = 1.0 - r / radius;
            b->applyImpulseAtPoint(d.normalized() * (power * falloff * b->mass * 0.55), b->position);
            b->wake();
            touched.push_back(b->id);
        }
        for (size_t i = 0; i < touched.size(); ++i) {
            RigidBody* b = world.body(touched[i]);
            if (!b) continue;
            Vec2 d = b->position - c;
            real falloff = clampr2(1.0 - d.length() / radius, 0.0, 1.0);
            ignite(touched[i], falloff * 0.8);
            damage(touched[i], power * falloff * 1.4, b->position, d, false);
        }
        for (int i = 0; i < 60; ++i) {
            Ember e;
            e.p = c + Vec2(rnd(-0.3, 0.3), rnd(-0.3, 0.3));
            real a = rnd(0.0, 6.2831853), sp = rnd(3.0, 16.0);
            e.v = Vec2(std::cos(a) * sp, std::sin(a) * sp + 3.0);
            e.max = e.life = rnd(0.5, 1.5);
            e.heat = rnd(1000.0, 1800.0);
            embers.push_back(e);
        }
        for (int i = 0; i < 40; ++i) {
            Spark s;
            s.p = c;
            real a = rnd(0.0, 6.2831853), sp = rnd(6.0, 26.0);
            s.v = Vec2(std::cos(a) * sp, std::sin(a) * sp);
            s.max = s.life = rnd(0.2, 0.6);
            sparks.push_back(s);
        }
        if (water.count() > 0 && water.count() + 60 < waterCap)
            for (int i = 0; i < 30; ++i)
                water.emit(c + Vec2(rnd(-0.4, 0.4), rnd(0.0, 0.5)), Vec2(rnd(-6.0, 6.0), rnd(2.0, 10.0)));
    }

    void fire(const Vec2& origin, const Vec2& aim) {
        const WeaponSpec& ws = WEAPONS[weapon];
        if (ws.pellets <= 0 || cooldown > 0.0) return;
        cooldown = ws.cooldown;
        ++shots;
        Vec2 dir = aim.lengthSq() > 1e-9 ? aim.normalized() : Vec2(1.0, 0.0);
        if (ws.lobbed) {
            BodyId id = add(Kind::Bomb, Shape::circle(0.26), origin + dir * 0.6, 0.0, BodyType::Dynamic, 10.0);
            RigidBody* b = world.body(id);
            if (b) b->velocity = dir * ws.speed;
            Bomb g;
            g.id = id;
            g.fuse = 1.5;
            bombs.push_back(g);
            return;
        }
        for (int p = 0; p < ws.pellets; ++p) {
            Vec2 d = dir.rotated(rnd(-ws.spread, ws.spread));
            if (ws.ignite) {
                Ember e;
                e.p = origin + d * 0.5;
                e.v = d * (ws.speed + rnd(-2.0, 2.0));
                e.max = e.life = rnd(0.6, 1.3);
                e.heat = rnd(900.0, 1500.0);
                embers.push_back(e);
                continue;
            }
            Vec2 to = origin + d * 70.0;
            RayHit hit;
            Tracer t;
            t.a = origin;
            t.b = to;
            t.life = 0.05;
            if (rayCastClosest(world, origin, to, hit) && hit.body) {
                t.b = hit.point;
                RigidBody* b = hit.body;
                BodyId victim = b->id;
                Vec2 hp = hit.point;
                b->applyImpulseAtPoint(d * (ws.impulse * 60.0), hp);
                b->wake();
                for (int i = 0; i < 6; ++i) {
                    Spark s;
                    s.p = hp;
                    s.v = hit.normal * rnd(1.0, 6.0) + Vec2(rnd(-3.0, 3.0), rnd(-1.0, 4.0));
                    s.max = s.life = rnd(0.1, 0.35);
                    sparks.push_back(s);
                }
                damage(victim, ws.damage, hp, d, true);
            }
            tracers.push_back(t);
        }
    }

    Vec2 windAt(const Vec2& p) const {
        real g = std::sin(windTime * 0.7 + p.x * 0.12) * gustAmp * 0.35 + std::sin(windTime * 1.9) * gustAmp * 0.2;
        return Vec2(wind + g, std::sin(windTime * 1.3 + p.y * 0.2) * gustAmp * 0.12);
    }

    // stability guard: no NaN, no runaway velocities, no escapers
    void sanitize() {
        std::vector<RigidBody*>& list = world.bodies();
        for (size_t i = 0; i < list.size(); ++i) {
            RigidBody* b = asBody(list[i]);
            if (!b || !b->isDynamic()) continue;
            bool bad = !std::isfinite(b->position.x) || !std::isfinite(b->position.y) ||
                       !std::isfinite(b->angle) || !std::isfinite(b->velocity.x) ||
                       !std::isfinite(b->velocity.y) || !std::isfinite(b->angularVelocity);
            if (bad) {
                b->position = Vec2(rnd(-10.0, 10.0), 16.0);
                b->prevPosition = b->position;
                b->angle = 0.0;
                b->prevAngle = 0.0;
                b->velocity = Vec2();
                b->angularVelocity = 0.0;
                b->acceleration = Vec2();
                continue;
            }
            if (b->position.x < -60.0 || b->position.x > 60.0 || b->position.y < -40.0 || b->position.y > 90.0) {
                b->position = Vec2(clampr2(b->position.x, -14.0, 14.0), 16.0);
                b->prevPosition = b->position;
                b->velocity = b->velocity * 0.1;
                b->angularVelocity = 0.0;
            }
            real sp = b->velocity.length();
            if (sp > 150.0) b->velocity = b->velocity * (150.0 / sp);
            b->angularVelocity = clampr2(b->angularVelocity, -70.0, 70.0);
            if (b->angle > 1e5 || b->angle < -1e5) b->angle = 0.0;
        }
    }

    void step(real dt) {
        if (paused || dt <= 0.0) return;
        simTime += dt;
        windTime += dt;
        cooldown = std::max((real)0.0, cooldown - dt);
        world.setGravity(Vec2(0.0, -9.81 * gravityMul));

        std::vector<RigidBody*>& list = world.bodies();
        for (size_t i = 0; i < list.size(); ++i) {
            RigidBody* b = asBody(list[i]);
            if (!b || !b->isDynamic() || b->sleeping) continue;
            Vec2 rel = windAt(b->position) - b->velocity;
            real sp = rel.length();
            if (sp < 0.05) continue;
            real area = 2.0 * b->boundingRadius;
            b->applyForce(rel.normalized() * (0.5 * 1.204 * 1.15 * area * sp * sp));
        }

        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        world.step(dt);
        ccd.update(dt);
        frac.update(dt);
        if (water.count() > 0) { real h = dt / 3.0; for (int i = 0; i < 3; ++i) water.update(h); }
        // ---- вода тушит огонь -------------------------------------------
        if (water.count() > 0) {
            for (std::unordered_map<BodyId, Meta>::iterator wit = meta.begin(); wit != meta.end(); ++wit) {
                if (wit->second.burn <= 0.0) continue;
                RigidBody* wb = world.body(wit->first);
                if (!wb) continue;
                const real wet = water.wetnessAt(wb->position, wb->boundingRadius + 0.35);
                if (wet <= 0.02) continue;
                wit->second.burn = std::max((real)0.0, wit->second.burn - dt * (3.0 + 9.0 * wet));
                wit->second.temp = std::max((real)293.0, wit->second.temp - dt * 400.0 * wet);
                if (sparks.size() < 900) {
                    Spark st;
                    st.p = wb->position + Vec2(rnd(-0.3, 0.3), rnd(-0.2, 0.4));
                    st.v = Vec2(rnd(-0.6, 0.6), rnd(0.9, 2.4));
                    st.life = 0.0;
                    st.max = 0.75;
                    st.r = 232; st.g = 236; st.b = 240;
                    sparks.push_back(st);
                }
            }
            for (size_t ei = 0; ei < embers.size();) {
                if (water.wetnessAt(embers[ei].p, 0.30) > 0.02) {
                    embers[ei] = embers.back();
                    embers.pop_back();
                } else {
                    ++ei;
                }
            }
        }
        blood.update(dt);
        sanitize();
        stepMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        std::vector<HitEvent> local;
        local.swap(hits);
        for (size_t i = 0; i < local.size(); ++i) {
            const HitEvent& hv = local[i];
            Meta* ma = metaOf(hv.a);
            Meta* mb = metaOf(hv.b);
            if (!ma || !mb) continue;
            real load = hv.impulse * 0.06 + hv.speed * 0.35;
            if (load < 1.2) continue;
            bool aSharp = ma->sharp, bSharp = mb->sharp;
            int aHuman = ma->human, bHuman = mb->human;
            Kind aKind = ma->kind, bKind = mb->kind;
            for (int side = 0; side < 2; ++side) {
                BodyId id = side ? hv.b : hv.a;
                int self = side ? bHuman : aHuman;
                bool otherSharp = side ? aSharp : bSharp;
                Kind kind = side ? bKind : aKind;
                if (kind == Kind::Ground) continue;
                if (!world.body(id)) continue;
                if (self >= 0) {
                    if (otherSharp && hv.speed > 3.2) { ++cuts; damage(id, std::min((real)16.0, load * 1.4), hv.point, hv.normal, false); }
                    else if (load > 6.0) damage(id, load * 0.55, hv.point, hv.normal, false);
                    continue;
                }
                if (load > 4.0) damage(id, load * 0.4, hv.point, hv.normal, false);
            }
        }

        for (int guard = 0; guard < 24 && !pendingBooms.empty(); ++guard) {
            std::vector<Boom> q;
            q.swap(pendingBooms);
            for (size_t i = 0; i < q.size(); ++i) explode(q[i].c, q[i].power, q[i].radius);
        }
        pendingBooms.clear();

        for (size_t i = 0; i < bombs.size();) {
            bombs[i].fuse -= dt;
            if (bombs[i].fuse <= 0.0) {
                RigidBody* b = world.body(bombs[i].id);
                Vec2 c = b ? b->position : Vec2();
                if (b) killBody(bombs[i].id);
                bombs[i] = bombs.back();
                bombs.pop_back();
                explode(c, 30.0, 6.0);
            } else ++i;
        }

        for (size_t i = 0; i < embers.size();) {
            Ember& e = embers[i];
            e.life -= dt;
            if (e.life <= 0.0) { e = embers.back(); embers.pop_back(); continue; }
            Vec2 w = windAt(e.p);
            e.v = e.v + (w - e.v) * std::min((real)1.0, dt * 2.2) + Vec2(0.0, 5.4 * dt);
            e.p = e.p + e.v * dt;
            e.heat *= std::max((real)0.0, 1.0 - dt * 0.8);
            RigidBody* hitBody = world.queryPoint(e.p);
            if (hitBody) { ignite(hitBody->id, dt * 1.4); e.v = e.v * 0.35; }
            ++i;
        }

        std::vector<BodyId> burning;
        for (std::unordered_map<BodyId, Meta>::iterator it = meta.begin(); it != meta.end(); ++it)
            if (it->second.burn > 0.0) burning.push_back(it->first);
        for (size_t i = 0; i < burning.size(); ++i) {
            Meta* m = metaOf(burning[i]);
            RigidBody* b = world.body(burning[i]);
            if (!m || !b) continue;
            m->burn = std::max((real)0.0, m->burn - dt * 0.05);
            m->temp = 293.0 + m->burn * 700.0;
            if (m->human >= 0 && m->human < (int)humans.size()) humans[(size_t)m->human].hp -= dt * 6.0 * m->burn;
            m->hp -= dt * 9.0 * m->burn;
            if (rnd01() < m->burn * dt * 26.0) {
                Ember e;
                e.p = b->position + Vec2(rnd(-0.3, 0.3), rnd(-0.3, 0.3));
                e.v = Vec2(rnd(-1.0, 1.0), rnd(1.0, 3.5));
                e.max = e.life = rnd(0.4, 1.0);
                e.heat = 900.0 + m->burn * 700.0;
                embers.push_back(e);
            }
            if (m->hp <= 0.0 && m->human < 0) {
                Kind k = m->kind;
                Vec2 pos = b->position;
                killBody(burning[i]);
                if (k == Kind::Bomb) explode(pos, 30.0, 6.0);
            }
        }

        for (size_t i = 0; i < sparks.size();) {
            Spark& s = sparks[i];
            s.life -= dt;
            if (s.life <= 0.0) { s = sparks.back(); sparks.pop_back(); continue; }
            s.v = s.v + Vec2(0.0, -16.0 * dt);
            s.p = s.p + s.v * dt;
            ++i;
        }
        for (size_t i = 0; i < tracers.size();) {
            tracers[i].life -= dt;
            if (tracers[i].life <= 0.0) { tracers[i] = tracers.back(); tracers.pop_back(); }
            else ++i;
        }

        std::vector<BodyId> gone;
        gone.swap(doomed);
        for (size_t i = 0; i < gone.size(); ++i) {
            BodyId id = gone[i];
            Meta* m = metaOf(id);
            if (m && m->human >= 0) continue;
            if (world.body(id)) world.destroyBody(id);
            forget(id);
        }
    }

    void drawBody(Canvas& cv, const Camera& cam, RigidBody* b) {
        Meta* m = metaOf(b->id);
        Kind k = m ? m->kind : Kind::Ground;
        real burn = m ? m->burn : 0.0;
        int r, g, bl;
        colorOfKind(k, burn, r, g, bl);
        real wet = blood.wetnessOf(b->id);
        if (wet > 0.01) {
            real t = clampr2(wet, 0.0, 1.0) * 0.75;
            r = (int)(r * (1.0 - t) + 120 * t);
            g = (int)(g * (1.0 - t) + 12 * t);
            bl = (int)(bl * (1.0 - t) + 16 * t);
        }
        real alpha = (k == Kind::Glass) ? 0.55 : (k == Kind::Ice ? 0.75 : 1.0);
        const std::vector<Vec2>& wv = b->worldVertices;
        if (!wv.empty()) {
            std::vector<Vec2> pts;
            pts.reserve(wv.size());
            for (size_t i = 0; i < wv.size(); ++i) pts.push_back(cam.toScreen(wv[i]));
            cv.poly(pts, r, g, bl, alpha);
            cv.outline(pts, 1.6, (int)(r * 0.45), (int)(g * 0.45), (int)(bl * 0.45), 0.9);
            if (pts.size() >= 2)
                cv.line(pts[0].x, pts[0].y, pts[1].x, pts[1].y, 1.4,
                        std::min(255, r + 60), std::min(255, g + 60), std::min(255, bl + 60), 0.5);
        } else if (b->shape.halfLength > 0.0) {
            Vec2 a, c;
            b->shape.capsuleSegment(b->transform(), a, c);
            Vec2 sa = cam.toScreen(a), sc = cam.toScreen(c);
            cv.line(sa.x, sa.y, sc.x, sc.y, b->shape.radius * 2.0 * cam.ppm, r, g, bl, alpha);
        } else {
            Vec2 s = cam.toScreen(b->position);
            real rad = b->shape.radius * cam.ppm;
            cv.disc(s.x, s.y, rad, r, g, bl, alpha);
            cv.disc(s.x - rad * 0.28, s.y - rad * 0.3, rad * 0.4,
                    std::min(255, r + 70), std::min(255, g + 70), std::min(255, bl + 70), 0.45);
            Vec2 mark = cam.toScreen(b->position + Vec2(std::cos(b->angle), std::sin(b->angle)) * b->shape.radius * 0.85);
            cv.line(s.x, s.y, mark.x, mark.y, 1.6, (int)(r * 0.4), (int)(g * 0.4), (int)(bl * 0.4), 0.8);
        }
        if (burn > 0.05) {
            Vec2 s = cam.toScreen(b->position);
            cv.softDisc(s.x, s.y, b->boundingRadius * cam.ppm * 1.5, 255, 150, 40, 0.35 * burn, true);
        }
    }

    void drawWater(Canvas& cv, const Camera& cam) {
        const std::vector<FluidParticle>& ps = water.particles();
        real rad = waterSpacing * cam.ppm * 1.5;
        for (size_t i = 0; i < ps.size(); ++i) {
            if (!ps[i].active) continue;
            Vec2 s = cam.toScreen(ps[i].position);
            if (s.x < -8 || s.y < -8 || s.x > cv.w + 8 || s.y > cv.h + 8) continue;
            real dens = clampr2(ps[i].density / std::max((real)1.0, water.restDensity), 0.0, 1.6);
            real sp = ps[i].velocity.length();
            int r = (int)(30 + 40 * (1.0 - dens));
            int g = (int)(110 + 60 * (1.0 - dens) + std::min((real)70.0, sp * 6.0));
            int b = (int)(200 + 40 * dens);
            cv.softDisc(s.x, s.y, rad, r, std::min(255, g), std::min(255, b), 0.55);
            if (sp > 4.5) cv.softDisc(s.x, s.y, rad * 0.7, 235, 245, 255, 0.35, true);
        }
    }

    void drawBlood(Canvas& cv, const Camera& cam) {
        if (!showBlood) return;
        const std::vector<BloodPool>& pools = blood.pools();
        for (size_t i = 0; i < pools.size(); ++i) {
            uint8_t r, g, b;
            BloodSystem::colorOf(pools[i].oxygen, pools[i].wetness, r, g, b);
            Vec2 s = cam.toScreen(pools[i].center);
            real hw = pools[i].halfWidth * cam.ppm;
            for (int k = 0; k < 3; ++k)
                cv.softDisc(s.x, s.y + k * 1.0, hw * (1.0 - k * 0.18), (int)r, (int)g, (int)b, 0.55);
        }
        const std::vector<BloodDecal>& decals = blood.decals();
        std::vector<Vec2> pts;
        for (size_t i = 0; i < decals.size(); ++i) {
            const BloodDecal& d = decals[i];
            uint8_t r, g, b;
            BloodSystem::colorOf(d.oxygen, d.wetness, r, g, b);
            Vec2 base = d.anchor;
            real ang = 0.0;
            if (d.body != INVALID_BODY) {
                RigidBody* bb = world.body(d.body);
                if (!bb) continue;
                ang = bb->angle;
                base = bb->position + d.anchor.rotated(ang);
            }
            if (d.outline.size() >= 3) {
                pts.clear();
                for (size_t k = 0; k < d.outline.size(); ++k) pts.push_back(cam.toScreen(base + d.outline[k].rotated(ang)));
                cv.poly(pts, (int)r, (int)g, (int)b, 0.85);
            } else {
                Vec2 s = cam.toScreen(base);
                cv.softDisc(s.x, s.y, 0.09 * cam.ppm, (int)r, (int)g, (int)b, 0.8);
            }
        }
        const std::vector<BloodDroplet>& drops = blood.droplets();
        for (size_t i = 0; i < drops.size(); ++i) {
            if (!drops[i].active) continue;
            uint8_t r, g, b;
            BloodSystem::colorOf(drops[i].oxygen, 1.0, r, g, b);
            Vec2 s = cam.toScreen(drops[i].position);
            Vec2 pv = cam.toScreen(drops[i].previous);
            real rad = std::max((real)0.9, drops[i].radius * cam.ppm);
            if (drops[i].kind == BloodKind::Mist) cv.softDisc(s.x, s.y, rad * 2.2, (int)r, (int)g, (int)b, 0.30);
            else {
                cv.line(pv.x, pv.y, s.x, s.y, rad * 1.2, (int)r, (int)g, (int)b, 0.8);
                cv.disc(s.x, s.y, rad, (int)r, (int)g, (int)b, 0.95);
            }
        }
    }

    void drawEffects(Canvas& cv, const Camera& cam) {
        for (size_t i = 0; i < embers.size(); ++i) {
            const Ember& e = embers[i];
            real t = clampr2(e.life / std::max((real)0.001, e.max), 0.0, 1.0);
            Vec2 s = cam.toScreen(e.p);
            real heat = clampr2(e.heat / 1800.0, 0.0, 1.0);
            cv.softDisc(s.x, s.y, (0.22 + 0.5 * (1.0 - t)) * cam.ppm,
                        (int)(180 + 75 * heat), (int)(70 + 110 * heat), (int)(20 + 40 * heat), 0.55 * t, true);
        }
        for (size_t i = 0; i < sparks.size(); ++i) {
            const Spark& s = sparks[i];
            real t = clampr2(s.life / std::max((real)0.001, s.max), 0.0, 1.0);
            Vec2 a = cam.toScreen(s.p), b = cam.toScreen(s.p - s.v * 0.02);
            cv.line(a.x, a.y, b.x, b.y, 1.6, s.r, s.g, s.b, t);
        }
        for (size_t i = 0; i < tracers.size(); ++i) {
            Vec2 a = cam.toScreen(tracers[i].a), b = cam.toScreen(tracers[i].b);
            cv.line(a.x, a.y, b.x, b.y, 1.4, 255, 235, 170, 0.7);
        }
    }

    void drawHud(Canvas& cv, const Camera& cam, const Input& in) {
        (void)cam;
        const int pw = 168;
        cv.rect(0, 0, pw, cv.h - 1, 16, 18, 24, 0.82);
        cv.text("SPAWNER", 12, 10, 2, 235, 235, 245);
        for (int i = 0; i < SP_COUNT; ++i) {
            int y = 34 + i * 22;
            bool sel = (i == spawnSel);
            if (sel) cv.rect(6, y - 4, pw - 6, y + 14, 60, 96, 150, 0.9);
            cv.text(SPAWN_NAMES[i], 14, y, 2, sel ? 255 : 190, sel ? 255 : 195, sel ? 255 : 205);
        }
        int wy = 34 + SP_COUNT * 22 + 14;
        cv.text("WEAPON", 12, wy, 2, 235, 225, 190);
        for (int i = 0; i < WEAPON_COUNT; ++i) {
            int y = wy + 22 + i * 20;
            bool sel = (i == weapon);
            if (sel) cv.rect(6, y - 4, pw - 6, y + 12, 150, 70, 60, 0.9);
            char buf[48];
            std::snprintf(buf, sizeof(buf), "%d %s", i + 1, WEAPONS[i].name);
            cv.text(buf, 14, y, 2, sel ? 255 : 185, sel ? 235 : 185, sel ? 220 : 195);
        }
        int sy = wy + 26 + WEAPON_COUNT * 20;
        char buf[192];
        std::snprintf(buf, sizeof(buf), "WIND %+.1f", (double)wind);
        cv.text(buf, 12, sy, 2, 170, 210, 235);
        std::snprintf(buf, sizeof(buf), "GUST %.1f", (double)gustAmp);
        cv.text(buf, 12, sy + 20, 2, 170, 210, 235);
        std::snprintf(buf, sizeof(buf), "GRAV %.2f", (double)gravityMul);
        cv.text(buf, 12, sy + 40, 2, 170, 210, 235);
        std::snprintf(buf, sizeof(buf), "TIME %.2f", (double)timeScale);
        cv.text(buf, 12, sy + 60, 2, 170, 210, 235);
        cv.text("Q E WIND   Z X GUST", 12, sy + 86, 1, 140, 150, 165);
        cv.text("G GRAV  T SLOWMO  SPACE PAUSE", 12, sy + 98, 1, 140, 150, 165);
        cv.text("LMB FIRE  RMB SPAWN  MMB DRAG", 12, sy + 110, 1, 140, 150, 165);
        cv.text("V DEBUG  B BLOOD  F5 RESET  ESC MENU", 12, sy + 122, 1, 140, 150, 165);

        cv.rect(pw + 1, 0, cv.w - 1, 40, 14, 16, 22, 0.78);
        BloodStats bs = blood.stats();
        std::snprintf(buf, sizeof(buf),
            "MAP %d  FPS %.0f  STEP %.2f MS  BODIES %zu  AWAKE %zu  CONTACTS %zu  JOINTS %zu",
            map, (double)fps, (double)stepMs, world.bodyCount(), world.awakeCount(),
            world.contactCount(), world.constraintCount());
        cv.text(buf, pw + 12, 8, 2, 225, 230, 240);
        std::snprintf(buf, sizeof(buf),
            "WATER %zu DROPS %zu DECALS %zu POOLS %zu WOUNDS %zu FIRE %zu BREAKS %d CUTS %d SHOTS %d KILLS %d",
            water.count(), bs.droplets, bs.decals, bs.pools, bs.wounds, embers.size(),
            fractures, cuts, shots, kills);
        cv.text(buf, pw + 12, 24, 2, 200, 215, 235);

        cv.line(in.mx - 9, in.my, in.mx + 9, in.my, 1.4, 250, 240, 180, 0.8);
        cv.line(in.mx, in.my - 9, in.mx, in.my + 9, 1.4, 250, 240, 180, 0.8);
        if (paused) cv.text("PAUSED", cv.w / 2 - 40, 60, 3, 255, 220, 120);
    }

    void draw(Canvas& cv, const Camera& cam, const Input& in) {
        cv.clearGradient(18, 20, 30, 44, 48, 62);
        drawBlood(cv, cam);
        std::vector<RigidBody*>& list = world.bodies();
        for (size_t i = 0; i < list.size(); ++i) {
            RigidBody* b = asBody(list[i]);
            if (b) drawBody(cv, cam, b);
        }
        drawWater(cv, cam);
        drawEffects(cv, cam);
        if (showDebug) {
            for (size_t i = 0; i < list.size(); ++i) {
                RigidBody* b = asBody(list[i]);
                if (!b) continue;
                Vec2 a = cam.toScreen(Vec2(b->aabb.min.x, b->aabb.max.y));
                Vec2 c = cam.toScreen(Vec2(b->aabb.max.x, b->aabb.min.y));
                cv.rect((int)a.x, (int)a.y, (int)c.x, (int)a.y + 1, 90, 200, 120, 0.5);
                cv.rect((int)a.x, (int)c.y, (int)c.x, (int)c.y + 1, 90, 200, 120, 0.5);
                cv.rect((int)a.x, (int)a.y, (int)a.x + 1, (int)c.y, 90, 200, 120, 0.5);
                cv.rect((int)c.x, (int)a.y, (int)c.x + 1, (int)c.y, 90, 200, 120, 0.5);
                Vec2 s = cam.toScreen(b->position);
                Vec2 v = cam.toScreen(b->position + b->velocity * 0.15);
                cv.line(s.x, s.y, v.x, v.y, 1.2, 240, 120, 90, 0.7);
            }
        }
        drawHud(cv, cam, in);
    }
};

struct Menu {
    int sel = 6;
    int map = 1;
    int humansExtra = 0;
    bool water = true;
    int quality = 1;
    real wind = 0.0;
    real gravity = 1.0;
    bool start = false;

    static const int ROWS = 7;
    const char* rowName(int i) const {
        switch (i) {
            case 0: return "MAP";
            case 1: return "EXTRA HUMANS";
            case 2: return "WATER";
            case 3: return "QUALITY";
            case 4: return "WIND";
            case 5: return "GRAVITY";
            default: return "START GAME";
        }
    }
    void rowValue(int i, char* out, size_t n) const {
        switch (i) {
            case 0: std::snprintf(out, n, "%s", map == 1 ? "1 STEEL BOX" : (map == 2 ? "2 WATER POOL" : "3 TOWER")); break;
            case 1: std::snprintf(out, n, "%d", humansExtra); break;
            case 2: std::snprintf(out, n, "%s", water ? "ON" : "OFF"); break;
            case 3: std::snprintf(out, n, "%s", quality == 0 ? "LOW" : (quality == 1 ? "NORMAL" : "HEAVY")); break;
            case 4: std::snprintf(out, n, "%+.1f M/S", (double)wind); break;
            case 5: std::snprintf(out, n, "%.2f G", (double)gravity); break;
            default: std::snprintf(out, n, "%s", "ENTER"); break;
        }
    }
    void move(int dir) { sel = (sel + dir + ROWS) % ROWS; }
    void adjust(int dir) {
        switch (sel) {
            case 0: map = 1 + ((map - 1 + dir + 3) % 3); break;
            case 1: humansExtra = (int)clampr2(humansExtra + dir, 0.0, 12.0); break;
            case 2: water = !water; break;
            case 3: quality = (int)clampr2(quality + dir, 0.0, 2.0); break;
            case 4: wind = clampr2(wind + dir * 1.0, -25.0, 25.0); break;
            case 5: gravity = clampr2(gravity + dir * 0.1, -1.0, 3.0); break;
            default: start = true; break;
        }
    }
    void draw(Canvas& cv, real t) const {
        cv.clearGradient(10, 11, 18, 30, 16, 20);
        for (int i = 0; i < 90; ++i) {
            real x = std::fmod(i * 137.0 + t * 20.0, (real)cv.w);
            real y = std::fmod(i * 71.0 + t * 8.0, (real)cv.h);
            cv.softDisc(x, y, 26.0, 120, 20, 24, 0.06, true);
        }
        const char* title = "GORELAB REMAKE";
        cv.text(title, cv.w / 2 - cv.textW(title, 7) / 2, 70, 7, 232, 60, 52);
        const char* sub = "PHYS2D SANDBOX - BODIES JOINTS CCD FRACTURE SPH WATER RAGDOLLS BLOOD FIRE WIND";
        cv.text(sub, cv.w / 2 - cv.textW(sub, 1) / 2, 132, 1, 170, 176, 190);
        char val[64];
        for (int i = 0; i < ROWS; ++i) {
            int y = 190 + i * 44;
            bool s = (i == sel);
            if (s) cv.rect(cv.w / 2 - 320, y - 10, cv.w / 2 + 320, y + 26, 60, 30, 34, 0.85);
            cv.text(rowName(i), cv.w / 2 - 300, y, 3, s ? 255 : 190, s ? 210 : 190, s ? 190 : 200);
            rowValue(i, val, sizeof(val));
            cv.text(val, cv.w / 2 + 60, y, 3, s ? 255 : 175, s ? 240 : 180, s ? 200 : 190);
        }
        const char* help = "ARROWS SELECT AND CHANGE   ENTER START   ESC QUIT";
        cv.text(help, cv.w / 2 - cv.textW(help, 2) / 2, 190 + ROWS * 44 + 30, 2, 150, 158, 172);
    }
};

static void applyMenu(Game& g, const Menu& mn) {
    g.wind = mn.wind;
    g.gravityMul = mn.gravity;
    g.waterCap = mn.quality == 0 ? 2500u : (mn.quality == 1 ? 6000u : 12000u);
    g.blood.maxDroplets = mn.quality == 0 ? 4000u : (mn.quality == 1 ? 9000u : 20000u);
    g.frac.maxFragmentsPerStep = mn.quality == 0 ? 12 : (mn.quality == 1 ? 24 : 60);
    g.world.config().solver.velocityIterations = mn.quality == 2 ? 18 : 14;
    g.buildMap(mn.map);
    if (!mn.water) g.water.clear();
    for (int i = 0; i < mn.humansExtra; ++i) g.spawnHuman(Vec2(rnd(-12.0, 12.0), rnd(6.0, 14.0)));
}

static void handleGameKeys(Game& g, Input& in, Camera& cam, real dt, bool& backToMenu, bool& reset) {
    for (int i = 0; i < WEAPON_COUNT && i < 9; ++i) if (in.pressed['1' + i]) g.weapon = i;
    if (in.pressed['Q']) g.wind = clampr2(g.wind - 1.0, -25.0, 25.0);
    if (in.pressed['E']) g.wind = clampr2(g.wind + 1.0, -25.0, 25.0);
    if (in.pressed['Z']) g.gustAmp = clampr2(g.gustAmp - 0.5, 0.0, 20.0);
    if (in.pressed['X']) g.gustAmp = clampr2(g.gustAmp + 0.5, 0.0, 20.0);
    if (in.pressed['G']) g.gravityMul = (g.gravityMul > 0.5 ? 0.0 : 1.0);
    if (in.pressed['T']) g.timeScale = (g.timeScale > 0.9 ? 0.25 : (g.timeScale > 0.2 ? 2.0 : 1.0));
    if (in.pressed['B']) g.showBlood = !g.showBlood;
    if (in.pressed['V']) g.showDebug = !g.showDebug;
    if (in.pressed[' ']) g.paused = !g.paused;
    if (in.pressed[0x74]) reset = true;
    if (in.pressed[0x1B]) backToMenu = true;
    real pan = 14.0 * dt;
    if (in.key['A']) cam.center.x -= pan;
    if (in.key['D']) cam.center.x += pan;
    if (in.key['W']) cam.center.y += pan;
    if (in.key['S']) cam.center.y -= pan;
    if (in.wheel != 0) cam.ppm = clampr2(cam.ppm * (in.wheel > 0 ? 1.12 : 0.89), 8.0, 160.0);
}

static void headlessRun(int mapId, int steps, bool useWater) {
    Game g;
    g.init();
    Menu mn;
    mn.map = mapId;
    mn.water = useWater;
    mn.humansExtra = 2;
    mn.wind = 6.0;
    applyMenu(g, mn);

    Camera cam;
    Canvas cv;
    cv.resize(cam.w, cam.h);
    Input in;
    const real dt = 1.0 / 60.0;
    std::printf("\n=== MAP %d ===\nbodies %zu  water %zu  humans %zu\n",
                mapId, g.world.bodyCount(), g.water.count(), g.humans.size());
    for (int i = 0; i < steps; ++i) {
        if (i % 24 == 0) {
            g.weapon = (i / 24) % (WEAPON_COUNT - 2);
            g.cooldown = 0.0;
            Vec2 origin(-15.0, 6.0 + std::sin(i * 0.1) * 3.0);
            Vec2 target(rnd(-6.0, 14.0), rnd(0.6, 10.0));
            g.fire(origin, target - origin);
        }
        if (i % 40 == 20) g.spawnAt((i / 40) % SP_COUNT, Vec2(rnd(-12.0, 12.0), 15.0));
        if (i == 150) g.explode(Vec2(0.0, 2.0), 34.0, 7.0);
        if (i == 300) g.explode(Vec2(-8.0, 1.5), 26.0, 6.0);
        g.step(dt);
        if ((i + 1) % 100 == 0) {
            BloodStats bs = g.blood.stats();
            std::printf("step %4d  bodies %4zu  awake %4zu  contacts %4zu  joints %3zu  water %5zu  drops %5zu"
                        "  decals %5zu  pools %4zu  fire %4zu  breaks %3d  cuts %3d  kills %2d  step %5.2f ms\n",
                        i + 1, g.world.bodyCount(), g.world.awakeCount(), g.world.contactCount(),
                        g.world.constraintCount(), g.water.count(), bs.droplets, bs.decals, bs.pools,
                        g.embers.size(), g.fractures, g.cuts, g.kills, (double)g.stepMs);
        }
    }
    int bad = 0;
    std::vector<RigidBody*>& list = g.world.bodies();
    for (size_t i = 0; i < list.size(); ++i) {
        RigidBody* b = asBody(list[i]);
        if (!b) continue;
        if (!std::isfinite(b->position.x) || !std::isfinite(b->position.y) || !std::isfinite(b->angle)) ++bad;
    }
    g.draw(cv, cam, in);
    char path[64];
    std::snprintf(path, sizeof(path), "gorelab_map%d.ppm", mapId);
    cv.savePpm(path);
    std::printf("map %d done: non finite bodies %d, screenshot %s (%dx%d)\n", mapId, bad, path, cv.w, cv.h);
}

#if defined(GORELAB_WIN)
static Input g_in;
static bool g_running = true;

static LRESULT CALLBACK wndProc(HWND hw, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_CLOSE: case WM_DESTROY: g_running = false; return 0;
        case WM_KEYDOWN: if (wp < 256 && !g_in.key[wp]) { g_in.key[wp] = true; g_in.pressed[wp] = true; } return 0;
        case WM_KEYUP:   if (wp < 256) g_in.key[wp] = false; return 0;
        case WM_MOUSEMOVE: g_in.mx = (int)(short)LOWORD(lp); g_in.my = (int)(short)HIWORD(lp); return 0;
        case WM_LBUTTONDOWN: g_in.lmb = true; g_in.lclick = true; return 0;
        case WM_LBUTTONUP:   g_in.lmb = false; return 0;
        case WM_RBUTTONDOWN: g_in.rmb = true; g_in.rclick = true; return 0;
        case WM_RBUTTONUP:   g_in.rmb = false; return 0;
        case WM_MBUTTONDOWN: g_in.key[0x04] = true; return 0;
        case WM_MBUTTONUP:   g_in.key[0x04] = false; return 0;
        case WM_MOUSEWHEEL:  g_in.wheel += (int)((short)HIWORD(wp)) / 120; return 0;
        default: break;
    }
    return DefWindowProc(hw, msg, wp, lp);
}

static int runWindowed() {
    const int W = 1440, H = 860;
    WNDCLASSA wc;
    std::memset(&wc, 0, sizeof(wc));
    wc.lpfnWndProc = wndProc;
    wc.hInstance = GetModuleHandleA(nullptr);
    wc.lpszClassName = "gorelabRemakeWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_CROSS);
    RegisterClassA(&wc);
    RECT r = { 0, 0, W, H };
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    HWND hw = CreateWindowA(wc.lpszClassName, "GoreLab Remake - phys2d sandbox",
                            WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
                            r.right - r.left, r.bottom - r.top, nullptr, nullptr, wc.hInstance, nullptr);
    if (!hw) return 1;
    HDC hdc = GetDC(hw);

    Canvas cv;
    cv.resize(W, H);
    Camera cam;
    cam.w = W; cam.h = H;
    Menu mn;
    std::unique_ptr<Game> game;
    bool inMenu = true;
    real menuTime = 0.0;

    BITMAPINFO bi;
    std::memset(&bi, 0, sizeof(bi));
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = W;
    bi.bmiHeader.biHeight = -H;
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;

    std::chrono::steady_clock::time_point prev = std::chrono::steady_clock::now();
    while (g_running) {
        g_in.newFrame();
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) { TranslateMessage(&msg); DispatchMessage(&msg); }
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        real dt = clampr2(std::chrono::duration<double>(now - prev).count(), 1.0 / 600.0, 1.0 / 20.0);
        prev = now;

        if (inMenu) {
            menuTime += dt;
            if (g_in.pressed[VK_UP]) mn.move(-1);
            if (g_in.pressed[VK_DOWN]) mn.move(1);
            if (g_in.pressed[VK_LEFT]) mn.adjust(-1);
            if (g_in.pressed[VK_RIGHT]) mn.adjust(1);
            if (g_in.pressed[VK_RETURN]) mn.start = true;
            if (g_in.pressed[VK_ESCAPE]) g_running = false;
            if (mn.start) {
                mn.start = false;
                game.reset(new Game());
                game->init();
                applyMenu(*game, mn);
                cam.center = Vec2(0.0, 8.0);
                inMenu = false;
            }
            mn.draw(cv, menuTime);
        } else {
            Game& g = *game;
            bool back = false, reset = false;
            handleGameKeys(g, g_in, cam, dt, back, reset);
            Vec2 mouseWorld = cam.toWorld((real)g_in.mx, (real)g_in.my);
            bool overPanel = (g_in.mx < 170);
            if (g_in.lclick && overPanel) {
                int idx = (g_in.my - 30) / 22;
                if (idx >= 0 && idx < SP_COUNT) g.spawnSel = idx;
                int wy = 34 + SP_COUNT * 22 + 14;
                int widx = (g_in.my - (wy + 18)) / 20;
                if (widx >= 0 && widx < WEAPON_COUNT) g.weapon = widx;
            } else if (g_in.lmb && !overPanel) {
                Vec2 muzzle = cam.center + Vec2(0.0, -1.0);
                g.fire(muzzle, mouseWorld - muzzle);
            }
            if (g_in.rclick && !overPanel) g.spawnAt(g.spawnSel, mouseWorld);
            if (g_in.key[0x04] && !overPanel) {
                if (g.held == INVALID_BODY) {
                    RigidBody* pick = g.world.queryPoint(mouseWorld);
                    if (pick && pick->isDynamic()) g.held = pick->id;
                }
                RigidBody* hb = g.world.body(g.held);
                if (hb) { hb->velocity = (mouseWorld - hb->position) * 12.0; hb->wake(); }
            } else g.held = INVALID_BODY;
            if (reset) { game.reset(new Game()); game->init(); applyMenu(*game, mn); }
            else {
                g.step(dt * g.timeScale);
                g.fps = 1.0 / std::max((real)1e-4, dt);
                g.draw(cv, cam, g_in);
            }
            if (back) inMenu = true;
        }
        StretchDIBits(hdc, 0, 0, W, H, 0, 0, W, H, cv.px.data(), &bi, DIB_RGB_COLORS, SRCCOPY);
    }
    ReleaseDC(hw, hdc);
    return 0;
}
#endif

int main(int argc, char** argv) {
    bool headless = false;
    int steps = 600;
    int only = 0;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--headless")) headless = true;
        else if (!std::strcmp(argv[i], "--steps") && i + 1 < argc) steps = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--map") && i + 1 < argc) only = std::atoi(argv[++i]);
    }
#if defined(GORELAB_WIN)
    if (!headless) return runWindowed();
#endif
    (void)headless;
    std::printf("GoreLab Remake 2 - headless engine stress test (%d steps per map)\n", steps);
    if (only >= 1 && only <= 3) headlessRun(only, steps, true);
    else { headlessRun(1, steps, false); headlessRun(2, steps, true); headlessRun(3, steps, false); }
    return 0;
}
