// ===========================================================================
//  CAR CRASH 2D - destructible cars on the phys2d engine
//  Rendering: Direct3D 9 only (no GDI). Console test build: CARCRASH_HEADLESS.
// ===========================================================================
#include "phys2d/World.h"
#include "phys2d/Body.h"
#include "phys2d/Shape.h"
#include "phys2d/Constraints.h"
#include "phys2d/Collision.h"
#include "phys2d/Extras.h"
#include "phys2d/Blood.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <chrono>

using namespace phys2d;

// ------------------------------------------------------------------ helpers
static uint32_t g_rng = 0xC0FFEEu;
static inline float rnd01(){
    g_rng = g_rng * 1664525u + 1013904223u;
    return (float)((g_rng >> 8) & 0xFFFFFFu) / (float)0xFFFFFF;
}
static inline float rnd(float a,float b){ return a + (b - a) * rnd01(); }
static inline double RC(double v,double lo,double hi){ return v < lo ? lo : (v > hi ? hi : v); }

// ------------------------------------------------------------------- canvas
// Software canvas. On Windows its pixels are uploaded to a D3D9 dynamic
// texture once per frame; nothing here touches GDI.
struct Canvas {
    int w = 0, h = 0;
    std::vector<uint32_t> px;

    void resize(int W,int H){ w = W; h = H; px.assign((size_t)W * H, 0u); }
    void clear(uint32_t c){ std::fill(px.begin(), px.end(), c); }

    inline void setpx(int x,int y,uint32_t c){
        if(x < 0 || y < 0 || x >= w || y >= h) return;
        px[(size_t)y * w + x] = c;
    }
    inline void blend(int x,int y,uint8_t r,uint8_t g,uint8_t b,uint8_t a){
        if(x < 0 || y < 0 || x >= w || y >= h || a == 0) return;
        uint32_t& d = px[(size_t)y * w + x];
        if(a == 255){ d = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; return; }
        uint32_t dr = (d >> 16) & 255, dg = (d >> 8) & 255, db = d & 255;
        uint32_t ia = 255u - a;
        dr = (r * a + dr * ia) / 255u;
        dg = (g * a + dg * ia) / 255u;
        db = (b * a + db * ia) / 255u;
        d = (dr << 16) | (dg << 8) | db;
    }
    void rect(int x,int y,int W,int H,uint32_t c,uint8_t a = 255){
        if(W <= 0 || H <= 0) return;
        uint8_t r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
        int x1 = std::min(w, x + W), y1 = std::min(h, y + H);
        for(int yy = std::max(0, y); yy < y1; yy++)
            for(int xx = std::max(0, x); xx < x1; xx++) blend(xx, yy, r, g, b, a);
    }
    void disc(int cx,int cy,int rad,uint8_t r,uint8_t g,uint8_t b,uint8_t a = 255){
        if(rad <= 0 || rad > 4000) return;
        if(cx + rad < 0 || cx - rad > w || cy + rad < 0 || cy - rad > h) return;
        int r2 = rad * rad;
        for(int y = -rad; y <= rad; y++){
            int yy = cy + y;
            if(yy < 0 || yy >= h) continue;
            int span = (int)std::sqrt((float)std::max(0, r2 - y * y));
            for(int x = -span; x <= span; x++) blend(cx + x, yy, r, g, b, a);
        }
    }
    void line(int x0,int y0,int x1,int y1,uint32_t c,int thick = 1){
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        if(dx > 20000 || dy > 20000) return;
        uint8_t r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        for(;;){
            if(thick <= 1) blend(x0, y0, r, g, b, 255);
            else for(int oy = 0; oy < thick; oy++)
                     for(int ox = 0; ox < thick; ox++) blend(x0 + ox, y0 + oy, r, g, b, 255);
            if(x0 == x1 && y0 == y1) break;
            int e2 = err * 2;
            if(e2 > -dy){ err -= dy; x0 += sx; }
            if(e2 <  dx){ err += dx; y0 += sy; }
        }
    }
    void fillPoly(const std::vector<std::pair<int,int> >& p,uint32_t c,uint8_t a = 255){
        if(p.size() < 3) return;
        int ymin = p[0].second, ymax = p[0].second;
        for(size_t i = 1; i < p.size(); i++){
            ymin = std::min(ymin, p[i].second);
            ymax = std::max(ymax, p[i].second);
        }
        if(ymax < 0 || ymin > h) return;
        ymin = std::max(0, ymin); ymax = std::min(h - 1, ymax);
        if(ymax - ymin > 8000) return;
        uint8_t r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
        std::vector<int> xs;
        for(int y = ymin; y <= ymax; y++){
            xs.clear();
            for(size_t i = 0; i < p.size(); i++){
                size_t j = (i + 1) % p.size();
                int y0 = p[i].second, y1 = p[j].second;
                if(y0 == y1) continue;
                if((y >= y0 && y < y1) || (y >= y1 && y < y0)){
                    double t = (double)(y - y0) / (double)(y1 - y0);
                    xs.push_back((int)(p[i].first + t * (p[j].first - p[i].first)));
                }
            }
            if(xs.size() < 2) continue;
            std::sort(xs.begin(), xs.end());
            for(size_t k = 0; k + 1 < xs.size(); k += 2){
                int xa = std::max(0, xs[k]), xb = std::min(w - 1, xs[k + 1]);
                for(int x = xa; x <= xb; x++) blend(x, y, r, g, b, a);
            }
        }
    }

    static uint8_t* fontTable(){
        static uint8_t F[96][6];
        static bool ready = false;
        if(ready) return &F[0][0];
        std::memset(F, 0, sizeof F);
        struct G { char c; uint8_t r[6]; };
        static const G defs[] = {
            {'.',{0,0,0,0,0,4}},        {',',{0,0,0,0,4,8}},
            {':',{0,4,0,0,4,0}},        {'-',{0,0,0,14,0,0}},
            {'/',{1,1,2,4,8,8}},        {'%',{9,2,4,9,0,0}},
            {'+',{0,4,14,4,0,0}},       {'!',{4,4,4,4,0,4}},
            {'0',{6,9,9,9,9,6}},        {'1',{2,6,2,2,2,7}},
            {'2',{14,1,2,4,8,15}},      {'3',{14,1,6,1,1,14}},
            {'4',{2,6,10,15,2,2}},      {'5',{15,8,14,1,1,14}},
            {'6',{6,8,14,9,9,6}},       {'7',{15,1,2,4,4,4}},
            {'8',{6,9,6,9,9,6}},        {'9',{6,9,9,7,1,6}},
            {'A',{6,9,9,15,9,9}},       {'B',{14,9,14,9,9,14}},
            {'C',{6,9,8,8,9,6}},        {'D',{14,9,9,9,9,14}},
            {'E',{15,8,14,8,8,15}},     {'F',{15,8,14,8,8,8}},
            {'G',{6,9,8,11,9,6}},       {'H',{9,9,15,9,9,9}},
            {'I',{14,4,4,4,4,14}},      {'J',{7,2,2,2,10,4}},
            {'K',{9,10,12,12,10,9}},    {'L',{8,8,8,8,8,15}},
            {'M',{9,15,15,9,9,9}},      {'N',{9,13,15,11,9,9}},
            {'O',{6,9,9,9,9,6}},        {'P',{14,9,9,14,8,8}},
            {'Q',{6,9,9,11,6,1}},       {'R',{14,9,9,14,10,9}},
            {'S',{7,8,6,1,1,14}},       {'T',{14,4,4,4,4,4}},
            {'U',{9,9,9,9,9,6}},        {'V',{9,9,9,9,6,6}},
            {'W',{9,9,9,15,15,9}},      {'X',{9,9,6,6,9,9}},
            {'Y',{9,9,6,4,4,4}},        {'Z',{15,1,2,4,8,15}},
        };
        for(size_t i = 0; i < sizeof(defs) / sizeof(defs[0]); i++){
            int idx = defs[i].c - 32;
            if(idx < 0 || idx >= 96) continue;
            for(int k = 0; k < 6; k++) F[idx][k] = defs[i].r[k];
        }
        ready = true;
        return &F[0][0];
    }
    void glyph(int x,int y,char ch,uint32_t c,int s){
        if(ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
        int idx = (int)(unsigned char)ch - 32;
        if(idx < 0 || idx >= 96) return;
        const uint8_t* rows = fontTable() + idx * 6;
        uint8_t r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
        for(int ry = 0; ry < 6; ry++){
            uint8_t bits = rows[ry];
            for(int rx = 0; rx < 4; rx++){
                if(!(bits & (8 >> rx))) continue;
                for(int oy = 0; oy < s; oy++)
                    for(int ox = 0; ox < s; ox++)
                        blend(x + rx * s + ox, y + ry * s + oy, r, g, b, 255);
            }
        }
    }
    void text(int x,int y,const char* str,uint32_t c,int s = 1){
        int cx = x;
        for(const char* p = str; *p; p++){
            if(*p == '\n'){ cx = x; y += 7 * s; continue; }
            glyph(cx, y, *p, c, s);
            cx += 5 * s;
        }
    }
    int textW(const char* str,int s = 1) const {
        int n = 0; for(const char* p = str; *p; p++) n++;
        return n * 5 * s;
    }
};

// ------------------------------------------------------------------- camera
struct Camera {
    float cx = 0.f, cy = 5.f, ppm = 34.f;
    int   W = 1360, H = 768;

    void toScreen(float wx,float wy,int& sx,int& sy) const {
        double x = (double)(wx - cx) * ppm + W * 0.5;
        double y = H * 0.5 - (double)(wy - cy) * ppm;
        sx = (int)RC(x, -1e6, 1e6);
        sy = (int)RC(y, -1e6, 1e6);
    }
    void toWorld(int sx,int sy,float& wx,float& wy) const {
        wx = cx + (sx - W * 0.5f) / ppm;
        wy = cy - (sy - H * 0.5f) / ppm;
    }
    void zoom(float k){ ppm = (float)RC(ppm * k, 8.0, 220.0); }
    int  m2p(float m) const { return (int)RC((double)m * ppm, 0.0, 4000.0); }
};

// -------------------------------------------------------------------- input
struct Input {
    bool  key[256], prev[256];
    int   mx = 0, my = 0;
    float wheel = 0.f;
    bool  lmb = false, rmb = false;

    Input(){ std::memset(key, 0, sizeof key); std::memset(prev, 0, sizeof prev); }
    void newFrame(){ std::memcpy(prev, key, sizeof key); wheel = 0.f; }
    bool pressed(int k) const { return k >= 0 && k < 256 && key[k] && !prev[k]; }
};

// ------------------------------------------------------------------ effects
struct Spark { Vec2 p, v; float life = 0.f, max = 1.f; uint8_t r = 255, g = 200, b = 80; };
struct Smoke { Vec2 p, v; float life = 0.f, max = 1.f, radius = 0.2f; uint8_t r = 90, g = 90, b = 90; float alpha = 0.8f; };
struct Skid  { Vec2 p; float life = 0.f, max = 6.f, strength = 1.f; };
struct Boom  { Vec2 c; float power = 200.f, radius = 7.f; };

// --------------------------------------------------------------- car models
enum class CarKind { Sedan = 0, Truck, Sport, Monster, Bus };

struct CarSpec {
    const char* name;
    float cw, ch;          // chassis half width / half height, metres
    float wr;              // wheel radius
    float mass;            // kg
    float torque;          // N*m at the driven wheels
    float topWheelSpeed;   // rad/s cap
    float suspStiff;       // N/m
    float suspDamp;        // N*s/m
    float toughness;       // structural strength multiplier
    uint32_t bodyCol, roofCol, rimCol;
    int driveWheels;       // 1 = rear, 2 = all
};

static const CarSpec SPECS[5] = {
    { "SEDAN",   1.70f, 0.42f, 0.34f, 1100.f,  900.f, 26.f, 26000.f,  900.f,  45.f, 0xC02840, 0xE04868, 0xC8C8D0, 1 },
    { "TRUCK",   2.40f, 0.62f, 0.46f, 3200.f, 2100.f, 19.f, 62000.f, 2200.f,  95.f, 0x1F7A46, 0x2C9457, 0x9A9AA2, 2 },
    { "SPORT",   1.55f, 0.34f, 0.30f,  850.f,  780.f, 33.f, 21000.f,  780.f,  32.f, 0xD81C1C, 0xF03A3A, 0xE8C24A, 1 },
    { "MONSTER", 1.90f, 0.55f, 0.62f, 2400.f, 2600.f, 17.f, 48000.f, 1800.f,  78.f, 0x5B2080, 0x7A2FA6, 0x2A2A2E, 2 },
    { "BUS",     3.10f, 0.80f, 0.42f, 5200.f, 2400.f, 15.f, 88000.f, 3000.f, 120.f, 0xE0A020, 0xF0BE40, 0x8C8C94, 1 },
};

// ------------------------------------------------------- deformable panels
// The chassis stays a single rigid box for the solver, but its skin is a set
// of panels whose vertices are pushed inwards at the impact point. That is
// what makes the car visibly crumple exactly where it was hit.
struct DamageMesh {
    struct Panel {
        Vec2     orig[4];
        Vec2     cur[4];
        float    dent = 0.f;
        uint32_t col = 0xC02840;
    };
    std::vector<Panel>  panels;
    std::vector<Vec2>   cracks;
    float               total = 0.f;

    void init(float cw,float ch,uint32_t body,uint32_t roof){
        panels.clear(); cracks.clear(); total = 0.f;
        const float seg = cw * 2.f / 3.f;
        for(int i = 0; i < 3; i++){
            Panel p;
            float x0 = -cw + seg * i, x1 = x0 + seg;
            float top = (i == 1) ? ch * 1.75f : ch;   // middle panel is the cabin
            p.orig[0] = Vec2((real)x0, (real)-ch);
            p.orig[1] = Vec2((real)x1, (real)-ch);
            p.orig[2] = Vec2((real)x1, (real)top);
            p.orig[3] = Vec2((real)x0, (real)top);
            for(int k = 0; k < 4; k++) p.cur[k] = p.orig[k];
            p.col = (i == 1) ? roof : body;
            panels.push_back(p);
        }
    }

    void dent(const Vec2& local,float energy){
        if(panels.empty()) return;
        float e = (float)RC(energy, 0.0, 3.0);
        if(e <= 0.f) return;
        total += e;
        for(size_t pi = 0; pi < panels.size(); pi++){
            Panel& p = panels[pi];
            for(int k = 0; k < 4; k++){
                Vec2 d = p.cur[k] - local;
                float dist2 = (float)(d.x * d.x + d.y * d.y);
                float weight = 1.f / (1.f + dist2 * 3.f);
                if(weight < 0.04f) continue;
                float push = (float)RC(e * weight * 0.22, 0.0, 0.28);
                float len = std::sqrt(std::max(1e-6f, dist2));
                Vec2 dir((real)((float)d.x / len), (real)((float)d.y / len));
                Vec2 next = p.cur[k] + Vec2(dir.x * (real)push, dir.y * (real)push);
                // never let a vertex cross the panel centre (sign flip guard)
                if(next.x * p.orig[k].x < 0.0 && std::fabs(p.orig[k].x) > 0.05) continue;
                p.cur[k] = next;
                p.dent += push;
            }
        }
        if(cracks.size() < 64 && e > 0.15f){
            float a = rnd(0.f, 6.283f), l = rnd(0.10f, 0.45f);
            cracks.push_back(local);
            cracks.push_back(local + Vec2((real)(std::cos(a) * l), (real)(std::sin(a) * l)));
        }
    }
};

// ------------------------------------------------------------ car instance
struct Wheel {
    BodyId              id    = INVALID_BODY;
    SpringConstraint*   susp  = nullptr;   // suspension
    DistanceConstraint* arm   = nullptr;   // breakable control arm
    AngularConstraint*  motor = nullptr;   // drive / brake
    bool                driven = false, lost = false;
    float               slip = 0.f;
};

struct Car {
    CarKind             kind = CarKind::Sedan;
    const CarSpec*      spec = &SPECS[0];
    BodyId              chassis = INVALID_BODY;
    BodyId              driver  = INVALID_BODY;
    Wheel               wheels[2];
    RevoluteConstraint* seatbelt = nullptr;
    DamageMesh          shell;

    float health = 1.f, fuel = 1.f, nitro = 1.f, heat = 0.f, fire = 0.f;
    bool  alive = true, dead = false, driverOut = false, bleeding = false;
    int   player = -1;                     // 0 = human, -1 = AI
    float aiTimer = 0.f, aiTarget = 0.f;
    int   aiMode = 0;
    float engineRpm = 0.f, throttle = 0.f;
};

// -------------------------------------------------------------------- game
struct Game {
    World          world;
    FractureSystem frac;
    CcdSystem      ccd;
    BloodSystem    blood;

    std::vector<Car>   cars;
    std::vector<Spark> sparks;
    std::vector<Smoke> smokes;
    std::vector<Skid>  skids;
    std::vector<Boom>  booms;
    std::vector<BodyId> doomed;

    std::unordered_map<BodyId,int> owner;     // body -> car index
    std::unordered_map<BodyId,int> propKind;  // body -> visual kind

    struct Hit { BodyId a, b; Vec2 point, normal; float impulse, speed; };
    std::vector<Hit> hits;

    int   map = 0;
    float time = 0.f, windX = 0.f, gustPhase = 0.f, timeScale = 1.f;
    bool  paused = false, debugDraw = false, slowmo = false;
    int   kills = 0, wrecks = 0, fragments = 0, lastContacts = 0, culled = 0;
    float camX = 0.f, camY = 5.f, shake = 0.f, stepMs = 0.f;
    float bboxX = 120.f;

    void   init(int mapId);
    void   buildMap(int mapId);
    float  groundAt(float x) const;
    BodyId prop(Vec2 pos,float w,float h,float density,int kind,float breakAt);
    void   spawnCar(CarKind kind,float x,int playerIdx);
    void   drive(Car& car,bool gas,bool brake,bool leanL,bool leanR,bool boost,bool handbrake,float dt);
    void   ai(Car& car,float dt);
    void   crash(int idx,const Vec2& worldPoint,float impulse,float speed);
    void   wreck(int idx);
    void   ejectDriver(Car& car);
    void   spark(Vec2 p,Vec2 dir,int n,float spd,uint8_t r,uint8_t g,uint8_t b);
    void   puff(Vec2 p,Vec2 v,float radius,uint8_t gray,float alpha);
    void   enforceLimits();
    void   step(float dt);
    Car*   playerCar();
    void   drawWorld(Canvas& C,const Camera& cam);
    void   drawCar(Canvas& C,const Camera& cam,const Car& car);
    void   drawBlood(Canvas& C,const Camera& cam);
    void   drawFx(Canvas& C,const Camera& cam);
    void   drawHud(Canvas& C,float fps);
};

Car* Game::playerCar(){
    for(size_t i = 0; i < cars.size(); i++) if(cars[i].player == 0) return &cars[i];
    return nullptr;
}

float Game::groundAt(float x) const {
    if(map != 1) return 0.f;
    return 2.6f * std::sin(x * 0.11f) + 1.3f * std::sin(x * 0.31f + 1.f) + 0.5f * std::sin(x * 0.77f);
}

// -------------------------------------------------------------------- setup
void Game::init(int mapId){
    WorldConfig cfg;
    cfg.solver.velocityIterations   = 18;
    cfg.solver.positionIterations   = 12;
    cfg.solver.baumgarte            = 0.22;
    cfg.solver.restitutionThreshold = 1.20;
    cfg.broadPhase   = BroadPhaseMode::Hybrid;
    cfg.gridCellSize = 2.2;
    cfg.aabbMargin   = 0.08;
    cfg.sleepTime    = 0.8;
    cfg.threadCount  = 0;
    world = World(cfg);

    cars.clear(); sparks.clear(); smokes.clear(); skids.clear();
    booms.clear(); doomed.clear(); hits.clear();
    owner.clear(); propKind.clear();
    blood.clear();
    map = mapId; time = 0.f; windX = 0.f; gustPhase = 0.f;
    kills = 0; wrecks = 0; fragments = 0; lastContacts = 0; culled = 0;
    camX = 0.f; camY = 5.f; shake = 0.f; stepMs = 0.f;
    paused = false; slowmo = false;

    blood.maxDroplets  = 1200;
    blood.maxDecals    = 600;
    blood.maxPools     = 240;
    blood.groundLevel  = 0.0;
    blood.stainBodies  = true;

    frac.attach(world);
    frac.impulseThreshold   = 42.0;
    frac.pieces             = 5;
    frac.maxFragmentsPerStep = 14;
    frac.minFragmentArea    = 0.020;
    frac.maxGeneration      = 2;
    frac.setCallback([this](const FractureEvent& ev){
        fragments += (int)ev.fragments.size();
        for(size_t i = 0; i < ev.fragments.size(); i++) propKind[ev.fragments[i]] = 3;
        spark(ev.impactPoint, Vec2(0, 1), 8, 6.f, 230, 230, 245);
        puff(ev.impactPoint, Vec2(0, 1.2), 0.20f, 190, 0.5f);
    });

    ccd.attach(world);
    ccd.speedThreshold = 7.0;
    ccd.maxSubsteps    = 8;

    // A car must never collide with its own wheels or driver.
    world.setContactFilter([this](const RigidBody& a,const RigidBody& b) -> bool {
        std::unordered_map<BodyId,int>::const_iterator ia = owner.find(a.id);
        if(ia == owner.end()) return true;
        std::unordered_map<BodyId,int>::const_iterator ib = owner.find(b.id);
        if(ib == owner.end()) return true;
        return ia->second != ib->second;
    });

    world.setBeginContactCallback([this](const CollisionEvent& e){
        if(!e.a || !e.b) return;
        if(e.normalImpulse < 4.0 && e.relativeSpeed < 3.0) return;
        if(hits.size() >= 400) return;
        Hit h;
        h.a = e.a->id; h.b = e.b->id;
        h.point = e.point; h.normal = e.normal;
        h.impulse = (float)e.normalImpulse;
        h.speed   = (float)e.relativeSpeed;
        hits.push_back(h);
    });

    buildMap(mapId);

    spawnCar(CarKind::Sport, 6.f, 0);
    const CarKind ai[5] = { CarKind::Sedan, CarKind::Truck, CarKind::Monster, CarKind::Sedan, CarKind::Bus };
    for(int i = 0; i < 5; i++) spawnCar(ai[i], 6.f + (i + 1) * 9.f, -1);
    camX = 6.f;
}

BodyId Game::prop(Vec2 pos,float w,float h,float density,int kind,float breakAt){
    BodyDef bd;
    bd.type = BodyType::Dynamic;
    bd.position = pos;
    if(kind == 1) bd.shape = Shape::circle((real)(h * 0.5f));
    else          bd.shape = Shape::box((real)w, (real)h);
    bd.material.density = (real)density;
    if(kind == 2){                       // glass
        bd.material.restitution     = 0.10;
        bd.material.staticFriction  = 0.50;
        bd.material.dynamicFriction = 0.40;
    } else if(kind == 1){                // barrel
        bd.material.restitution     = 0.28;
        bd.material.staticFriction  = 0.55;
        bd.material.dynamicFriction = 0.42;
        bd.material.rollingFriction = 0.05;
    } else {
        bd.material.restitution     = 0.16;
        bd.material.staticFriction  = 0.70;
        bd.material.dynamicFriction = 0.55;
    }
    bd.material.hysteresis    = 0.20;
    bd.material.linearDrag    = 0.03;
    bd.material.quadraticDrag = 0.02;
    BodyId id = world.createBody(bd);
    propKind[id] = kind;
    if(breakAt > 0.f) frac.makeBreakable(id, (real)breakAt);
    return id;
}

static BodyId staticBox(World& w,Vec2 pos,float hw,float hh,float angle = 0.f){
    BodyDef bd;
    bd.type = BodyType::Static;
    bd.shape = Shape::box((real)(hw * 2.f), (real)(hh * 2.f));
    bd.position = pos;
    bd.angle = (real)angle;
    bd.material.staticFriction  = 0.95;
    bd.material.dynamicFriction = 0.80;
    bd.material.restitution     = 0.05;
    return w.createBody(bd);
}

void Game::buildMap(int mapId){
    if(mapId == 0){
        // ---------------- MAP 1: open track ----------------
        bboxX = 120.f;
        staticBox(world, Vec2(0, -1.0), 122.f, 1.0f);
        staticBox(world, Vec2(-118, 4.0), 1.0f, 5.0f);
        staticBox(world, Vec2( 118, 4.0), 1.0f, 5.0f);
        const float rampX[6] = { -80.f, -34.f, 18.f, 52.f, 84.f, 104.f };
        const float rampA[6] = { 0.30f, -0.34f, 0.26f, -0.30f, 0.36f, -0.26f };
        for(int i = 0; i < 6; i++) staticBox(world, Vec2((real)rampX[i], 0.9), 4.2f, 0.35f, rampA[i]);
        for(int i = 0; i < 13; i++) staticBox(world, Vec2((real)(-100.f + i * 16.f), 0.10), 0.7f, 0.16f);
        for(int wall = 0; wall < 3; wall++){
            float bx = -46.f + wall * 44.f;
            for(int cy = 0; cy < 3; cy++)
                for(int cx = 0; cx < 5; cx++)
                    prop(Vec2((real)(bx + cx * 0.86f), (real)(0.45f + cy * 0.86f)), 0.8f, 0.8f, 420.f, 0, 55.f);
        }
        for(int i = 0; i < 8; i++) prop(Vec2((real)(-70.f + i * 20.f), 1.05), 0.25f, 2.0f, 2500.f, 2, 26.f);
        for(int i = 0; i < 14; i++) prop(Vec2((real)(-90.f + i * 13.f), 0.6),  0.7f, 0.7f, 240.f, 1, 0.f);
        for(int i = 0; i < 5;  i++) prop(Vec2((real)(-24.f + i * 26.f), 0.8),  1.4f, 1.4f, 2300.f, 0, 220.f);
    } else if(mapId == 1){
        // ---------------- MAP 2: sine hills ----------------
        // The terrain is built from slab segments that follow groundAt(x), and
        // every spawn is lifted above it (this is what used to blow up).
        bboxX = 124.f;
        for(int i = 1; i <= 208; i++){
            float x0 = -120.f + (i - 1) * 1.16f;
            float x1 = x0 + 1.16f;
            float y0 = groundAt(x0), y1 = groundAt(x1);
            float mx = (x0 + x1) * 0.5f, my = (y0 + y1) * 0.5f;
            float ang = std::atan2(y1 - y0, x1 - x0);
            staticBox(world, Vec2((real)mx, (real)(my - 0.6f)), 0.72f, 0.62f, ang);
        }
        staticBox(world, Vec2(-124, 8.0), 1.0f, 9.0f);
        staticBox(world, Vec2( 124, 8.0), 1.0f, 9.0f);
        for(int i = 0; i < 26; i++){
            float x = -104.f + i * 8.2f;
            prop(Vec2((real)x, (real)(groundAt(x) + 6.0f + rnd(0.f, 4.f))), 0.8f, 0.8f, 430.f, 0, 60.f);
        }
        for(int i = 0; i < 18; i++){
            float x = -96.f + i * 11.f;
            prop(Vec2((real)x, (real)(groundAt(x) + 4.2f)), 0.7f, 0.7f, 250.f, 1, 0.f);
        }
        for(int i = 0; i < 6; i++){
            float x = -60.f + i * 24.f;
            prop(Vec2((real)x, (real)(groundAt(x) + 2.2f)), 0.25f, 2.0f, 2500.f, 2, 26.f);
        }
    } else {
        // ---------------- MAP 3: closed arena ----------------
        bboxX = 76.f;
        staticBox(world, Vec2(0, -1.0), 76.f, 1.0f);          // floor
        staticBox(world, Vec2(0, 34.0), 76.f, 1.0f);          // ceiling
        staticBox(world, Vec2(-74, 17.0), 1.0f, 18.f);
        staticBox(world, Vec2( 74, 17.0), 1.0f, 18.f);
        staticBox(world, Vec2(-34, 7.0), 12.f, 0.4f);
        staticBox(world, Vec2( 30, 11.0), 10.f, 0.4f);
        staticBox(world, Vec2(  2, 16.0),  9.f, 0.4f);
        for(int t = 0; t < 5; t++){
            float bx = -50.f + t * 24.f;
            for(int cy = 0; cy < 9; cy++)
                for(int cx = 0; cx < 2; cx++)
                    prop(Vec2((real)(bx + cx * 0.86f), (real)(0.45f + cy * 0.86f)), 0.8f, 0.8f, 400.f, 0, 50.f);
        }
        for(int i = 0; i < 10; i++) prop(Vec2((real)(-60.f + i * 13.f), (real)(20.f + i * 1.1f)), 0.9f, 0.9f, 520.f, 0, 70.f);
        for(int i = 0; i < 10; i++) prop(Vec2((real)(-58.f + i * 12.f), 0.6), 0.7f, 0.7f, 240.f, 1, 0.f);
        for(int i = 0; i < 6;  i++) prop(Vec2((real)(-20.f + i * 9.f), 18.0), 0.25f, 2.0f, 2500.f, 2, 26.f);

        // 4.2 t wrecking ball on a rope, kicked into the towers on frame 0
        BodyId anchor = staticBox(world, Vec2(-10, 32.0), 0.5f, 0.5f);
        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.shape = Shape::circle(0.75);
        bd.position = Vec2(-10, 20.0);
        bd.material.density         = 5200.0;
        bd.material.restitution     = 0.20;
        bd.material.staticFriction  = 0.70;
        bd.material.dynamicFriction = 0.60;
        BodyId ball = world.createBody(bd);
        propKind[ball] = 4;
        world.createRope(anchor, ball, Vec2(0, 0), Vec2(0, 0), 13.0);
        RigidBody* bb = world.body(ball);
        if(bb) bb->applyImpulse(Vec2(90000.0, 0.0));
    }
}

void Game::spawnCar(CarKind kind,float x,int playerIdx){
    const CarSpec& sp = SPECS[(int)kind];
    int idx = (int)cars.size();
    Car car;
    car.kind = kind;
    car.spec = &SPECS[(int)kind];
    car.player = playerIdx;
    car.aiTimer = rnd(0.3f, 2.0f);
    car.shell.init(sp.cw, sp.ch, sp.bodyCol, sp.roofCol);

    float baseY = groundAt(x) + 2.2f;

    // ---- chassis
    BodyDef bd;
    bd.type = BodyType::Dynamic;
    bd.shape = Shape::box((real)(sp.cw * 2.f), (real)(sp.ch * 2.f));
    bd.position = Vec2((real)x, (real)baseY);
    bd.material.density         = (real)(sp.mass / (sp.cw * 2.f * sp.ch * 2.f));
    bd.material.restitution     = 0.15;
    bd.material.staticFriction  = 0.55;
    bd.material.dynamicFriction = 0.45;
    bd.material.linearDrag      = 0.05;
    bd.material.quadraticDrag   = 0.06;
    bd.material.hysteresis      = 0.18;
    bd.name = sp.name;
    car.chassis = world.createBody(bd);
    owner[car.chassis] = idx;

    // ---- wheels: spring suspension + breakable control arm + drive motor
    const float travel = sp.ch + sp.wr * 0.55f;
    for(int i = 0; i < 2; i++){
        float ax = (i == 0) ? -sp.cw * 0.72f : sp.cw * 0.72f;
        BodyDef wd;
        wd.type = BodyType::Dynamic;
        wd.shape = Shape::circle((real)sp.wr);
        wd.position = Vec2((real)(x + ax), (real)(baseY - travel));
        wd.material.density         = 260.0;
        wd.material.restitution     = 0.08;
        wd.material.staticFriction  = 1.90;
        wd.material.dynamicFriction = 1.45;
        wd.material.rollingFriction = 0.010;
        wd.material.hysteresis      = 0.25;
        BodyId wid = world.createBody(wd);
        owner[wid] = idx;

        Wheel wh;
        wh.id = wid;
        wh.driven = (sp.driveWheels == 2) || (i == 1);
        wh.susp = world.createSpring(car.chassis, wid,
                                     Vec2((real)ax, (real)(-sp.ch * 0.4f)), Vec2(0, 0),
                                     (real)(travel - sp.ch * 0.4f),
                                     (real)sp.suspStiff, (real)sp.suspDamp);
        wh.arm = world.createDistance(car.chassis, wid,
                                      Vec2((real)ax, (real)(-sp.ch * 0.4f)), Vec2(0, 0),
                                      (real)(travel - sp.ch * 0.4f));
        if(wh.arm) wh.arm->breakForce = (real)(sp.mass * 260.f);
        wh.motor = world.createAngular(car.chassis, wid, -1e9, 1e9);
        if(wh.motor){ wh.motor->motorSpeed = 0.0; wh.motor->maxMotorTorque = 0.0; }
        car.wheels[i] = wh;
    }

    // ---- driver: own body, held by a breakable seatbelt
    BodyDef dd;
    dd.type = BodyType::Dynamic;
    dd.shape = Shape::box(0.28, 0.52);
    dd.position = Vec2((real)(x + sp.cw * 0.10f), (real)(baseY + sp.ch * 0.85f));
    dd.material.density         = 1050.0;
    dd.material.restitution     = 0.05;
    dd.material.staticFriction  = 0.80;
    dd.material.dynamicFriction = 0.60;
    dd.name = "driver";
    car.driver = world.createBody(dd);
    owner[car.driver] = idx;
    car.seatbelt = world.createRevolute(car.chassis, car.driver,
                                       Vec2((real)(sp.cw * 0.10f), (real)(sp.ch * 0.85f)), Vec2(0, 0));
    if(car.seatbelt) car.seatbelt->breakForce = (real)(sp.mass * 95.f);

    cars.push_back(car);
}

void Game::drive(Car& car,bool gas,bool brake,bool leanL,bool leanR,bool boost,bool handbrake,float dt){
    if(!car.alive) return;
    RigidBody* cb = world.body(car.chassis);
    if(!cb) return;
    const CarSpec& sp = *car.spec;

    float boostMul = 1.f;
    if(boost && car.nitro > 0.f && car.fuel > 0.f){
        boostMul = 2.3f;
        car.nitro = (float)RC(car.nitro - dt * 0.35, 0.0, 1.0);
        car.heat += dt * 0.45f;
        Vec2 back(-std::cos((real)cb->angle), -std::sin((real)cb->angle));
        puff(cb->position + Vec2(back.x * (real)sp.cw, back.y * (real)sp.cw + 0.1),
             Vec2(back.x * 7.0, back.y * 7.0 + 0.6), 0.20f, 235, 0.75f);
        spark(cb->position + Vec2(back.x * (real)sp.cw, back.y * (real)sp.cw), back, 3, 7.f, 120, 170, 255);
    }

    car.throttle = 0.f;
    float torque = 0.f, targetSpeed = 0.f;
    if(gas && car.fuel > 0.f){
        torque = sp.torque * boostMul * (0.55f + 0.45f * car.health);
        targetSpeed = -sp.topWheelSpeed * boostMul;
        car.fuel = (float)RC(car.fuel - dt * (0.010 + 0.020 * (boostMul - 1.f)), 0.0, 1.0);
        car.heat += dt * 0.10f;
        car.throttle = 1.f;
    } else if(brake){
        torque = sp.torque * 0.85f;
        targetSpeed = 0.f;
        car.throttle = -1.f;
    }

    for(int i = 0; i < 2; i++){
        Wheel& wh = car.wheels[i];
        if(wh.lost || wh.id == INVALID_BODY || !wh.motor) continue;
        RigidBody* wb = world.body(wh.id);
        if(!wb){ wh.lost = true; continue; }
        if(handbrake && i == 1){
            wh.motor->motorSpeed = 0.0;
            wh.motor->maxMotorTorque = (real)(sp.torque * 1.6f);
        } else if(wh.driven && (gas || brake)){
            wh.motor->motorSpeed = (real)targetSpeed;
            wh.motor->maxMotorTorque = (real)torque;
        } else {
            wh.motor->motorSpeed = 0.0;
            wh.motor->maxMotorTorque = 0.0;
        }
        float surf = (float)std::fabs(wb->angularVelocity) * sp.wr;
        float road = (float)std::sqrt(cb->velocity.x * cb->velocity.x + cb->velocity.y * cb->velocity.y);
        wh.slip = std::fabs(surf - road);
        if(wh.slip > 4.f && road > 0.5f){
            if(skids.size() < 700){
                Skid s; s.p = wb->position; s.life = 0.f; s.max = 6.f;
                s.strength = std::min(1.f, wh.slip * 0.08f);
                skids.push_back(s);
            }
            if(rnd01() < 0.30f) puff(wb->position, Vec2((real)rnd(-1.f, 1.f), (real)rnd(0.3f, 1.4f)), 0.16f, 205, 0.32f);
        }
    }

    float lean = 0.f;
    if(leanL) lean += 1.f;
    if(leanR) lean -= 1.f;
    if(lean != 0.f) cb->applyTorque((real)(lean * sp.mass * 7.f));

    car.engineRpm = car.engineRpm * 0.9f + (gas ? 1.f : 0.f) * boostMul * 0.1f;
    car.heat = (float)RC(car.heat - dt * 0.16, 0.0, 1.0);
    if(car.heat > 0.9f){
        puff(cb->position + Vec2(0, (real)(sp.ch * 1.2f)), Vec2(0, 1.4), 0.18f, 120, 0.5f);
        car.health = (float)RC(car.health - dt * 0.02, 0.0, 1.0);
    }
}

void Game::ai(Car& car,float dt){
    if(!car.alive) return;
    RigidBody* cb = world.body(car.chassis);
    if(!cb) return;
    car.aiTimer -= dt;
    if(car.aiTimer <= 0.f){
        car.aiTimer = rnd(1.2f, 3.5f);
        Car* p = playerCar();
        RigidBody* pb = p ? world.body(p->chassis) : nullptr;
        if(pb && rnd01() < 0.7f) car.aiTarget = (float)pb->position.x + rnd(-3.f, 3.f);
        else                     car.aiTarget = rnd(-bboxX * 0.7f, bboxX * 0.7f);
        car.aiMode = (rnd01() < 0.12f) ? 1 : 0;
    }
    float dx = car.aiTarget - (float)cb->position.x;
    bool gas   = std::fabs(dx) > 2.5f && car.aiMode == 0;
    bool brake = (car.aiMode == 1);
    float ang  = (float)cb->angle;
    bool leanL = (ang < -0.35f), leanR = (ang > 0.35f);
    bool boost = (std::fabs(dx) > 25.f && rnd01() < 0.03f);
    drive(car, gas, brake, leanL, leanR, boost, false, dt);
    if(gas) cb->applyForce(Vec2((real)((dx > 0.f ? 1.f : -1.f) * car.spec->mass * 3.2f), 0.0));
}

void Game::crash(int idx,const Vec2& worldPoint,float impulse,float speed){
    if(idx < 0 || idx >= (int)cars.size()) return;
    Car& car = cars[idx];
    if(!car.alive || car.dead) return;
    RigidBody* cb = world.body(car.chassis);
    if(!cb) return;
    const CarSpec& sp = *car.spec;

    // Damage from real collision energy, not from mere touching.
    float energy = impulse * std::max(1.f, speed) * 0.02f;
    float ref    = sp.mass * sp.toughness * 0.55f;
    float damage = std::min(0.45f, energy / std::max(1.f, ref));
    if(damage < 0.0005f) return;

    float s = std::sin(-(float)cb->angle), c = std::cos(-(float)cb->angle);
    Vec2 d = worldPoint - cb->position;
    Vec2 local((real)(c * (float)d.x - s * (float)d.y), (real)(s * (float)d.x + c * (float)d.y));
    car.shell.dent(local, energy / std::max(1.f, sp.toughness * 0.35f));
    car.health = (float)RC(car.health - damage, 0.0, 1.0);

    int n = (int)std::min(24.f, 3.f + impulse * 0.05f);
    spark(worldPoint, Vec2(-d.x, -d.y), n, 3.f + speed * 0.35f, 255, 205, 90);
    if(impulse > 140.f) puff(worldPoint, Vec2(0, 0.8), 0.26f, 110, 0.6f);
    if(damage > 0.02f) shake = std::min(1.4f, shake + damage * 2.2f);

    RigidBody* db = world.body(car.driver);
    if(db && damage > 0.035f){
        Vec2 dir((real)rnd(-1.f, 1.f), (real)rnd(0.2f, 1.f));
        blood.addWound(car.driver, db->position, dir, (real)std::min(1.f, damage * 6.f), damage > 0.12f);
        blood.spray(db->position, dir, (real)(2.5f + damage * 18.f), 0.5,
                    (int)std::min(30.f, 5.f + damage * 90.f), (real)(3.0 + damage * 40.0));
        car.bleeding = true;
    }
    if(car.seatbelt && car.seatbelt->broken && !car.driverOut) ejectDriver(car);
    if(car.health < 0.30f && car.fire <= 0.f && rnd01() < 0.5f) car.fire = rnd(5.f, 14.f);

    for(int i = 0; i < 2; i++){
        Wheel& wh = car.wheels[i];
        if(!wh.lost && wh.arm && wh.arm->broken){
            wh.lost = true;
            if(wh.motor) wh.motor->maxMotorTorque = 0.0;
            spark(worldPoint, Vec2(0, 1), 12, 7.f, 220, 220, 235);
        }
    }
}

void Game::ejectDriver(Car& car){
    if(car.driverOut) return;
    car.driverOut = true;
    if(car.seatbelt){ world.destroyConstraint(car.seatbelt); car.seatbelt = nullptr; }
    owner.erase(car.driver);
    RigidBody* db = world.body(car.driver);
    if(db){
        blood.addWound(car.driver, db->position, Vec2(0, 1), 0.85, true);
        blood.spray(db->position, Vec2((real)rnd(-1.f, 1.f), 1.0), 8.0, 0.9, 40, 70.0);
    }
    car.bleeding = true;
}

void Game::wreck(int idx){
    if(idx < 0 || idx >= (int)cars.size()) return;
    Car& car = cars[idx];
    if(car.dead) return;
    car.dead = true; car.alive = false;

    RigidBody* cb = world.body(car.chassis);
    Vec2 c = cb ? cb->position : Vec2(0, 5);
    if(!c.isFinite()) c = Vec2(0, 5);

    Boom bm; bm.c = c; bm.power = 160.f + car.spec->mass * 0.05f; bm.radius = 6.5f;
    booms.push_back(bm);

    for(int i = 0; i < 60; i++){
        float a = rnd(0.f, 6.283f), s = rnd(3.f, 16.f);
        Spark sp;
        sp.p = c; sp.v = Vec2((real)(std::cos(a) * s), (real)(std::sin(a) * s));
        sp.life = 0.f; sp.max = rnd(0.4f, 2.0f);
        sp.r = 255; sp.g = (uint8_t)rnd(90.f, 205.f); sp.b = 30;
        sparks.push_back(sp);
    }
    for(int i = 0; i < 14; i++)
        puff(c, Vec2((real)rnd(-4.f, 4.f), (real)rnd(1.f, 7.f)), rnd(0.4f, 1.1f), 50, 0.9f);

    for(int i = 0; i < 6; i++){
        float a = rnd(0.f, 6.283f), s = rnd(4.f, 12.f);
        BodyId f = prop(c + Vec2((real)(std::cos(a) * 0.6f), (real)(std::sin(a) * 0.6f)),
                        rnd(0.25f, 0.55f), rnd(0.15f, 0.40f), 1400.f, 3, 0.f);
        RigidBody* fb = world.body(f);
        if(fb) fb->applyImpulse(Vec2((real)(std::cos(a) * s * (float)fb->mass),
                                     (real)(std::sin(a) * s * (float)fb->mass)));
    }

    if(!car.driverOut) ejectDriver(car);
    doomed.push_back(car.chassis);
    for(int i = 0; i < 2; i++) if(car.wheels[i].id != INVALID_BODY) doomed.push_back(car.wheels[i].id);
    wrecks++;
    if(car.player < 0) kills++;
    shake = std::min(2.0f, shake + 0.9f);
}

void Game::spark(Vec2 p,Vec2 dir,int n,float spd,uint8_t r,uint8_t g,uint8_t b){
    if(sparks.size() > 2000 || !p.isFinite()) return;
    float base = std::atan2((float)dir.y, (float)dir.x);
    if(!(base == base)) base = 1.57f;
    n = std::min(n, 40);
    for(int i = 0; i < n; i++){
        float a = base + rnd(-1.1f, 1.1f), s = spd * rnd(0.35f, 1.7f);
        Spark sp;
        sp.p = p; sp.v = Vec2((real)(std::cos(a) * s), (real)(std::sin(a) * s));
        sp.life = 0.f; sp.max = rnd(0.25f, 1.1f);
        sp.r = r; sp.g = g; sp.b = b;
        sparks.push_back(sp);
    }
}

void Game::puff(Vec2 p,Vec2 v,float radius,uint8_t gray,float alpha){
    if(smokes.size() > 600 || !p.isFinite()) return;
    Smoke s;
    s.p = p; s.v = v; s.life = 0.f; s.max = rnd(0.9f, 2.4f); s.radius = radius;
    s.r = gray; s.g = gray; s.b = gray; s.alpha = alpha;
    smokes.push_back(s);
}

// Hard safety net: nothing may leave the play field or exceed sane speeds.
// One bad contact can otherwise throw a body to 1e9 metres and the broad
// phase will try to allocate a grid that spans it.
void Game::enforceLimits(){
    const real vmax = 180.0, xmax = (real)bboxX + 40.0, ymin = -60.0, ymax = 220.0;

    // PASS 1: only clamp numbers and collect verdicts. Nothing here may create
    // or destroy a body, because world.bodies() would be reallocated under us.
    std::vector<int> wreckList;
    {
        std::vector<RigidBody*>& bs = world.bodies();
        for(size_t i = 0; i < bs.size(); i++){
            RigidBody* b = bs[i];
            if(!b || b->type != BodyType::Dynamic) continue;
            bool bad = (!b->position.isFinite() || !b->velocity.isFinite());
            if(bad){
                b->velocity = Vec2(0, 0);
                b->angularVelocity = 0.0;
                b->position = Vec2(0, 10);
            } else {
                real v2 = b->velocity.x * b->velocity.x + b->velocity.y * b->velocity.y;
                if(v2 > vmax * vmax){
                    real k = vmax / std::sqrt(v2);
                    b->velocity = Vec2(b->velocity.x * k, b->velocity.y * k);
                }
                if(b->angularVelocity >  200.0) b->angularVelocity =  200.0;
                if(b->angularVelocity < -200.0) b->angularVelocity = -200.0;
            }
            bool out = bad || (b->position.x < -xmax || b->position.x > xmax ||
                               b->position.y <  ymin || b->position.y > ymax);
            if(!out) continue;
            std::unordered_map<BodyId,int>::iterator it = owner.find(b->id);
            if(it != owner.end()) wreckList.push_back(it->second);
            else { doomed.push_back(b->id); culled++; }
        }
    }
    // PASS 2: safe to mutate the world now.
    for(size_t i = 0; i < wreckList.size(); i++) wreck(wreckList[i]);
}

void Game::step(float dt){
    if(paused) return;
    float sdt = dt * timeScale * (slowmo ? 0.25f : 1.f);
    if(sdt <= 0.f) return;
    time += sdt;
    gustPhase += sdt;
    windX = 3.2f * std::sin(gustPhase * 0.23f) + 1.4f * std::sin(gustPhase * 0.91f + 1.f);

    for(size_t i = 0; i < cars.size(); i++){
        Car& car = cars[i];
        if(!car.alive) continue;
        RigidBody* cb = world.body(car.chassis);
        if(!cb){ wreck((int)i); continue; }
        // aerodynamic drag against the gusting wind: F = 0.5*rho*Cd*A*|v|v
        real relx = cb->velocity.x - (real)windX;
        real q = 0.5 * 1.204 * 1.05 * ((real)car.spec->ch * 2.0) * std::fabs(relx) * relx;
        cb->applyForce(Vec2(-q, 0));
        if(car.fire > 0.f){
            car.fire -= sdt;
            car.health = (float)RC(car.health - sdt * 0.055, 0.0, 1.0);
            if(rnd01() < 0.6f){
                puff(cb->position + Vec2((real)rnd(-0.6f, 0.6f), (real)(car.spec->ch * 1.1f)),
                     Vec2((real)(windX * 0.4f + rnd(-0.5f, 0.5f)), (real)rnd(1.4f, 3.6f)),
                     rnd(0.18f, 0.42f), 45, 0.85f);
                spark(cb->position + Vec2((real)rnd(-0.6f, 0.6f), (real)rnd(-0.2f, 0.6f)),
                      Vec2(0, 1), 2, 4.f, 255, (uint8_t)rnd(110.f, 190.f), 20);
            }
        }
        for(int wi = 0; wi < 2; wi++){
            Wheel& wh = car.wheels[wi];
            if(wh.id != INVALID_BODY && !world.body(wh.id)) wh.lost = true;
        }
    }

    enforceLimits();
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    world.step(sdt * 0.5f);
    world.step(sdt * 0.5f);
    ccd.update(sdt);
    frac.update(sdt);
    blood.update(sdt);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    stepMs = std::chrono::duration<float,std::milli>(t1 - t0).count();
    lastContacts = (int)world.contactCount();
    enforceLimits();

    for(size_t i = 0; i < hits.size(); i++){
        const Hit& h = hits[i];
        if(h.impulse < 10.f && h.speed < 4.f) continue;
        std::unordered_map<BodyId,int>::iterator ia = owner.find(h.a);
        std::unordered_map<BodyId,int>::iterator ib = owner.find(h.b);
        if(ia != owner.end()) crash(ia->second, h.point, h.impulse, h.speed);
        if(ib != owner.end()) crash(ib->second, h.point, h.impulse, h.speed);
        if(ia == owner.end() && ib == owner.end() && h.speed > 7.f)
            spark(h.point, h.normal, 4, h.speed * 0.3f, 210, 210, 220);
    }
    hits.clear();

    if(!booms.empty()){
        std::vector<Boom> pending;
        pending.swap(booms);
        for(size_t i = 0; i < pending.size(); i++){
            const Boom bm = pending[i];
            std::vector<RigidBody*> bs = world.bodies();   // snapshot on purpose
            for(size_t j = 0; j < bs.size(); j++){
                RigidBody* b = bs[j];
                if(!b || b->type != BodyType::Dynamic) continue;
                Vec2 d = b->position - bm.c;
                real len = std::sqrt(d.x * d.x + d.y * d.y);
                if(len >= (real)bm.radius || len < 0.05) continue;
                real f = (real)bm.power * (1.0 - len / (real)bm.radius) * b->mass / len;
                b->applyImpulse(Vec2(d.x * f, d.y * f));
                b->wake();
            }
            for(int k = 0; k < 22; k++){
                float a = rnd(0.f, 6.283f), s = rnd(2.f, 12.f);
                Spark sp;
                sp.p = bm.c; sp.v = Vec2((real)(std::cos(a) * s), (real)(std::sin(a) * s));
                sp.life = 0.f; sp.max = rnd(0.3f, 1.6f);
                sp.r = 255; sp.g = (uint8_t)rnd(80.f, 170.f); sp.b = 25;
                if(sparks.size() < 2200) sparks.push_back(sp);
            }
            for(int k = 0; k < 8; k++)
                puff(bm.c, Vec2((real)rnd(-3.f, 3.f), (real)rnd(1.f, 5.f)), rnd(0.5f, 1.4f), 35, 0.9f);
        }
    }

    for(size_t i = 0; i < cars.size(); i++)
        if(cars[i].alive && !cars[i].dead && cars[i].health <= 0.f) wreck((int)i);

    for(size_t i = 0; i < cars.size(); i++){
        Car& car = cars[i];
        if(!car.bleeding || car.driver == INVALID_BODY) continue;
        RigidBody* db = world.body(car.driver);
        if(db && rnd01() < 0.22f)
            blood.spray(db->position, Vec2((real)rnd(-0.4f, 0.4f), -0.6), 1.6, 0.7, 2, 1.2);
    }

    for(size_t i = 0; i < cars.size(); i++) if(cars[i].player < 0) ai(cars[i], sdt);

    if(!doomed.empty()){
        std::vector<BodyId> list;
        list.swap(doomed);
        for(size_t i = 0; i < list.size(); i++){
            BodyId id = list[i];
            if(id == INVALID_BODY) continue;
            if(world.body(id)){
                blood.forgetBody(id);
                owner.erase(id);
                propKind.erase(id);
                world.destroyBody(id);
            }
        }
    }

    for(size_t i = 0; i < cars.size(); i++)
        if(cars[i].alive) cars[i].nitro = (float)RC(cars[i].nitro + sdt * 0.012, 0.0, 1.0);

    for(size_t i = 0; i < sparks.size(); i++){
        Spark& s = sparks[i];
        s.v.y -= (real)(22.0 * sdt);
        s.v.x *= 0.985; s.v.y *= 0.985;
        s.p.x += s.v.x * sdt; s.p.y += s.v.y * sdt;
        if(s.p.y < 0.02 && map != 1){ s.p.y = 0.02; s.v.y = std::fabs(s.v.y) * 0.25; s.v.x *= 0.6; }
        s.life += sdt;
    }
    sparks.erase(std::remove_if(sparks.begin(), sparks.end(),
                 [](const Spark& s){ return s.life >= s.max; }), sparks.end());

    for(size_t i = 0; i < smokes.size(); i++){
        Smoke& s = smokes[i];
        s.v.x += (real)(windX * sdt * 0.55);
        s.v.y += (real)(0.55 * sdt);
        s.p.x += s.v.x * sdt; s.p.y += s.v.y * sdt;
        s.radius += sdt * 0.42f;
        s.life += sdt;
        s.alpha *= (1.f - sdt * 0.85f);
    }
    smokes.erase(std::remove_if(smokes.begin(), smokes.end(),
                 [](const Smoke& s){ return s.life >= s.max || s.alpha < 0.02f; }), smokes.end());

    for(size_t i = 0; i < skids.size(); i++) skids[i].life += sdt;
    skids.erase(std::remove_if(skids.begin(), skids.end(),
                [](const Skid& s){ return s.life >= s.max; }), skids.end());

    shake *= (1.f - std::min(1.f, sdt * 3.2f));
}

void Game::drawWorld(Canvas& C,const Camera& cam){
    for(size_t i = 0; i < skids.size(); i++){
        const Skid& s = skids[i];
        int sx, sy; cam.toScreen((float)s.p.x, (float)s.p.y, sx, sy);
        float f = 1.f - s.life / s.max;
        C.disc(sx, sy, std::max(1, cam.m2p(0.10f)), 20, 20, 22, (uint8_t)(150 * f * s.strength));
    }
    std::vector<RigidBody*>& bs = world.bodies();
    for(size_t i = 0; i < bs.size(); i++){
        RigidBody* b = bs[i];
        if(!b || !b->position.isFinite()) continue;
        if(owner.find(b->id) != owner.end()) continue;   // cars draw themselves
        bool isStatic = (b->type == BodyType::Static);
        int kind = 0;
        std::unordered_map<BodyId,int>::iterator it = propKind.find(b->id);
        if(it != propKind.end()) kind = it->second;
        uint32_t col;
        if(isStatic)      col = 0x4A4238;
        else if(kind == 1) col = 0xB4632A;
        else if(kind == 2) col = 0x9FD8E8;
        else if(kind == 3) col = 0x6E6E76;
        else if(kind == 4) col = 0x50505A;
        else              col = 0x8C6A3C;

        const std::vector<Vec2>& vs = b->worldVertices;
        if(vs.empty()){
            int sx, sy; cam.toScreen((float)b->position.x, (float)b->position.y, sx, sy);
            int r = cam.m2p((float)b->shape.radius);
            if(sx < -r - 40 || sx > C.w + r + 40 || sy < -r - 40 || sy > C.h + r + 40) continue;
            C.disc(sx, sy, r, (col >> 16) & 255, (col >> 8) & 255, col & 255, kind == 2 ? 170 : 255);
            float a = (float)b->angle;
            C.line(sx, sy, sx + (int)(r * std::cos(a)), sy - (int)(r * std::sin(a)), 0x2A2A2E, 1);
        } else {
            std::vector<std::pair<int,int> > poly;
            poly.reserve(vs.size());
            bool onScreen = false;
            for(size_t k = 0; k < vs.size(); k++){
                int sx, sy; cam.toScreen((float)vs[k].x, (float)vs[k].y, sx, sy);
                if(sx > -80 && sx < C.w + 80 && sy > -80 && sy < C.h + 80) onScreen = true;
                poly.push_back(std::make_pair(sx, sy));
            }
            if(!onScreen) continue;
            C.fillPoly(poly, col, kind == 2 ? 150 : 255);
            for(size_t k = 0; k < poly.size(); k++){
                size_t k2 = (k + 1) % poly.size();
                C.line(poly[k].first, poly[k].second, poly[k2].first, poly[k2].second,
                       isStatic ? 0x6A6055 : 0x24242A, 1);
            }
        }
        if(debugDraw){
            int x0, y0, x1, y1;
            cam.toScreen((float)b->aabb.min.x, (float)b->aabb.max.y, x0, y0);
            cam.toScreen((float)b->aabb.max.x, (float)b->aabb.min.y, x1, y1);
            uint32_t ac = b->sleeping ? 0x2A6E2A : 0x2A5A9E;
            C.line(x0, y0, x1, y0, ac, 1); C.line(x1, y0, x1, y1, ac, 1);
            C.line(x1, y1, x0, y1, ac, 1); C.line(x0, y1, x0, y0, ac, 1);
        }
    }
}

void Game::drawCar(Canvas& C,const Camera& cam,const Car& car){
    RigidBody* cb = world.body(car.chassis);
    const CarSpec& sp = *car.spec;

    for(int i = 0; i < 2; i++){
        const Wheel& wh = car.wheels[i];
        if(wh.id == INVALID_BODY) continue;
        RigidBody* wb = world.body(wh.id);
        if(!wb || !wb->position.isFinite()) continue;
        int wx, wy; cam.toScreen((float)wb->position.x, (float)wb->position.y, wx, wy);
        int r = cam.m2p(sp.wr);
        if(wx < -r - 40 || wx > C.w + r + 40) continue;
        C.disc(wx, wy, r, 26, 26, 30, 255);
        C.disc(wx, wy, std::max(1, (int)(r * 0.55f)),
               (sp.rimCol >> 16) & 255, (sp.rimCol >> 8) & 255, sp.rimCol & 255, 255);
        float a = (float)wb->angle;
        for(int k = 0; k < 4; k++){
            float aa = a + k * 1.5708f;
            C.line(wx, wy, wx + (int)(r * 0.5f * std::cos(aa)), wy - (int)(r * 0.5f * std::sin(aa)), 0x3C3C42, 1);
        }
        if(wh.slip > 4.f) C.disc(wx, wy + r / 2, std::max(1, r / 3), 200, 200, 200, 90);
    }
    if(!cb || !cb->position.isFinite()) return;

    float cs = std::cos((float)cb->angle), sn = std::sin((float)cb->angle);
    float bx0 = (float)cb->position.x, by0 = (float)cb->position.y;

    for(size_t pi = 0; pi < car.shell.panels.size(); pi++){
        const DamageMesh::Panel& p = car.shell.panels[pi];
        std::vector<std::pair<int,int> > poly;
        for(int k = 0; k < 4; k++){
            float lx = (float)p.cur[k].x, ly = (float)p.cur[k].y;
            int sx, sy; cam.toScreen(bx0 + lx * cs - ly * sn, by0 + lx * sn + ly * cs, sx, sy);
            poly.push_back(std::make_pair(sx, sy));
        }
        float df = std::min(1.f, p.dent * 0.55f);
        uint8_t R = (uint8_t)(((p.col >> 16) & 255) * (1.f - df) + 26 * df);
        uint8_t G = (uint8_t)(((p.col >>  8) & 255) * (1.f - df) + 26 * df);
        uint8_t B = (uint8_t)(( p.col        & 255) * (1.f - df) + 30 * df);
        C.fillPoly(poly, ((uint32_t)R << 16) | ((uint32_t)G << 8) | B, 255);
        for(size_t k = 0; k < poly.size(); k++){
            size_t k2 = (k + 1) % poly.size();
            C.line(poly[k].first, poly[k].second, poly[k2].first, poly[k2].second, 0x1E1E22, 1);
        }
    }
    {
        std::vector<std::pair<int,int> > win;
        const float lxs[4] = { -sp.cw * 0.24f, sp.cw * 0.34f, sp.cw * 0.28f, -sp.cw * 0.18f };
        const float lys[4] = {  sp.ch * 0.60f, sp.ch * 0.60f, sp.ch * 1.55f,  sp.ch * 1.55f };
        for(int k = 0; k < 4; k++){
            int sx, sy; cam.toScreen(bx0 + lxs[k] * cs - lys[k] * sn, by0 + lxs[k] * sn + lys[k] * cs, sx, sy);
            win.push_back(std::make_pair(sx, sy));
        }
        C.fillPoly(win, 0x22303C, (uint8_t)(car.health > 0.5f ? 210 : 120));
    }
    for(size_t k = 0; k + 1 < car.shell.cracks.size(); k += 2){
        float ax = (float)car.shell.cracks[k].x,     ay = (float)car.shell.cracks[k].y;
        float bx = (float)car.shell.cracks[k + 1].x, by = (float)car.shell.cracks[k + 1].y;
        int x0, y0, x1, y1;
        cam.toScreen(bx0 + ax * cs - ay * sn, by0 + ax * sn + ay * cs, x0, y0);
        cam.toScreen(bx0 + bx * cs - by * sn, by0 + bx * sn + by * cs, x1, y1);
        C.line(x0, y0, x1, y1, 0x14141A, 1);
    }
    {
        float hlx = -sp.cw * 0.98f, tlx = sp.cw * 0.98f;
        int hx, hy, tx, ty;
        cam.toScreen(bx0 + hlx * cs, by0 + hlx * sn, hx, hy);
        cam.toScreen(bx0 + tlx * cs, by0 + tlx * sn, tx, ty);
        C.disc(hx, hy, std::max(1, cam.m2p(0.11f)), 255, 244, 190, (uint8_t)(car.health > 0.3f ? 255 : 90));
        C.disc(tx, ty, std::max(1, cam.m2p(0.09f)), 210,  40,  40, (uint8_t)(car.health > 0.3f ? 255 : 90));
    }
    if(car.driver != INVALID_BODY){
        RigidBody* db = world.body(car.driver);
        if(db && db->position.isFinite()){
            const std::vector<Vec2>& dv = db->worldVertices;
            if(!dv.empty()){
                std::vector<std::pair<int,int> > poly;
                for(size_t k = 0; k < dv.size(); k++){
                    int sx, sy; cam.toScreen((float)dv[k].x, (float)dv[k].y, sx, sy);
                    poly.push_back(std::make_pair(sx, sy));
                }
                C.fillPoly(poly, car.bleeding ? 0x9A5A50 : 0xC8A882, 255);
            }
            int hx, hy; cam.toScreen((float)db->position.x, (float)db->position.y + 0.34f, hx, hy);
            C.disc(hx, hy, std::max(1, cam.m2p(0.15f)), 222, 190, 150, 255);
        }
    }
    if(car.fire > 0.f){
        int fx, fy; cam.toScreen(bx0 - sp.ch * 1.1f * sn, by0 + sp.ch * 1.1f * cs, fx, fy);
        C.disc(fx, fy, std::max(2, cam.m2p(0.45f + rnd01() * 0.22f)), 255, (uint8_t)rnd(110.f, 190.f), 25, 175);
        C.disc(fx, fy - cam.m2p(0.2f), std::max(1, cam.m2p(0.26f)), 255, 235, 140, 150);
    }
    {
        int bxp, byp; cam.toScreen(bx0 - sp.ch * 2.15f * sn, by0 + sp.ch * 2.15f * cs, bxp, byp);
        int bw = cam.m2p(sp.cw * 1.6f), bh = 4;
        if(bw > 6){
            C.rect(bxp - bw / 2, byp, bw, bh, 0x181820, 220);
            uint32_t hc = car.health > 0.5f ? 0x30D050 : (car.health > 0.22f ? 0xE0C030 : 0xE04030);
            C.rect(bxp - bw / 2, byp, (int)(bw * car.health), bh, hc, 255);
            if(car.player == 0){
                C.rect(bxp - bw / 2, byp + bh + 1, bw, 3, 0x101028, 220);
                C.rect(bxp - bw / 2, byp + bh + 1, (int)(bw * car.nitro), 3, 0x4878FF, 255);
                C.disc(bxp, byp - cam.m2p(0.35f), 4, 80, 220, 255, 220);
            }
        }
    }
}

void Game::drawBlood(Canvas& C,const Camera& cam){
    const std::vector<BloodPool>& pools = blood.pools();
    for(size_t i = 0; i < pools.size(); i++){
        const BloodPool& p = pools[i];
        uint8_t r, g, b; BloodSystem::colorOf(p.oxygen, p.wetness, r, g, b);
        int sx, sy; cam.toScreen((float)p.center.x, (float)p.center.y, sx, sy);
        int hw = std::max(1, cam.m2p((float)p.halfWidth));
        C.rect(sx - hw, sy - 2, hw * 2, 4, ((uint32_t)r << 16) | ((uint32_t)g << 8) | b, 220);
    }
    const std::vector<BloodDecal>& decals = blood.decals();
    for(size_t i = 0; i < decals.size(); i++){
        const BloodDecal& d = decals[i];
        uint8_t r, g, b; BloodSystem::colorOf(d.oxygen, d.wetness, r, g, b);
        float bx = (float)d.anchor.x, by = (float)d.anchor.y, ca = 1.f, sa = 0.f;
        if(d.body != INVALID_BODY){
            RigidBody* bb = world.body(d.body);
            if(!bb || !bb->position.isFinite()) continue;
            ca = std::cos((float)bb->angle); sa = std::sin((float)bb->angle);
            bx = (float)bb->position.x + (float)d.anchor.x * ca - (float)d.anchor.y * sa;
            by = (float)bb->position.y + (float)d.anchor.x * sa + (float)d.anchor.y * ca;
        }
        if(d.outline.size() < 3){
            int sx, sy; cam.toScreen(bx, by, sx, sy);
            C.disc(sx, sy, std::max(1, cam.m2p(0.05f)), r, g, b, 230);
            continue;
        }
        std::vector<std::pair<int,int> > poly;
        for(size_t k = 0; k < d.outline.size(); k++){
            float ox = (float)d.outline[k].x, oy = (float)d.outline[k].y;
            int sx, sy; cam.toScreen(bx + ox * ca - oy * sa, by + ox * sa + oy * ca, sx, sy);
            poly.push_back(std::make_pair(sx, sy));
        }
        C.fillPoly(poly, ((uint32_t)r << 16) | ((uint32_t)g << 8) | b, 235);
    }
    const std::vector<BloodDroplet>& drops = blood.droplets();
    for(size_t i = 0; i < drops.size(); i++){
        const BloodDroplet& d = drops[i];
        if(!d.active) continue;
        uint8_t r, g, b; BloodSystem::colorOf(d.oxygen, 1.0, r, g, b);
        int sx, sy; cam.toScreen((float)d.position.x, (float)d.position.y, sx, sy);
        C.disc(sx, sy, std::max(1, cam.m2p((float)d.radius * 1.6f)), r, g, b, 240);
    }
}

void Game::drawFx(Canvas& C,const Camera& cam){
    for(size_t i = 0; i < smokes.size(); i++){
        const Smoke& s = smokes[i];
        int sx, sy; cam.toScreen((float)s.p.x, (float)s.p.y, sx, sy);
        int r = cam.m2p(s.radius);
        if(r < 1 || sx < -r - 40 || sx > C.w + r + 40) continue;
        C.disc(sx, sy, r, s.r, s.g, s.b, (uint8_t)(std::min(1.f, s.alpha) * 190));
    }
    for(size_t i = 0; i < sparks.size(); i++){
        const Spark& s = sparks[i];
        int sx, sy, px, py;
        cam.toScreen((float)s.p.x, (float)s.p.y, sx, sy);
        cam.toScreen((float)(s.p.x - s.v.x * 0.02), (float)(s.p.y - s.v.y * 0.02), px, py);
        float t = 1.f - s.life / s.max;
        C.line(px, py, sx, sy, ((uint32_t)s.r << 16) | ((uint32_t)s.g << 8) | s.b, 1);
        C.disc(sx, sy, t > 0.6f ? 2 : 1, s.r, s.g, s.b, (uint8_t)(255 * t));
    }
}

void Game::drawHud(Canvas& C,float fps){
    char buf[256];
    C.rect(0, 0, C.w, 24, 0x0E0E14, 215);
    std::snprintf(buf, sizeof buf,
        "CAR CRASH 2D   MAP %d   WRECKS %d   FRAGMENTS %d   BODIES %d   AWAKE %d   CONTACTS %d",
        map + 1, wrecks, fragments, (int)world.bodyCount(), (int)world.awakeCount(), lastContacts);
    C.text(6, 5, buf, 0xE8E8F0, 1);
    std::snprintf(buf, sizeof buf, "D3D9  FPS %.0f  STEP %.2f MS", fps, stepMs);
    C.text(C.w - C.textW(buf, 1) - 8, 5, buf, 0x9AE89A, 1);

    Car* p = playerCar();
    if(p){
        C.rect(0, C.h - 72, 236, 72, 0x0E0E14, 205);
        C.text(8, C.h - 66, p->spec->name, 0xFFD24A, 1);
        const char* stt = p->dead ? "WRECKED" : (p->fire > 0.f ? "ON FIRE" : (p->health < 0.4f ? "DAMAGED" : "OK"));
        C.text(8 + C.textW(p->spec->name, 1) + 10, C.h - 66, stt,
               p->dead ? 0xE04030 : (p->fire > 0.f ? 0xFF8020 : 0x60D060), 1);
        const char* labels[4] = { "HP", "NOS", "FUEL", "HEAT" };
        float vals[4] = { p->health, p->nitro, p->fuel, p->heat };
        uint32_t cols[4] = { 0x30D050, 0x4878FF, 0xFF9A20, 0xE0402A };
        for(int i = 0; i < 4; i++){
            int y = C.h - 52 + i * 12;
            C.text(8, y, labels[i], 0x9098A8, 1);
            C.rect(40, y, 180, 8, 0x22222C, 255);
            C.rect(40, y, (int)(180 * std::max(0.f, std::min(1.f, vals[i]))), 8, cols[i], 255);
        }
        RigidBody* cb = world.body(p->chassis);
        if(cb){
            float kmh = (float)std::sqrt(cb->velocity.x * cb->velocity.x + cb->velocity.y * cb->velocity.y) * 3.6f;
            std::snprintf(buf, sizeof buf, "%3.0f", kmh);
            C.text(C.w - C.textW(buf, 4) - 70, C.h - 52, buf, kmh > 140.f ? 0xFF5030 : 0xFFFFFF, 4);
            C.text(C.w - 58, C.h - 24, "KM/H", 0x9098A8, 1);
        }
        if(p->dead)             C.text(C.w / 2 - C.textW("WRECKED - PRESS R", 3) / 2, C.h / 2 - 20, "WRECKED - PRESS R", 0xE04030, 3);
        else if(p->fuel <= 0.f) C.text(C.w / 2 - C.textW("OUT OF FUEL", 2) / 2, C.h / 2, "OUT OF FUEL", 0xFF9A20, 2);
    }
    if(paused) C.text(C.w / 2 - C.textW("PAUSED", 3) / 2, C.h / 2 - 60, "PAUSED", 0xFFFFFF, 3);
    if(slowmo) C.text(C.w - 90, 30, "SLOW MO", 0x80C0FF, 1);
    C.text(6, C.h - 12,
        "W GAS  S BRAKE  A D LEAN  SHIFT NOS  SPACE HANDBRAKE  E BOOM  RMB DRAG  1 2 3 MAP  R RESET  T SLOWMO  F DEBUG  P PAUSE",
        0x707888, 1);
}

static void renderFrame(Game& game,Canvas& C,Camera& cam,float fps){
    for(int y = 0; y < C.h; y++){
        float t = (float)y / (float)std::max(1, C.h);
        uint32_t col = ((uint32_t)(18 + t * 44) << 16) | ((uint32_t)(20 + t * 40) << 8) | (uint32_t)(34 + t * 58);
        uint32_t* row = &C.px[(size_t)y * C.w];
        for(int x = 0; x < C.w; x++) row[x] = col;
    }
    game.drawWorld(C, cam);
    game.drawBlood(C, cam);
    for(size_t i = 0; i < game.cars.size(); i++)
        if(!game.cars[i].dead) game.drawCar(C, cam, game.cars[i]);
    game.drawFx(C, cam);
    game.drawHud(C, fps);
}

#if defined(CARCRASH_HEADLESS)
// ------------------------------------------------------- console test build
int main(int argc,char** argv){
    int mapId = 0, steps = 600;
    for(int i = 1; i < argc; i++){
        if(!std::strcmp(argv[i], "--map")   && i + 1 < argc) mapId = std::atoi(argv[++i]);
        else if(!std::strcmp(argv[i], "--steps") && i + 1 < argc) steps = std::atoi(argv[++i]);
    }
    Game game;
    game.init(mapId);
    Canvas canvas; canvas.resize(640, 360);
    Camera cam; cam.W = 640; cam.H = 360; cam.ppm = 18.f;
    const float dt = 1.f / 60.f;
    float worst = 0.f, total = 0.f;
    for(int s = 0; s < steps; s++){
        Car* p = game.playerCar();
        if(p && p->alive){
            bool gas   = (s % 240) < 170;
            bool boost = ((s % 240) > 60 && (s % 240) < 95);
            game.drive(*p, gas, !gas, false, false, boost, false, dt);
        }
        game.step(dt);
        total += game.stepMs;
        if(game.stepMs > worst) worst = game.stepMs;
        cam.cx = game.camX; cam.cy = 6.f;
        renderFrame(game, canvas, cam, 60.f);
        if(s > 0 && s % 150 == 0){
            int nf = 0;
            std::vector<RigidBody*>& bs = game.world.bodies();
            for(size_t i = 0; i < bs.size(); i++)
                if(bs[i] && (!bs[i]->position.isFinite() || !bs[i]->velocity.isFinite())) nf++;
            BloodStats bst = game.blood.stats();
            std::printf("step %4d  bodies %3d  awake %3d  contacts %4d  frag %3d  wrecks %d  culled %d  sparks %4d  blood %4d/%3d  nf %d  %.2f ms\n",
                        s, (int)game.world.bodyCount(), (int)game.world.awakeCount(), game.lastContacts,
                        game.fragments, game.wrecks, game.culled, (int)game.sparks.size(),
                        (int)bst.droplets, (int)bst.decals, nf, game.stepMs);
        }
    }
    int nf = 0;
    std::vector<RigidBody*>& bs = game.world.bodies();
    for(size_t i = 0; i < bs.size(); i++)
        if(bs[i] && (!bs[i]->position.isFinite() || !bs[i]->velocity.isFinite())) nf++;
    std::printf("map %d done: wrecks %d  fragments %d  culled %d  non-finite %d  avg %.2f ms  worst %.2f ms\n",
                mapId + 1, game.wrecks, game.fragments, game.culled, nf, total / steps, worst);
    return nf ? 1 : 0;
}

#elif defined(_WIN32)
// =========================================================================
//  Direct3D 9 presentation. There is no GDI here: no HDC, no BITMAPINFO,
//  no StretchDIBits, no GDI brushes or fonts. The software canvas is copied
//  into a D3D9 dynamic texture and drawn as one pre-transformed quad.
// =========================================================================
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d9.h>
#undef min
#undef max

struct QuadVtx { float x, y, z, rhw, u, v; };
static const DWORD QUAD_FVF = D3DFVF_XYZRHW | D3DFVF_TEX1;

struct D3DView {
    IDirect3D9*           d3d = nullptr;
    IDirect3DDevice9*     dev = nullptr;
    IDirect3DTexture9*    tex = nullptr;
    D3DPRESENT_PARAMETERS pp;
    int  w = 0, h = 0;
    bool ok = false;

    bool create(HWND hwnd,int W,int H){
        w = W; h = H;
        d3d = Direct3DCreate9(D3D_SDK_VERSION);
        if(!d3d) return false;
        std::memset(&pp, 0, sizeof pp);
        pp.Windowed             = TRUE;
        pp.SwapEffect           = D3DSWAPEFFECT_DISCARD;
        pp.BackBufferFormat     = D3DFMT_X8R8G8B8;
        pp.BackBufferWidth      = (UINT)W;
        pp.BackBufferHeight     = (UINT)H;
        pp.hDeviceWindow        = hwnd;
        pp.PresentationInterval = D3DPRESENT_INTERVAL_ONE;
        // FPU_PRESERVE matters: the engine runs on double precision.
        DWORD flags = D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE;
        HRESULT hr = d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hwnd, flags, &pp, &dev);
        if(FAILED(hr)){
            flags = D3DCREATE_SOFTWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE;
            hr = d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hwnd, flags, &pp, &dev);
        }
        if(FAILED(hr) || !dev) return false;
        if(!makeTexture()) return false;
        applyStates();
        ok = true;
        return true;
    }
    bool makeTexture(){
        if(!dev) return false;
        return SUCCEEDED(dev->CreateTexture((UINT)w, (UINT)h, 1, D3DUSAGE_DYNAMIC,
                                            D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &tex, NULL));
    }
    void applyStates(){
        if(!dev) return;
        dev->SetRenderState(D3DRS_LIGHTING, FALSE);
        dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
        dev->SetRenderState(D3DRS_ZENABLE, FALSE);
        dev->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
        dev->SetSamplerState(0, D3DSAMP_MINFILTER, D3DTEXF_POINT);
        dev->SetSamplerState(0, D3DSAMP_MAGFILTER, D3DTEXF_POINT);
        dev->SetSamplerState(0, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
        dev->SetSamplerState(0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
        dev->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1);
        dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
        dev->SetTextureStageState(0, D3DTSS_ALPHAOP,   D3DTOP_SELECTARG1);
        dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
    }
    void upload(const Canvas& C){
        if(!tex || C.w != w || C.h != h) return;
        D3DLOCKED_RECT lr;
        if(FAILED(tex->LockRect(0, &lr, NULL, D3DLOCK_DISCARD))) return;
        uint8_t* base = (uint8_t*)lr.pBits;
        for(int y = 0; y < h; y++){
            uint32_t* dst = (uint32_t*)(base + (size_t)y * lr.Pitch);
            const uint32_t* src = &C.px[(size_t)y * w];
            for(int x = 0; x < w; x++) dst[x] = 0xFF000000u | src[x];
        }
        tex->UnlockRect(0);
    }
    void frame(const Canvas& C){
        if(!ok || !dev) return;
        HRESULT co = dev->TestCooperativeLevel();
        if(co == D3DERR_DEVICELOST){ Sleep(8); return; }
        if(co == D3DERR_DEVICENOTRESET){
            if(tex){ tex->Release(); tex = nullptr; }
            if(FAILED(dev->Reset(&pp))) return;
            if(!makeTexture()) return;
            applyStates();
        }
        upload(C);
        dev->Clear(0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0, 0, 0), 1.f, 0);
        if(SUCCEEDED(dev->BeginScene())){
            dev->SetFVF(QUAD_FVF);
            dev->SetTexture(0, tex);
            const float fw = (float)w, fh = (float)h;
            QuadVtx v[4] = {
                { -0.5f,      -0.5f,      0.f, 1.f, 0.f, 0.f },
                { fw - 0.5f,  -0.5f,      0.f, 1.f, 1.f, 0.f },
                { -0.5f,      fh - 0.5f,  0.f, 1.f, 0.f, 1.f },
                { fw - 0.5f,  fh - 0.5f,  0.f, 1.f, 1.f, 1.f },
            };
            dev->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, v, sizeof(QuadVtx));
            dev->SetTexture(0, NULL);
            dev->EndScene();
        }
        dev->Present(NULL, NULL, NULL, NULL);
    }
    void destroy(){
        if(tex){ tex->Release(); tex = nullptr; }
        if(dev){ dev->Release(); dev = nullptr; }
        if(d3d){ d3d->Release(); d3d = nullptr; }
        ok = false;
    }
};

static Input g_in;
static bool  g_running = true;

static LRESULT CALLBACK WndProc(HWND hw,UINT m,WPARAM wp,LPARAM lp){
    switch(m){
    case WM_CLOSE: case WM_DESTROY: g_running = false; PostQuitMessage(0); return 0;
    case WM_ERASEBKGND:  return 1;                  // D3D owns the client area
    case WM_PAINT:       ValidateRect(hw, NULL); return 0;
    case WM_KEYDOWN:     if(wp < 256) g_in.key[wp] = true;  return 0;
    case WM_KEYUP:       if(wp < 256) g_in.key[wp] = false; return 0;
    case WM_LBUTTONDOWN: g_in.lmb = true;  return 0;
    case WM_LBUTTONUP:   g_in.lmb = false; return 0;
    case WM_RBUTTONDOWN: g_in.rmb = true;  return 0;
    case WM_RBUTTONUP:   g_in.rmb = false; return 0;
    case WM_MOUSEMOVE:   g_in.mx = (short)LOWORD(lp); g_in.my = (short)HIWORD(lp); return 0;
    case WM_MOUSEWHEEL:  g_in.wheel += (float)GET_WHEEL_DELTA_WPARAM(wp) / 120.f; return 0;
    }
    return DefWindowProcA(hw, m, wp, lp);
}

int WINAPI WinMain(HINSTANCE hInst,HINSTANCE,LPSTR,int){
    const int W = 1360, H = 768;
    WNDCLASSA wc;
    std::memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hInst;
    wc.lpszClassName = "carCrash2dWindow";
    wc.hCursor       = LoadCursorA(NULL, IDC_ARROW);
    wc.hbrBackground = NULL;                        // no GDI brush
    RegisterClassA(&wc);

    RECT rc = { 0, 0, W, H };
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
    HWND hwnd = CreateWindowA("carCrash2dWindow", "CAR CRASH 2D - phys2d - Direct3D 9",
                              WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
                              rc.right - rc.left, rc.bottom - rc.top, NULL, NULL, hInst, NULL);
    if(!hwnd) return 1;
    ShowWindow(hwnd, SW_SHOW);

    D3DView view;
    if(!view.create(hwnd, W, H)){
        MessageBoxA(hwnd, "Direct3D 9 device could not be created.", "CAR CRASH 2D", MB_ICONERROR);
        return 2;
    }

    Canvas C; C.resize(W, H);
    Camera cam; cam.W = W; cam.H = H; cam.ppm = 34.f;
    Game game; game.init(0);

    std::chrono::steady_clock::time_point prev = std::chrono::steady_clock::now();
    float fps = 60.f;

    while(g_running){
        MSG msg;
        while(PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)){ TranslateMessage(&msg); DispatchMessageA(&msg); }

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - prev).count();
        prev = now;
        dt = (float)RC(dt, 1.0 / 600.0, 1.0 / 20.0);
        fps = fps * 0.9f + 0.1f / dt;

        if(g_in.pressed('1')) game.init(0);
        if(g_in.pressed('2')) game.init(1);
        if(g_in.pressed('3')) game.init(2);
        if(g_in.pressed('R')) game.init(game.map);
        if(g_in.pressed('P')) game.paused    = !game.paused;
        if(g_in.pressed('T')) game.slowmo    = !game.slowmo;
        if(g_in.pressed('F')) game.debugDraw = !game.debugDraw;
        if(g_in.pressed(VK_ESCAPE)) g_running = false;
        if(g_in.wheel != 0.f) cam.zoom(g_in.wheel > 0.f ? 1.12f : 0.89f);

        Car* p = game.playerCar();
        if(p && p->alive)
            game.drive(*p,
                       g_in.key['W'] || g_in.key[VK_UP],
                       g_in.key['S'] || g_in.key[VK_DOWN],
                       g_in.key['A'] || g_in.key[VK_LEFT],
                       g_in.key['D'] || g_in.key[VK_RIGHT],
                       g_in.key[VK_SHIFT], g_in.key[VK_SPACE], dt);

        if(g_in.pressed('E')){
            float wx, wy; cam.toWorld(g_in.mx, g_in.my, wx, wy);
            Boom bm; bm.c = Vec2((real)wx, (real)wy); bm.power = 320.f; bm.radius = 9.f;
            game.booms.push_back(bm);
        }
        if(g_in.rmb){
            float wx, wy; cam.toWorld(g_in.mx, g_in.my, wx, wy);
            RigidBody* b = game.world.queryPoint(Vec2((real)wx, (real)wy));
            if(b && b->type == BodyType::Dynamic){
                b->velocity = Vec2(((real)wx - b->position.x) * 9.0, ((real)wy - b->position.y) * 9.0);
                b->wake();
            }
        }

        game.step(dt);
        g_in.newFrame();

        if(p){
            RigidBody* cb = game.world.body(p->chassis);
            if(cb && cb->position.isFinite()){
                game.camX += ((float)cb->position.x - game.camX) * std::min(1.f, dt * 7.f);
                game.camY += (((float)cb->position.y + 2.2f) - game.camY) * std::min(1.f, dt * 5.f);
            }
        }
        cam.cx = game.camX + rnd(-1.f, 1.f) * game.shake * 0.35f;
        cam.cy = game.camY + rnd(-1.f, 1.f) * game.shake * 0.35f;

        renderFrame(game, C, cam, fps);
        view.frame(C);
    }
    view.destroy();
    return 0;
}

#else
int main(){ std::printf("build with -DCARCRASH_HEADLESS for the console test\n"); return 0; }
#endif
