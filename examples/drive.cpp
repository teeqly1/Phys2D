// ===========================================================================
//  PHYS2D DRIVE - car + track. No explosions, no damage, no gore.
//  Rendering: Direct3D 9 only (no GDI). Console test: -DDRIVE_HEADLESS.
//  Maps: 1 free road | 2 bumps (suspension test) | 3 crate yard.
//  Suspension: custom sliding strut solved in code - wheels cannot fall off.
// ===========================================================================
#include "phys2d/World.h"
#include "phys2d/Body.h"
#include "phys2d/Shape.h"
#include "phys2d/Constraints.h"
#include "phys2d/Collision.h"
#include "phys2d/Extras.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <chrono>

using namespace phys2d;

static uint32_t g_rng = 0x5EED1234u;
static inline float rnd01(){
    g_rng = g_rng * 1664525u + 1013904223u;
    return (float)((g_rng >> 8) & 0xFFFFFFu) / (float)0xFFFFFF;
}
static inline float rnd(float a,float b){ return a + (b - a) * rnd01(); }
static inline double RC(double v,double lo,double hi){ return v < lo ? lo : (v > hi ? hi : v); }

struct Canvas {
    int w = 0, h = 0;
    std::vector<uint32_t> px;

    void resize(int W,int H){ w = W; h = H; px.assign((size_t)W * H, 0u); }
    void clear(uint32_t c){ std::fill(px.begin(), px.end(), c); }

    inline void blend(int x,int y,uint8_t r,uint8_t g,uint8_t b,uint8_t a){
        if(x < 0 || y < 0 || x >= w || y >= h || a == 0) return;
        uint32_t& d = px[(size_t)y * w + x];
        if(a == 255){ d = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; return; }
        uint32_t dr = (d >> 16) & 255, dg = (d >> 8) & 255, db = d & 255;
        uint32_t ia = 255u - a;
        d = (((r * a + dr * ia) / 255u) << 16) | (((g * a + dg * ia) / 255u) << 8) | ((b * a + db * ia) / 255u);
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
    void ring(int cx,int cy,int rad,int thick,uint32_t c,uint8_t a = 255){
        if(rad <= 0 || rad > 4000) return;
        uint8_t r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
        int inr = std::max(0, rad - thick);
        int outer = rad * rad, inner = inr * inr;
        for(int y = -rad; y <= rad; y++){
            int yy = cy + y;
            if(yy < 0 || yy >= h) continue;
            for(int x = -rad; x <= rad; x++){
                int d2 = x * x + y * y;
                if(d2 <= outer && d2 >= inner) blend(cx + x, yy, r, g, b, a);
            }
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

struct Camera {
    float cx = 0.f, cy = 5.f, ppm = 34.f;
    int   W = 1360, H = 768;
    void toScreen(float wx,float wy,int& sx,int& sy) const {
        sx = (int)RC((double)(wx - cx) * ppm + W * 0.5, -1e6, 1e6);
        sy = (int)RC(H * 0.5 - (double)(wy - cy) * ppm, -1e6, 1e6);
    }
    void zoom(float k){ ppm = (float)RC(ppm * k, 8.0, 220.0); }
    int  m2p(float m) const { return (int)RC((double)m * ppm, 0.0, 4000.0); }
};

struct Input {
    bool  key[256], prev[256];
    int   mx = 0, my = 0;
    float wheel = 0.f;
    bool  lmb = false, rmb = false;
    Input(){ std::memset(key, 0, sizeof key); std::memset(prev, 0, sizeof prev); }
    void newFrame(){ std::memcpy(prev, key, sizeof key); wheel = 0.f; }
    bool pressed(int k) const { return k >= 0 && k < 256 && key[k] && !prev[k]; }
};

struct Dust { Vec2 p, v; float life = 0.f, max = 1.f, radius = 0.2f; uint8_t gray = 190; float alpha = 0.6f; };
struct Skid { Vec2 p; float life = 0.f, max = 8.f, strength = 1.f; };

struct CarSpec {
    const char* name;
    float cw, ch;        // chassis half size, m
    float wr;            // wheel radius, m
    float mass;          // kg
    float driveAccel;    // m/s^2 at full throttle
    float topMs;         // top speed, m/s
    float brakeAccel;    // m/s^2 braking
    float travel;        // total suspension travel, m
    float grip;
    uint32_t bodyCol, roofCol, rimCol;
    int driveWheels;     // 1 = rear, 2 = all
};

static const CarSpec SPECS[5] = {
    { "SEDAN",   1.70f, 0.42f, 0.34f, 1100.f,  7.0f, 46.f, 11.0f, 0.30f, 1.00f, 0x2C6EC8, 0x4A90E2, 0xC8C8D0, 1 },
    { "SPORT",   1.55f, 0.32f, 0.30f,  850.f,  9.5f, 58.f, 13.0f, 0.22f, 1.15f, 0xD81C1C, 0xF03A3A, 0xE8C24A, 1 },
    { "OFFROAD", 1.85f, 0.50f, 0.52f, 1900.f,  6.2f, 36.f,  9.5f, 0.46f, 1.25f, 0x2F8B4A, 0x3FA65D, 0x8C8C94, 2 },
    { "MONSTER", 1.95f, 0.55f, 0.72f, 2400.f,  5.6f, 32.f,  9.0f, 0.62f, 1.35f, 0x5B2080, 0x7A2FA6, 0x2A2A2E, 2 },
    { "TRUCK",   2.40f, 0.62f, 0.46f, 3200.f,  4.2f, 28.f,  8.0f, 0.36f, 1.05f, 0xE0A020, 0xF0BE40, 0x9A9AA2, 2 },
};

// One strut. The wheel is a real dynamic body, but it slides along the chassis
// axis instead of hanging on joints, so it can never be pulled off the car.
struct Wheel {
    BodyId id = INVALID_BODY;
    Vec2   localAnchor;
    float  restLen = 0.4f;
    float  minLen  = 0.2f;
    float  maxLen  = 0.5f;
    float  len     = 0.4f;
    float  vel     = 0.f;
    float  comp    = 0.f;    // 0 = drooped, 1 = bottomed out
    float  load    = 0.f;
    float  spin    = 0.f;
    float  slip    = 0.f;
    bool   driven  = false;
    bool   ground  = false;
};

struct Car {
    const CarSpec* spec = &SPECS[0];
    int    kind = 0;
    BodyId chassis = INVALID_BODY;
    Wheel  wheels[2];
    float  springK = 30000.f, springC = 3000.f;
    float  engineRpm = 0.f, throttle = 0.f, airTime = 0.f;
    float  odo = 0.f, topSpeed = 0.f, bestAir = 0.f;
};

struct Game {
    World world;
    CcdSystem ccd;

    Car  car;
    std::vector<Dust> dust;
    std::vector<Skid> skids;
    std::unordered_map<BodyId,int> owner;
    std::unordered_map<BodyId,int> propKind;

    int   map = 0, carKind = 0;
    float time = 0.f, timeScale = 1.f;
    bool  paused = false, slowmo = false, debugDraw = false;
    float camX = 0.f, camY = 5.f, stepMs = 0.f;
    float trackMin = -150.f, trackMax = 150.f;
    int   lastContacts = 0;
    float spawnX = 0.f;
    bool  wheelTouch[2];
    float wheelImpulse[2];

    Game(){ wheelTouch[0] = wheelTouch[1] = false; wheelImpulse[0] = wheelImpulse[1] = 0.f; }

    float  groundAt(float x) const;
    void   init(int mapId,int kind);
    void   buildMap(int mapId);
    BodyId crate(Vec2 pos,float w,float h,float density,int kind);
    void   spawnCar(int kind,float x);
    void   respawn();
    Vec2   strutTop(int i) const;
    Vec2   strutAxis() const;
    void   suspensionForces(float dt);
    void   suspensionProject();
    void   drive(bool gas,bool brake,bool leanL,bool leanR,bool handbrake,float dt);
    void   puff(Vec2 p,Vec2 v,float radius,uint8_t gray,float alpha);
    void   enforceLimits();
    void   step(float dt);
    void   drawWorld(Canvas& C,const Camera& cam);
    void   drawCar(Canvas& C,const Camera& cam);
    void   drawFx(Canvas& C,const Camera& cam);
    void   drawHud(Canvas& C,float fps);
};

float Game::groundAt(float x) const {
    if(map == 0) return 1.15f * std::sin(x * 0.055f) + 0.55f * std::sin(x * 0.13f + 0.7f);
    return 0.f;
}

static BodyId staticBox(World& w,Vec2 pos,float hw,float hh,float angle,float friction){
    BodyDef bd;
    bd.type = BodyType::Static;
    bd.shape = Shape::box((real)(hw * 2.f), (real)(hh * 2.f));
    bd.position = pos;
    bd.angle = (real)angle;
    bd.material.staticFriction  = (real)friction;
    bd.material.dynamicFriction = (real)(friction * 0.82f);
    bd.material.restitution     = 0.04;
    return w.createBody(bd);
}

BodyId Game::crate(Vec2 pos,float w,float h,float density,int kind){
    BodyDef bd;
    bd.type = BodyType::Dynamic;
    bd.position = pos;
    if(kind == 3) bd.shape = Shape::circle((real)(h * 0.5f));
    else          bd.shape = Shape::box((real)w, (real)h);
    bd.material.density         = (real)density;
    bd.material.restitution     = 0.06;
    bd.material.staticFriction  = 0.78;
    bd.material.dynamicFriction = 0.62;
    bd.material.rollingFriction = (kind == 3) ? 0.06 : 0.02;
    bd.material.linearDrag      = 0.03;
    bd.material.quadraticDrag   = 0.02;
    bd.material.hysteresis      = 0.22;
    BodyId id = world.createBody(bd);
    propKind[id] = kind;
    return id;
}

void Game::buildMap(int mapId){
    if(mapId == 0){
        trackMin = -160.f; trackMax = 160.f;
        for(int i = 0; i < 290; i++){
            float x0 = -162.f + i * 1.12f, x1 = x0 + 1.12f;
            float y0 = groundAt(x0), y1 = groundAt(x1);
            float ang = std::atan2(y1 - y0, x1 - x0);
            staticBox(world, Vec2((real)((x0 + x1) * 0.5f), (real)((y0 + y1) * 0.5f - 0.7f)), 0.70f, 0.70f, ang, 0.95f);
        }
        staticBox(world, Vec2(-164, 6.0), 1.0f, 7.0f, 0.f, 0.6f);
        staticBox(world, Vec2( 164, 6.0), 1.0f, 7.0f, 0.f, 0.6f);
        const float rx[4] = { -95.f, -20.f, 45.f, 110.f };
        const float ra[4] = { 0.22f, 0.26f, 0.20f, 0.28f };
        for(int i = 0; i < 4; i++)
            staticBox(world, Vec2((real)rx[i], (real)(groundAt(rx[i]) + 0.70f)), 5.0f, 0.30f, ra[i], 0.95f);
        for(int i = 0; i < 26; i++){
            float x = -140.f + i * 11.f;
            crate(Vec2((real)x, (real)(groundAt(x) + 0.35f)), 0.34f, 0.50f, 120.f, 1);
        }
    } else if(mapId == 1){
        trackMin = -125.f; trackMax = 125.f;
        staticBox(world, Vec2(0, -0.8), 128.f, 0.8f, 0.f, 0.98f);
        staticBox(world, Vec2(-128, 7.0), 1.0f, 8.0f, 0.f, 0.6f);
        staticBox(world, Vec2( 128, 7.0), 1.0f, 8.0f, 0.f, 0.6f);
        for(int i = 0; i < 34; i++)
            staticBox(world, Vec2((real)(-100.f + i * 1.20f), -0.02), 0.09f, 0.09f, 0.7854f, 0.98f);
        for(int i = 0; i < 20; i++)
            staticBox(world, Vec2((real)(-50.f + i * 2.00f), -0.05), 0.18f, 0.18f, 0.7854f, 0.98f);
        const float bh[4] = { 0.14f, 0.22f, 0.32f, 0.44f };
        for(int g = 0; g < 4; g++)
            for(int i = 0; i < 5; i++){
                float x = 6.f + g * 16.f + i * 2.6f;
                staticBox(world, Vec2((real)x, (real)(-bh[g] * 0.35f)), bh[g], bh[g], 0.7854f, 0.98f);
            }
        staticBox(world, Vec2(84.0, -0.06), 0.22f, 0.22f, 0.7854f, 0.98f);
        staticBox(world, Vec2(104.0, 0.62), 4.20f, 0.28f, 0.15f, 0.95f);
        for(int i = 0; i < 12; i++)
            crate(Vec2((real)(-110.f + i * 18.f), 0.35), 0.34f, 0.50f, 120.f, 1);
    } else {
        trackMin = -110.f; trackMax = 110.f;
        staticBox(world, Vec2(0, -0.8), 112.f, 0.8f, 0.f, 0.95f);
        staticBox(world, Vec2(-112, 6.0), 1.0f, 7.0f, 0.f, 0.6f);
        staticBox(world, Vec2( 112, 6.0), 1.0f, 7.0f, 0.f, 0.6f);
        for(int st = 0; st < 7; st++)
            for(int k = 0; k <= st; k++)
                crate(Vec2((real)(-70.f + st * 1.05f), (real)(0.5f + k * 1.02f)), 1.0f, 1.0f, 260.f, 2);
        for(int wallI = 0; wallI < 3; wallI++){
            float bx = -30.f + wallI * 26.f;
            for(int cy = 0; cy < 5; cy++)
                for(int cx = 0; cx < 3; cx++)
                    crate(Vec2((real)(bx + cx * 0.92f), (real)(0.45f + cy * 0.92f)), 0.88f, 0.88f, 210.f, 0);
        }
        for(int i = 0; i < 5; i++)  crate(Vec2((real)(60.f + i * 3.2f), 0.75), 1.4f, 1.4f, 900.f, 0);
        for(int i = 0; i < 14; i++) crate(Vec2((real)(-95.f + i * 4.0f), 0.5), 0.8f, 0.8f, 90.f, 3);
        for(int i = 0; i < 3; i++)
            staticBox(world, Vec2((real)(-34.f + i * 26.f), 0.35), 2.6f, 0.20f, 0.30f, 0.95f);
    }
}

Vec2 Game::strutAxis() const {
    const RigidBody* cb = world.body(car.chassis);
    float a = cb ? (float)cb->angle : 0.f;
    return Vec2((real)std::sin(a), (real)(-std::cos(a)));
}

Vec2 Game::strutTop(int i) const {
    const RigidBody* cb = world.body(car.chassis);
    if(!cb) return Vec2(0, 0);
    const Wheel& wh = car.wheels[i];
    float a = (float)cb->angle, cs = std::cos(a), sn = std::sin(a);
    return Vec2(cb->position.x + wh.localAnchor.x * cs - wh.localAnchor.y * sn,
                cb->position.y + wh.localAnchor.x * sn + wh.localAnchor.y * cs);
}

void Game::spawnCar(int kind,float x){
    kind = (int)RC(kind, 0, 4);
    const CarSpec& sp = SPECS[kind];
    car = Car();
    car.kind = kind;
    car.spec = &SPECS[kind];

    // 60% of the travel is compression, 40% droop; the ride height keeps the
    // chassis clear of the road even with the strut fully bottomed out
    float downT = sp.travel * 0.60f;
    float upT   = sp.travel * 0.40f;
    float restLen = sp.ch * 0.35f + downT + 0.06f;
    float baseY = groundAt(x) + restLen + sp.wr + 0.30f;

    // spring rate follows the mass: the car settles at ~35% of the compression
    // travel under its own weight, so it can never ride on the bump stop
    car.springK = 2.6f * (sp.mass * 9.81f * 0.5f) / std::max(0.05f, 0.35f * downT);
    car.springC = 0.75f * 2.f * std::sqrt(car.springK * sp.mass * 0.5f);

    BodyDef bd;
    bd.type = BodyType::Dynamic;
    bd.shape = Shape::box((real)(sp.cw * 2.f), (real)(sp.ch * 2.f));
    bd.position = Vec2((real)x, (real)baseY);
    bd.material.density         = (real)(sp.mass / (sp.cw * 2.f * sp.ch * 2.f));
    bd.material.restitution     = 0.05;
    bd.material.staticFriction  = 0.35;
    bd.material.dynamicFriction = 0.25;
    bd.material.linearDrag      = 0.04;
    bd.material.quadraticDrag   = 0.05;
    bd.material.hysteresis      = 0.18;
    bd.name = sp.name;
    car.chassis = world.createBody(bd);
    owner[car.chassis] = 1;

    for(int i = 0; i < 2; i++){
        float ax = (i == 0) ? -sp.cw * 0.72f : sp.cw * 0.72f;
        Wheel wh;
        wh.localAnchor = Vec2((real)ax, (real)(-sp.ch * 0.35f));
        wh.restLen = restLen;
        wh.maxLen  = restLen + upT;
        wh.minLen  = std::max(0.06f, restLen - downT);
        wh.len     = restLen;
        wh.driven  = (sp.driveWheels == 2) || (i == 1);

        BodyDef wd;
        wd.type = BodyType::Dynamic;
        wd.shape = Shape::circle((real)sp.wr);
        wd.position = Vec2((real)(x + ax), (real)(baseY - restLen));
        wd.material.density         = 190.0;
        wd.material.restitution     = 0.02;
        // the wheel is locked to the strut, so a high friction value would just
        // scrub along the road and stall the car; drive and brake are forces
        wd.material.staticFriction  = 0.30;
        wd.material.dynamicFriction = 0.20;
        wd.material.rollingFriction = 0.0;
        wd.material.linearDrag      = 0.0;
        wd.material.quadraticDrag   = 0.0;
        wd.material.hysteresis      = 0.20;
        wd.fixedRotation = true;      // spin is drawn, not simulated
        wd.allowSleep    = false;
        wh.id = world.createBody(wd);
        owner[wh.id] = 1;
        car.wheels[i] = wh;
    }
    spawnX = x;
}

void Game::init(int mapId,int kind){
    WorldConfig cfg;
    cfg.solver.velocityIterations   = 18;
    cfg.solver.positionIterations   = 12;
    cfg.solver.baumgarte            = 0.22;
    cfg.solver.restitutionThreshold = 1.20;
    cfg.broadPhase   = BroadPhaseMode::Hybrid;
    cfg.gridCellSize = 2.2;
    cfg.aabbMargin   = 0.08;
    cfg.sleepTime    = 0.7;
    cfg.threadCount  = 0;
    world = World(cfg);

    dust.clear(); skids.clear(); owner.clear(); propKind.clear();
    map = mapId; carKind = kind;
    time = 0.f; stepMs = 0.f; lastContacts = 0;
    paused = false; slowmo = false;
    wheelTouch[0] = wheelTouch[1] = false;
    wheelImpulse[0] = wheelImpulse[1] = 0.f;

    ccd.attach(world);
    ccd.speedThreshold = 8.0;
    ccd.maxSubsteps    = 6;

    world.setContactFilter([this](const RigidBody& a,const RigidBody& b) -> bool {
        return !(owner.find(a.id) != owner.end() && owner.find(b.id) != owner.end());
    });
    std::function<void(const CollisionEvent&)> onContact = [this](const CollisionEvent& e){
        for(int i = 0; i < 2; i++){
            BodyId id = car.wheels[i].id;
            if(id == INVALID_BODY) continue;
            if((e.a && e.a->id == id) || (e.b && e.b->id == id)){
                wheelTouch[i] = true;
                wheelImpulse[i] += (float)e.normalImpulse;
            }
        }
    };
    world.setBeginContactCallback(onContact);
    world.setPersistContactCallback(onContact);

    buildMap(mapId);
    float sx = (mapId == 1) ? -118.f : ((mapId == 2) ? -100.f : trackMin + 22.f);
    spawnCar(kind, sx);
    camX = sx; camY = groundAt(sx) + 3.f;
}

void Game::respawn(){
    RigidBody* cb = world.body(car.chassis);
    float x = (cb && cb->position.isFinite()) ? (float)cb->position.x : spawnX;
    x = (float)RC(x, trackMin + 6.f, trackMax - 6.f);
    std::vector<BodyId> old;
    old.push_back(car.chassis);
    for(int i = 0; i < 2; i++) old.push_back(car.wheels[i].id);
    for(size_t i = 0; i < old.size(); i++){
        if(old[i] == INVALID_BODY) continue;
        if(world.body(old[i])){ owner.erase(old[i]); world.destroyBody(old[i]); }
    }
    spawnCar(car.kind, x);
}

// ---------------------------------------------------------------------------
//  Suspension: spring + damper force along the strut, applied to both bodies.
// ---------------------------------------------------------------------------
void Game::suspensionForces(float dt){
    RigidBody* cb = world.body(car.chassis);
    if(!cb) return;
    const CarSpec& sp = *car.spec;
    Vec2 down = strutAxis();

    for(int i = 0; i < 2; i++){
        Wheel& wh = car.wheels[i];
        RigidBody* wb = world.body(wh.id);
        if(!wb) continue;
        Vec2 top = strutTop(i);
        Vec2 d(wb->position.x - top.x, wb->position.y - top.y);
        float len = (float)(d.x * down.x + d.y * down.y);
        len = (float)RC(len, wh.minLen, wh.maxLen);

        Vec2 topVel = cb->velocityAtPoint(top);
        Vec2 rel(wb->velocity.x - topVel.x, wb->velocity.y - topVel.y);
        float relAlong = (float)(rel.x * down.x + rel.y * down.y);

        float x = wh.restLen - len;                        // + = compressed
        float force = car.springK * x - car.springC * relAlong;
        if(len < wh.minLen + 0.04f) force += car.springK * 6.f * (wh.minLen + 0.04f - len);
        force = (float)RC(force, -sp.mass * 30.0, sp.mass * 140.0);

        wh.len  = len;
        wh.vel  = relAlong;
        wh.load = force;
        wh.comp = (float)RC((wh.maxLen - len) / std::max(0.02f, wh.maxLen - wh.minLen), 0.0, 1.0);

        Vec2 F(down.x * (real)force, down.y * (real)force);
        wb->applyForce(F);
        cb->applyForceAtPoint(Vec2(-F.x, -F.y), top);
    }
}

// Hard part of the strut: after the solver ran, the wheel is snapped back onto
// the strut axis, so no impact can ever tear it off the car.
void Game::suspensionProject(){
    RigidBody* cb = world.body(car.chassis);
    if(!cb || !cb->position.isFinite()) return;
    Vec2 down = strutAxis();

    for(int i = 0; i < 2; i++){
        Wheel& wh = car.wheels[i];
        RigidBody* wb = world.body(wh.id);
        if(!wb) continue;
        Vec2 top = strutTop(i);
        Vec2 d(wb->position.x - top.x, wb->position.y - top.y);
        float along = (float)(d.x * down.x + d.y * down.y);
        along = (float)RC(along, wh.minLen, wh.maxLen);
        wb->position = Vec2(top.x + down.x * (real)along, top.y + down.y * (real)along);
        wh.len  = along;
        wh.comp = (float)RC((wh.maxLen - along) / std::max(0.02f, wh.maxLen - wh.minLen), 0.0, 1.0);

        Vec2 topVel = cb->velocityAtPoint(top);
        Vec2 rel(wb->velocity.x - topVel.x, wb->velocity.y - topVel.y);
        float relDown = (float)(rel.x * down.x + rel.y * down.y);
        if(along <= wh.minLen + 1e-4f && relDown < 0.f) relDown = 0.f;
        if(along >= wh.maxLen - 1e-4f && relDown > 0.f) relDown = 0.f;
        wb->velocity = Vec2(topVel.x + down.x * (real)relDown,
                            topVel.y + down.y * (real)relDown);
        wb->angularVelocity = 0.0;
        wb->angle = 0.0;
        wb->wake();
    }
}

void Game::puff(Vec2 p,Vec2 v,float radius,uint8_t gray,float alpha){
    if(dust.size() > 420 || !p.isFinite()) return;
    Dust d;
    d.p = p; d.v = v; d.life = 0.f; d.max = rnd(0.7f, 1.9f);
    d.radius = radius; d.gray = gray; d.alpha = alpha;
    dust.push_back(d);
}

void Game::drive(bool gas,bool brake,bool leanL,bool leanR,bool handbrake,float dt){
    RigidBody* cb = world.body(car.chassis);
    if(!cb) return;
    const CarSpec& sp = *car.spec;

    float ca = (float)cb->angle;
    Vec2  fwd((real)std::cos(ca), (real)std::sin(ca));
    float vfwd = (float)(cb->velocity.x * fwd.x + cb->velocity.y * fwd.y);
    float speed = (float)std::sqrt(cb->velocity.x * cb->velocity.x + cb->velocity.y * cb->velocity.y);

    car.throttle = gas ? 1.f : (brake ? -1.f : 0.f);
    cb->wake();
    for(int i = 0; i < 2; i++){ RigidBody* wb = world.body(car.wheels[i].id); if(wb) wb->wake(); }

    int drivenGrounded = 0, groundedCount = 0;
    for(int i = 0; i < 2; i++){
        Wheel& wh = car.wheels[i];
        wh.ground = wheelTouch[i] || wh.comp > 0.08f;
        if(wh.ground){ groundedCount++; if(wh.driven) drivenGrounded++; }
    }
    bool grounded = groundedCount > 0;
    car.airTime = grounded ? 0.f : (car.airTime + dt);
    if(car.airTime > car.bestAir) car.bestAir = car.airTime;

    int drivenTotal = (sp.driveWheels == 2) ? 2 : 1;
    float share = drivenTotal > 0 ? (float)drivenGrounded / (float)drivenTotal : 0.f;

    if(gas && share > 0.f){
        float fade = 1.f - (float)RC(std::fabs(vfwd) / sp.topMs, 0.0, 1.0);
        float force = sp.mass * sp.driveAccel * (0.45f + 0.55f * fade) * share * sp.grip * 3.0f;
        for(int i = 0; i < 2; i++){
            Wheel& wh = car.wheels[i];
            RigidBody* wb = world.body(wh.id);
            if(!wb || !wh.driven || !wh.ground) continue;
            cb->applyForceAtPoint(Vec2(fwd.x * (real)(force / drivenTotal), fwd.y * (real)(force / drivenTotal)),
                                  wb->position);
            wh.slip = (fade > 0.85f && speed < 4.f) ? 6.f : std::max(0.f, 4.f * fade - speed * 0.2f);
        }
    } else if(brake && grounded){
        float force = sp.mass * sp.brakeAccel;
        if(speed > 0.05f)
            cb->applyForce(Vec2(-cb->velocity.x / speed * (real)force, -cb->velocity.y / speed * (real)force));
        for(int i = 0; i < 2; i++) car.wheels[i].slip = speed > 6.f ? speed * 0.5f : 0.f;
    } else {
        for(int i = 0; i < 2; i++) car.wheels[i].slip *= 0.85f;
    }

    if(handbrake && grounded){
        cb->velocity.x *= (real)std::pow(0.02, (double)dt);
        cb->angularVelocity *= (real)std::pow(0.2, (double)dt);
        for(int i = 0; i < 2; i++) car.wheels[i].slip = speed * 0.6f;
    }
    // coasting: rolling resistance only, so the car keeps its momentum
    if(grounded && !gas && !brake && !handbrake){
        float roll = sp.mass * 0.35f;
        if(speed > 0.02f)
            cb->applyForce(Vec2(-cb->velocity.x / speed * (real)roll, -cb->velocity.y / speed * (real)roll));
    }

    for(int i = 0; i < 2; i++){
        Wheel& wh = car.wheels[i];
        RigidBody* wb = world.body(wh.id);
        float spinRate = -vfwd / std::max(0.05f, sp.wr);
        if(handbrake) spinRate = 0.f;
        else if(gas && wh.driven && wh.slip > 4.f) spinRate *= 1.7f;
        wh.spin += spinRate * dt;
        if(wh.spin >  6.2831853f) wh.spin -= 6.2831853f;
        if(wh.spin < -6.2831853f) wh.spin += 6.2831853f;

        if(wb && wh.slip > 3.5f && speed > 0.5f && wh.ground){
            if(skids.size() < 900){
                Skid sk; sk.p = wb->position; sk.strength = std::min(1.f, wh.slip * 0.09f);
                skids.push_back(sk);
            }
            if(rnd01() < 0.35f)
                puff(wb->position, Vec2((real)rnd(-1.2f, 1.2f), (real)rnd(0.3f, 1.5f)), 0.15f, 205, 0.35f);
        }
    }

    float lean = (leanL ? 1.f : 0.f) - (leanR ? 1.f : 0.f);
    if(lean != 0.f) cb->applyTorque((real)(lean * sp.mass * (grounded ? 3.0f : 9.0f)));

    car.engineRpm = car.engineRpm * 0.88f +
                    (float)(gas ? RC(0.35 + std::fabs(vfwd) / sp.topMs, 0.0, 1.0) : 0.0) * 0.12f;
}

void Game::enforceLimits(){
    std::vector<RigidBody*>& bs = world.bodies();
    for(size_t i = 0; i < bs.size(); i++){
        RigidBody* b = bs[i];
        if(!b || b->type != BodyType::Dynamic) continue;
        if(!b->position.isFinite() || !b->velocity.isFinite()){
            b->velocity = Vec2(0, 0);
            b->angularVelocity = 0.0;
            b->position = Vec2((real)RC(spawnX, trackMin + 5.0, trackMax - 5.0), (real)(groundAt(spawnX) + 3.0));
            continue;
        }
        real v2 = b->velocity.x * b->velocity.x + b->velocity.y * b->velocity.y;
        if(v2 > 160.0 * 160.0){
            real k = 160.0 / std::sqrt(v2);
            b->velocity = Vec2(b->velocity.x * k, b->velocity.y * k);
        }
        if(b->angularVelocity >  180.0) b->angularVelocity =  180.0;
        if(b->angularVelocity < -180.0) b->angularVelocity = -180.0;
    }
}

void Game::step(float dt){
    if(paused) return;
    float sdt = dt * timeScale * (slowmo ? 0.3f : 1.f);
    if(sdt <= 0.f) return;
    time += sdt;

    RigidBody* cb = world.body(car.chassis);
    Vec2 before = cb ? cb->position : Vec2(0, 0);

    for(int i = 0; i < 2; i++){ wheelTouch[i] = false; wheelImpulse[i] = 0.f; }
    enforceLimits();
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    for(int sub = 0; sub < 2; sub++){
        suspensionForces(sdt * 0.5f);
        world.step(sdt * 0.5f);
        suspensionProject();
    }
    ccd.update(sdt);
    suspensionProject();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    stepMs = std::chrono::duration<float,std::milli>(t1 - t0).count();
    lastContacts = (int)world.contactCount();
    enforceLimits();

    cb = world.body(car.chassis);
    if(cb && cb->position.isFinite()){
        Vec2 d = cb->position - before;
        car.odo += (float)std::sqrt(d.x * d.x + d.y * d.y);
        float spd = (float)std::sqrt(cb->velocity.x * cb->velocity.x + cb->velocity.y * cb->velocity.y) * 3.6f;
        if(spd < 400.f && spd > car.topSpeed) car.topSpeed = spd;
        if(cb->position.x < trackMin){ cb->position.x = (real)trackMin; cb->velocity.x =  std::fabs(cb->velocity.x) * 0.3; }
        if(cb->position.x > trackMax){ cb->position.x = (real)trackMax; cb->velocity.x = -std::fabs(cb->velocity.x) * 0.3; }
        if(cb->position.y < -25.0) respawn();
    }

    for(size_t i = 0; i < dust.size(); i++){
        Dust& d = dust[i];
        d.v.y += (real)(0.5 * sdt);
        d.p.x += d.v.x * sdt; d.p.y += d.v.y * sdt;
        d.radius += sdt * 0.35f;
        d.alpha *= (1.f - sdt * 0.95f);
        d.life += sdt;
    }
    dust.erase(std::remove_if(dust.begin(), dust.end(),
               [](const Dust& d){ return d.life >= d.max || d.alpha < 0.02f; }), dust.end());
    for(size_t i = 0; i < skids.size(); i++) skids[i].life += sdt;
    skids.erase(std::remove_if(skids.begin(), skids.end(),
                [](const Skid& s){ return s.life >= s.max; }), skids.end());
}

// ---------------------------------------------------------------------------
//  Rendering
// ---------------------------------------------------------------------------
static const char* MAPNAME[3] = { "FREE ROAD", "BUMPS - SUSPENSION TEST", "CRATE YARD" };

void Game::drawWorld(Canvas& C,const Camera& cam){
    for(int y = 0; y < C.h; y++){
        float t = (float)y / (float)std::max(1, C.h);
        uint8_t r = (uint8_t)(40 + 90 * t), g = (uint8_t)(70 + 105 * t), b = (uint8_t)(120 + 90 * t);
        C.rect(0, y, C.w, 1, ((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
    }
    for(int i = 0; i < C.w; i += 4){
        float wx = cam.cx + (i - C.w * 0.5f) / cam.ppm * 2.2f;
        float hy = 6.f + 2.6f * std::sin(wx * 0.05f) + 1.3f * std::sin(wx * 0.11f);
        int sx, sy; cam.toScreen(cam.cx + (i - C.w * 0.5f) / cam.ppm, hy, sx, sy);
        C.rect(i, sy, 4, C.h - sy, 0x38506A, 190);
    }

    for(size_t i = 0; i < skids.size(); i++){
        const Skid& s = skids[i];
        int sx, sy; cam.toScreen((float)s.p.x, (float)s.p.y, sx, sy);
        uint8_t a = (uint8_t)RC(150.0 * (1.0 - s.life / s.max) * s.strength, 0.0, 150.0);
        C.rect(sx - 2, sy, 4, 2, 0x1A1A1E, a);
    }

    std::vector<RigidBody*>& bs = world.bodies();
    for(size_t i = 0; i < bs.size(); i++){
        RigidBody* b = bs[i];
        if(!b || !b->position.isFinite()) continue;
        if(owner.find(b->id) != owner.end()) continue;

        uint32_t col = 0x54503F;
        if(b->type == BodyType::Dynamic){
            std::unordered_map<BodyId,int>::iterator it = propKind.find(b->id);
            int k = (it == propKind.end()) ? 0 : it->second;
            col = (k == 1) ? 0xE2622A : (k == 2) ? 0xA07A44 : (k == 3) ? 0x3F7F9F : 0x8C6A3C;
        }
        if(b->shape.type == ShapeType::Circle){
            int sx, sy; cam.toScreen((float)b->position.x, (float)b->position.y, sx, sy);
            int r = cam.m2p((float)b->shape.radius);
            C.disc(sx, sy, r, (col >> 16) & 255, (col >> 8) & 255, col & 255);
            C.ring(sx, sy, r, 2, 0x101014, 160);
            int hx = sx + (int)(r * 0.6f * std::cos((float)b->angle));
            int hy = sy - (int)(r * 0.6f * std::sin((float)b->angle));
            C.line(sx, sy, hx, hy, 0x101014, 1);
        } else {
            const std::vector<Vec2>& vs = b->worldVertices;
            if(vs.size() < 3) continue;
            std::vector<std::pair<int,int> > pts;
            pts.reserve(vs.size());
            for(size_t k = 0; k < vs.size(); k++){
                int sx, sy; cam.toScreen((float)vs[k].x, (float)vs[k].y, sx, sy);
                pts.push_back(std::make_pair(sx, sy));
            }
            C.fillPoly(pts, col);
            for(size_t k = 0; k < pts.size(); k++){
                size_t j = (k + 1) % pts.size();
                C.line(pts[k].first, pts[k].second, pts[j].first, pts[j].second, 0x1A1A1E, 1);
            }
        }
        if(debugDraw){
            int ax, ay, bx2, by2;
            cam.toScreen((float)b->aabb.min.x, (float)b->aabb.max.y, ax, ay);
            cam.toScreen((float)b->aabb.max.x, (float)b->aabb.min.y, bx2, by2);
            uint32_t dc = b->sleeping ? 0x606060 : 0x30E060;
            C.line(ax, ay, bx2, ay, dc); C.line(ax, by2, bx2, by2, dc);
            C.line(ax, ay, ax, by2, dc); C.line(bx2, ay, bx2, by2, dc);
        }
    }
}

void Game::drawCar(Canvas& C,const Camera& cam){
    RigidBody* cb = world.body(car.chassis);
    if(!cb || !cb->position.isFinite()) return;
    const CarSpec& sp = *car.spec;

    for(int i = 0; i < 2; i++){
        Wheel& wh = car.wheels[i];
        RigidBody* wb = world.body(wh.id);
        if(!wb) continue;
        Vec2 top = strutTop(i);
        int tx, ty, wx, wy;
        cam.toScreen((float)top.x, (float)top.y, tx, ty);
        cam.toScreen((float)wb->position.x, (float)wb->position.y, wx, wy);

        int mx = (tx + wx) / 2, my = (ty + wy) / 2;
        C.line(tx, ty, mx, my, 0x9AA0A8, std::max(2, cam.m2p(0.05f)));
        C.line(mx, my, wx, wy, 0x5A6068, std::max(3, cam.m2p(0.08f)));

        const int turns = 7;
        float amp = (float)cam.m2p(0.11f + 0.05f * wh.comp);
        uint32_t coil = wh.comp > 0.85f ? 0xFF6A3A : (wh.comp > 0.55f ? 0xFFC24A : 0xD8DCE2);
        int px = tx, py = ty;
        for(int s = 1; s <= turns * 2; s++){
            float t = (float)s / (float)(turns * 2);
            int bx = (int)(tx + (wx - tx) * t);
            int by = (int)(ty + (wy - ty) * t);
            int off = ((s % 2) ? 1 : -1) * (int)amp;
            int nx = bx + off, ny = by;
            C.line(px, py, nx, ny, coil, 2);
            px = nx; py = ny;
        }
        C.line(px, py, wx, wy, coil, 2);

        int gh = cam.m2p(wh.maxLen - wh.minLen);
        if(gh > 6){
            int gx = tx + cam.m2p(0.22f);
            C.rect(gx, ty, 3, gh, 0x20242A, 170);
            int fill = (int)(gh * wh.comp);
            C.rect(gx, ty + gh - fill, 3, fill, coil, 220);
        }

        int r = cam.m2p(sp.wr);
        C.disc(wx, wy, r, 26, 26, 30);
        C.ring(wx, wy, r, std::max(2, r / 5), 0x101014, 255);
        int rimR = (int)(r * 0.55f);
        C.disc(wx, wy, rimR, (sp.rimCol >> 16) & 255, (sp.rimCol >> 8) & 255, sp.rimCol & 255);
        for(int s = 0; s < 5; s++){
            float a = wh.spin + s * 1.2566371f;
            C.line(wx, wy, wx + (int)(rimR * std::cos(a)), wy - (int)(rimR * std::sin(a)), 0x2A2A30, 2);
        }
        if(wh.ground) C.disc(wx, wy + r, std::max(2, r / 4), 255, 220, 120, 70);
    }

    const std::vector<Vec2>& vs = cb->worldVertices;
    if(vs.size() >= 3){
        std::vector<std::pair<int,int> > pts;
        for(size_t k = 0; k < vs.size(); k++){
            int sx, sy; cam.toScreen((float)vs[k].x, (float)vs[k].y, sx, sy);
            pts.push_back(std::make_pair(sx, sy));
        }
        C.fillPoly(pts, sp.bodyCol);
        for(size_t k = 0; k < pts.size(); k++){
            size_t j = (k + 1) % pts.size();
            C.line(pts[k].first, pts[k].second, pts[j].first, pts[j].second, 0x101014, 2);
        }
    }
    float ca = (float)cb->angle, cs = std::cos(ca), sn = std::sin(ca);
    float rw = sp.cw * 0.52f, rh = sp.ch * 0.85f, ry = sp.ch + rh * 0.9f;
    std::vector<std::pair<int,int> > roof;
    const float rc[4][2] = { { -rw, 0.f }, { rw * 0.7f, 0.f }, { rw * 0.45f, rh }, { -rw * 0.75f, rh } };
    for(int k = 0; k < 4; k++){
        float lx = rc[k][0], ly = rc[k][1] + ry - rh * 0.5f;
        int sx, sy;
        cam.toScreen((float)(cb->position.x + lx * cs - ly * sn),
                     (float)(cb->position.y + lx * sn + ly * cs), sx, sy);
        roof.push_back(std::make_pair(sx, sy));
    }
    C.fillPoly(roof, sp.roofCol);
    for(size_t k = 0; k < roof.size(); k++){
        size_t j = (k + 1) % roof.size();
        C.line(roof[k].first, roof[k].second, roof[j].first, roof[j].second, 0x101014, 2);
    }
    int hx, hy;
    float lx = sp.cw * 0.92f, ly = 0.f;
    cam.toScreen((float)(cb->position.x + lx * cs - ly * sn),
                 (float)(cb->position.y + lx * sn + ly * cs), hx, hy);
    C.disc(hx, hy, std::max(2, cam.m2p(0.10f)), 255, 240, 170);
}

void Game::drawFx(Canvas& C,const Camera& cam){
    for(size_t i = 0; i < dust.size(); i++){
        const Dust& d = dust[i];
        int sx, sy; cam.toScreen((float)d.p.x, (float)d.p.y, sx, sy);
        C.disc(sx, sy, cam.m2p(d.radius), d.gray, d.gray, (uint8_t)(d.gray - 10),
               (uint8_t)RC(d.alpha * 255.0, 0.0, 255.0));
    }
}

void Game::drawHud(Canvas& C,float fps){
    char buf[300];
    RigidBody* cb = world.body(car.chassis);
    float kmh = 0.f;
    if(cb && cb->velocity.isFinite())
        kmh = (float)std::sqrt(cb->velocity.x * cb->velocity.x + cb->velocity.y * cb->velocity.y) * 3.6f;

    C.rect(0, 0, C.w, 26, 0x10131A, 205);
    std::snprintf(buf, sizeof buf, "PHYS2D DRIVE   MAP %d %s   CAR %s   BODIES %d   AWAKE %d   CONTACTS %d",
                  map + 1, MAPNAME[(int)RC(map, 0, 2)], car.spec->name,
                  (int)world.bodyCount(), (int)world.awakeCount(), lastContacts);
    C.text(10, 9, buf, 0xE6EAF2, 1);
    std::snprintf(buf, sizeof buf, "D3D9  FPS %.0f  STEP %.2f MS", fps, stepMs);
    C.text(C.w - C.textW(buf, 1) - 10, 9, buf, 0x9FB4D8, 1);

    int bx = 14, by = C.h - 116, bw = 190, bh = 13;
    for(int i = 0; i < 2; i++){
        const Wheel& wh = car.wheels[i];
        float sag = (wh.restLen - wh.len) * 100.f;
        C.rect(bx, by + i * 22, bw, bh, 0x181C24, 210);
        uint32_t col = wh.comp < 0.45f ? 0x3FD07A : (wh.comp < 0.8f ? 0xFFC24A : 0xFF5A3A);
        C.rect(bx, by + i * 22, (int)(bw * wh.comp), bh, col, 235);
        C.rect(bx + (int)(bw * 0.8f), by + i * 22, 2, bh, 0xFFFFFF, 150);
        std::snprintf(buf, sizeof buf, "%s SUSP %+.0f CM  %s",
                      i == 0 ? "REAR " : "FRONT", sag, wh.ground ? "GROUND" : "AIR");
        C.text(bx + 6, by + i * 22 + 3, buf, 0x0B0D12, 1);
    }
    std::snprintf(buf, sizeof buf, "AIR %.2f S   BEST %.2f S", car.airTime, car.bestAir);
    C.text(bx, by + 50, buf, 0xC8D2E4, 1);
    std::snprintf(buf, sizeof buf, "ODO %.0f M   TOP %.0f KM/H", car.odo, car.topSpeed);
    C.text(bx, by + 62, buf, 0xC8D2E4, 1);

    std::snprintf(buf, sizeof buf, "%.0f", kmh);
    C.text(C.w - 150, C.h - 96, buf, kmh > 130.f ? 0xFF8A3A : 0xE6EAF2, 5);
    C.text(C.w - 150, C.h - 52, "KM/H", 0x9FB4D8, 1);
    C.rect(C.w - 150, C.h - 38, 130, 8, 0x181C24, 210);
    C.rect(C.w - 150, C.h - 38, (int)(130 * RC(car.engineRpm, 0.0, 1.0)), 8,
           car.engineRpm > 0.85f ? 0xFF5A3A : 0x3FD07A, 235);

    C.rect(0, C.h - 18, C.w, 18, 0x10131A, 195);
    C.text(8, C.h - 13,
           "W GAS  S BRAKE  A D LEAN  SPACE HANDBRAKE  1 2 3 MAP  C CAR  R RESPAWN  T SLOWMO  F DEBUG  P PAUSE  WHEEL ZOOM",
           0x8FA2C4, 1);
    if(paused) C.text(C.w / 2 - 60, C.h / 2, "PAUSED", 0xFFC24A, 4);
}

static void renderFrame(Game& g,Canvas& C,Camera& cam,float fps){
    cam.W = C.w; cam.H = C.h;
    cam.cx = g.camX; cam.cy = g.camY;
    g.drawWorld(C, cam);
    g.drawCar(C, cam);
    g.drawFx(C, cam);
    g.drawHud(C, fps);
}

// ===========================================================================
#if defined(DRIVE_HEADLESS)

int main(int argc,char** argv){
    int mapId = 0, steps = 900, kind = 0;
    for(int i = 1; i < argc; i++){
        if(!std::strcmp(argv[i], "--map")  && i + 1 < argc) mapId = std::atoi(argv[++i]);
        if(!std::strcmp(argv[i], "--steps")&& i + 1 < argc) steps = std::atoi(argv[++i]);
        if(!std::strcmp(argv[i], "--car")  && i + 1 < argc) kind  = std::atoi(argv[++i]);
    }
    mapId = (int)RC(mapId, 0, 2);
    kind  = (int)RC(kind, 0, 4);

    Game g;
    g.init(mapId, kind);
    Canvas C; C.resize(640, 360);
    Camera cam; cam.ppm = 18.f;

    float dt = 1.f / 60.f, avg = 0.f, worst = 0.f, maxComp = 0.f, maxSag = 0.f, restSag = 0.f;
    int nonFinite = 0, detached = 0;
    for(int s = 0; s < steps; s++){
        bool gas = (s % 900) < 820;
        g.drive(gas, false, false, false, false, dt);
        g.step(dt);
        avg += g.stepMs;
        if(g.stepMs > worst) worst = g.stepMs;
        for(int i = 0; i < 2; i++){
            const Wheel& wh = g.car.wheels[i];
            if(wh.comp > maxComp) maxComp = wh.comp;
            float sag = wh.restLen - wh.len;
            if(sag > maxSag) maxSag = sag;
            if(s == 120) restSag = sag;
            RigidBody* wb = g.world.body(wh.id);
            if(wb){
                Vec2 top = g.strutTop(i);
                Vec2 d(wb->position.x - top.x, wb->position.y - top.y);
                float dist = (float)std::sqrt(d.x * d.x + d.y * d.y);
                if(dist > wh.maxLen + 0.02f) detached++;
            }
        }
        std::vector<RigidBody*>& bs = g.world.bodies();
        for(size_t i = 0; i < bs.size(); i++)
            if(bs[i] && (!bs[i]->position.isFinite() || !bs[i]->velocity.isFinite())) nonFinite++;

        if(s % 150 == 0){
            RigidBody* cb = g.world.body(g.car.chassis);
            std::printf("step %4d  x %7.1f  kmh %6.1f  comp %.2f/%.2f  len %.2f/%.2f  air %.2f  contacts %4d  %.2f ms\n",
                        s, cb ? (float)cb->position.x : 0.f,
                        cb ? (float)std::sqrt(cb->velocity.x * cb->velocity.x + cb->velocity.y * cb->velocity.y) * 3.6f : 0.f,
                        g.car.wheels[0].comp, g.car.wheels[1].comp,
                        g.car.wheels[0].len, g.car.wheels[1].len,
                        g.car.airTime, g.lastContacts, g.stepMs);
        }
        renderFrame(g, C, cam, 60.f);
    }
    std::printf("map %d car %s: odo %.0f m  top %.0f km/h  best air %.2f s  static sag %.0f cm  max sag %.0f cm  max comp %.2f  detached %d  nf %d  avg %.2f ms\n",
                mapId + 1, g.car.spec->name, g.car.odo, g.car.topSpeed, g.car.bestAir,
                restSag * 100.f, maxSag * 100.f, maxComp, detached, nonFinite,
                avg / (float)std::max(1, steps));
    return (nonFinite || detached) ? 1 : 0;
}

#elif defined(_WIN32)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d9.h>

struct QuadVtx { float x, y, z, rhw, u, v; };
static const DWORD QUAD_FVF = D3DFVF_XYZRHW | D3DFVF_TEX1;

struct D3DView {
    IDirect3D9*         d3d = NULL;
    IDirect3DDevice9*   dev = NULL;
    IDirect3DTexture9*  tex = NULL;
    D3DPRESENT_PARAMETERS pp;
    int  w = 0, h = 0;
    bool ok = false;

    bool create(HWND hwnd,int W,int H){
        w = W; h = H;
        d3d = Direct3DCreate9(D3D_SDK_VERSION);
        if(!d3d) return false;
        ZeroMemory(&pp, sizeof pp);
        pp.Windowed = TRUE;
        pp.SwapEffect = D3DSWAPEFFECT_DISCARD;
        pp.BackBufferFormat = D3DFMT_X8R8G8B8;
        pp.BackBufferWidth = W;
        pp.BackBufferHeight = H;
        pp.hDeviceWindow = hwnd;
        pp.PresentationInterval = D3DPRESENT_INTERVAL_ONE;
        HRESULT hr = d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hwnd,
                                       D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &pp, &dev);
        if(FAILED(hr))
            hr = d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hwnd,
                                   D3DCREATE_SOFTWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &pp, &dev);
        if(FAILED(hr) || !dev) return false;
        if(!makeTexture()) return false;
        applyStates();
        ok = true;
        return true;
    }
    bool makeTexture(){
        if(!dev) return false;
        return SUCCEEDED(dev->CreateTexture(w, h, 1, D3DUSAGE_DYNAMIC, D3DFMT_A8R8G8B8,
                                            D3DPOOL_DEFAULT, &tex, NULL));
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
        dev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
        dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
        dev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
        dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
    }
    void upload(const Canvas& C){
        if(!tex) return;
        D3DLOCKED_RECT lr;
        if(FAILED(tex->LockRect(0, &lr, NULL, D3DLOCK_DISCARD))) return;
        for(int y = 0; y < h; y++){
            uint32_t* dst = (uint32_t*)((uint8_t*)lr.pBits + y * lr.Pitch);
            const uint32_t* src = &C.px[(size_t)y * C.w];
            for(int x = 0; x < w; x++) dst[x] = 0xFF000000u | src[x];
        }
        tex->UnlockRect(0);
    }
    void frame(const Canvas& C){
        if(!dev) return;
        HRESULT co = dev->TestCooperativeLevel();
        if(co == D3DERR_DEVICELOST){ Sleep(8); return; }
        if(co == D3DERR_DEVICENOTRESET){
            if(tex){ tex->Release(); tex = NULL; }
            if(FAILED(dev->Reset(&pp))) { Sleep(8); return; }
            makeTexture(); applyStates();
        }
        upload(C);
        dev->Clear(0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0, 0, 0), 1.0f, 0);
        if(SUCCEEDED(dev->BeginScene())){
            dev->SetFVF(QUAD_FVF);
            dev->SetTexture(0, tex);
            QuadVtx v[4] = {
                { -0.5f,          -0.5f,          0.f, 1.f, 0.f, 0.f },
                { (float)w - 0.5f,-0.5f,          0.f, 1.f, 1.f, 0.f },
                { -0.5f,          (float)h - 0.5f,0.f, 1.f, 0.f, 1.f },
                { (float)w - 0.5f,(float)h - 0.5f,0.f, 1.f, 1.f, 1.f },
            };
            dev->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, v, sizeof(QuadVtx));
            dev->SetTexture(0, NULL);
            dev->EndScene();
        }
        dev->Present(NULL, NULL, NULL, NULL);
    }
    void destroy(){
        if(tex){ tex->Release(); tex = NULL; }
        if(dev){ dev->Release(); dev = NULL; }
        if(d3d){ d3d->Release(); d3d = NULL; }
        ok = false;
    }
};

static Input g_in;
static bool  g_running = true;

static LRESULT CALLBACK WndProc(HWND hwnd,UINT msg,WPARAM wp,LPARAM lp){
    switch(msg){
        case WM_CLOSE: case WM_DESTROY: g_running = false; return 0;
        case WM_KEYDOWN: if(wp < 256) g_in.key[wp] = true;  return 0;
        case WM_KEYUP:   if(wp < 256) g_in.key[wp] = false; return 0;
        case WM_MOUSEMOVE: g_in.mx = LOWORD(lp); g_in.my = HIWORD(lp); return 0;
        case WM_LBUTTONDOWN: g_in.lmb = true;  return 0;
        case WM_LBUTTONUP:   g_in.lmb = false; return 0;
        case WM_RBUTTONDOWN: g_in.rmb = true;  return 0;
        case WM_RBUTTONUP:   g_in.rmb = false; return 0;
        case WM_MOUSEWHEEL:  g_in.wheel += (float)GET_WHEEL_DELTA_WPARAM(wp) / 120.f; return 0;
        case WM_ERASEBKGND:  return 1;
        case WM_PAINT:       ValidateRect(hwnd, NULL); return 0;
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

int WINAPI WinMain(HINSTANCE hInst,HINSTANCE,LPSTR,int){
    const int W = 1360, H = 768;
    WNDCLASSA wc; ZeroMemory(&wc, sizeof wc);
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInst;
    wc.lpszClassName = "phys2dDriveWindow";
    wc.hbrBackground = NULL;
    wc.hCursor = LoadCursorA(NULL, IDC_ARROW);
    RegisterClassA(&wc);

    RECT rc = { 0, 0, W, H };
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
    HWND hwnd = CreateWindowA("phys2dDriveWindow", "PHYS2D DRIVE - Direct3D 9",
                              WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
                              rc.right - rc.left, rc.bottom - rc.top, NULL, NULL, hInst, NULL);
    if(!hwnd) return 1;
    ShowWindow(hwnd, SW_SHOW);

    D3DView view;
    if(!view.create(hwnd, W, H)){
        MessageBoxA(hwnd, "Direct3D 9 device could not be created.", "PHYS2D DRIVE", MB_OK | MB_ICONERROR);
        view.destroy();
        return 2;
    }

    Canvas C; C.resize(W, H);
    Camera cam; cam.W = W; cam.H = H; cam.ppm = 34.f;
    Game g;
    g.init(0, 0);

    LARGE_INTEGER freq, prev;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);
    float fps = 60.f;

    while(g_running){
        MSG msg;
        while(PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)){
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }
        LARGE_INTEGER now; QueryPerformanceCounter(&now);
        float dt = (float)((double)(now.QuadPart - prev.QuadPart) / (double)freq.QuadPart);
        prev = now;
        dt = (float)RC(dt, 1.0 / 600.0, 1.0 / 20.0);
        fps = fps * 0.92f + (1.f / dt) * 0.08f;

        if(g_in.pressed('1')) g.init(0, g.carKind);
        if(g_in.pressed('2')) g.init(1, g.carKind);
        if(g_in.pressed('3')) g.init(2, g.carKind);
        if(g_in.pressed('C')) g.init(g.map, (g.carKind + 1) % 5);
        if(g_in.pressed('R')) g.respawn();
        if(g_in.pressed('P')) g.paused = !g.paused;
        if(g_in.pressed('T')) g.slowmo = !g.slowmo;
        if(g_in.pressed('F')) g.debugDraw = !g.debugDraw;
        if(g_in.pressed(VK_ESCAPE)) g_running = false;
        if(g_in.wheel > 0.f) cam.zoom(1.12f);
        if(g_in.wheel < 0.f) cam.zoom(0.89f);

        bool gas   = g_in.key['W'] || g_in.key[VK_UP];
        bool brake = g_in.key['S'] || g_in.key[VK_DOWN];
        bool leanL = g_in.key['A'] || g_in.key[VK_LEFT];
        bool leanR = g_in.key['D'] || g_in.key[VK_RIGHT];
        bool hand  = g_in.key[VK_SPACE];

        g.drive(gas, brake, leanL, leanR, hand, dt);
        g.step(dt);

        RigidBody* cb = g.world.body(g.car.chassis);
        if(cb && cb->position.isFinite()){
            g.camX += ((float)cb->position.x - g.camX) * (float)RC(dt * 6.0, 0.0, 1.0);
            g.camY += ((float)cb->position.y + 1.6f - g.camY) * (float)RC(dt * 4.0, 0.0, 1.0);
        }

        renderFrame(g, C, cam, fps);
        view.frame(C);
        g_in.newFrame();
    }
    view.destroy();
    return 0;
}

#else
int main(){
    std::printf("PHYS2D DRIVE needs Windows + Direct3D 9, or build with -DDRIVE_HEADLESS.\n");
    return 0;
}
#endif
