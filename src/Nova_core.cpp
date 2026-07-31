#include "phys2d/Nova.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <thread>

namespace phys2d {
namespace nova {

static inline real cl0(real v, real a, real b){ return v < a ? a : (v > b ? b : v); }

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 1u) {}
    uint32_t next(){ s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    real uni(){ return (real)(next() % 1000000u) / 1000000.0; }
    real range(real a, real b){ return a + (b - a) * uni(); }
    int irange(int a, int b){ return a + (int)(next() % (uint32_t)(b - a + 1)); }
};

static uint32_t lighten(uint32_t c, real k)
{
    real r = (real)((c >> 16) & 0xFF), g = (real)((c >> 8) & 0xFF), b = (real)(c & 0xFF);
    r = cl0(r + (255 - r) * k, 0, 255); g = cl0(g + (255 - g) * k, 0, 255); b = cl0(b + (255 - b) * k, 0, 255);
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

struct MatRow {
    const char* name;
    double dens, rest, sf, df;
    uint32_t col;
    float rough, gloss, hz, absorb;
    int fracture;
    double tough;
    int shards;
    float transparency;
};

static const MatRow kRows[] = {
    {"water",1000,0.00,0.02,0.02,0x2E6FA8,0.10f,0.85f,190,0.55f,0,1e4,0,0.75f},
    {"saltwater",1025,0.00,0.02,0.02,0x246288,0.10f,0.85f,185,0.55f,0,1e4,0,0.72f},
    {"oil",900,0.00,0.01,0.01,0x27221A,0.20f,0.70f,120,0.70f,0,1e4,0,0.45f},
    {"ice",917,0.10,0.08,0.05,0xC9E8F5,0.08f,0.90f,1500,0.06f,1,4e4,10,0.40f},
    {"snow",250,0.02,0.55,0.45,0xF2F6FA,0.95f,0.02f,90,0.85f,4,6e3,12,0.0f},
    {"packedsnow",500,0.05,0.45,0.35,0xE4EDF4,0.85f,0.05f,140,0.70f,4,2e4,10,0.0f},
    {"sand",1600,0.02,0.65,0.55,0xC2A050,0.98f,0.02f,110,0.65f,4,8e3,14,0.0f},
    {"gravel",1900,0.08,0.75,0.62,0x8C8880,0.95f,0.04f,260,0.50f,4,3e4,10,0.0f},
    {"soil",1500,0.03,0.70,0.58,0x6B4A2A,0.95f,0.02f,100,0.72f,4,1e4,12,0.0f},
    {"clay",1750,0.02,0.72,0.60,0x9A6A4A,0.90f,0.06f,130,0.60f,4,2e4,10,0.0f},
    {"mud",1650,0.01,0.50,0.40,0x4E3A22,0.98f,0.08f,80,0.80f,0,5e3,0,0.0f},
    {"asphalt",2300,0.12,0.85,0.72,0x35383C,0.88f,0.06f,330,0.28f,4,9e4,8,0.0f},
    {"concrete",2400,0.15,0.80,0.68,0x9C9C98,0.85f,0.05f,480,0.15f,4,1.4e5,9,0.0f},
    {"brick",1900,0.14,0.78,0.66,0xA24B32,0.90f,0.05f,520,0.18f,4,8e4,10,0.0f},
    {"granite",2700,0.18,0.72,0.60,0x7A7A80,0.60f,0.25f,900,0.05f,1,3e5,8,0.0f},
    {"marble",2650,0.20,0.55,0.45,0xE8E6E0,0.25f,0.70f,1050,0.04f,1,2.4e5,8,0.0f},
    {"sandstone",2200,0.10,0.80,0.68,0xC8A878,0.92f,0.04f,420,0.30f,4,7e4,12,0.0f},
    {"limestone",2400,0.12,0.76,0.64,0xD8D2C0,0.80f,0.10f,560,0.22f,4,9e4,10,0.0f},
    {"basalt",2900,0.16,0.75,0.63,0x3A3A3E,0.70f,0.15f,840,0.08f,1,3.2e5,8,0.0f},
    {"glass",2500,0.22,0.35,0.25,0xBEE0EE,0.05f,0.95f,2100,0.03f,5,1.2e4,26,0.85f},
    {"temperedglass",2500,0.24,0.35,0.25,0xAFD8E8,0.05f,0.95f,2400,0.03f,5,6e4,40,0.85f},
    {"ceramic",2300,0.20,0.55,0.42,0xE6E0D2,0.20f,0.75f,1700,0.06f,1,3e4,14,0.0f},
    {"porcelain",2400,0.22,0.45,0.35,0xFAF8F4,0.10f,0.90f,1950,0.05f,1,2.6e4,18,0.0f},
    {"oak",750,0.18,0.62,0.50,0x8A5A32,0.75f,0.15f,420,0.15f,2,5e4,9,0.0f},
    {"pine",520,0.20,0.58,0.46,0xC8A472,0.80f,0.12f,380,0.18f,2,3e4,10,0.0f},
    {"plywood",600,0.16,0.60,0.48,0xB08C58,0.82f,0.10f,340,0.22f,2,2.6e4,11,0.0f},
    {"bamboo",700,0.30,0.55,0.44,0xC8C070,0.70f,0.25f,650,0.16f,2,4e4,8,0.0f},
    {"cork",240,0.35,0.70,0.60,0xC8A070,0.95f,0.03f,150,0.75f,4,6e3,10,0.0f},
    {"steel",7850,0.30,0.55,0.42,0x9AA0A8,0.40f,0.60f,1200,0.02f,3,9e5,4,0.0f},
    {"stainless",7900,0.32,0.50,0.38,0xC0C6CC,0.25f,0.80f,1350,0.02f,3,1e6,4,0.0f},
    {"iron",7300,0.26,0.58,0.46,0x7A7E84,0.55f,0.40f,1000,0.03f,3,7e5,5,0.0f},
    {"aluminium",2700,0.28,0.48,0.36,0xCFD4D8,0.30f,0.72f,1450,0.02f,3,3e5,5,0.0f},
    {"titanium",4500,0.34,0.52,0.40,0xA8AEB4,0.35f,0.68f,1600,0.02f,3,1.2e6,4,0.0f},
    {"copper",8900,0.22,0.52,0.40,0xB87333,0.35f,0.70f,900,0.03f,3,5e5,5,0.0f},
    {"brass",8500,0.24,0.50,0.38,0xC8A030,0.30f,0.75f,1150,0.03f,3,4.5e5,5,0.0f},
    {"lead",11300,0.05,0.60,0.50,0x6A6E76,0.60f,0.30f,320,0.10f,3,1e5,3,0.0f},
    {"gold",19300,0.20,0.48,0.36,0xE8C040,0.20f,0.90f,980,0.03f,3,2e5,4,0.0f},
    {"rubber",1100,0.80,0.95,0.85,0x28282C,0.92f,0.08f,180,0.55f,0,2e4,0,0.0f},
    {"softrubber",950,0.88,0.98,0.88,0x3A3A40,0.95f,0.05f,140,0.65f,0,1e4,0,0.0f},
    {"plastic",1200,0.45,0.42,0.32,0x4A80C8,0.45f,0.55f,760,0.20f,2,3e4,7,0.15f},
    {"nylon",1150,0.50,0.38,0.28,0xE0E0D8,0.40f,0.50f,820,0.25f,6,2.4e4,6,0.10f},
    {"foam",60,0.10,0.80,0.70,0xF0E8C8,0.99f,0.01f,60,0.95f,6,2e3,6,0.0f},
    {"cloth",300,0.05,0.75,0.65,0xC85A5A,0.98f,0.02f,120,0.85f,6,4e3,8,0.0f},
    {"leather",900,0.20,0.72,0.60,0x6A4028,0.85f,0.15f,220,0.60f,6,1.2e4,6,0.0f},
    {"flesh",1050,0.08,0.80,0.70,0xB4544A,0.90f,0.10f,140,0.75f,6,8e3,8,0.0f},
    {"bone",1900,0.25,0.60,0.48,0xF0E8D8,0.55f,0.35f,1250,0.12f,1,9e4,7,0.0f},
    {"wax",950,0.05,0.55,0.45,0xF2EAD0,0.70f,0.30f,240,0.55f,7,5e3,5,0.0f},
    {"lava",3100,0.02,0.85,0.75,0xFF5A18,0.80f,0.30f,140,0.40f,7,4e3,6,0.0f},
};

MaterialLibrary::MaterialLibrary()
{
    const int n = (int)(sizeof(kRows) / sizeof(kRows[0]));
    mats_.reserve((size_t)n);
    for(int i = 0; i < n; ++i){
        const MatRow& r = kRows[i];
        MaterialDef d;
        d.name = r.name;
        d.phys.density = r.dens;
        d.phys.restitution = r.rest;
        d.phys.staticFriction = r.sf;
        d.phys.dynamicFriction = r.df;
        d.look.color = r.col;
        d.look.accent = lighten(r.col, 0.25);
        d.look.roughness = r.rough;
        d.look.gloss = r.gloss;
        d.look.transparency = r.transparency;
        d.look.emissive = (r.name[0] == 'l' && r.name[1] == 'a') ? 0.9f : 0.0f;
        d.sound.baseHz = r.hz;
        d.sound.decay = 2.0f + r.gloss * 10.0f;
        d.sound.absorption = r.absorb;
        d.sound.footstepHz = r.hz * 0.45f;
        d.fracture = (FractureStyle)r.fracture;
        d.toughness = r.tough;
        d.shardCount = r.shards;
        mats_.push_back(d);
        index_[d.name] = i;
    }
}

const MaterialLibrary& MaterialLibrary::instance()
{
    static MaterialLibrary lib;
    return lib;
}
const MaterialDef& MaterialLibrary::byIndex(int i) const
{
    if(i < 0 || i >= (int)mats_.size()) i = 0;
    return mats_[(size_t)i];
}
int MaterialLibrary::indexOf(const std::string& name) const
{
    std::unordered_map<std::string, int>::const_iterator it = index_.find(name);
    return it == index_.end() ? -1 : it->second;
}
const MaterialDef& MaterialLibrary::byName(const std::string& name) const
{
    int i = indexOf(name);
    return byIndex(i < 0 ? 0 : i);
}
real MaterialLibrary::impactHz(int a, int b, real impulse) const
{
    real fa = byIndex(a).sound.baseHz, fb = byIndex(b).sound.baseHz;
    real base = std::sqrt(fa * fb);
    real boost = cl0(std::log10(1.0 + std::fabs((double)impulse)) * 0.12, 0.0, 0.6);
    return base * (1.0 + boost);
}

static uint64_t pairKey(int a, int b){ return ((uint64_t)(uint32_t)(a + 1024) << 32) | (uint32_t)(b + 1024); }
static bool pairAllowsIn(const std::unordered_map<uint64_t, bool>& m, int a, int b)
{
    if(a == b) return true;
    std::unordered_map<uint64_t, bool>::const_iterator it = m.find(pairKey(a, b));
    if(it != m.end()) return it->second;
    it = m.find(pairKey(b, a));
    if(it != m.end()) return it->second;
    return false;
}
static BodyId idOfBody(World* w, const RigidBody* target)
{
    if(!w || !target) return INVALID_BODY;
    uint32_t limit = (uint32_t)w->bodyCount() + 64u;
    for(uint32_t id = 0; id <= limit; ++id)
        if(w->body((BodyId)id) == target) return (BodyId)id;
    return INVALID_BODY;
}

void DepthWorld::attach(World& w)
{
    world_ = &w;
    LayerInfo back; back.z = 6; back.parallax = 0.55; back.fog = 0.45f; back.tint = 0x9FB0C8;
    LayerInfo mid;
    LayerInfo front; front.z = -3; front.parallax = 1.35; front.fog = 0.10f;
    layers_[-1] = back; layers_[0] = mid; layers_[1] = front;
    DepthWorld* self = this;
    World* wp = &w;
    w.setContactFilter([self, wp](const RigidBody& a, const RigidBody& b) -> bool {
        BodyId ia = idOfBody(wp, &a), ib = idOfBody(wp, &b);
        std::unordered_map<uint32_t, int>::const_iterator ita = self->layerOf_.find((uint32_t)ia);
        std::unordered_map<uint32_t, int>::const_iterator itb = self->layerOf_.find((uint32_t)ib);
        int la = ita == self->layerOf_.end() ? 0 : ita->second;
        int lb = itb == self->layerOf_.end() ? 0 : itb->second;
        if(la == lb) return true;
        return pairAllowsIn(self->pairs_, la, lb);
    });
}
void DepthWorld::configure(int layer, const LayerInfo& info){ layers_[layer] = info; }
LayerInfo DepthWorld::info(int layer) const
{
    std::unordered_map<int, LayerInfo>::const_iterator it = layers_.find(layer);
    return it == layers_.end() ? LayerInfo() : it->second;
}
void DepthWorld::setLayer(BodyId id, int layer){ layerOf_[(uint32_t)id] = layer; }
int DepthWorld::layer(BodyId id) const
{
    std::unordered_map<uint32_t, int>::const_iterator it = layerOf_.find((uint32_t)id);
    return it == layerOf_.end() ? 0 : it->second;
}
void DepthWorld::linkLayers(int a, int b, bool collide){ pairs_[pairKey(a, b)] = collide; }
bool DepthWorld::canCollide(int la, int lb) const { return pairAllowsIn(pairs_, la, lb); }
std::vector<BodyId> DepthWorld::drawOrder() const
{
    std::vector<std::pair<real, BodyId> > tmp;
    for(std::unordered_map<uint32_t, int>::const_iterator it = layerOf_.begin(); it != layerOf_.end(); ++it)
        tmp.push_back(std::make_pair(info(it->second).z, (BodyId)it->first));
    std::sort(tmp.begin(), tmp.end(), [](const std::pair<real, BodyId>& a, const std::pair<real, BodyId>& b){
        if(a.first != b.first) return a.first > b.first;
        return a.second < b.second;
    });
    std::vector<BodyId> out;
    for(size_t i = 0; i < tmp.size(); ++i) out.push_back(tmp[i].second);
    return out;
}
real DepthWorld::parallax(int layer) const { return info(layer).parallax; }
real DepthWorld::scaleFor(int layer, real cameraZ) const
{
    const real focal = 20.0;
    real z = info(layer).z;
    return focal / (focal + (z - cameraZ));
}

std::vector<uint8_t> ProcGen::caveGrid(const CaveParams& p)
{
    Rng rng(p.seed);
    std::vector<uint8_t> g((size_t)(p.width * p.height), 0);
    for(int y = 0; y < p.height; ++y)
        for(int x = 0; x < p.width; ++x){
            bool border = (x == 0 || y == 0 || x == p.width - 1 || y == p.height - 1);
            g[(size_t)(y * p.width + x)] = (uint8_t)((border || rng.uni() < p.fill) ? 1 : 0);
        }
    std::vector<uint8_t> tmp = g;
    for(int pass = 0; pass < p.smoothPasses; ++pass){
        for(int y = 0; y < p.height; ++y)
            for(int x = 0; x < p.width; ++x){
                int n = 0;
                for(int dy = -1; dy <= 1; ++dy)
                    for(int dx = -1; dx <= 1; ++dx){
                        if(!dx && !dy) continue;
                        int nx = x + dx, ny = y + dy;
                        if(nx < 0 || ny < 0 || nx >= p.width || ny >= p.height) { ++n; continue; }
                        n += g[(size_t)(ny * p.width + nx)] ? 1 : 0;
                    }
                uint8_t keep = g[(size_t)(y * p.width + x)];
                tmp[(size_t)(y * p.width + x)] = (uint8_t)(n > 4 ? 1 : (n < 4 ? 0 : keep));
            }
        g.swap(tmp);
        for(int x = 0; x < p.width; ++x){ g[(size_t)x] = 1; g[(size_t)((p.height - 1) * p.width + x)] = 1; }
        for(int y = 0; y < p.height; ++y){ g[(size_t)(y * p.width)] = 1; g[(size_t)(y * p.width + p.width - 1)] = 1; }
    }
    return g;
}

std::vector<uint8_t> ProcGen::mazeGrid(const MazeParams& p)
{
    Rng rng(p.seed);
    int w = p.cols | 1, h = p.rows | 1;
    std::vector<uint8_t> g((size_t)(w * h), 1);
    std::vector<std::pair<int,int> > stack;
    stack.push_back(std::make_pair(1, 1));
    g[(size_t)(1 * w + 1)] = 0;
    while(!stack.empty()){
        std::pair<int,int> cur = stack.back();
        int dirs[4] = {0, 1, 2, 3};
        for(int i = 3; i > 0; --i){ int j = rng.irange(0, i); int t = dirs[i]; dirs[i] = dirs[j]; dirs[j] = t; }
        bool moved = false;
        for(int i = 0; i < 4 && !moved; ++i){
            int dx = 0, dy = 0;
            if(dirs[i] == 0) dx = 2; else if(dirs[i] == 1) dx = -2; else if(dirs[i] == 2) dy = 2; else dy = -2;
            int nx = cur.first + dx, ny = cur.second + dy;
            if(nx <= 0 || ny <= 0 || nx >= w - 1 || ny >= h - 1) continue;
            if(!g[(size_t)(ny * w + nx)]) continue;
            g[(size_t)(ny * w + nx)] = 0;
            g[(size_t)((cur.second + dy / 2) * w + (cur.first + dx / 2))] = 0;
            stack.push_back(std::make_pair(nx, ny));
            moved = true;
        }
        if(!moved) stack.pop_back();
    }
    if(p.braid){
        for(int y = 1; y < h - 1; ++y)
            for(int x = 1; x < w - 1; ++x)
                if(g[(size_t)(y * w + x)] && rng.uni() < 0.15) g[(size_t)(y * w + x)] = 0;
    }
    return g;
}

static int buildGridBodies(World& w, const std::vector<uint8_t>& g, int gw, int gh, real cell, const Vec2& origin, int material)
{
    const MaterialLibrary& lib = MaterialLibrary::instance();
    int made = 0;
    for(int y = 0; y < gh; ++y){
        int x = 0;
        while(x < gw){
            if(!g[(size_t)(y * gw + x)]){ ++x; continue; }
            int run = 0;
            while(x + run < gw && g[(size_t)(y * gw + x + run)]) ++run;
            BodyDef bd;
            bd.type = BodyType::Static;
            bd.shape = Shape::box(cell * run, cell);
            bd.material = lib.physics(material);
            bd.position = origin + Vec2(((real)x + run * 0.5) * cell, ((real)y + 0.5) * cell);
            bd.name = "terrain";
            w.createBody(bd);
            ++made;
            x += run;
        }
    }
    return made;
}

int ProcGen::buildCave(World& w, const CaveParams& p)
{
    std::vector<uint8_t> g = caveGrid(p);
    return buildGridBodies(w, g, p.width, p.height, p.cell, p.origin, MaterialLibrary::instance().indexOf("granite"));
}
int ProcGen::buildMaze(World& w, const MazeParams& p)
{
    int gw = p.cols | 1, gh = p.rows | 1;
    std::vector<uint8_t> g = mazeGrid(p);
    return buildGridBodies(w, g, gw, gh, p.cell, p.origin, MaterialLibrary::instance().indexOf("brick"));
}
int ProcGen::buildCity(World& w, const CityParams& p)
{
    const MaterialLibrary& lib = MaterialLibrary::instance();
    Rng rng(p.seed);
    int made = 0;
    real x = p.origin.x;
    int iConcrete = lib.indexOf("concrete"), iGlass = lib.indexOf("glass"), iAsphalt = lib.indexOf("asphalt"), iBrick = lib.indexOf("brick");
    {
        BodyDef road;
        road.type = BodyType::Static;
        road.shape = Shape::box((real)p.blocks * (p.maxBuilding + p.streetWidth) + 20.0, 1.0);
        road.material = lib.physics(iAsphalt);
        road.position = Vec2(p.origin.x + (real)p.blocks * (p.maxBuilding + p.streetWidth) * 0.5, p.origin.y - 0.5);
        w.createBody(road);
        ++made;
    }
    for(int b = 0; b < p.blocks; ++b){
        real bw = rng.range(p.minBuilding, p.maxBuilding);
        real bh = rng.range(p.minHeight, p.maxHeight);
        BodyDef wall;
        wall.type = BodyType::Static;
        wall.material = lib.physics(b % 3 == 0 ? iBrick : iConcrete);
        wall.shape = Shape::box(0.4, bh);
        wall.position = Vec2(x, p.origin.y + bh * 0.5);
        w.createBody(wall); ++made;
        wall.position = Vec2(x + bw, p.origin.y + bh * 0.5);
        w.createBody(wall); ++made;
        for(real fy = 3.2; fy < bh; fy += 3.2){
            BodyDef floorDef;
            floorDef.type = BodyType::Static;
            floorDef.material = lib.physics(iConcrete);
            floorDef.shape = Shape::box(bw, 0.25);
            floorDef.position = Vec2(x + bw * 0.5, p.origin.y + fy);
            w.createBody(floorDef); ++made;
            if(rng.uni() < 0.6){
                BodyDef win;
                win.type = BodyType::Dynamic;
                win.material = lib.physics(iGlass);
                win.shape = Shape::box(bw * 0.6, 1.2);
                win.position = Vec2(x + bw * 0.5, p.origin.y + fy + 1.0);
                win.name = "window";
                w.createBody(win); ++made;
            }
        }
        BodyDef roof;
        roof.type = BodyType::Static;
        roof.material = lib.physics(iConcrete);
        roof.shape = Shape::box(bw + 0.8, 0.4);
        roof.position = Vec2(x + bw * 0.5, p.origin.y + bh);
        w.createBody(roof); ++made;
        if(rng.uni() < p.debrisChance){
            BodyDef d;
            d.type = BodyType::Dynamic;
            d.material = lib.physics(iBrick);
            d.shape = Shape::box(0.6, 0.6);
            d.position = Vec2(x + bw + p.streetWidth * 0.5, p.origin.y + 0.4);
            w.createBody(d); ++made;
        }
        x += bw + p.streetWidth;
    }
    return made;
}
Vec2 ProcGen::findSpawn(const std::vector<uint8_t>& grid, int w, int h, real cell, const Vec2& origin)
{
    for(int y = h - 2; y > 0; --y)
        for(int x = 1; x < w - 1; ++x)
            if(!grid[(size_t)(y * w + x)] && !grid[(size_t)((y - 1) * w + x)])
                return origin + Vec2(((real)x + 0.5) * cell, ((real)y + 0.5) * cell);
    return origin;
}

CpuFeatures detectCpu()
{
    CpuFeatures f;
#if defined(__ARM_NEON) || defined(__ARM_NEON__) || defined(__aarch64__)
    f.neon = true; f.isa = "neon";
#endif
#if defined(__SSE2__)
    f.sse2 = true; f.isa = "sse2";
#endif
#if defined(__SSE4_1__) || defined(__SSE4_2__)
    f.sse4 = true; f.isa = "sse4";
#endif
#if defined(__AVX__)
    f.avx = true; f.isa = "avx";
#endif
#if defined(__AVX2__)
    f.avx2 = true; f.isa = "avx2";
#endif
    unsigned hc = std::thread::hardware_concurrency();
    f.cores = hc ? (int)hc : 1;
    return f;
}
const char* simdBackend()
{
    static std::string s = detectCpu().isa;
    return s.c_str();
}

void PowerManager::update(real dt)
{
    real a = 1.0;
    if(world_){
        int total = (int)world_->bodyCount();
        int awake = (int)world_->awakeCount();
        a = total > 0 ? (real)awake / (real)total : 0.0;
    }
    activity_ = activity_ * 0.9 + a * 0.1;
    if(activity_ < 0.05) idle_ += dt; else idle_ = 0;
    real base = 1.0 / 60.0;
    real stretch = 1.0;
    if(idle_ > 0.5){
        if(profile_ == Profile::Battery) stretch = 3.0;
        else if(profile_ == Profile::Mobile) stretch = 1.5;
    }
    step_ = cl0(base * stretch, 1.0 / 120.0, 1.0 / 15.0);
    ++frame_;
}
int PowerManager::suggestIterations() const
{
    real a = cl0(activity_, 0.0, 1.0);
    if(profile_ == Profile::Desktop) return (int)(8 + 8 * a);
    if(profile_ == Profile::Mobile) return (int)(4 + 6 * a);
    return (int)(4 + 2 * a);
}
bool PowerManager::shouldRenderFrame()
{
    int every = 1;
    if(profile_ == Profile::Mobile) every = 2;
    else if(profile_ == Profile::Battery) every = activity_ < 0.05 ? 4 : 2;
    if(every <= 1) return true;
    if(frame_ % every == 0) return true;
    ++skipped_;
    return false;
}
real PowerManager::estimatedPowerW() const
{
    real base = profile_ == Profile::Desktop ? 45.0 : (profile_ == Profile::Mobile ? 3.2 : 1.6);
    return base * (0.35 + 0.65 * cl0(activity_, 0.0, 1.0));
}

void EditorCore::attach(World& w)
{
    world_ = &w;
    const MaterialLibrary& lib = MaterialLibrary::instance();
    PaletteItem it;
    it.name = "wood box";   it.kind = PaletteItem::Box;       it.w = 0.8; it.h = 0.8; it.materialIndex = lib.indexOf("oak");      palette_.push_back(it);
    it.name = "steel box";  it.kind = PaletteItem::Box;       it.w = 0.7; it.h = 0.7; it.materialIndex = lib.indexOf("steel");    palette_.push_back(it);
    it.name = "glass pane"; it.kind = PaletteItem::Box;       it.w = 0.2; it.h = 1.6; it.materialIndex = lib.indexOf("glass");    palette_.push_back(it);
    it.name = "stone ball"; it.kind = PaletteItem::Circle;    it.w = 0.4; it.h = 0.4; it.materialIndex = lib.indexOf("granite");  palette_.push_back(it);
    it.name = "rubber ball";it.kind = PaletteItem::Circle;    it.w = 0.35;it.h = 0.35;it.materialIndex = lib.indexOf("rubber");   palette_.push_back(it);
    it.name = "platform";   it.kind = PaletteItem::StaticBox; it.w = 6.0; it.h = 0.4; it.materialIndex = lib.indexOf("concrete"); palette_.push_back(it);
    it.name = "ramp";       it.kind = PaletteItem::Ramp;      it.w = 4.0; it.h = 0.3; it.materialIndex = lib.indexOf("concrete"); palette_.push_back(it);
    it.name = "cloth";      it.kind = PaletteItem::Cloth;     it.w = 1.6; it.h = 1.2; it.materialIndex = lib.indexOf("cloth");    palette_.push_back(it);
    it.name = "sand pile";  it.kind = PaletteItem::Grain;     it.w = 1.0; it.h = 0.8; it.materialIndex = lib.indexOf("sand");     palette_.push_back(it);
}
void EditorCore::step(real dt)
{
    if(!world_) return;
    if(paused_){
        if(sel_ != INVALID_BODY){
            RigidBody* b = world_->body(sel_);
            if(b){ b->position = dragTarget_; b->velocity = Vec2(0, 0); }
        }
        return;
    }
    if(sel_ != INVALID_BODY){
        RigidBody* b = world_->body(sel_);
        if(b){
            b->wake();
            b->velocity = (dragTarget_ - b->position) * 12.0;
        }
    }
    world_->step(dt * timeScale_);
}
bool EditorCore::pick(const Vec2& at)
{
    if(!world_) return false;
    uint32_t limit = (uint32_t)world_->bodyCount() + 64u;
    real best = 1e30;
    BodyId bestId = INVALID_BODY;
    for(uint32_t id = 0; id <= limit; ++id){
        RigidBody* b = world_->body((BodyId)id);
        if(!b || b->type == BodyType::Static) continue;
        Vec2 d = b->position - at;
        real dist = std::sqrt((double)(d.x * d.x + d.y * d.y));
        if(dist <= b->boundingRadius + 0.2 && dist < best){ best = dist; bestId = (BodyId)id; }
    }
    sel_ = bestId;
    if(sel_ != INVALID_BODY) dragTarget_ = at;
    return sel_ != INVALID_BODY;
}
void EditorCore::drag(const Vec2& to){ dragTarget_ = to; }
void EditorCore::deleteSelection()
{
    if(!world_ || sel_ == INVALID_BODY) return;
    pushUndo();
    world_->destroyBody(sel_);
    sel_ = INVALID_BODY;
}
BodyId EditorCore::duplicateSelection()
{
    if(!world_ || sel_ == INVALID_BODY) return INVALID_BODY;
    RigidBody* b = world_->body(sel_);
    if(!b) return INVALID_BODY;
    pushUndo();
    BodyDef bd;
    bd.type = b->type;
    bd.shape = b->shape;
    bd.material = b->material;
    bd.position = b->position + Vec2(0.4, 0.4);
    bd.angle = b->angle;
    return world_->createBody(bd);
}
BodyId EditorCore::spawn(int paletteIndex, const Vec2& at)
{
    if(!world_ || paletteIndex < 0 || paletteIndex >= (int)palette_.size()) return INVALID_BODY;
    const PaletteItem& it = palette_[(size_t)paletteIndex];
    BodyDef bd;
    bd.material = MaterialLibrary::instance().physics(it.materialIndex);
    bd.position = at;
    bd.name = it.name;
    switch(it.kind){
        case PaletteItem::Circle: bd.shape = Shape::circle(it.w); bd.type = BodyType::Dynamic; break;
        case PaletteItem::StaticBox: bd.shape = Shape::box(it.w, it.h); bd.type = BodyType::Static; break;
        case PaletteItem::Ramp: bd.shape = Shape::box(it.w, it.h); bd.type = BodyType::Static; bd.angle = 0.35; break;
        default: bd.shape = Shape::box(it.w, it.h); bd.type = BodyType::Dynamic; break;
    }
    pushUndo();
    return world_->createBody(bd);
}
void EditorCore::setGravity(const Vec2& g){ if(world_) world_->setGravity(g); }
void EditorCore::setGlobalFriction(real f)
{
    if(!world_) return;
    uint32_t limit = (uint32_t)world_->bodyCount() + 64u;
    for(uint32_t id = 0; id <= limit; ++id){
        RigidBody* b = world_->body((BodyId)id);
        if(!b) continue;
        b->material.staticFriction = f;
        b->material.dynamicFriction = f * 0.8;
    }
}
void EditorCore::setGlobalRestitution(real r)
{
    if(!world_) return;
    uint32_t limit = (uint32_t)world_->bodyCount() + 64u;
    for(uint32_t id = 0; id <= limit; ++id){
        RigidBody* b = world_->body((BodyId)id);
        if(b) b->material.restitution = r;
    }
}
void EditorCore::setSolverIterations(int velocity, int position)
{
    if(!world_) return;
    world_->config().solver.velocityIterations = velocity;
    world_->config().solver.positionIterations = position;
}
void EditorCore::pushUndo()
{
    if(!world_) return;
    undo_.push_back(world_->toJson(false));
    if(undo_.size() > 32) undo_.erase(undo_.begin());
    redo_.clear();
}
bool EditorCore::undo()
{
    if(!world_ || undo_.empty()) return false;
    redo_.push_back(world_->toJson(false));
    std::string s = undo_.back();
    undo_.pop_back();
    sel_ = INVALID_BODY;
    return world_->fromJson(s);
}
bool EditorCore::redo()
{
    if(!world_ || redo_.empty()) return false;
    undo_.push_back(world_->toJson(false));
    std::string s = redo_.back();
    redo_.pop_back();
    sel_ = INVALID_BODY;
    return world_->fromJson(s);
}
bool EditorCore::saveScene(const std::string& path){ return world_ ? world_->saveToFile(path) : false; }
bool EditorCore::loadScene(const std::string& path){ return world_ ? world_->loadFromFile(path) : false; }
std::string EditorCore::statusLine() const
{
    char buf[256];
    char sel[32];
    if(sel_ == INVALID_BODY) std::snprintf(sel, sizeof(sel), "-");
    else std::snprintf(sel, sizeof(sel), "%u", (unsigned)sel_);
    std::snprintf(buf, sizeof(buf), "%s bodies %d  sel %s  time x%.2f  undo %d  palette %d",
        paused_ ? "PAUSED" : "RUN",
        world_ ? (int)world_->bodyCount() : 0,
        sel,
        (double)timeScale_,
        (int)undo_.size(),
        (int)palette_.size());
    return std::string(buf);
}

} // namespace nova
} // namespace phys2d
