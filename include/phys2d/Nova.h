// Nova.h - NOVA extension pack for phys2d (12 subsystems)
#pragma once
#include "phys2d/World.h"
#include "phys2d/Body.h"
#include "phys2d/Shape.h"
#include "phys2d/Vec2.h"
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace phys2d {
namespace nova {

// =============================================================== 4. MATERIALS
enum class FractureStyle : uint8_t { None, Shards, Splinters, Dent, Crumble, Shatter, Tear, Melt };

struct MaterialLook {
    uint32_t color = 0x909090;
    uint32_t accent = 0xB0B0B0;
    float roughness = 0.5f;
    float gloss = 0.2f;
    float emissive = 0.0f;
    float transparency = 0.0f;
};
struct MaterialSound {
    float baseHz = 320.0f;
    float decay = 6.0f;
    float gain = 0.7f;
    float rolloff = 1.0f;
    float absorption = 0.25f;
    float footstepHz = 180.0f;
};
struct MaterialDef {
    std::string name;
    Material phys;
    MaterialLook look;
    MaterialSound sound;
    FractureStyle fracture = FractureStyle::None;
    real toughness = 1e5;
    int shardCount = 6;
};

class MaterialLibrary {
public:
    static const MaterialLibrary& instance();
    int count() const { return (int)mats_.size(); }
    const MaterialDef& byIndex(int index) const;
    int indexOf(const std::string& name) const;
    const MaterialDef& byName(const std::string& name) const;
    Material physics(int index) const { return byIndex(index).phys; }
    real impactHz(int a, int b, real impulse) const;
private:
    MaterialLibrary();
    std::vector<MaterialDef> mats_;
    std::unordered_map<std::string, int> index_;
};

// ==================================================================== 2. 2.5D
struct LayerInfo {
    real z = 0;
    real thickness = 1;
    real parallax = 1;
    uint32_t tint = 0xFFFFFF;
    float fog = 0.0f;
    bool collides = true;
};
class DepthWorld {
public:
    void attach(World& w);
    void configure(int layer, const LayerInfo& info);
    LayerInfo info(int layer) const;
    void setLayer(BodyId id, int layer);
    int layer(BodyId id) const;
    void linkLayers(int a, int b, bool collide);
    bool canCollide(int la, int lb) const;
    std::vector<BodyId> drawOrder() const;
    real parallax(int layer) const;
    real scaleFor(int layer, real cameraZ = 0) const;
    size_t trackedBodies() const { return layerOf_.size(); }
private:
    World* world_ = nullptr;
    std::unordered_map<int, LayerInfo> layers_;
    std::unordered_map<uint32_t, int> layerOf_;
    std::unordered_map<uint64_t, bool> pairs_;
};

// ================================================================= 5. PROCGEN
struct CaveParams {
    int width = 96, height = 48;
    real cell = 0.5, fill = 0.45;
    int smoothPasses = 5;
    uint32_t seed = 1337;
    Vec2 origin = Vec2(-24, 0);
};
struct CityParams {
    int blocks = 8;
    real streetWidth = 6, minBuilding = 4, maxBuilding = 12, minHeight = 6, maxHeight = 26, debrisChance = 0.25;
    uint32_t seed = 2024;
    Vec2 origin = Vec2(-60, 0);
};
struct MazeParams {
    int cols = 21, rows = 15;
    real cell = 2;
    uint32_t seed = 7;
    Vec2 origin = Vec2(-20, 0);
    bool braid = false;
};
class ProcGen {
public:
    static std::vector<uint8_t> caveGrid(const CaveParams& p);
    static std::vector<uint8_t> mazeGrid(const MazeParams& p);
    static int buildCave(World& w, const CaveParams& p);
    static int buildMaze(World& w, const MazeParams& p);
    static int buildCity(World& w, const CityParams& p);
    static Vec2 findSpawn(const std::vector<uint8_t>& grid, int w, int h, real cell, const Vec2& origin);
};

// ================================================================== 11. MOBILE
struct CpuFeatures {
    bool sse2 = false, sse4 = false, avx = false, avx2 = false, neon = false;
    int cores = 1;
    std::string isa = "scalar";
};
CpuFeatures detectCpu();
const char* simdBackend();

class PowerManager {
public:
    enum class Profile { Desktop, Mobile, Battery };
    void attach(World& w) { world_ = &w; }
    void setProfile(Profile p) { profile_ = p; }
    Profile profile() const { return profile_; }
    void update(real dt);
    real suggestTimestep() const { return step_; }
    int suggestIterations() const;
    bool shouldRenderFrame();
    real activity() const { return activity_; }
    real estimatedPowerW() const;
    int skippedFrames() const { return skipped_; }
private:
    World* world_ = nullptr;
    Profile profile_ = Profile::Desktop;
    real activity_ = 1;
    real idle_ = 0;
    real step_ = 1.0 / 60.0;
    int skipped_ = 0;
    long long frame_ = 0;
};

// ================================================================== 3. EDITOR
struct PaletteItem {
    enum Kind { Box, Circle, Polygon, StaticBox, Ramp, Cloth, Grain, Vehicle };
    std::string name;
    Kind kind = Box;
    real w = 1, h = 1;
    int materialIndex = 0;
};
class EditorCore {
public:
    void attach(World& w);
    void setPaused(bool p) { paused_ = p; }
    bool paused() const { return paused_; }
    void step(real dt);
    bool pick(const Vec2& at);
    void drag(const Vec2& to);
    void release() { sel_ = INVALID_BODY; }
    bool hasSelection() const { return sel_ != INVALID_BODY; }
    BodyId selection() const { return sel_; }
    void deleteSelection();
    BodyId duplicateSelection();
    const std::vector<PaletteItem>& palette() const { return palette_; }
    void addPaletteItem(const PaletteItem& item) { palette_.push_back(item); }
    BodyId spawn(int paletteIndex, const Vec2& at);
    void setGravity(const Vec2& g);
    void setGlobalFriction(real f);
    void setGlobalRestitution(real r);
    void setSolverIterations(int velocity, int position);
    void setTimeScale(real s) { timeScale_ = s; }
    real timeScale() const { return timeScale_; }
    void pushUndo();
    bool undo();
    bool redo();
    int undoDepth() const { return (int)undo_.size(); }
    bool saveScene(const std::string& path);
    bool loadScene(const std::string& path);
    std::string statusLine() const;
private:
    World* world_ = nullptr;
    bool paused_ = false;
    BodyId sel_ = INVALID_BODY;
    Vec2 dragTarget_;
    std::vector<PaletteItem> palette_;
    real timeScale_ = 1.0;
    std::vector<std::string> undo_, redo_;
};

// ============================================================ 7. CLOTH & HAIR
struct ClothNode {
    Vec2 p, prev;
    real invMass = 1;
    bool pinned = false;
    bool alive = true;
};
struct ClothLink {
    int a = 0, b = 0;
    real rest = 0.1;
    real stiffness = 1;
    bool alive = true;
    bool shear = false;
};
struct ClothPiece {
    int cols = 0, rows = 0;
    real spacing = 0.12;
    real damping = 0.02;
    real tearFactor = 0;
    real thickness = 0.04;
    bool selfCollide = false;
    bool hair = false;
    uint32_t color = 0xC8C8D8;
    std::vector<ClothNode> nodes;
    std::vector<ClothLink> links;
};
class ClothSystem {
public:
    Vec2 gravity = Vec2(0, -9.81);
    void attach(World& w) { world_ = &w; }
    int createFlag(const Vec2& topLeft, int cols, int rows, real spacing = 0.18, real stiffness = 1.0);
    int createCurtain(const Vec2& topLeft, int cols, int rows, real spacing = 0.18);
    int createHair(const Vec2& root, int nodes, real length, real stiffness = 0.9);
    int createRopeStrand(const Vec2& a, const Vec2& b, int nodes, real stiffness = 1.0);
    void pinToBody(int piece, int node, BodyId body);
    void setWind(const Vec2& wind, real turbulence);
    void update(real dt, int iterations = 6);
    void applyImpulse(const Vec2& at, real radius, const Vec2& impulse);
    int pieceCount() const { return (int)pieces_.size(); }
    const ClothPiece& piece(int i) const { return pieces_[(size_t)i]; }
    size_t nodeCount() const;
    int tornLinks() const { return torn_; }
private:
    void solveBodies(ClothPiece& piece);
    World* world_ = nullptr;
    std::vector<ClothPiece> pieces_;
    std::vector<std::pair<std::pair<int, int>, BodyId> > pins_;
    Vec2 wind_;
    real turbulence_ = 0;
    real time_ = 0;
    int torn_ = 0;
};

// =============================================== 8. DEM PARTICLES AND WEATHER
enum class GrainKind : uint8_t { Sand, Gravel, Snow, Spark, Smoke, Ember, Rain, Hail, Dust };
struct Grain {
    Vec2 p, v;
    real radius = 0.05;
    real mass = 0.02;
    real life = 0;
    real maxLife = 1e30;
    real heat = 0;
    GrainKind kind = GrainKind::Sand;
    uint32_t color = 0xC2A050;
    bool resting = false;
};
class GrainSystem {
public:
    Vec2 gravity = Vec2(0, -9.81);
    real stiffness = 8000, damping = 12, friction = 0.55, airDrag = 0.02;
    size_t maxCount = 40000;
    real bounds[4] = {-400, -80, 400, 260};
    void attach(World& w) { world_ = &w; }
    void reserve(size_t n) { grains_.reserve(n); }
    void emitPile(const Vec2& center, real radius, real height, real grainRadius = 0.05, GrainKind kind = GrainKind::Sand);
    void emitBurst(const Vec2& at, int count, real speed, GrainKind kind);
    void emitJet(const Vec2& at, const Vec2& dir, int count, real speed, real spread, GrainKind kind);
    void update(real dt);
    void clear() { grains_.clear(); }
    void applyExplosion(const Vec2& at, real radius, real energy);
    size_t count() const { return grains_.size(); }
    std::vector<Grain>& grains() { return grains_; }
    const std::vector<Grain>& grains() const { return grains_; }
    real packedFraction() const;
    int contactsLastStep() const { return contacts_; }
private:
    void collideGrains(real dt);
    void collideBodies(real dt);
    World* world_ = nullptr;
    std::vector<Grain> grains_;
    std::unordered_map<uint64_t, std::vector<int> > cells_;
    real cellSize_ = 0.2;
    uint32_t rng_ = 12345;
    int contacts_ = 0;
};
class WeatherSystem {
public:
    enum class Kind { Clear, Rain, Snowfall, Hail, Sandstorm };
    void configure(Kind k, real intensity, const Vec2& wind = Vec2(0, 0));
    void setArea(const Vec2& mn, const Vec2& mx) { min_ = mn; max_ = mx; }
    void update(real dt, GrainSystem& grains);
    Kind kind() const { return kind_; }
    real intensity() const { return intensity_; }
    int spawnedTotal() const { return spawned_; }
    Vec2 wind() const { return wind_; }
private:
    Kind kind_ = Kind::Clear;
    real intensity_ = 0;
    Vec2 wind_;
    Vec2 min_ = Vec2(-40, 0), max_ = Vec2(40, 30);
    real accum_ = 0;
    int spawned_ = 0;
    uint32_t rng_ = 999;
};

// ================================================================ 9. VEHICLES
struct WheelSetup {
    Vec2 anchor;
    real radius = 0.35;
    real travel = 0.30;
    real stiffnessScale = 1;
    real damperScale = 1;
    bool driven = true;
    bool steered = false;
    bool brakes = true;
};
struct WheelState {
    BodyId body = INVALID_BODY;
    Vec2 localAnchor;
    real restLen = 0.4, minLen = 0.2, maxLen = 0.6, len = 0.4;
    real load = 0, compression = 0, spin = 0, slip = 0;
    bool grounded = false;
    WheelSetup setup;
};
class WheeledVehicle {
public:
    struct Config {
        Vec2 position;
        real halfWidth = 1.7, halfHeight = 0.42, mass = 1100;
        real driveAccel = 7, topSpeed = 46, brakeAccel = 11;
        real ridePercent = 0.35, damperRatio = 0.75, grip = 1, downforce = 0;
        int materialIndex = 0;
    };
    void create(World& w, const Config& cfg, const std::vector<WheelSetup>& setups);
    void destroy();
    void update(real dt, real throttle, real brake, real lean);
    void postStep();
    BodyId chassis() const { return chassis_; }
    const std::vector<WheelState>& wheels() const { return wheels_; }
    real speedKmh() const;
    real suspensionTravelUsed() const;
    bool airborne() const;
    real odometer() const { return odo_; }
    real springRate() const { return springK_; }
    real damperRate() const { return springC_; }
private:
    Vec2 strutAxis() const;
    Vec2 strutTop(const WheelState& ws) const;
    World* world_ = nullptr;
    Config cfg_;
    BodyId chassis_ = INVALID_BODY;
    std::vector<WheelState> wheels_;
    real springK_ = 0, springC_ = 0, odo_ = 0, topKmh_ = 0;
};
class TrackedVehicle {
public:
    struct Config {
        Vec2 position;
        real halfLength = 3, halfHeight = 0.6, mass = 24000;
        int roadWheels = 6;
        real wheelRadius = 0.45, trackGrip = 2.2, drivePower = 520000, turretMass = 3000;
    };
    void create(World& w, const Config& cfg);
    void update(real dt, real leftThrottle, real rightThrottle);
    void postStep();
    BodyId hull() const { return hull_; }
    BodyId turret() const { return turret_; }
    real trackSpeed() const { return trackSpeed_; }
    real groundPressure() const;
    const std::vector<BodyId>& roadWheels() const { return wheels_; }
private:
    World* world_ = nullptr;
    Config cfg_;
    BodyId hull_ = INVALID_BODY, turret_ = INVALID_BODY;
    std::vector<BodyId> wheels_;
    real trackSpeed_ = 0;
    real contact_ = 0;
};
class Aircraft {
public:
    struct Wing {
        Vec2 offset;
        real area = 6, liftSlope = 5.8, stallAngle = 0.28, dragBase = 0.02, controlGain = 0;
    };
    struct Config {
        Vec2 position;
        real mass = 900, halfWidth = 3.2, halfHeight = 0.5, thrust = 5200, airDensity = 1.225;
        bool rotorcraft = false;
        real rotorThrust = 14000;
    };
    void create(World& w, const Config& cfg, const std::vector<Wing>& wings);
    void update(real dt, real throttle, real pitchInput);
    BodyId body() const { return body_; }
    real angleOfAttack() const { return aoa_; }
    real lastLift() const { return lift_; }
    real lastDrag() const { return drag_; }
    bool stalled() const { return stalled_; }
    real airspeed() const { return airspeed_; }
    void setWind(const Vec2& w) { wind_ = w; }
private:
    World* world_ = nullptr;
    Config cfg_;
    std::vector<Wing> wings_;
    BodyId body_ = INVALID_BODY;
    Vec2 wind_;
    real aoa_ = 0, lift_ = 0, drag_ = 0, airspeed_ = 0;
    bool stalled_ = false;
};

// =================================================================== 10. OCEAN
class Ocean {
public:
    struct Wave { real amplitude = 0.4, length = 12, speed = 4, phase = 0, steepness = 0.6; };
    real fluidDensity = 1025, dragCoefficient = 1.1, angularDrag = 0.6;
    void attach(World& w) { world_ = &w; }
    void clearWaves() { waves_.clear(); }
    void addWave(const Wave& w) { waves_.push_back(w); }
    void makeSea(int waveCount, real windSpeed, uint32_t seed = 4242);
    void setShore(real shoreX, real slope, real deepDepth);
    void setLevel(real l) { level_ = l; }
    real level() const { return level_; }
    real height(real x, real t) const;
    Vec2 surfacePoint(real x, real t) const;
    Vec2 orbitalVelocity(const Vec2& at, real t) const;
    real depth(real x) const;
    real breakingIntensity(real x, real t) const;
    void applyBuoyancy(real t, real dt);
    int floatingBodies() const { return floating_; }
    int waveCount() const { return (int)waves_.size(); }
private:
    World* world_ = nullptr;
    std::vector<Wave> waves_;
    real level_ = 0;
    real shoreX_ = 1e30, slope_ = 0.05, deepDepth_ = 20;
    int floating_ = 0;
};

// ================================================================= 1. NETWORK
struct NetBodyState {
    uint32_t id = 0;
    float x = 0, y = 0, angle = 0, vx = 0, vy = 0, w = 0;
};
struct NetSnapshot {
    uint32_t tick = 0;
    double time = 0;
    std::vector<NetBodyState> bodies;
    const NetBodyState* find(uint32_t id) const;
};
struct NetInput {
    uint32_t tick = 0;
    uint32_t client = 0;
    uint32_t buttons = 0;
    float axisX = 0, axisY = 0;
};
class SnapshotCodec {
public:
    static std::vector<uint8_t> encode(const NetSnapshot& cur, const NetSnapshot* base);
    static bool decode(const std::vector<uint8_t>& data, const NetSnapshot* base, NetSnapshot& out);
    static size_t rawSize(const NetSnapshot& s);
    static real posQuantum() { return 1.0 / 512.0; }
    static real angQuantum() { return 1.0 / 2048.0; }
    static real velQuantum() { return 1.0 / 256.0; }
};
using NetInputHandler = std::function<void(World&, const NetInput&)>;
class ServerAuthority {
public:
    void attach(World& w) { world_ = &w; }
    void setHistoryLength(int n) { history_ = n; }
    void setInputHandler(NetInputHandler h) { handler_ = h; }
    void pushInput(const NetInput& in) { pending_.push_back(in); }
    void step(real dt);
    NetSnapshot capture() const;
    bool rewind(uint32_t tick);
    void restore();
    RigidBody* rewindQueryPoint(uint32_t tick, const Vec2& at);
    uint32_t tick() const { return tick_; }
    size_t historySize() const { return snaps_.size(); }
private:
    World* world_ = nullptr;
    NetInputHandler handler_;
    std::vector<NetInput> pending_;
    std::vector<NetSnapshot> snaps_;
    NetSnapshot saved_;
    bool savedValid_ = false;
    int history_ = 64;
    uint32_t tick_ = 0;
};
class ClientPredictor {
public:
    void attach(World& w) { world_ = &w; }
    void setInputHandler(NetInputHandler h) { handler_ = h; }
    void pushLocalInput(const NetInput& in) { inputs_.push_back(in); }
    void predict(real dt);
    void applySnapshot(const NetSnapshot& s, uint32_t ackTick);
    real reconciliationError() const { return error_; }
    void setSmoothing(real s) { smoothing_ = s; }
    uint32_t tick() const { return tick_; }
    size_t unacked() const { return inputs_.size(); }
private:
    World* world_ = nullptr;
    NetInputHandler handler_;
    std::vector<NetInput> inputs_;
    real smoothing_ = 0.25;
    real dt_ = 1.0 / 60.0;
    real error_ = 0;
    uint32_t tick_ = 0;
};
class InterpolationBuffer {
public:
    void setDelay(real d) { delay_ = d; }
    void push(const NetSnapshot& s);
    bool sample(real now, NetSnapshot& out) const;
    size_t size() const { return snaps_.size(); }
    void clear() { snaps_.clear(); }
private:
    std::vector<NetSnapshot> snaps_;
    real delay_ = 0.1;
};

// =============================================================== 12. ACOUSTICS
struct Voice {
    Vec2 position;
    real gain = 1, pitch = 1, delay = 0, pan = 0, reverbTail = 0, frequency = 320;
    int materialA = 0, materialB = 0;
};
class Acoustics {
public:
    struct Wall { Vec2 a, b; int material = 0; };
    void attach(World& w) { world_ = &w; }
    void setListener(const Vec2& p, const Vec2& v) { listener_ = p; listenerVel_ = v; }
    void addWall(const Vec2& a, const Vec2& b, int material);
    void clearWalls() { walls_.clear(); }
    void setSpeedOfSound(real s) { speed_ = s; }
    void setRoom(real volume, real surface);
    Voice impact(const Vec2& at, real impulse, int matA, int matB);
    Voice footstep(const Vec2& at, int surfaceMaterial);
    void update(real dt);
    const std::vector<Voice>& voices() const { return voices_; }
    real reverbTime60() const;
    real dopplerFor(const Vec2& sourcePos, const Vec2& sourceVel) const;
    int wallCount() const { return (int)walls_.size(); }
    real masterLevel() const;
private:
    World* world_ = nullptr;
    std::vector<Wall> walls_;
    std::vector<Voice> voices_;
    Vec2 listener_, listenerVel_;
    real speed_ = 343, volume_ = 400, surface_ = 320;
};

// ============================================================= 6. LUA SCRIPTS
struct LuaValue {
    enum Type { Nil, Number, Bool, Str };
    Type type = Nil;
    double num = 0;
    bool b = false;
    std::string str;
    LuaValue() {}
    LuaValue(double n) : type(Number), num(n) {}
    LuaValue(bool v) : type(Bool), b(v) {}
    LuaValue(const std::string& s) : type(Str), str(s) {}
    LuaValue(const char* s) : type(Str), str(s ? s : "") {}
    bool truthy() const { return !(type == Nil || (type == Bool && !b)); }
    double toNumber() const;
    std::string toString() const;
};
using LuaNative = std::function<LuaValue(const std::vector<LuaValue>&)>;
class LuaVM {
public:
    LuaVM();
    ~LuaVM();
    LuaVM(const LuaVM&) = delete;
    LuaVM& operator=(const LuaVM&) = delete;
    void bindWorld(World& w);
    void registerFunction(const std::string& name, LuaNative fn);
    bool doString(const std::string& code, std::string* error = nullptr);
    bool doFile(const std::string& path, std::string* error = nullptr);
    bool hasFunction(const std::string& name) const;
    LuaValue callFunction(const std::string& name, const std::vector<LuaValue>& args = std::vector<LuaValue>());
    void setGlobal(const std::string& name, const LuaValue& v);
    LuaValue getGlobal(const std::string& name) const;
    const std::string& output() const;
    void clearOutput();
    int nativeCount() const;
    struct Impl;
private:
    Impl* impl_;
};

// ==================================================================== SUITE
struct NovaSuite {
    DepthWorld depth;
    ServerAuthority server;
    ClientPredictor client;
    InterpolationBuffer interp;
    ClothSystem cloth;
    GrainSystem grains;
    WeatherSystem weather;
    Ocean ocean;
    Acoustics audio;
    PowerManager power;
    EditorCore editor;
    LuaVM lua;
    void attachAll(World& w);
    void update(real dt);
    std::string report() const;
};

} // namespace nova
} // namespace phys2d
