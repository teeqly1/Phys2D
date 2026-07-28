// phys2d - detailed blood / gore simulation layer.
//
// Everything here is built on top of the phys2d rigid body world:
// droplets are ballistic particles with air drag, they are traced against the
// world with the engine ray caster, and every impact turns into a decal that is
// stored in the LOCAL space of the body it hit - so splatter rotates and moves
// with the object it landed on. Wounds are attached to bodies, bleed with a
// heartbeat pulse, run out of blood and eventually clot.
#pragma once

#include "phys2d/Extras.h"

#include <cstdint>
#include <vector>

namespace phys2d {

// ---------------------------------------------------------------- primitives
enum class BloodKind : uint8_t {
    Droplet = 0,   // normal ballistic drop
    Mist    = 1,   // fine spray, high drag, short life
    Gush    = 2,   // heavy arterial blob, makes big splats
    Clot    = 3    // coagulated chunk, bounces, does not smear
};

struct BloodDroplet {
    Vec2      position, previous, velocity;
    real      radius   = 0.035;
    real      volume   = 0.35;    // millilitres
    real      age      = 0.0;
    real      life     = 14.0;
    real      spin     = 0.0;
    BloodKind kind     = BloodKind::Droplet;
    uint8_t   oxygen   = 210;     // 255 = bright arterial, 0 = dark venous
    bool      active   = true;
};

// A splatter mark. Outline is a closed polygon in the local frame of `body`
// (or in world space when body == INVALID_BODY).
struct BloodDecal {
    BodyId            body = INVALID_BODY;
    Vec2              anchor;          // local (or world) attach point
    std::vector<Vec2> outline;         // local offsets around the anchor
    real              volume  = 0.4;
    real              age     = 0.0;
    real              wetness = 1.0;   // 1 wet and glossy -> 0 dried and dark
    real              soak    = 0.0;   // how much has been absorbed
    uint8_t           oxygen  = 210;
};

// Blood that reached a horizontal surface and spread out into a puddle.
struct BloodPool {
    Vec2   center;
    real   halfWidth = 0.05;
    real   volume    = 0.0;    // millilitres
    real   age       = 0.0;
    real   wetness   = 1.0;
    real   depth     = 0.01;
    uint8_t oxygen   = 200;
};

// An open bleeding wound anchored to a body.
struct Wound {
    BodyId id        = INVALID_BODY;
    Vec2   local;                  // attach point in body space
    Vec2   localDir;               // outward direction in body space
    real   severity  = 0.5;        // 0..1, drives flow rate and spray cone
    real   reservoir = 900.0;      // millilitres of blood left in this wound
    real   age       = 0.0;
    real   clot      = 0.0;        // 0 open -> 1 fully clotted
    real   emitAcc   = 0.0;
    bool   arterial  = false;      // pulsing jet vs steady ooze
    bool   severed   = false;      // stump: wide, violent gusher
};

struct BloodStain {
    real wetness   = 0.0;   // 0..1 how soaked the body surface is
    real coverage  = 0.0;   // accumulated splatter amount
    real dripTimer = 0.0;
};

struct BloodStats {
    size_t droplets = 0, decals = 0, pools = 0, wounds = 0;
    real   airborneMl = 0.0, decalMl = 0.0, pooledMl = 0.0, spilledMl = 0.0;
    int    impactsThisStep = 0;
    int    dropletsSpawned = 0;
};

// ---------------------------------------------------------------- the system
class BloodSystem : public SimulationSystem {
public:
    // --- tuning -----------------------------------------------------------
    Vec2 gravity{0.0, -9.81};
    real airDrag          = 0.30;   // quadratic drag on droplets
    real mistDrag         = 2.60;
    real dropletRadius    = 0.035;
    real dropletVolume    = 0.35;   // ml per spawned droplet
    real stickiness       = 0.78;   // chance a hit sticks instead of bouncing
    real bounceRestitution= 0.22;
    real castOffSpeed     = 7.0;    // above this an impact throws satellites
    real dryTime          = 30.0;   // seconds until a decal is fully dry
    real poolDryTime      = 70.0;
    real poolSpread       = 0.070;  // puddle half width per sqrt(ml)
    real poolMergeDist    = 0.55;
    real groundLevel      = -1.0e30;// optional flat floor for pooling
    real pulseHz          = 1.30;   // heartbeat
    real pulseStrength    = 0.85;
    real clotRate         = 0.035;  // per second
    real smearRate        = 1.20;   // wet bodies leave marks while sliding
    real dripRate         = 0.60;   // wet bodies drip
    int  splatterLobes    = 9;      // detail of a single splat outline
    size_t maxDroplets    = 24000;
    size_t maxDecals      = 7000;
    size_t maxPools       = 900;
    size_t maxWounds      = 256;   // hard cap, oldest/driest wounds are recycled
    real   woundMergeDist = 0.28;  // new hits near an old wound just deepen it
    bool stainBodies      = true;
    bool enablePools      = true;
    bool enableCastOff    = true;
    bool enableMist       = true;
    uint32_t seed         = 0x51ED2701u;

    // --- emission ---------------------------------------------------------
    void emit(const Vec2& p, const Vec2& v, real volumeMl, BloodKind kind, uint8_t oxygen = 210);
    void spray(const Vec2& origin, const Vec2& dir, real speed, real coneRad,
               int count, real volumeMl, uint8_t oxygen = 225);
    void splash(const Vec2& center, real speed, int count, real volumeMl, uint8_t oxygen = 200);

    // --- wounds -----------------------------------------------------------
    Wound* addWound(BodyId body, const Vec2& worldPoint, const Vec2& worldDir,
                    real severity, bool arterial);
    // Big open stump: instant burst plus a strong pulsing jet.
    Wound* sever(BodyId body, const Vec2& worldPoint, const Vec2& worldDir, real severity = 1.0);
    void   healWound(BodyId body);
    void   stainBody(BodyId body, real amount);

    // --- lifecycle --------------------------------------------------------
    void update(real dt) override;
    void clear();
    void forgetBody(BodyId body);   // call when a body is destroyed

    // --- queries ----------------------------------------------------------
    const std::vector<BloodDroplet>& droplets() const { return m_drops; }
    const std::vector<BloodDecal>&   decals()   const { return m_decals; }
    const std::vector<BloodPool>&    pools()    const { return m_pools; }
    const std::vector<Wound>&        wounds()   const { return m_wounds; }
    real       wetnessOf(BodyId body) const;
    BloodStats stats() const { return m_stats; }
    real       pulsePhase() const { return m_pulse; }

    // Colour ramp: fresh oxygenated blood -> dark dried crust.
    static void colorOf(uint8_t oxygen, real wetness, uint8_t& r, uint8_t& g, uint8_t& b);

    const char* className() const override { return "BloodSystem"; }

private:
    real  rnd();
    real  rnd(real lo, real hi);
    void  integrateDroplets(real dt);
    void  resolveImpact(BloodDroplet& d, const RayHit& hit, real dt);
    void  makeDecal(BodyId body, const Vec2& worldPoint, const Vec2& normal,
                    const Vec2& incoming, real volumeMl, uint8_t oxygen, real speed);
    void  addToPool(const Vec2& worldPoint, real volumeMl, uint8_t oxygen);
    void  bleedWounds(real dt);
    void  ageMarks(real dt);
    void  updateStains(real dt);
    void  budget();

    std::vector<BloodDroplet> m_drops;
    std::vector<BloodDroplet> m_pending;   // spawned while iterating m_drops
    bool                      m_iterating = false;
    std::vector<BloodDecal>   m_decals;
    std::vector<BloodPool>    m_pools;
    std::vector<Wound>        m_wounds;
    std::unordered_map<BodyId, BloodStain> m_stains;
    BloodStats m_stats;
    uint32_t   m_rng   = 0x51ED2701u;
    real       m_pulse = 0.0;
};

} // namespace phys2d
