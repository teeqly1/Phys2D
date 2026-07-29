#pragma once
// phys2d - Advanced physics pack (block 1).
//
// Wind with adjustable strength and mass-dependent response, wind-driven
// deformation, electrical conduction through poured water, impact
// fragmentation with exact momentum conservation, a material-science lab
// (fatigue / creep / relaxation / viscoelasticity / plasticity / shape
// memory / corrosion / oxidation / radiation / ablation) and surface
// physics (adhesion / cohesion / capillary / diffusion / osmosis).
//
// Everything plugs into the existing World through the usual
// SimulationSystem::attach(world) + update(dt) pattern. The engine core is
// untouched.

#include "phys2d/World.h"
#include "phys2d/Extras.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace phys2d {
namespace adv {

// ================================================================ RNG
/// Deterministic xorshift64* generator with a Gaussian tail.
class Rng {
public:
    Rng(uint64_t seed = 0x9E3779B97F4A7C15ull) : m_s(seed ? seed : 0x9E3779B97F4A7C15ull) {}
    uint64_t next();
    real     uniform(real a, real b);
    real     gauss(real mu, real sigma);
    void     reseed(uint64_t s) { m_s = s ? s : 1ull; }

private:
    uint64_t m_s;
};

// ================================================================ WIND
/// Global wind state. Strength is in m/s and can be changed at any time.
struct WindProfile {
    real strength      = 10.0;    ///< mean wind speed, m/s
    Vec2 direction     = Vec2(1.0, 0.0);
    real turbulence    = 0.25;    ///< fractal-noise gust fraction
    real gustPeriod    = 4.0;     ///< s
    real gustAmplitude = 0.35;    ///< fraction of the mean speed
    real airDensity    = 1.225;   ///< kg/m3
    real shearHeight   = 0.0;     ///< >0 makes wind grow with height
};

/// Per-body aerodynamic description.
struct AeroProfile {
    real dragCoefficient = 1.2;   ///< Cd
    real area            = -1.0;  ///< <0 = derive the silhouette from the shape
    real liftCoefficient = 0.0;
    real flexibility     = 0.0;   ///< 0 rigid .. 1 fully deformable
    real tearPressure    = 1.0e9; ///< Pa, dynamic pressure that tears the part
    bool sail            = true;  ///< recompute the silhouette as it tumbles
};

/// Measured wind response of one body.
struct WindReport {
    BodyId body         = INVALID_BODY;
    real   windSpeed    = 0.0;
    real   area         = 0.0;
    real   force        = 0.0;
    real   mass         = 0.0;
    real   acceleration = 0.0;   ///< force / mass: light things fly, heavy crawl
    real   deformation  = 0.0;
    real   terminalSpeed = 0.0;
};

/// Wind: F = 0.5 * rho * Cd * A * |v_rel| * v_rel, applied off-centre so
/// bodies also tumble. Flexible parts bend with the dynamic pressure.
class WindSystem : public ForceSystem {
public:
    WindProfile     profile;
    SoftBodySystem* softBodies = nullptr;

    real deformationGain     = 1.0;
    real deformationRecovery = 4.0;   ///< 1/s elastic spring-back
    real maxDeformation      = 0.45;
    bool enableDeformation   = true;

    const char* className() const override { return "WindSystem"; }

    void setStrength(real s) { profile.strength = s; }
    real strength() const { return profile.strength; }
    void setDirectionDegrees(real deg);

    void         configure(BodyId id, const AeroProfile& p) { m_aero[id] = p; }
    AeroProfile& aero(BodyId id);
    bool         hasProfile(BodyId id) const { return m_aero.count(id) != 0; }

    Vec2       windVelocityAt(const Vec2& p) const;
    real       projectedArea(const RigidBody& b) const;
    Vec2       dragForceOn(const RigidBody& b) const;
    WindReport report(const RigidBody& b) const;

    int  tornCount() const { return m_torn; }
    void update(real dt) override;

private:
    struct DeformState {
        std::vector<Vec2> rest;
        real bend = 0.0;
        real set  = 0.0;      ///< permanent plastic set
    };

    void applyToSoftBodies(real dt);
    void deformBodies(real dt);

    std::unordered_map<BodyId, AeroProfile> m_aero;
    std::unordered_map<BodyId, DeformState> m_deform;
    Rng  m_rng{0xC0FFEEull};
    real m_time = 0.0;
    int  m_torn = 0;
};

// ================================================================ ELECTRICITY
struct Electrode {
    Vec2 position;
    real voltage = 220.0;
    real radius  = 0.6;
    bool ground  = false;
};

struct ShockEvent {
    BodyId body = INVALID_BODY;
    real   voltage = 0.0;
    real   current = 0.0;
    real   power   = 0.0;
    Vec2   point;
};

struct ArcEvent {
    Vec2 from, to;
    real voltage = 0.0;
    real energy  = 0.0;
    real life    = 0.06;
};

/// Turns a body of water into a resistor network: every SPH particle is a
/// node, neighbours are linked with G = A / (rho * L), electrodes pin their
/// nodes, and div(sigma grad V) = 0 is relaxed with Gauss-Seidel. The current
/// therefore spreads through the whole puddle and shocks anything in it.
class ElectricSystem : public SimulationSystem {
public:
    ParticleSystem*      fluid    = nullptr;
    BodyPhysicsRegistry* registry = nullptr;
    ThermalSystem*       thermal  = nullptr;

    real waterResistivity = 20.0;    ///< ohm*m (tap water ~20, sea water ~0.2)
    real linkRadius       = 0.42;    ///< particle connectivity radius
    real crossSection     = 0.02;    ///< m2 of the conducting channel
    real bodyResistance   = 1500.0;  ///< ohm through a body standing in water
    real jouleHeating     = 1.0;
    real electrokinetic   = 6.0e-5;
    real arcVoltage       = 900.0;
    real triboRate        = 4.0e-8;  ///< charge per joule of friction work
    int  relaxIterations  = 28;
    bool superconducting  = false;
    real criticalTemp     = 92.0;    ///< K

    const char* className() const override { return "ElectricSystem"; }

    void addElectrode(const Electrode& e) { m_electrodes.push_back(e); }
    void clearElectrodes() { m_electrodes.clear(); }
    const std::vector<Electrode>& electrodes() const { return m_electrodes; }

    void update(real dt) override;

    real   voltageAt(const Vec2& p) const;
    const  std::vector<real>& particleVoltages() const { return m_voltage; }
    const  std::vector<real>& particleTemperature() const { return m_temp; }
    real   totalCurrent() const { return m_current; }
    real   dissipatedPower() const { return m_power; }
    size_t wetNodeCount() const { return m_pos.size(); }

    const std::vector<ShockEvent>& shocks() const { return m_shocks; }
    const std::vector<ArcEvent>&   arcs() const { return m_arcs; }

    void notifyContactBreak(const Vec2& p, const Vec2& q, real voltage);
    void addFrictionWork(BodyId a, BodyId b, real joules);
    real chargeOf(BodyId id) const;

private:
    struct Link { int a; int b; real g; };

    void buildGraph();
    void solveField();
    void couple(real dt);

    std::vector<Electrode>  m_electrodes;
    std::vector<Vec2>       m_pos;
    std::vector<real>       m_voltage;
    std::vector<real>       m_temp;
    std::vector<char>       m_fixed;
    std::vector<Link>       m_links;
    std::vector<ShockEvent> m_shocks;
    std::vector<ArcEvent>   m_arcs;
    std::unordered_map<BodyId, real> m_charge;
    real m_current = 0.0;
    real m_power   = 0.0;
};

// ================================================================ MATERIALS
struct MaterialProfile {
    real youngModulus   = 2.0e9;
    real yieldStress    = 2.5e7;
    real ultimateStress = 4.0e7;
    real fatigueLimit   = 8.0e6;
    real fatigueExponent = 3.0;    ///< Basquin exponent
    real creepRate      = 1.0e-12; ///< Norton coefficient
    real creepExponent  = 4.0;
    real relaxationTime = 8.0;     ///< s, exponential stress decay
    real kelvinViscosity = 4.0e6;  ///< Kelvin-Voigt damper
    real cubicStiffness = -1.5e-3; ///< nonlinear elasticity term
    real cyclicSoftening = 0.02;
    real brittleness    = 0.5;     ///< >=0.5 shatters, below stretches
    real toughnessRateExp = 0.08;  ///< strength gain per decade of strain rate
    real adhesion       = 0.0;     ///< Pa, different materials
    real cohesion       = 0.0;     ///< Pa, identical materials
    real corrosionRate  = 0.0;
    real oxidationRate  = 0.0;
    real ablationRate   = 0.0;
    real radiationSoftening = 0.0;
    bool shapeMemory    = false;
    real austeniteTemp  = 340.0;   ///< K
};

struct MaterialState {
    real stress = 0.0;
    real elasticStrain = 0.0;
    real plasticStrain = 0.0;
    real creepStrain = 0.0;
    real damage = 0.0;
    real fatigue = 0.0;
    real cycles = 0.0;
    real peakStress = 0.0;
    real lastStress = 0.0;
    real oxide = 0.0;
    real corroded = 0.0;
    real ablated = 0.0;
    real dose = 0.0;
    real temperature = 293.0;
    real strengthScale = 1.0;
    int  impacts = 0;
    bool failed = false;
};

/// Material science: cumulative damage, fatigue, creep, relaxation,
/// viscoelasticity, nonlinear elasticity, plasticity, cyclic softening,
/// shape memory, corrosion, oxidation, radiation and ablation.
class MaterialLab : public SimulationSystem {
public:
    real aggressiveMedium = 0.0;   ///< 0..1 acid concentration
    real radiationField   = 0.0;   ///< Gy/s

    const char* className() const override { return "MaterialLab"; }

    void registerBody(BodyId id, const MaterialProfile& p);
    bool tracks(BodyId id) const { return m_profile.count(id) != 0; }

    MaterialProfile&     profile(BodyId id);
    MaterialState&       state(BodyId id);
    const MaterialState* find(BodyId id) const;

    void applyStress(BodyId id, real sigma, real dt);
    real effectiveStrength(BodyId id, real strainRate) const;
    void onImpact(BodyId id, real impulse, real speed, const Vec2& point);
    void addFrictionWork(BodyId id, real joules);
    void irradiate(BodyId id, real gray);

    real viscoelasticStress(BodyId id, real strain, real strainRate) const;
    real shapeMemoryRecovery(BodyId id) const;

    void update(real dt) override;
    const std::vector<BodyId>& failedThisStep() const { return m_failed; }

private:
    std::unordered_map<BodyId, MaterialProfile> m_profile;
    std::unordered_map<BodyId, MaterialState>   m_state;
    std::vector<BodyId> m_failed;
};

// ================================================================ FRAGMENTS
struct ShatterResult {
    std::vector<BodyId> fragments;
    real momentumResidual = 0.0;   ///< |sum(m_i v_i) - P0|
    real angularResidual  = 0.0;
    real massResidual     = 0.0;
    bool brittle = true;
};

/// Splits a body into fragments on a hard impact. Brittle materials produce
/// radial wedges, ductile ones stretched threads. Linear momentum, angular
/// momentum and mass are conserved exactly (to double round-off).
class ImpactFragmentation : public SimulationSystem {
public:
    MaterialLab* lab = nullptr;

    int  maxFragments      = 9;
    real minFragmentRadius = 0.06;
    real spreadSpeed       = 1.6;
    real energyThreshold   = 12.0;   ///< J
    Rng  rng{0x1234ABCDull};

    const char* className() const override { return "ImpactFragmentation"; }

    ShatterResult shatter(BodyId id, const Vec2& impactPoint, const Vec2& impactDir);
    void onCollision(const CollisionEvent& e);
    void update(real dt) override;

    int totalFragments() const { return m_total; }
    const std::vector<ShatterResult>& history() const { return m_history; }

private:
    std::vector<BodyId> m_queue;
    std::vector<Vec2>   m_queuePoint;
    std::vector<Vec2>   m_queueDir;
    std::vector<ShatterResult> m_history;
    int m_total = 0;
};

// ================================================================ SURFACES
/// Adhesion, cohesion, capillary bridges, diffusion and osmosis.
class ContactChemistry : public SimulationSystem {
public:
    MaterialLab* lab = nullptr;

    real surfaceTension     = 0.0728;  ///< N/m, water at 20 C
    real contactAngle       = 0.0;     ///< rad
    real capillaryRange     = 0.05;    ///< m, how far a bridge stretches
    real capillaryMaxRadius = 0.25;    ///< only small parts get bridges
    real diffusionRate      = 0.02;
    real osmosisRate        = 0.01;
    bool enableAdhesion     = true;
    bool enableCapillary    = true;

    const char* className() const override { return "ContactChemistry"; }

    void setWetness(BodyId id, real w) { m_wet[id] = w; }
    real wetness(BodyId id) const;
    void setConcentration(BodyId id, real c) { m_conc[id] = c; }
    real concentration(BodyId id) const;
    void setSemiPermeable(BodyId id, bool v) { m_perm[id] = v; }

    void onContact(const CollisionEvent& e);
    void update(real dt) override;

    real lastAdhesionForce() const { return m_lastAdhesion; }
    int  bridgeCount() const { return m_bridges; }

private:
    struct Pair {
        BodyId a = INVALID_BODY, b = INVALID_BODY;
        Vec2 point, normal;
        real gap = 0.0;
    };

    std::vector<Pair> m_pairs;
    std::unordered_map<BodyId, real> m_wet;
    std::unordered_map<BodyId, real> m_conc;
    std::unordered_map<BodyId, bool> m_perm;
    real m_lastAdhesion = 0.0;
    int  m_bridges = 0;
};

// ================================================================ presets
MaterialProfile materialSteel();
MaterialProfile materialGlass();
MaterialProfile materialConcrete();
MaterialProfile materialRubber();
MaterialProfile materialNitinol();
MaterialProfile materialCloth();
MaterialProfile materialIce();
MaterialProfile materialWood();

AeroProfile aeroScarf();
AeroProfile aeroCrate();
AeroProfile aeroSphere();
AeroProfile aeroFlag();

} // namespace adv
} // namespace phys2d
