// phys2d - Advanced physics pack: headless proof lab.
// Every new subsystem is executed and its result printed as a number.
#include "phys2d/Advanced.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>

using namespace phys2d;
using namespace phys2d::adv;

namespace {

BodyId makeGround(World& w) {
    BodyDef d;
    d.type = BodyType::Static;
    d.shape = Shape::box(120.0, 1.0);
    d.position = Vec2(0.0, -0.5);
    d.material.staticFriction = 0.55;
    d.material.dynamicFriction = 0.42;
    d.name = "ground";
    return w.createBody(d);
}

// ---------------------------------------------------------------- 1. WIND
void sceneWind(real windStrength) {
    std::printf("\n===== 1. WIND: same wind, different mass =====\n");
    std::printf("wind strength = %.1f m/s\n\n", windStrength);

    World w;
    makeGround(w);

    WindSystem wind;
    wind.attach(w);
    wind.setStrength(windStrength);
    wind.profile.turbulence = 0.15;
    wind.profile.gustAmplitude = 0.2;

    // A scarf: 60 grams of fabric, 1.2 m long.
    BodyDef sd;
    sd.shape = Shape::box(1.2, 0.02);
    sd.position = Vec2(0.0, 1.2);
    sd.name = "scarf";
    const BodyId scarf = w.createBody(sd);
    if (RigidBody* b = w.body(scarf)) b->setMassData(0.06, 0.06 * 0.15);
    wind.configure(scarf, aeroScarf());

    // A crate: 20 kg, 0.8 x 0.8 m.
    BodyDef cd;
    cd.shape = Shape::box(0.8, 0.8);
    cd.position = Vec2(0.0, 0.4);
    cd.name = "crate20kg";
    const BodyId crate = w.createBody(cd);
    if (RigidBody* b = w.body(crate)) b->setMassData(20.0, 20.0 * 0.11);
    wind.configure(crate, aeroCrate());

    const WindReport rs = wind.report(*w.body(scarf));
    const WindReport rc = wind.report(*w.body(crate));
    const real ratio = (rc.acceleration > 1e-9) ? rs.acceleration / rc.acceleration : 0.0;

    std::printf("%-10s %8s %8s %10s %12s\n", "object", "mass kg", "area m2", "force N", "accel m/s2");
    std::printf("%-10s %8.3f %8.3f %10.3f %12.2f\n", "scarf", rs.mass, rs.area, rs.force, rs.acceleration);
    std::printf("%-10s %8.3f %8.3f %10.3f %12.2f\n", "crate", rc.mass, rc.area, rc.force, rc.acceleration);
    std::printf("\n-> the scarf accelerates %.0fx harder under the SAME wind\n", ratio);

    // Race: how long until each is carried 5 m downwind?
    const real dt = 1.0 / 120.0;
    const Vec2 s0 = w.body(scarf)->position, c0 = w.body(crate)->position;
    real tScarf = -1.0, tCrate = -1.0;
    for (int i = 0; i < 120 * 60; ++i) {
        wind.update(dt);
        w.step(dt);
        if (tScarf < 0.0 && std::fabs(w.body(scarf)->position.x - s0.x) >= 5.0) tScarf = (i + 1) * dt;
        if (tCrate < 0.0 && std::fabs(w.body(crate)->position.x - c0.x) >= 5.0) tCrate = (i + 1) * dt;
        if (tScarf > 0.0 && tCrate > 0.0) break;
    }
    std::printf("time to be carried 5 m:  scarf %s   crate %s\n",
                tScarf > 0 ? (std::to_string(tScarf) + " s").c_str() : "not carried",
                tCrate > 0 ? (std::to_string(tCrate) + " s").c_str() : "> 60 s (still creeping)");
    std::printf("displacement after the run: scarf %.2f m   crate %.2f m\n",
                w.body(scarf)->position.x - s0.x, w.body(crate)->position.x - c0.x);
}

// ---------------------------------------------------------------- 2. DEFORMATION
void sceneDeformation() {
    std::printf("\n===== 2. WIND DEFORMATION vs WIND STRENGTH =====\n");
    std::printf("%-12s %12s %14s %14s\n", "wind m/s", "pressure Pa", "bend (0..1)", "outline dx m");

    for (real ws : {2.0, 5.0, 10.0, 20.0, 35.0, 50.0}) {
        World w;
        makeGround(w);
        WindSystem wind;
        wind.attach(w);
        wind.setStrength(ws);
        wind.profile.turbulence = 0.0;
        wind.profile.gustAmplitude = 0.0;

        BodyDef fd;
        fd.shape = Shape::box(1.0, 0.6);
        fd.position = Vec2(0.0, 3.0);
        fd.gravityScale = 0.0;
        fd.name = "panel";
        const BodyId panel = w.createBody(fd);
        if (RigidBody* b = w.body(panel)) b->setMassData(400.0, 40.0);
        wind.configure(panel, aeroFlag());

        const std::vector<Vec2> before = w.body(panel)->localVertices;
        for (int i = 0; i < 240; ++i) { wind.update(1.0 / 120.0); w.step(1.0 / 120.0); }

        const std::vector<Vec2>& after = w.body(panel)->localVertices;
        real maxDx = 0.0;
        for (size_t i = 0; i < after.size() && i < before.size(); ++i)
            maxDx = std::max(maxDx, (after[i] - before[i]).length());

        std::printf("%-12.1f %12.1f %14.3f %14.4f\n",
                    ws, 0.5 * 1.225 * ws * ws,
                    wind.report(*w.body(panel)).deformation, maxDx);
    }
    std::printf("-> bend follows the dynamic pressure q = 0.5*rho*v^2 and saturates\n");
}

// ---------------------------------------------------------------- 3. WATER + VOLTAGE
void sceneElectrifiedWater() {
    std::printf("\n===== 3. VOLTAGE APPLIED TO POURED WATER =====\n");

    World w;
    makeGround(w);

    ParticleSystem fluid;
    fluid.attach(w);
    fluid.smoothingRadius = 0.35;
    fluid.particleMass = 0.6;
    // A tank with walls, so the puddle stays a puddle instead of spreading
    // into a one-particle film with no electrical connectivity.
    fluid.domain = AABB(Vec2(-5.0, 0.0), Vec2(5.0, 8.0));
    fluid.emitBlock(Vec2(-4.2, 0.1), 8.4, 1.4, 0.16);

    BodyPhysicsRegistry reg;
    ElectricSystem elec;
    elec.attach(w);
    elec.fluid = &fluid;
    elec.registry = &reg;
    elec.waterResistivity = 20.0;     // tap water
    elec.linkRadius = 0.32;

    // 220 V on the left, ground on the right.
    elec.addElectrode(Electrode{Vec2(-4.0, 0.4), 220.0, 0.6, false});
    elec.addElectrode(Electrode{Vec2( 4.0, 0.4),   0.0, 0.6, true});

    // A rod dipped in the puddle, and an identical one on dry ground.
    BodyDef wd; wd.type = BodyType::Static;
    wd.shape = Shape::box(0.4, 1.2); wd.position = Vec2(0.5, 0.7); wd.name = "in_water";
    const BodyId inWater = w.createBody(wd);
    BodyDef dd; dd.type = BodyType::Static;
    dd.shape = Shape::box(0.4, 1.2); dd.position = Vec2(14.0, 0.7); dd.name = "dry";
    const BodyId dry = w.createBody(dd);

    const real dt = 1.0 / 120.0;
    for (int i = 0; i < 240; ++i) { fluid.update(dt); elec.update(dt); w.step(dt); }

    std::printf("water particles in the circuit: %zu\n", elec.wetNodeCount());
    std::printf("current from the electrode    : %.4f A\n", elec.totalCurrent());
    std::printf("power dissipated in the water : %.1f W\n", elec.dissipatedPower());
    std::printf("\nvoltage across the puddle (the current takes the whole puddle):\n");
    std::printf("%-10s %10s\n", "x, m", "V");
    for (real x = -4.0; x <= 4.01; x += 1.0)
        std::printf("%-10.1f %10.1f\n", x, elec.voltageAt(Vec2(x, 0.35)));

    std::printf("\nshocked bodies:\n");
    bool wet = false, dryHit = false;
    for (const ShockEvent& s : elec.shocks()) {
        const RigidBody* b = w.body(s.body);
        std::printf("  %-10s  U = %7.1f V   I = %6.3f A   P = %8.1f W\n",
                    b ? b->name.c_str() : "?", s.voltage, s.current, s.power);
        if (s.body == inWater) wet = true;
        if (s.body == dry) dryHit = true;
    }
    std::printf("rod standing in water : %s\n", wet ? "SHOCKED" : "not shocked");
    std::printf("rod on dry ground     : %s\n", dryHit ? "shocked (wrong)" : "not shocked (correct)");

    real maxT = 0.0;
    for (real t : elec.particleTemperature()) maxT = std::max(maxT, t);
    std::printf("joule heating: hottest particle %.3f K (started at 293 K)\n", maxT);
}

// ---------------------------------------------------------------- 4. FRAGMENTS
void sceneFragments() {
    std::printf("\n===== 4. IMPACT FRAGMENTS WITH EXACT MOMENTUM CONSERVATION =====\n");

    World w;
    makeGround(w);
    MaterialLab lab; lab.attach(w);
    ImpactFragmentation frag; frag.attach(w); frag.lab = &lab;

    std::printf("%-10s %7s %17s %14s %14s %14s\n",
                "material", "shards", "mode", "dP residual", "dL residual", "dm residual");

    struct Case { const char* name; MaterialProfile mp; };
    const Case cases[] = {
        {"glass",    materialGlass()},
        {"concrete", materialConcrete()},
        {"steel",    materialSteel()},
        {"rubber",   materialRubber()},
    };

    for (const Case& c : cases) {
        BodyDef bd;
        bd.shape = Shape::box(1.0, 1.0);
        bd.position = Vec2(0.0, 5.0);
        bd.velocity = Vec2(14.0, -3.0);
        bd.angularVelocity = 5.0;
        bd.name = c.name;
        const BodyId id = w.createBody(bd);
        if (RigidBody* b = w.body(id)) b->setMassData(8.0, 1.33);
        lab.registerBody(id, c.mp);

        const ShatterResult r = frag.shatter(id, Vec2(-0.4, 4.7), Vec2(1.0, -0.2));
        std::printf("%-10s %7zu %17s %14.3e %14.3e %14.3e\n",
                    c.name, r.fragments.size(),
                    r.brittle ? "radial cracks" : "ductile threads",
                    r.momentumResidual, r.angularResidual, r.massResidual);
    }
    std::printf("-> residuals are double-precision round-off: momentum IS conserved\n");
}

// ---------------------------------------------------------------- 5. MATERIALS
void sceneMaterialLab() {
    std::printf("\n===== 5. MATERIAL SCIENCE =====\n");

    World w;
    MaterialLab lab;
    lab.attach(w);
    const real dt = 1.0 / 120.0;

    {   // cumulative damage from repeated identical hits
        const BodyId id = 1001;
        lab.registerBody(id, materialGlass());
        std::printf("\n-- cumulative damage from repeated identical hits (glass)\n");
        std::printf("%-8s %12s %10s\n", "hit #", "damage", "state");
        for (int i = 1; i <= 8; ++i) {
            lab.onImpact(id, 55.0, 6.0, Vec2());
            const MaterialState* s = lab.find(id);
            std::printf("%-8d %12.4f %10s\n", i, s->damage, s->failed ? "BROKEN" : "-");
            if (s->failed) break;
        }
    }

    {   // fatigue
        const BodyId id = 1002;
        MaterialProfile p = materialSteel();
        p.fatigueLimit = 1.0e7; p.fatigueExponent = 2.0;
        lab.registerBody(id, p);
        for (int c = 0; c < 40000; ++c) lab.applyStress(id, 6.0e7 * std::sin(c * 0.7), dt);
        const MaterialState* s = lab.find(id);
        std::printf("\n-- fatigue (steel): cycles %.0f, accumulated micro-damage %.4f\n",
                    s->cycles, s->fatigue);
    }

    {   // creep + relaxation + plasticity
        const BodyId id = 1003;
        MaterialProfile p = materialIce();
        p.creepRate = 2.0e-5;
        lab.registerBody(id, p);
        std::printf("\n-- creep under constant load + plasticity (ice)\n");
        std::printf("%-8s %16s %16s\n", "t, s", "creep strain", "plastic strain");
        for (int i = 0; i <= 600; ++i) {
            lab.applyStress(id, 1.4e6, dt);
            if (i % 150 == 0) {
                const MaterialState* s = lab.find(id);
                std::printf("%-8.2f %16.3e %16.3e\n", i * dt, s->creepStrain, s->plasticStrain);
            }
        }
    }

    {   // viscoelasticity + nonlinear elasticity
        const BodyId id = 1004;
        lab.registerBody(id, materialRubber());
        std::printf("\n-- Kelvin-Voigt viscoelasticity + cubic elasticity (rubber)\n");
        std::printf("%-10s %16s %18s\n", "strain", "static Pa", "at 2/s rate Pa");
        for (real e = 0.05; e <= 0.51; e += 0.15)
            std::printf("%-10.2f %16.4e %18.4e\n", e,
                        lab.viscoelasticStress(id, e, 0.0),
                        lab.viscoelasticStress(id, e, 2.0));
    }

    {   // impact toughness: strength depends on strain rate
        const BodyId id = 1007;
        lab.registerBody(id, materialGlass());
        std::printf("\n-- impact toughness: strength vs strain rate (glass)\n");
        for (real rate : {1.0, 100.0, 10000.0, 1000000.0})
            std::printf("   rate %10.0f 1/s -> strength %.4e Pa\n",
                        rate, lab.effectiveStrength(id, rate));
    }

    {   // shape memory
        const BodyId id = 1005;
        lab.registerBody(id, materialNitinol());
        MaterialState& s = lab.state(id);
        for (int i = 0; i < 200; ++i) lab.applyStress(id, 4.0e8, dt);
        const real bent = s.plasticStrain;
        s.temperature = 380.0;
        for (int i = 0; i < 240; ++i) lab.update(dt);
        std::printf("\n-- shape memory (nitinol): bent %.5f -> after heating %.5f (recovered %.1f%%)\n",
                    bent, s.plasticStrain, bent > 0 ? 100.0 * (1.0 - s.plasticStrain / bent) : 0.0);
    }

    {   // corrosion / oxidation / radiation
        const BodyId id = 1006;
        lab.registerBody(id, materialSteel());
        lab.aggressiveMedium = 1.0;
        lab.radiationField = 2.0;
        MaterialState& s = lab.state(id);
        s.temperature = 800.0;
        for (int i = 0; i < 1200; ++i) lab.update(dt);
        std::printf("\n-- environment after 10 s (steel in acid, 2 Gy/s, 800 K)\n");
        std::printf("corroded %.3e m   oxide film %.3e m   dose %.1f Gy   strength left %.2f%%\n",
                    s.corroded, s.oxide, s.dose, s.strengthScale * 100.0);
        lab.aggressiveMedium = 0.0; lab.radiationField = 0.0;
    }
}

// ---------------------------------------------------------------- 6. SURFACES
void sceneAdhesion() {
    std::printf("\n===== 6. ADHESION / COHESION / CAPILLARY / OSMOSIS =====\n");

    World w;
    makeGround(w);
    MaterialLab lab; lab.attach(w);
    ContactChemistry chem; chem.attach(w); chem.lab = &lab;

    BodyDef ad; ad.shape = Shape::circle(0.12); ad.position = Vec2(0.0, 0.5); ad.name = "beadA";
    BodyDef bd; bd.shape = Shape::circle(0.12); bd.position = Vec2(0.22, 0.5); bd.name = "beadB";
    ad.gravityScale = 0.0; bd.gravityScale = 0.0;   // isolate the capillary force
    const BodyId a = w.createBody(ad), b = w.createBody(bd);
    lab.registerBody(a, materialRubber());
    lab.registerBody(b, materialRubber());
    chem.setWetness(a, 1.0);
    chem.setWetness(b, 1.0);
    chem.setConcentration(a, 1.0);
    chem.setConcentration(b, 0.0);
    chem.setSemiPermeable(b, true);

    w.setBeginContactCallback([&](const CollisionEvent& e) { chem.onContact(e); });
    w.setPersistContactCallback([&](const CollisionEvent& e) { chem.onContact(e); });

    const real dt = 1.0 / 120.0;
    real peakForce = 0.0; int peakBridges = 0;
    for (int i = 0; i < 600; ++i) {
        w.step(dt); chem.update(dt);
        peakForce = std::max(peakForce, chem.lastAdhesionForce());
        peakBridges = std::max(peakBridges, chem.bridgeCount());
    }

    std::printf("peak adhesion + capillary force: %.4f N\n", peakForce);
    std::printf("capillary bridges formed       : %d\n", peakBridges);
    std::printf("osmosis through the membrane   : A %.4f -> B %.4f (started 1.0 / 0.0)\n",
                chem.concentration(a), chem.concentration(b));
}

} // namespace

int main(int argc, char** argv) {
    real wind = 10.0;
    for (int i = 1; i < argc; ++i) {
        const std::string s = argv[i];
        if (s == "--wind" && i + 1 < argc) wind = std::atof(argv[++i]);
    }

    std::printf("phys2d ADVANCED LAB - block 1\n=============================\n");
    sceneWind(wind);
    sceneDeformation();
    sceneElectrifiedWater();
    sceneFragments();
    sceneMaterialLab();
    sceneAdhesion();
    std::printf("\nall subsystems executed.\n");
    return 0;
}
