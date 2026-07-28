// phys2d v2 - демонстрация всего расширенного API + бенчмарки.
#include "phys2d/Extras.h"

#include <cstdio>
#include <fstream>
#include <iostream>

using namespace phys2d;

namespace {

BodyId spawnBox(World& w, const Vec2& p, real hw, real hh, BodyType type = BodyType::Dynamic,
                const char* name = "box") {
    BodyDef def;
    def.type = type;
    def.shape = Shape::box(hw * 2.0, hh * 2.0);
    def.position = p;
    def.name = name;
    return w.createBody(def);
}

BodyId spawnCircle(World& w, const Vec2& p, real r, real density = 1.0) {
    BodyDef def;
    def.type = BodyType::Dynamic;
    def.shape = Shape::circle(r);
    def.position = p;
    def.material.density = density;
    def.name = "circle";
    return w.createBody(def);
}

void buildStage(Engine& engine) {
    World& w = engine.world();

    // Пол и стены.
    spawnBox(w, Vec2(0.0, -0.5), 40.0, 0.5, BodyType::Static, "ground");
    spawnBox(w, Vec2(-40.0, 15.0), 0.5, 16.0, BodyType::Static, "wall_l");
    spawnBox(w, Vec2(40.0, 15.0), 0.5, 16.0, BodyType::Static, "wall_r");

    // Рампы из вогнутого контура (разбиение на выпуклые части).
    const std::vector<Vec2> concave = {
        Vec2(-6.0, 0.0), Vec2(6.0, 0.0), Vec2(6.0, 1.0), Vec2(1.0, 1.2),
        Vec2(0.0, 3.0), Vec2(-1.0, 1.2), Vec2(-6.0, 1.0)};
    int piece = 0;
    for (const Shape& s : shapesFromConcave(concave)) {
        BodyDef def;
        def.type = BodyType::Static;
        def.shape = s;
        def.position = Vec2(-18.0, 0.6);
        def.name = "ramp_piece";
        w.createBody(def);
        ++piece;
    }
    std::printf("  вогнутый контур -> %d выпуклых частей\n", piece);

    // Ландшафт из шума Перлина.
    const Terrain terrain = buildNoiseTerrain(w, 8.0, 38.0, 1.2, 2.2, 4242);
    std::printf("  ландшафт: %zu сегментов\n", terrain.segments.size());

    // Кривые Безье и NURBS — направляющая горка.
    const std::vector<Vec2> ctrl = {Vec2(-34.0, 12.0), Vec2(-26.0, 16.0), Vec2(-18.0, 6.0), Vec2(-10.0, 9.0)};
    const std::vector<Vec2> curve = sampleBezier(ctrl, 24);
    for (size_t i = 0; i + 1 < curve.size(); ++i) {
        const Vec2 a = curve[i], b = curve[i + 1];
        const Vec2 d = b - a;
        BodyDef def;
        def.type = BodyType::Static;
        def.shape = Shape::capsule(0.12, std::max((real)0.05, d.length()));
        def.position = (a + b) * 0.5;
        def.angle = std::atan2(d.y, d.x);
        def.name = "bezier";
        w.createBody(def);
    }
    const std::vector<Vec2> nurbs = sampleNurbs(ctrl, {1.0, 2.5, 0.6, 1.0}, 3, 16);
    std::printf("  безье: %zu точек, NURBS: %zu точек\n", curve.size(), nurbs.size());
}

void buildJoints(Engine& engine) {
    World& w = engine.world();

    // Мотор + шестерёнки с передаточным числом.
    BodyDef gearDef;
    gearDef.type = BodyType::Dynamic;
    gearDef.shape = Shape::gear(14, 1.1, 1.5);
    gearDef.position = Vec2(-6.0, 8.0);
    gearDef.name = "gear_a";
    const BodyId gearA = w.createBody(gearDef);

    gearDef.shape = Shape::gear(10, 0.8, 1.1);
    gearDef.position = Vec2(-2.8, 8.0);
    gearDef.name = "gear_b";
    const BodyId gearB = w.createBody(gearDef);

    const BodyId axleA = spawnBox(w, Vec2(-6.0, 8.0), 0.12, 0.12, BodyType::Static, "axle_a");
    const BodyId axleB = spawnBox(w, Vec2(-2.8, 8.0), 0.12, 0.12, BodyType::Static, "axle_b");
    w.createRevolute(axleA, gearA, Vec2(), Vec2());
    w.createRevolute(axleB, gearB, Vec2(), Vec2());

    createMotor(w, axleA, gearA, 3.0, 400.0);
    createGear(w, gearA, gearB, -1.4);

    // Призматический джойнт (поршень).
    const BodyId railBase = spawnBox(w, Vec2(6.0, 6.0), 0.3, 0.3, BodyType::Static, "rail");
    const BodyId slider = spawnBox(w, Vec2(8.0, 6.0), 0.6, 0.25, BodyType::Dynamic, "slider");
    createPrismatic(w, railBase, slider, Vec2(6.0, 6.0), Vec2(1.0, 0.0));

    // Жёсткая сварка с мягкой жёсткостью.
    const BodyId weldA = spawnBox(w, Vec2(12.0, 9.0), 0.5, 0.2, BodyType::Static, "weld_anchor");
    const BodyId weldB = spawnBox(w, Vec2(13.2, 9.0), 0.6, 0.2, BodyType::Dynamic, "weld_arm");
    SoftParams soft;
    soft.frequencyHz = 6.0;
    soft.dampingRatio = 0.4;
    WeldJoint* weld = createWeld(w, weldA, weldB, Vec2(12.6, 9.0), soft);

    // Блок (полиспаст) с передаточным отношением 2:1.
    const BodyId loadA = spawnBox(w, Vec2(18.0, 8.0), 0.4, 0.4, BodyType::Dynamic, "pulley_load_a");
    const BodyId loadB = spawnBox(w, Vec2(22.0, 10.0), 0.4, 0.4, BodyType::Dynamic, "pulley_load_b");
    createPulley(w, loadA, loadB, Vec2(18.0, 14.0), Vec2(22.0, 14.0), Vec2(), Vec2(), 2.0);

    // Трос с провисанием по цепной линии.
    buildCatenaryRope(w, Vec2(26.0, 13.0), Vec2(34.0, 13.0), 11.0, 18, 0.09, 1.2);

    // Разрушаемый джойнт.
    const BodyId hangAnchor = spawnBox(w, Vec2(0.0, 14.0), 0.3, 0.3, BodyType::Static, "hang_anchor");
    const BodyId heavy = spawnCircle(w, Vec2(0.0, 11.5), 0.7, 25.0);
    Constraint* rope = w.createDistance(hangAnchor, heavy, Vec2(), Vec2(), 2.5);
    engine.breakables().watch(rope, 900.0);
    engine.breakables().setCallback([](const JointBreakEvent& e) {
        std::printf("  [событие] джойнт разорван, сила = %.1f\n", (double)e.force);
    });

    std::printf("  джойнты: motor+gear+prismatic+weld(%s)+pulley+catenary+breakable\n",
                weld ? "ok" : "fail");
    (void)slider;
}

void buildDestruction(Engine& engine) {
    World& w = engine.world();

    engine.fracture().enabled = true;
    engine.fracture().impulseThreshold = 12.0;
    engine.fracture().pieces = 7;
    engine.fracture().setCallback([](const FractureEvent& e) {
        std::printf("  [событие] тело %u разрушено на %zu осколков\n", e.original, e.fragments.size());
    });

    for (int i = 0; i < 8; ++i) {
        const BodyId id = spawnBox(w, Vec2(-12.0 + i * 1.6, 2.0), 0.7, 0.7);
        engine.fracture().makeBreakable(id, 10.0);
    }
    // Снаряд с CCD.
    const BodyId bullet = spawnCircle(w, Vec2(-20.0, 3.0), 0.25, 30.0);
    w.body(bullet)->velocity = Vec2(90.0, 0.0);
    engine.ccd().enabled = true;
    engine.ccd().speedThreshold = 20.0;
}

void buildSoftBodies(Engine& engine) {
    engine.softBodies().enabled = true;

    SoftBody cloth = SoftBody::grid(Vec2(-4.0, 16.0), 6.0, 4.0, 12, 8, 6.0);
    cloth.model = SoftModel::Hybrid;
    cloth.tearStrain = 0.85;
    cloth.nodes[0].pinned = true;
    cloth.nodes[11].pinned = true;
    engine.softBodies().add(cloth);

    SoftBody blob = SoftBody::fromPolygon(
        {Vec2(0.0, 0.0), Vec2(2.4, 0.0), Vec2(3.0, 1.8), Vec2(1.2, 3.0), Vec2(-0.6, 1.8)}, 0.35, 8.0);
    blob.model = SoftModel::FEM;
    blob.youngModulus = 4.0e4;
    blob.pressure = 30.0;
    for (SoftNode& n : blob.nodes) n.position += Vec2(4.0, 12.0);
    blob.rebuildRest();
    engine.softBodies().add(blob);

    SoftBody cord = SoftBody::rope(Vec2(20.0, 16.0), Vec2(24.0, 16.0), 20, 2.0);
    cord.nodes.front().pinned = true;
    engine.softBodies().add(cord);

    RagdollConfig cfg;
    cfg.position = Vec2(30.0, 8.0);
    const Ragdoll doll = createRagdoll(engine.world(), cfg);

    std::printf("  мягкие тела: %zu, ragdoll: %zu частей / %zu джойнтов\n",
                engine.softBodies().count(), doll.parts.size(), doll.joints.size());
}

void buildFluid(Engine& engine) {
    ParticleSystem& fluid = engine.fluid();
    fluid.enabled = true;
    fluid.domain.min = Vec2(-38.0, 0.2);
    fluid.domain.max = Vec2(-24.0, 26.0);
    fluid.smoothingRadius = 0.45;
    fluid.particleMass = 1.4;
    fluid.viscosity = 14.0;
    fluid.surfaceTension = 1.2;
    fluid.emitBlock(Vec2(-36.0, 1.0), 10.0, 6.0, 0.36);
    std::printf("  SPH частиц: %zu\n", fluid.count());
}

void buildFields(Engine& engine) {
    FieldSystem& fields = engine.fields();
    fields.enabled = true;
    fields.registry = &engine.properties();
    fields.flags = FIELD_WIND | FIELD_ELECTROSTATIC | FIELD_MAGNETIC | FIELD_TIDAL | FIELD_LIGHT;
    fields.windBase = Vec2(3.0, 0.0);
    fields.windTurbulence = 6.0;
    fields.tidalSources.push_back(PointSource{Vec2(0.0, 384400.0), 7.3e22});   // Луна на реальной дистанции (км -> масштаб сцены)
    fields.lightSources.push_back(PointSource{Vec2(0.0, 1.496e11), 3.8e26});

    ThermalSystem& thermal = engine.thermal();
    thermal.enabled = true;
    thermal.registry = &engine.properties();

    // Заряженные и горячие тела.
    World& w = engine.world();
    for (int i = 0; i < 6; ++i) {
        const BodyId id = spawnCircle(w, Vec2(-2.0 + i * 1.1, 20.0), 0.3, 2.0);
        BodyPhysics& p = engine.properties().ref(id);
        p.charge = (i % 2 == 0) ? 2.0e-5 : -2.0e-5;
        p.magneticMoment = 0.4;
        p.temperature = (i == 0) ? 1500.0 : 293.0;   // первое тело — расплавленое
        p.piezoCoefficient = 2.3e-12;
    }
}

void buildSensorsAndFilters(Engine& engine) {
    World& w = engine.world();

    // Слои столкновений: мусор (0x2) не сталкивается с мусором.
    for (int i = 0; i < 30; ++i) {
        const BodyId id = spawnCircle(w, Vec2(10.0 + (i % 10) * 0.5, 18.0 + (i / 10) * 0.6), 0.18);
        Filter f;
        f.category = 0x2;
        f.mask = 0xFFFFFFFFu & ~0x2u;
        engine.filters().set(id, f);
    }
    engine.filters().install(w);

    // Зона-триггер.
    const BodyId zone = spawnBox(w, Vec2(16.0, 3.0), 3.0, 3.0, BodyType::Static, "trigger_zone");
    Filter sensorFilter;
    sensorFilter.sensor = true;
    engine.filters().set(zone, sensorFilter);
    engine.sensors().enabled = true;
    engine.sensors().setFilterRegistry(&engine.filters());
    engine.sensors().add(zone);

    static int entered = 0, exited = 0;
    engine.sensors().setCallback([](const SensorEvent& e) {
        if (e.phase == SensorPhase::Enter) ++entered;
        else if (e.phase == SensorPhase::Exit) ++exited;
    });

    // Материалы поверхности: анизотропное трение (лёд вдоль, резина поперёк).
    const BodyId anisoPlate = spawnBox(w, Vec2(24.0, 1.4), 4.0, 0.3, BodyType::Static, "aniso");
    SurfaceAssignment sa;
    sa.defaultMaterial = materialByName("steel");
    sa.anisotropyAxis = Vec2(1.0, 0.0);
    sa.frictionAlong = 0.05;
    sa.frictionAcross = 1.4;
    engine.surfaces().set(anisoPlate, sa);

    for (int i = 0; i < 12; ++i) spawnBox(w, Vec2(21.0 + i * 0.55, 2.6), 0.2, 0.2);
}

void runSimulation(Engine& engine, int steps, real dt) {
    engine.recorder().enabled = true;
    engine.recorder().recordContacts = false;   // иначе лог растёт на десятки МБ
    engine.recorder().snapshotInterval = 4;
    engine.islands().enabled = true;
    engine.audio().enabled = true;
    engine.graph().enabled = true;
    engine.safety().enabled = true;
    engine.gc().enabled = true;
    engine.gc().domain.min = Vec2(-200.0, -100.0);
    engine.gc().domain.max = Vec2(200.0, 300.0);

    for (int i = 0; i < steps; ++i) {
        engine.step(dt);

        if (i == steps / 3) {
            // Луч с фильтрацией по слоям: все попадания.
            std::vector<RayHit> hits;
            rayCastAll(engine.world(), Vec2(-38.0, 4.0), Vec2(38.0, 4.0), hits, &engine.filters(), 0xFFFFFFFFu);
            std::printf("  луч (шаг %d): %zu попаданий", i, hits.size());
            if (!hits.empty())
                std::printf(", ближайшее t=%.3f в (%.2f, %.2f)",
                            (double)hits.front().fraction, (double)hits.front().point.x,
                            (double)hits.front().point.y);
            std::printf("\n");

            // Ручной взрыв — разлом по Voronoi.
            for (RigidBody* b : engine.world().bodies()) {
                if (b->name == "box" && b->isDynamic()) {
                    engine.fracture().fracture(b->id, b->position, 9);
                    break;
                }
            }
        }
    }
}

void printSummary(Engine& engine) {
    World& w = engine.world();
    const ProfileData& pd = w.profile();

    std::printf("\n=== Итоги симуляции ===\n");
    std::printf("%s\n", engine.statusLine().c_str());
    std::printf("тел: %zu, контактов: %zu, активных: %zu, арена: %.1f МБ\n",
                w.bodyCount(), w.contactCount(), w.awakeCount(), w.arenaBytes() / 1048576.0);
    std::printf("островов: %zu (крупнейший %d), разрушений: %d, разрывов ткани: %d\n",
                engine.islands().islandCount(), engine.islands().largestIsland(),
                engine.fracture().fracturesTotal(), engine.softBodies().tearsThisStep());
    std::printf("CCD сработало: %d, ограничено скоростей: %d, NaN-починок: %d, собрано тел: %d\n",
                engine.ccd().resolvedThisStep(), engine.safety().clampedThisStep(), engine.safety().repairedTotal(),
                engine.gc().collectedTotal());
    std::printf("расплавленных тел: %d, аудиособытий: %zu, семплов: %zu\n",
                engine.thermal().moltenCount(), engine.audio().events().size(),
                engine.audio().buffer().size());
    std::printf("снапшотов записи: %zu, записей лога: %zu, кадр: %llu\n",
                engine.recorder().snapshotCount(), engine.recorder().records().size(),
                (unsigned long long)engine.recorder().frame());
    std::printf("таблицы: sin[%zu], материалов %d, статические %.1f КБ, ядро: %s\n",
                trigTableEntries(), materialCount(), staticTableBytes() / 1024.0,
                kernelSignature(KERNEL_WARMSTART | KERNEL_FRICTION | KERNEL_RESTITUTION, true));
    std::printf("\n%s\n", pd.toString().c_str());
    std::printf("График Total (мс):\n%s\n", engine.graph().asciiPlot(Stage::Total, 58, 7).c_str());
}

void runExports(Engine& engine) {
    World& w = engine.world();

    w.saveToFile("demo_v2_state.json");
    exportBox2D(w, "demo_v2_scene.b2d");
    exportBullet(w, "demo_v2_scene.bullet");
    engine.audio().writeWav("demo_v2_audio.wav");
    engine.recorder().saveBinary("demo_v2_events.p2log");

    // Перемотка назад на 10 кадров.
    const bool rewound = engine.recorder().rewind(10);
    std::printf("экспорт: json / .b2d / .bullet / .wav / .p2log; перемотка: %s\n",
                rewound ? "успешно" : "нет снапшотов");
}

void runScripting(Engine& engine) {
    engine.script().enabled = true;
    const std::string out = engine.script().execute(
        "radius = 0.3 * 2\n"
        "spawn circle 0 24 radius\n"
        "gravity 0 -9.81\n"
        "iterations 14 12\n"
        "repeat 3 step 1\n"
        "stats\n");
    std::printf("\n=== Скриптовый движок ===\n%s", out.c_str());

    engine.console().script = &engine.script();
    engine.console().enabled = true;
    engine.console().pushCommand("print консоль работает");
    engine.console().update(0.0);
    for (const std::string& line : engine.console().log()) std::printf("console: %s\n", line.c_str());
}

} // namespace

int main(int argc, char** argv) {
    const CommandLineOptions opt = parseCommandLine(argc, argv);
    if (opt.extra.count("help")) {
        std::printf("%s", commandLineHelp().c_str());
        return 0;
    }

    if (opt.runBenchmark) {
        BenchmarkSuite suite;
        std::printf("Запуск бенчмарков: %zu сцен\n", suite.sceneNames().size());
        const std::vector<BenchmarkResult> results =
            opt.scene.empty() ? suite.runAll(std::max(60, opt.steps / 4), opt.dt)
                              : std::vector<BenchmarkResult>{suite.run(opt.scene, opt.steps, opt.dt)};
        const std::string report = suite.report(results);
        std::printf("\n%s\n", report.c_str());

        std::ofstream(opt.report.empty() ? "benchmark_report.md" : opt.report) << report;
        std::ofstream("benchmark_report.csv") << suite.csv(results);
        return 0;
    }

    WorldConfig cfg;
    cfg.fixedTimeStep = opt.dt;
    cfg.threadCount = opt.threads;
    Engine engine(cfg);

    std::printf("=== phys2d v2: сборка сцены ===\n");
    buildStage(engine);
    buildJoints(engine);
    buildDestruction(engine);
    buildSoftBodies(engine);
    buildFluid(engine);
    buildFields(engine);
    buildSensorsAndFilters(engine);

    const int steps = std::max(60, opt.steps);
    std::printf("\n=== Симуляция: %d шагов по %.4f с ===\n", steps, (double)opt.dt);
    runSimulation(engine, steps, opt.dt);

    printSummary(engine);
    runExports(engine);
    runScripting(engine);

    if (ErrorStack::instance().hasError())
        std::printf("\nТрассировка ошибок:\n%s\n", ErrorStack::instance().trace().c_str());

    std::printf("\nГотово.\n");
    return 0;
}
