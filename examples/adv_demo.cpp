// phys2d v3 — демо расширенного блока: составные тела, поверхности, фильтры,
// управление, ветер, диагностика, стабильность, запросы, сервисные системы.
#include "phys2d/Advanced2.h"

#include <cstdio>

using namespace phys2d;

int main() {
    WorldConfig cfg;
    applyPreset(cfg, Preset::Balance);
    cfg.gridCellSize = 3.0;
    cfg.aabbMargin   = 0.12;
    World w(cfg);

    // ---- единицы
    UnitScale px = UnitScale::of(Units::Pixels, 32.0);
    std::printf("units: 64 px = %.3f m\n", (double)px.len(64.0));

    // ---- дерево материалов
    MaterialTree mats;
    mats.loadStandardLibrary();
    Material steel = mats.resolve("steel");
    Material balsa = mats.resolve("balsa");
    Material ice   = mats.resolve("ice");
    Material oak   = mats.resolve("oak");
    Material concrete = mats.resolve("concrete");
    std::printf("materials: %zu, steel rho=%.0f, balsa rho=%.0f mu=%.2f (\u043d\u0430\u0441\u043b\u0435\u0434\u043e\u0432\u0430\u043d\u043e), ice mu=%.2f\n",
                mats.size(), (double)steel.density, (double)balsa.density,
                (double)balsa.staticFriction, (double)ice.staticFriction);

    // ---- пол
    BodyDef gd;
    gd.type     = BodyType::Static;
    gd.shape    = Shape::box(40.0, 1.0);
    gd.material = concrete;
    gd.position = Vec2(0.0, -0.5);
    BodyId ground = w.createBody(gd);

    // ---- составное тело: молоток (деревянная ручка + стальная голова)
    std::vector<CompoundPart> parts;
    CompoundPart handle;
    handle.shape   = Shape::box(0.2, 2.0);
    handle.offset  = Vec2(0.0, 0.0);
    handle.density = oak.density;
    parts.push_back(handle);
    CompoundPart head;
    head.shape   = Shape::box(1.0, 0.5);
    head.offset  = Vec2(0.0, 1.1);
    head.density = steel.density;
    parts.push_back(head);

    Compound hammer = createCompound(w, parts, Vec2(-6.0, 3.0), 0.3, BodyType::Dynamic, 45000.0);
    std::printf("compound: parts=%zu m=%.1f kg I=%.2f com=(%.3f, %.3f)\n",
                hammer.parts.size(), (double)hammer.mass, (double)hammer.inertia,
                (double)hammer.center.x, (double)hammer.center.y);

    // ---- правила столкновений и поверхности
    CollisionRules rules;
    ContactTuner   tuner;
    tuner.surfaces = &rules.surfaces;
    rules.install(w);

    SurfaceProfile& gp = rules.surfaces.ref(ground);
    gp.frictionAxis      = Vec2(1.0, 0.0);
    gp.frictionAlong     = 0.35;      // вдоль оси скользит
    gp.frictionAcross    = 2.20;      // поперёк цепляет
    gp.oneWayFriction    = 0.60;
    gp.spinFriction      = 0.50;
    gp.angleAbsorption   = 0.25;
    gp.restitutionAtRest = 0.55;
    gp.restitutionAtSpeed = 0.05;

    ControlSystem control;
    for (BodyId id : hammer.parts) {
        BodyControl& bc = control.ref(id);
        bc.limits.maxSpeed        = 40.0;
        bc.limits.maxAngularSpeed = 25.0;
        bc.limits.maxAngularStep  = 0.25;
    }

    // ---- случайные выпуклые полигоны + слои
    uint32_t seed = 12345u;
    rules.setLayerRule(1, 2, false);          // слой 1 и слой 2 не видят друг друга
    for (int i = 0; i < 40; ++i) {
        BodyDef d;
        d.type     = BodyType::Dynamic;
        d.shape    = randomConvexPolygon(seed, 3 + (i % 6), 0.28);
        d.material = (i % 3 == 0) ? ice : ((i % 3 == 1) ? balsa : oak);
        d.position = Vec2(-8.0 + (real)(i % 10) * 1.7, 2.0 + (real)(i / 10) * 1.3);
        BodyId id  = w.createBody(d);
        rules.surfaces.ref(id).layer = (i % 2) ? 1 : 2;
    }

    // ---- кинематическая платформа с целевой скоростью и сглаживанием
    BodyDef pd;
    pd.type     = BodyType::Kinematic;
    pd.shape    = Shape::box(4.0, 0.3);
    pd.material = steel;
    pd.position = Vec2(6.0, 1.5);
    BodyId platform = w.createBody(pd);
    BodyControl& pc = control.ref(platform);
    pc.useTargetVelocity = true;
    pc.targetVelocity    = Vec2(1.5, 0.0);
    pc.smoothing         = 0.25;

    // ---- AXIS LOCK + CONSTANT FORCE (лифт)
    BodyDef ld;
    ld.type     = BodyType::Dynamic;
    ld.shape    = Shape::box(0.6, 0.6);
    ld.material = steel;
    ld.position = Vec2(2.0, 4.0);
    BodyId lift = w.createBody(ld);
    BodyControl& lc = control.ref(lift);
    lc.lock.x                = true;
    lc.constantForce         = Vec2(0.0, 250.0);
    lc.limits.maxSpeed       = 6.0;
    lc.limits.maxAngularStep = 0.15;

    // ---- SOFT CONSTRAINT: удержание в точке пружиной с демпфированием
    BodyDef hd;
    hd.type     = BodyType::Dynamic;
    hd.shape    = Shape::circle(0.35);
    hd.material = oak;
    hd.position = Vec2(-2.0, 5.0);
    BodyId held = w.createBody(hd);
    BodyControl& hc = control.ref(held);
    hc.holdPosition  = true;
    hc.anchor        = Vec2(-2.0, 5.0);
    hc.holdStiffness = 120.0;
    hc.holdDamping   = 16.0;

    // ---- поля
    GravityController grav;
    grav.mode     = GravityMode::Constant;
    grav.constant = Vec2(0.0, -9.81);

    WindField wind;
    wind.base          = Vec2(7.0, 0.0);
    wind.gustAmplitude = 3.0;
    wind.turbulence    = 1.2;

    FrictionHeat heat;
    tuner.install(w);            // доводка контактов владеет persist-колбэком

    StabilityGuard guard;
    guard.bounds = AABB{Vec2(-60.0, -20.0), Vec2(60.0, 80.0)};

    MotionInterpolator interp;
    Diagnostics        diag;
    DirtyTracker       dirty;
    SceneKeeper        keeper;
    keeper.intervalSteps = 200;
    keeper.path          = "autosave.json";

    const real dt = cfg.fixedTimeStep;
    for (int step = 0; step < 600; ++step) {
        diag.beginStep();
        guard.snapshot(w);

        grav.apply(w, dt);
        wind.apply(w, dt, 0.6);
        control.applyBeforeStep(w, dt);
        adaptSolver(w, 6, 20);

        w.step(dt);

        control.applyAfterStep(w, dt);
        sweptTunnelGuard(w, 0.02);
        guard.verify(w);
        heat.update(dt);
        interp.capture(w);
        dirty.scan(w);
        keeper.tick(w);
        diag.endStep(w);

        if (step == 300) {
            reshapeBody(w, lift, Shape::circle(0.5));   // смена формы в рантайме
            control.kick(w, lift, Vec2(4.0, 8.0));      // управляемый импульс
        }
        if (step == 599) diag.print();
    }

    // ---- запросы
    ShapeCastHit hit;
    if (shapeCast(w, Shape::circle(0.25), Transform(Vec2(-9.0, 1.0), 0.0), Vec2(8.0, 0.0), hit))
        std::printf("shapecast: hit t=%.3f point=(%.2f, %.2f)\n",
                    (double)hit.t, (double)hit.point.x, (double)hit.point.y);
    else
        std::printf("shapecast: miss\n");

    RayHit rh;
    if (rayCastClosest(w, Vec2(-12.0, 0.4), Vec2(12.0, 0.4), rh))
        std::printf("raycast: hit frac=%.3f\n", (double)rh.fraction);

    std::vector<RigidBody*> pq = pointQueryAll(w, Vec2(0.0, 0.2));
    std::vector<RigidBody*> ov = overlapShape(w, Shape::box(6.0, 3.0), Transform(Vec2(0.0, 1.5), 0.0));
    std::printf("pointquery=%zu  overlap=%zu  filter rejected=%zu  tuner adjusted=%zu (dE=%.2f J)\n",
                pq.size(), ov.size(), rules.rejected(), tuner.adjusted(), (double)tuner.energyRemoved());

    // ---- геометрические утилиты
    Vec2 xp;
    bool crossed = segmentIntersect(Vec2(-1, -1), Vec2(1, 1), Vec2(-1, 1), Vec2(1, -1), xp);
    std::vector<Vec2> poly{Vec2(-1, -1), Vec2(1, -1), Vec2(1, 1), Vec2(-1, 1)};
    std::vector<Vec2> withCollinear{Vec2(-1, -1), Vec2(0, -1), Vec2(1, -1), Vec2(1, 1), Vec2(-1, 1)};
    std::vector<Vec2> cleaned = removeCollinear(withCollinear);
    std::printf("geom: segments cross at (%.2f, %.2f)  pointInPolygon=%d  collinear %zu->%zu  convex=%d\n",
                crossed ? (double)xp.x : 0.0, crossed ? (double)xp.y : 0.0,
                pointInPolygon(poly, Vec2(0.2, 0.1)) ? 1 : 0,
                withCollinear.size(), cleaned.size(), isConvex(poly) ? 1 : 0);

    // ---- интерполяция / экстраполяция для рендера
    RenderTransform mid = interp.at(hammer.parts[0], 0.5);
    RenderTransform fut = interp.at(hammer.parts[0], 1.5);
    std::printf("interp: hammer at alpha=0.5 -> (%.3f, %.3f) angle=%.3f | extrapolate 1.5 -> (%.3f, %.3f)\n",
                (double)mid.position.x, (double)mid.position.y, (double)mid.angle,
                (double)fut.position.x, (double)fut.position.y);

    // ---- конфиг, профайл, память, плагины
    saveConfigTxt("config.txt", w.config());
    WorldConfig loaded;
    loadConfigTxt("config.txt", loaded);
    diag.writeJson("profile.json");

    void* block = MemoryTracker::alloc(1024 * 1024);
    std::printf("%s\n", MemoryTracker::report().c_str());
    MemoryTracker::release(block, 1024 * 1024);

    std::printf("service: config roundtrip iters=%d/%d  autosaves=%zu  dirty=%zu  heat_max=%.1f K\n",
                loaded.solver.velocityIterations, loaded.solver.positionIterations,
                keeper.saves(), dirty.list().size(), (double)heat.hottest());

    PluginHost plugins;
    const bool pluginLoaded = plugins.load("./libphys2d_plugin.so");
    std::printf("service: orphan statics removed=%zu  rollbacks=%zu clamps=%zu  plugins=%s\n",
                removeOrphanStatics(w), guard.rollbacks(), guard.clamps(),
                pluginLoaded ? "loaded" : plugins.lastError().c_str());

    diag.endStep(w);
    std::printf("final: %s\n", diag.line().c_str());
    return 0;
}
