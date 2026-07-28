// phys2d — тестовая сцена:
//   1000 падающих окружностей, 500 прямоугольников с разными углами,
//   200 сложных многоугольников, 50 капсул, стенки 100x100,
//   препятствия внутри, вращающиеся шестерни.
#include "phys2d/World.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>

using namespace phys2d;

static std::mt19937_64 rng(1234567u);
static double urand(double a, double b) {
    return a + (b - a) * std::uniform_real_distribution<double>(0.0, 1.0)(rng);
}

int main(int argc, char** argv) {
    const int steps = (argc > 1) ? std::atoi(argv[1]) : 120;

    WorldConfig cfg;
    cfg.modules = MOD_ALL & ~(MOD_BUOYANCY | MOD_VISCOSITY | MOD_TRAILS | MOD_DEBUG_DRAW |
                              MOD_TOI | MOD_AERODYNAMICS | MOD_SOFT_BODY);
    cfg.integrator          = Integrator::SymplecticEuler;
    cfg.substeps            = 2;
    cfg.gridCellSize        = 3.0;
    cfg.broadPhase          = BroadPhaseMode::Hybrid;
    cfg.solver.velocityIterations = 8;
    cfg.solver.positionIterations = 10;   // минимум 10 итераций
    cfg.arenaBytes          = 32u * 1024u * 1024u;
    cfg.medium.fluidLevel   = -1e30;

    World world(cfg);

    // ---------------------------------------------------------- стенки 100x100
    const double H = 50.0, T = 2.0;
    auto addWall = [&](double x, double y, double w, double h) {
        BodyDef d;
        d.type = BodyType::Static;
        d.shape = Shape::box(w, h);
        d.position = Vec2(x, y);
        d.material.restitution = 0.15;
        d.material.staticFriction = 0.8;
        d.material.dynamicFriction = 0.6;
        d.name = "wall";
        return world.createBody(d);
    };
    addWall(0.0, -H - T * 0.5, 100.0 + 2 * T, T);   // пол
    addWall(0.0,  H + T * 0.5, 100.0 + 2 * T, T);   // потолок
    addWall(-H - T * 0.5, 0.0, T, 100.0);           // левая
    addWall( H + T * 0.5, 0.0, T, 100.0);           // правая

    // ---------------------------------------------------------- препятствия внутри
    for (int i = 0; i < 12; ++i) {
        BodyDef d;
        d.type = BodyType::Static;
        d.shape = Shape::box(24.0, 1.2);
        d.position = Vec2(urand(-30.0, 30.0), -40.0 + i * 6.0);
        d.angle = urand(-0.5, 0.5);
        d.material.staticFriction = 0.7;
        d.name = "obstacle";
        world.createBody(d);
    }
    for (int i = 0; i < 6; ++i) {   // статические круглые препятствия
        BodyDef d;
        d.type = BodyType::Static;
        d.shape = Shape::circle(urand(1.5, 3.5));
        d.position = Vec2(urand(-40.0, 40.0), urand(-35.0, 20.0));
        d.name = "pillar";
        world.createBody(d);
    }

    // ---------------------------------------------------------- вращающиеся шестерни
    for (int i = 0; i < 4; ++i) {
        BodyDef d;
        d.type = BodyType::Kinematic;               // кинематическое тело
        d.shape = Shape::gear(12, 2.2, 3.4);
        d.position = Vec2(-24.0 + i * 16.0, -30.0);
        d.angularVelocity = (i % 2 == 0) ? 1.6 : -1.6;
        d.allowSleep = false;
        d.name = "gear";
        world.createBody(d);
    }

    // ---------------------------------------------------------- 1000 окружностей
    for (int i = 0; i < 1000; ++i) {
        BodyDef d;
        d.shape = Shape::circle(urand(0.25, 0.55));
        d.position = Vec2(urand(-46.0, 46.0), urand(-10.0, 46.0));
        d.velocity = Vec2(urand(-2.0, 2.0), urand(-6.0, 0.0));
        d.material.restitution     = urand(0.1, 0.6);
        d.material.rollingFriction = 0.03;          // трение качения
        d.name = "circle";
        world.createBody(d);
    }

    // ---------------------------------------------------------- 500 прямоугольников
    for (int i = 0; i < 500; ++i) {
        BodyDef d;
        d.shape = Shape::box(urand(0.5, 1.4), urand(0.5, 1.4));
        d.position = Vec2(urand(-46.0, 46.0), urand(-20.0, 46.0));
        d.angle = urand(-PI, PI);                   // разные углы
        d.angularVelocity = urand(-1.5, 1.5);
        d.material.restitution = urand(0.05, 0.4);
        d.name = "box";
        world.createBody(d);
    }

    // ---------------------------------------------------------- 200 сложных многоугольников
    for (int i = 0; i < 200; ++i) {
        const int n = (int)urand(5, 9);
        std::vector<Vec2> pts;
        for (int k = 0; k < n; ++k) {
            const double a = 2.0 * PI * k / n + urand(-0.15, 0.15);
            const double r = urand(0.5, 1.3);
            pts.emplace_back(std::cos(a) * r, std::sin(a) * r);
        }
        BodyDef d;
        d.shape = Shape::polygon(pts);              // произвольные вершины -> выпуклая оболочка
        d.position = Vec2(urand(-45.0, 45.0), urand(0.0, 46.0));
        d.angle = urand(-PI, PI);
        d.material.restitution = 0.25;
        d.name = "poly";
        world.createBody(d);
    }

    // ---------------------------------------------------------- 50 капсул
    for (int i = 0; i < 50; ++i) {
        BodyDef d;
        d.shape = Shape::capsule(urand(0.25, 0.5), urand(1.0, 2.5));
        d.position = Vec2(urand(-44.0, 44.0), urand(10.0, 46.0));
        d.angle = urand(-PI, PI);
        d.material.rollingFriction = 0.04;
        d.name = "capsule";
        world.createBody(d);
    }

    // ---------------------------------------------------------- ограничения (цепочка на пружинах и тросе)
    {
        BodyDef anchorDef;
        anchorDef.type = BodyType::Static;
        anchorDef.shape = Shape::circle(0.4);
        anchorDef.position = Vec2(0.0, 44.0);
        anchorDef.name = "anchor";
        const BodyId anchor = world.createBody(anchorDef);

        BodyId prev = anchor;
        for (int i = 1; i <= 6; ++i) {
            BodyDef d;
            d.shape = Shape::box(1.0, 0.4);
            d.position = Vec2(i * 1.6, 44.0);
            d.allowSleep = false;
            d.name = "link";
            const BodyId link = world.createBody(d);
            if (i % 2 == 0) world.createDistance(prev, link, Vec2(), Vec2(), 1.6);
            else            world.createRevolute(prev, link, Vec2(0.5, 0.0), Vec2(-0.5, 0.0));
            prev = link;
        }
        BodyDef ballDef;
        ballDef.shape = Shape::circle(0.9);
        ballDef.position = Vec2(12.0, 40.0);
        ballDef.allowSleep = false;
        const BodyId ball = world.createBody(ballDef);
        world.createSpring(prev, ball, Vec2(), Vec2(), 2.0, 350.0, 8.0);
        world.createRope(anchor, ball, Vec2(), Vec2(), 16.0);
        world.createAngular(anchor, ball, -PI * 0.4, PI * 0.4);
    }

    // ---------------------------------------------------------- события столкновений
    long long beginEvents = 0, endEvents = 0;
    double maxImpulse = 0.0;
    world.setBeginContactCallback([&](const CollisionEvent& ev) {
        ++beginEvents;
        if (ev.normalImpulse > maxImpulse) maxImpulse = ev.normalImpulse;
    });
    world.setEndContactCallback([&](const CollisionEvent&) { ++endEvents; });

    std::printf("phys2d test scene\n");
    std::printf("тел: %zu, ограничений: %zu, память физики: %.2f МБ\n",
                world.bodyCount(), world.constraintCount(),
                (double)world.arenaBytes() / (1024.0 * 1024.0));

    // ---------------------------------------------------------- симуляция
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) {
        world.step(1.0 / 60.0);
        if (i % 30 == 0) {
            std::printf("шаг %4d | контакты %6zu | активных %5zu | шаг %6.2f мс | энергия %.3e\n",
                        i, world.contactCount(), world.awakeCount(),
                        world.profile().ms[(size_t)Stage::Total], world.totalEnergy());
        }
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    std::printf("\n%s\n", world.profile().toString().c_str());
    std::printf("всего: %d шагов за %.2f с (%.2f мс/шаг)\n", steps, wall, wall * 1000.0 / steps);
    std::printf("события: begin=%lld end=%lld, макс. нормальный импульс %.3f\n",
                beginEvents, endEvents, maxImpulse);
    std::printf("память под данные физики: %.2f МБ\n", (double)world.arenaBytes() / (1024.0 * 1024.0));

    // ---------------------------------------------------------- сериализация
    if (world.saveToFile("scene.json")) {
        std::ifstream f("scene.json", std::ios::ate | std::ios::binary);
        std::printf("scene.json записан (%.2f МБ)\n", (double)f.tellg() / (1024.0 * 1024.0));
    }
    {
        World reloaded;
        std::string err;
        const std::string json = world.toJson(false);
        if (reloaded.fromJson(json, &err))
            std::printf("десериализация OK: %zu тел, %zu ограничений\n",
                        reloaded.bodyCount(), reloaded.constraintCount());
        else
            std::printf("ошибка десериализации: %s\n", err.c_str());
    }

    // ---------------------------------------------------------- дебаг-визуализация в SVG
    world.debugBuffer().layers = DBG_SHAPES | DBG_CONTACTS | DBG_NORMALS | DBG_AABB |
                                 DBG_SAT_AXES | DBG_FORCES | DBG_JOINTS;
    world.buildDebugGeometry();
    AABB view;
    view.min = Vec2(-56.0, -56.0);
    view.max = Vec2( 56.0,  56.0);
    const std::string svg = debugBufferToSvg(world.debugBuffer(), view, 1400);
    std::ofstream out("debug_frame.svg", std::ios::binary);
    out << svg;
    std::printf("debug_frame.svg: %zu примитивов\n", world.debugBuffer().itemCount());
    return 0;
}
