// phys2d — демо рендера движка. Весь вывод идёт через phys2d::Renderer
// (Direct3D 9 на Windows, офлайн-бэкенд в остальных случаях) — в самом
// приложении нет ни одного вызова DirectX или WinAPI.
#include "phys2d/World.h"
#include "phys2d/Extras.h"
#include "phys2d/Render.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace phys2d;

namespace {

void palette(const RigidBody& body, void* user, RenderColor& fill, RenderColor& edge) {
    (void)user;
    if (body.isStatic()) { fill = rgba(70, 66, 60); edge = rgba(30, 28, 26); return; }
    switch (body.shape.type) {
        case ShapeType::Circle:  fill = rgba(214, 148, 96);  break;
        case ShapeType::Capsule: fill = rgba(126, 186, 148); break;
        default:                 fill = rgba(158, 108, 56);  break;
    }
    edge = rgba(20, 18, 16, 210);
}

BodyId addBox(World& world, const Vec2& p, real hw, real hh, real density = 300.0) {
    BodyDef def;
    def.type = BodyType::Dynamic;
    def.shape = Shape::box(hw, hh);
    def.material.density = density;
    def.material.restitution = 0.05;
    def.position = p;
    return world.createBody(def);
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    WorldConfig wcfg;
    wcfg.gravity = Vec2(0.0, -9.81);
    World world(wcfg);

    BodyDef ground;                     // пол и стены
    ground.type = BodyType::Static;
    ground.shape = Shape::box(20.0, 0.5);
    ground.position = Vec2(0.0, -0.5);
    world.createBody(ground);
    ground.shape = Shape::box(0.5, 10.0);
    ground.position = Vec2(-16.5, 9.5);
    world.createBody(ground);
    ground.position = Vec2(16.5, 9.5);
    world.createBody(ground);

    for (int i = 0; i < 6; ++i) addBox(world, Vec2(-4.0 + i * 1.2, 1.0 + i * 1.3), 0.5, 0.5);
    for (int i = 0; i < 5; ++i) {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.shape = Shape::circle(0.35);
        d.material.density = 250.0;
        d.material.restitution = 0.4;
        d.position = Vec2(3.0 + i * 0.9, 6.0);
        world.createBody(d);
    }
    {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.shape = Shape::capsule(0.8, 0.25);
        d.material.density = 400.0;
        d.position = Vec2(-7.0, 4.0);
        world.createBody(d);
    }

    ParticleSystem fluid;               // вода для drawWaterParticles
    fluid.smoothingRadius = 0.30;
    fluid.particleMass    = 0.55;
    fluid.restDensity     = 1000.0;
    fluid.stiffness       = 180.0;
    fluid.viscosity       = 34.0;
    fluid.surfaceTension  = 1.2;
    fluid.gravity         = Vec2(0.0, -9.81);
    fluid.boundaryDamping = 0.45;
    fluid.domain = AABB(Vec2(-15.8, 0.1), Vec2(15.8, 12.0));
    fluid.coupleWithRigid = true;
    fluid.emitBlock(Vec2(6.0, 0.4), 8.0, 1.6, 0.20);

    RenderConfig rcfg;
    rcfg.title  = "phys2d — Direct3D 9 renderer";
    rcfg.camera = Vec2(0.0, 6.0);
    rcfg.pixelsPerMeter = 40.0;
    rcfg.offlineFrames = (argc > 1) ? std::atoi(argv[1]) : 240;

    std::unique_ptr<Renderer> gfx = Renderer::create(rcfg);
    if (!gfx) { std::printf("renderer init failed\n"); return 1; }
    std::printf("phys2d renderer backend: %s\n", gfx->backendName());

    std::vector<Vec2> pos, vel;
    size_t frames = 0;

    while (gfx->pumpEvents()) {
        const RenderInput& in = gfx->input();
        if (in.keyPressed[27 /*Esc*/]) break;
        if (in.wheel != 0.0) gfx->zoomBy(in.wheel > 0.0 ? 1.1 : 0.9);
        if (in.leftPressed) addBox(world, in.mouseWorld, 0.4, 0.4);
        if (in.rightDown)   fluid.emit(in.mouseWorld, Vec2(0.0, -2.0));

        const real dt = gfx->frameSeconds();
        fluid.update(dt / 3.0);
        fluid.update(dt / 3.0);
        fluid.update(dt / 3.0);
        world.step(dt);

        pos.clear();
        vel.clear();
        for (const FluidParticle& q : fluid.particles()) {
            if (!q.active) continue;
            pos.push_back(q.position);
            vel.push_back(q.velocity);
        }

        gfx->beginFrame();
        gfx->drawWaterParticles(pos.data(), vel.data(), pos.size(), 0.20);
        gfx->drawWorld(world, palette);
        char line[256];
        std::snprintf(line, sizeof(line),
                      "PHYS2D %s  BODIES %zu  WATER %zu  TRIS %zu  FPS %.0f",
                      gfx->backendName(), world.bodyCount(), pos.size(),
                      gfx->triangleCount(), 1.0 / (dt > 0.0 ? dt : 1.0));
        gfx->rectPx(6.0f, 6.0f, (float)gfx->width() - 12.0f, 22.0f, rgba(8, 10, 14, 130));
        gfx->textPx(14.0f, 12.0f, line, 2.0f, rgba(236, 236, 240));
        gfx->textPx(14.0f, 30.0f, "LMB BOX   RMB WATER   WHEEL ZOOM   ESC QUIT", 2.0f,
                    rgba(170, 200, 230, 210));
        gfx->endFrame();
        ++frames;
    }

    std::printf("frames %zu, last frame triangles %zu, bodies %zu, water %zu\n",
                frames, gfx->lastFrameTriangles(), world.bodyCount(), fluid.count());
    return 0;
}
