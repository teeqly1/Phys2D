// SPH/PBF settle test: still water must come to rest instead of jittering.
#include "phys2d/World.h"
#include "phys2d/Extras.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace phys2d;

int main(int argc, char** argv) {
    const real visc  = argc > 1 ? std::atof(argv[1]) : 22.0;
    const real tens  = argc > 2 ? std::atof(argv[2]) : 0.3;
    const int  sub   = argc > 3 ? std::atoi(argv[3]) : 2;
    const real hmul  = argc > 4 ? std::atof(argv[4]) : 2.0;
    const real damp  = argc > 5 ? std::atof(argv[5]) : 0.3;
    const int  steps = argc > 6 ? std::atoi(argv[6]) : 900;

    World w;
    w.config().gravity = Vec2(0.0, -9.81);
    ParticleSystem water;
    water.attach(w);
    const real spacing = 0.20;
    water.smoothingRadius = spacing * hmul;
    water.restDensity = 1000.0;
    water.viscosity = visc;
    water.surfaceTension = tens;
    water.boundaryDamping = damp;
    water.coupleWithRigid = true;
    water.domain = AABB{ Vec2(-8.0, 0.02), Vec2(8.0, 20.0) };

    BodyDef fl; fl.type = BodyType::Static; fl.shape = Shape::box(20.0, 1.2); fl.position = Vec2(0.0, -0.6); w.createBody(fl);
    BodyDef lw; lw.type = BodyType::Static; lw.shape = Shape::box(0.6, 6.0); lw.position = Vec2(-6.0, 3.0); w.createBody(lw);
    BodyDef rw = lw; rw.position = Vec2(6.0, 3.0); w.createBody(rw);
    BodyDef bx; bx.type = BodyType::Dynamic; bx.shape = Shape::box(0.8, 0.8); bx.position = Vec2(0.0, 6.0);
    bx.material.density = 500.0; const BodyId floater = w.createBody(bx);

    water.emitBlock(Vec2(-5.4, 0.15), 10.8, 2.4, spacing);
    std::printf("particles %zu  mass %.3f kg\n", water.count(), (double)water.particleMass);
    const real dt = 1.0 / 60.0;
    for (int step = 0; step < steps; ++step) {
        w.step(dt);
        for (int k = 0; k < sub; ++k) water.update(dt / sub);
        if ((step + 1) % 150 == 0) {
            const std::vector<FluidParticle>& ps = water.particles();
            real avg = 0.0, mx = 0.0, top = -1e30, dmean = 0.0;
            int bad = 0;
            for (size_t i = 0; i < ps.size(); ++i) {
                const real sp = ps[i].velocity.length();
                if (!std::isfinite(sp) || !std::isfinite(ps[i].position.y)) { ++bad; continue; }
                avg += sp; if (sp > mx) mx = sp;
                if (ps[i].position.y > top) top = ps[i].position.y;
                dmean += ps[i].density;
            }
            avg /= (real)ps.size(); dmean /= (real)ps.size();
            RigidBody* b = w.body(floater);
            std::printf("step %4d  avg|v| %6.3f  max|v| %6.2f  top %5.2f  rho %6.1f  box_y %5.2f  bad %d\n",
                        step + 1, (double)avg, (double)mx, (double)top, (double)dmean,
                        b ? (double)b->position.y : 0.0, bad);
        }
    }
    return 0;
}
