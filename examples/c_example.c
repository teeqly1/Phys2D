/* phys2d - C API usage example (build: phys2d_c_example). */
#include "phys2d/phys2d_c_api.h"

#include <stdio.h>

static int g_beginCount = 0;

static void onBegin(const p2CollisionEvent* ev, void* user) {
    (void)user;
    if (g_beginCount < 3)
        printf("  contact %u <-> %u  n=(%.3f, %.3f)  depth=%.4f\n",
               ev->a, ev->b, ev->normalX, ev->normalY, ev->penetration);
    ++g_beginCount;
}

int main(void) {
    p2WorldConfig cfg = p2DefaultConfig();
    cfg.gravityY = -9.81;
    cfg.substeps = 2;
    cfg.threadCount = 2;

    p2WorldHandle world = p2CreateWorld(&cfg);
    if (!world) {
        printf("cannot create world\n");
        return 1;
    }

    p2SetContactCallback(world, 0, onBegin, NULL);

    /* ground + a small stack of shapes */
    p2CreateBox(world, P2_STATIC, 0.0, -1.0, 40.0, 2.0, 0.0, 1.0);

    p2BodyId ball = p2CreateCircle(world, P2_DYNAMIC, -2.0, 8.0, 0.5, 1.0);
    p2BodyId box  = p2CreateBox(world, P2_DYNAMIC, 0.0, 6.0, 1.2, 0.8, 0.3, 1.0);
    p2BodyId cap  = p2CreateCapsule(world, P2_DYNAMIC, 2.0, 10.0, 0.35, 1.4, 0.7, 1.0);

    const p2Vec2 tri[3] = {{-0.7, -0.5}, {0.7, -0.5}, {0.0, 0.8}};
    p2BodyId poly = p2CreatePolygon(world, P2_DYNAMIC, 1.0, 13.0, tri, 3, 1.0);

    p2SetMaterial(world, ball, 0.6, 0.5, 0.35);
    p2CreateSpring(world, ball, box, 0.0, 0.0, 0.0, 0.0, 2.0, 120.0, 4.0);

    for (int i = 0; i < 180; ++i) p2Step(world, 1.0 / 60.0);

    double x = 0.0, y = 0.0, angle = 0.0;
    p2GetTransform(world, ball, &x, &y, &angle);
    printf("ball:    (%.3f, %.3f) angle=%.3f mass=%.3f\n", x, y, angle, p2GetMass(world, ball));
    p2GetTransform(world, box, &x, &y, &angle);
    printf("box:     (%.3f, %.3f) angle=%.3f sleeping=%d\n", x, y, angle, p2IsSleeping(world, box));
    p2GetTransform(world, cap, &x, &y, &angle);
    printf("capsule: (%.3f, %.3f) angle=%.3f\n", x, y, angle);
    p2GetTransform(world, poly, &x, &y, &angle);
    printf("polygon: (%.3f, %.3f) angle=%.3f\n", x, y, angle);

    printf("bodies=%zu contacts=%zu arena=%.2f MB energy=%.3f begin-events=%d\n",
           p2BodyCount(world), p2ContactCount(world),
           (double)p2ArenaBytes(world) / (1024.0 * 1024.0),
           p2TotalEnergy(world), g_beginCount);

    char profile[4096];
    p2ProfileString(world, profile, (int)sizeof(profile));
    printf("%s\n", profile);

    p2SaveToFile(world, "c_example_scene.json");
    p2SetDebugLayers(world, 0xFFFFFFFFu);
    p2BuildDebugSvg(world, "c_example_frame.svg", 40.0);

    p2DestroyWorld(world);
    return 0;
}
