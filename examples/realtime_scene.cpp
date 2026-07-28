// phys2d - real-time interactive test scene.
//
//   Windows : native Win32 + GDI window (no external dependencies).
//   Other   : headless fallback, runs the same scene and writes debug SVG frames.
//
// Controls (Windows build):
//   SPACE          pause / resume            S    single step while paused
//   R              rebuild the scene         C    remove all dynamic bodies
//   1 2 3 4        spawn circle / box / polygon / capsule at the cursor
//   left mouse     grab and drag a body      right mouse  explosion at the cursor
//   mouse wheel    zoom                      arrows       pan the camera
//   G              gravity on/off            I    switch integrator
//   A              AABB overlay              N    contacts + normals overlay
//   T              trails                    M    multithreading on/off
//   F              friction on/off           E    restitution on/off
//   P              position correction on/off
//   [ ]            fewer / more solver velocity iterations
//   , .            fewer / more substeps
//   J              save scene.json           L    load scene.json
//   ESC            exit

#include "phys2d/World.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

using namespace phys2d;

// ============================================================ scene building
namespace {

std::mt19937 g_rng(1337);

real frand(real lo, real hi) {
    std::uniform_real_distribution<real> d(lo, hi);
    return d(g_rng);
}

struct SceneRefs {
    std::vector<BodyId> gears;
};

SceneRefs buildScene(World& world, int circles, int boxes, int polys, int capsules) {
    world.clear();
    SceneRefs refs;

    const real halfW = 20.0, halfH = 15.0, thickness = 1.0;

    auto addStatic = [&](const Shape& s, const Vec2& p, real angle) {
        BodyDef d;
        d.type = BodyType::Static;
        d.shape = s;
        d.position = p;
        d.angle = angle;
        d.material.staticFriction = 0.8;
        d.material.dynamicFriction = 0.6;
        d.material.restitution = 0.15;
        return world.createBody(d);
    };

    // arena walls
    addStatic(Shape::box(2.0 * halfW + 2.0 * thickness, thickness), Vec2(0.0, -halfH), 0.0);
    addStatic(Shape::box(2.0 * halfW + 2.0 * thickness, thickness), Vec2(0.0, halfH), 0.0);
    addStatic(Shape::box(thickness, 2.0 * halfH), Vec2(-halfW, 0.0), 0.0);
    addStatic(Shape::box(thickness, 2.0 * halfH), Vec2(halfW, 0.0), 0.0);

    // ramps and obstacles
    addStatic(Shape::box(16.0, 0.5), Vec2(-8.0, 4.0), -0.30);
    addStatic(Shape::box(16.0, 0.5), Vec2(8.0, -2.0), 0.28);
    addStatic(Shape::box(10.0, 0.5), Vec2(-10.0, -8.0), 0.18);
    addStatic(Shape::circle(1.6), Vec2(0.0, -6.0), 0.0);
    addStatic(Shape::circle(1.2), Vec2(6.0, -10.0), 0.0);
    addStatic(Shape::regularPolygon(6, 1.8), Vec2(-14.0, -2.0), 0.4);

    // rotating kinematic gears
    const Vec2 gearPos[3] = {Vec2(-6.0, -11.0), Vec2(4.0, 7.0), Vec2(13.0, 4.0)};
    const real gearSpin[3] = {1.8, -2.4, 1.2};
    for (int i = 0; i < 3; ++i) {
        BodyDef d;
        d.type = BodyType::Kinematic;
        d.shape = Shape::gear(10, 1.3, 2.1);
        d.position = gearPos[i];
        d.angularVelocity = gearSpin[i];
        d.allowSleep = false;
        d.name = "gear";
        refs.gears.push_back(world.createBody(d));
    }

    // dynamic bodies
    auto spawn = [&](const Shape& s, real restitution) {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.shape = s;
        d.position = Vec2(frand(-halfW + 2.0, halfW - 2.0), frand(1.0, halfH - 2.0));
        d.angle = frand(0.0, 6.28);
        d.angularVelocity = frand(-1.5, 1.5);
        d.velocity = Vec2(frand(-2.0, 2.0), 0.0);
        d.material.restitution = restitution;
        d.material.density = frand(0.6, 1.6);
        world.createBody(d);
    };

    for (int i = 0; i < circles; ++i) spawn(Shape::circle(frand(0.18, 0.42)), frand(0.25, 0.65));
    for (int i = 0; i < boxes; ++i) spawn(Shape::box(frand(0.4, 1.1), frand(0.3, 0.8)), frand(0.1, 0.45));
    for (int i = 0; i < polys; ++i)
        spawn(Shape::regularPolygon(3 + (int)frand(0.0, 5.0), frand(0.3, 0.7), frand(0.0, 1.5)), 0.3);
    for (int i = 0; i < capsules; ++i) spawn(Shape::capsule(frand(0.15, 0.3), frand(0.5, 1.2)), 0.3);

    // a swinging chain hanging from the ceiling
    BodyId prev = INVALID_BODY;
    Vec2 anchor(-4.0, halfH - 2.0);
    for (int i = 0; i < 8; ++i) {
        BodyDef d;
        d.type = BodyType::Dynamic;
        d.shape = Shape::box(0.8, 0.22);
        d.position = anchor + Vec2(0.9 * (i + 1), 0.0);
        d.allowSleep = false;
        const BodyId link = world.createBody(d);
        if (prev == INVALID_BODY) {
            const BodyId hook = addStatic(Shape::box(0.4, 0.4), anchor, 0.0);
            world.createRevolute(hook, link, Vec2(0.2, 0.0), Vec2(-0.4, 0.0));
        } else {
            world.createRevolute(prev, link, Vec2(0.4, 0.0), Vec2(-0.4, 0.0));
        }
        prev = link;
    }

    // spring pendulum
    {
        const BodyId top = addStatic(Shape::box(0.4, 0.4), Vec2(10.0, halfH - 2.0), 0.0);
        BodyDef d;
        d.shape = Shape::circle(0.6);
        d.position = Vec2(10.0, halfH - 8.0);
        d.material.restitution = 0.5;
        const BodyId bob = world.createBody(d);
        world.createSpring(top, bob, Vec2(), Vec2(), 4.0, 260.0, 6.0);
    }

    return refs;
}

BodyId spawnAt(World& world, const Vec2& p, int kind) {
    BodyDef d;
    d.type = BodyType::Dynamic;
    d.position = p;
    d.angle = frand(0.0, 6.28);
    d.material.restitution = 0.45;
    switch (kind) {
        case 0: d.shape = Shape::circle(frand(0.25, 0.6)); break;
        case 1: d.shape = Shape::box(frand(0.5, 1.2), frand(0.4, 0.9)); break;
        case 2: d.shape = Shape::regularPolygon(3 + (int)frand(0.0, 5.0), frand(0.4, 0.8)); break;
        default: d.shape = Shape::capsule(frand(0.18, 0.32), frand(0.6, 1.3)); break;
    }
    return world.createBody(d);
}

void explosion(World& world, const Vec2& center, real radius, real strength) {
    for (RigidBody* b : world.bodies()) {
        if (!b->isDynamic()) continue;
        Vec2 d = b->position - center;
        const real dist = d.length();
        if (dist > radius || dist < 1e-6) continue;
        const real falloff = 1.0 - dist / radius;
        b->applyImpulseAtPoint(d.normalized() * (strength * falloff * b->mass), b->position);
    }
}

} // namespace

// ============================================================ Windows renderer
#if defined(_WIN32)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// GET_X_LPARAM lives in windowsx.h; keep the example dependency-free.
#define GET_X_LPARAM_SAFE(lp) ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM_SAFE(lp) ((int)(short)HIWORD(lp))


namespace {

struct Camera {
    Vec2 center{0.0, 0.0};
    real scale = 22.0;   // pixels per meter
    int  width = 1280, height = 800;

    POINT toScreen(const Vec2& w) const {
        POINT p;
        p.x = (LONG)std::lround((w.x - center.x) * scale + width * 0.5);
        p.y = (LONG)std::lround(height * 0.5 - (w.y - center.y) * scale);
        return p;
    }
    Vec2 toWorld(int sx, int sy) const {
        return Vec2((sx - width * 0.5) / scale + center.x, (height * 0.5 - sy) / scale + center.y);
    }
};

struct AppState {
    World      world{[] {
        WorldConfig cfg;
        cfg.modules = MOD_DEFAULT | MOD_TOI | MOD_TRAILS;
        cfg.substeps = 4;
        cfg.gridCellSize = 2.5;
        cfg.solver.velocityIterations = 10;
        cfg.solver.positionIterations = 10;
        cfg.arenaBytes = 32u * 1024u * 1024u;
        return cfg;
    }()};
    Camera     cam;
    bool       paused = false;
    bool       stepOnce = false;
    bool       showAABB = false;
    bool       showContacts = true;
    bool       showTrails = false;
    bool       running = true;
    Vec2       gravitySaved{0.0, -9.81};
    BodyId     dragged = INVALID_BODY;
    Vec2       dragLocal;
    Vec2       mouseWorld;
    double     frameMs = 0.0;
    double     fps = 0.0;
    int        spawnKind = 0;
    int        beginEvents = 0;
};

AppState* g_app = nullptr;

void drawBody(HDC dc, const Camera& cam, const RigidBody& b) {
    const Transform xf = b.transform();

    HPEN pen;
    if (b.isStatic())          pen = CreatePen(PS_SOLID, 1, RGB(140, 150, 160));
    else if (b.isKinematic())  pen = CreatePen(PS_SOLID, 2, RGB(255, 170, 80));
    else if (b.sleeping)       pen = CreatePen(PS_SOLID, 1, RGB(90, 110, 130));
    else                       pen = CreatePen(PS_SOLID, 1, RGB(120, 205, 255));
    HGDIOBJ oldPen = SelectObject(dc, pen);

    if (b.shape.type == ShapeType::Circle) {
        const POINT c = cam.toScreen(b.position);
        const int r = (int)std::lround(b.shape.radius * cam.scale);
        Ellipse(dc, c.x - r, c.y - r, c.x + r, c.y + r);
        const POINT rim = cam.toScreen(b.position + Vec2::fromAngle(b.angle, b.shape.radius));
        MoveToEx(dc, c.x, c.y, nullptr);
        LineTo(dc, rim.x, rim.y);
    } else if (b.shape.type == ShapeType::Capsule) {
        Vec2 a, e;
        b.shape.capsuleSegment(xf, a, e);
        const int r = (int)std::lround(b.shape.radius * cam.scale);
        const POINT pa = cam.toScreen(a), pe = cam.toScreen(e);
        Ellipse(dc, pa.x - r, pa.y - r, pa.x + r, pa.y + r);
        Ellipse(dc, pe.x - r, pe.y - r, pe.x + r, pe.y + r);
        const Vec2 n = (e - a).normalized().perp() * b.shape.radius;
        POINT s1 = cam.toScreen(a + n), s2 = cam.toScreen(e + n);
        MoveToEx(dc, s1.x, s1.y, nullptr); LineTo(dc, s2.x, s2.y);
        s1 = cam.toScreen(a - n); s2 = cam.toScreen(e - n);
        MoveToEx(dc, s1.x, s1.y, nullptr); LineTo(dc, s2.x, s2.y);
    } else {
        const std::vector<Vec2>& vs = b.worldVertices;
        if (vs.size() >= 2) {
            std::vector<POINT> pts(vs.size());
            for (size_t i = 0; i < vs.size(); ++i) pts[i] = cam.toScreen(vs[i]);
            Polygon(dc, pts.data(), (int)pts.size());
        }
    }

    if (g_app->showTrails && b.trail.size() >= 2) {
        HPEN tp = CreatePen(PS_SOLID, 1, RGB(70, 130, 100));
        HGDIOBJ op = SelectObject(dc, tp);
        POINT p0 = cam.toScreen(b.trail[0]);
        MoveToEx(dc, p0.x, p0.y, nullptr);
        for (size_t i = 1; i < b.trail.size(); ++i) {
            const POINT p = cam.toScreen(b.trail[i]);
            LineTo(dc, p.x, p.y);
        }
        SelectObject(dc, op);
        DeleteObject(tp);
    }

    if (g_app->showAABB) {
        HPEN ap = CreatePen(PS_DOT, 1, RGB(70, 95, 115));
        HGDIOBJ op = SelectObject(dc, ap);
        const POINT lo = cam.toScreen(Vec2(b.aabb.min.x, b.aabb.max.y));
        const POINT hi = cam.toScreen(Vec2(b.aabb.max.x, b.aabb.min.y));
        MoveToEx(dc, lo.x, lo.y, nullptr);
        LineTo(dc, hi.x, lo.y); LineTo(dc, hi.x, hi.y); LineTo(dc, lo.x, hi.y); LineTo(dc, lo.x, lo.y);
        SelectObject(dc, op);
        DeleteObject(ap);
    }

    SelectObject(dc, oldPen);
    DeleteObject(pen);
}

void drawContacts(HDC dc, const Camera& cam, const World& world) {
    HPEN pen = CreatePen(PS_SOLID, 1, RGB(255, 90, 90));
    HPEN npen = CreatePen(PS_SOLID, 1, RGB(255, 205, 70));
    HGDIOBJ old = SelectObject(dc, pen);

    for (const Manifold& m : world.manifolds()) {
        if (!m.touching) continue;
        for (int i = 0; i < m.count; ++i) {
            const POINT p = cam.toScreen(m.points[i].point);
            SelectObject(dc, pen);
            Rectangle(dc, p.x - 2, p.y - 2, p.x + 2, p.y + 2);
            SelectObject(dc, npen);
            const POINT e = cam.toScreen(m.points[i].point + m.normal * 0.35);
            MoveToEx(dc, p.x, p.y, nullptr);
            LineTo(dc, e.x, e.y);
        }
    }
    SelectObject(dc, old);
    DeleteObject(pen);
    DeleteObject(npen);
}

void drawHud(HDC dc, AppState& app) {
    const ProfileData& pd = app.world.profile();
    const char* integ = (app.world.integrator() == Integrator::SymplecticEuler) ? "symplectic euler"
                      : (app.world.integrator() == Integrator::Verlet)         ? "verlet"
                                                                              : "rk4";
    char buf[2048];
    std::snprintf(buf, sizeof(buf),
        "fps %.0f   frame %.2f ms   step %.2f ms%s\n"
        "bodies %zu (awake %zu)   contacts %zu   pairs %d   points %d\n"
        "broad %.2f  narrow %.2f  vel %.2f  pos %.2f  constr %.2f  sleep %.2f ms\n"
        "integrator: %s   substeps %d   vel-iters %d   pos-iters %d\n"
        "gravity (%.2f, %.2f)   energy %.1f   arena %.1f MB   GJK %d  EPA %d  SAT %d (hits %d)  TOI %d\n"
        "friction %s  restitution %s  poscorr %s  threads %s  spawn %s\n"
        "SPACE pause  S step  R reset  C clear  1-4 spawn  LMB drag  RMB blast  G I A N T M F E P [ ] , . J L",
        app.fps, app.frameMs, pd.ms[(size_t)Stage::Total], app.paused ? "   [PAUSED]" : "",
        app.world.bodyCount(), app.world.awakeCount(), app.world.contactCount(),
        pd.broadPairs, pd.contactPoints,
        pd.ms[(size_t)Stage::BroadPhase], pd.ms[(size_t)Stage::NarrowPhase],
        pd.ms[(size_t)Stage::SolveVelocity], pd.ms[(size_t)Stage::SolvePosition],
        pd.ms[(size_t)Stage::Constraints], pd.ms[(size_t)Stage::Sleeping],
        integ, app.world.config().substeps,
        app.world.config().solver.velocityIterations, app.world.config().solver.positionIterations,
        app.world.gravity().x, app.world.gravity().y, app.world.totalEnergy(),
        (double)app.world.arenaBytes() / (1024.0 * 1024.0),
        pd.gjkCalls, pd.epaCalls, pd.satCalls, pd.satCacheHits, pd.toiCalls,
        app.world.moduleEnabled(MOD_FRICTION) ? "on" : "off",
        app.world.moduleEnabled(MOD_RESTITUTION) ? "on" : "off",
        app.world.moduleEnabled(MOD_POSITION_CORRECTION) ? "on" : "off",
        app.world.moduleEnabled(MOD_MULTITHREADING) ? "on" : "off",
        (app.spawnKind == 0) ? "circle" : (app.spawnKind == 1) ? "box" : (app.spawnKind == 2) ? "polygon" : "capsule");

    RECT r{10, 8, app.cam.width - 10, 190};
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(215, 225, 235));
    DrawTextA(dc, buf, -1, &r, DT_LEFT | DT_TOP | DT_NOCLIP);
}

void render(HWND hwnd, AppState& app) {
    PAINTSTRUCT ps;
    HDC dc = BeginPaint(hwnd, &ps);

    RECT rc;
    GetClientRect(hwnd, &rc);
    const int w = rc.right - rc.left, h = rc.bottom - rc.top;
    app.cam.width = w;
    app.cam.height = h;

    HDC mem = CreateCompatibleDC(dc);
    HBITMAP bmp = CreateCompatibleBitmap(dc, w, h);
    HGDIOBJ oldBmp = SelectObject(mem, bmp);

    HBRUSH bg = CreateSolidBrush(RGB(18, 22, 28));
    FillRect(mem, &rc, bg);
    DeleteObject(bg);
    SelectObject(mem, GetStockObject(NULL_BRUSH));

    for (RigidBody* b : app.world.bodies()) drawBody(mem, app.cam, *b);
    if (app.showContacts) drawContacts(mem, app.cam, app.world);

    if (app.dragged != INVALID_BODY) {
        RigidBody* b = app.world.body(app.dragged);
        if (b) {
            HPEN pen = CreatePen(PS_SOLID, 1, RGB(200, 120, 255));
            HGDIOBJ old = SelectObject(mem, pen);
            const POINT a = app.cam.toScreen(b->transform().apply(app.dragLocal));
            const POINT m = app.cam.toScreen(app.mouseWorld);
            MoveToEx(mem, a.x, a.y, nullptr);
            LineTo(mem, m.x, m.y);
            SelectObject(mem, old);
            DeleteObject(pen);
        }
    }

    drawHud(mem, app);

    BitBlt(dc, 0, 0, w, h, mem, 0, 0, SRCCOPY);
    SelectObject(mem, oldBmp);
    DeleteObject(bmp);
    DeleteDC(mem);
    EndPaint(hwnd, &ps);
}

void onKey(AppState& app, WPARAM key) {
    WorldConfig& cfg = app.world.config();
    switch (key) {
        case VK_ESCAPE: app.running = false; break;
        case VK_SPACE:  app.paused = !app.paused; break;
        case 'S':       app.stepOnce = true; break;
        case 'R':       buildScene(app.world, 120, 60, 40, 20); break;
        case 'C': {
            std::vector<BodyId> ids;
            for (RigidBody* b : app.world.bodies())
                if (b->isDynamic()) ids.push_back(b->id);
            for (BodyId id : ids) app.world.destroyBody(id);
            break;
        }
        case '1': app.spawnKind = 0; spawnAt(app.world, app.mouseWorld, 0); break;
        case '2': app.spawnKind = 1; spawnAt(app.world, app.mouseWorld, 1); break;
        case '3': app.spawnKind = 2; spawnAt(app.world, app.mouseWorld, 2); break;
        case '4': app.spawnKind = 3; spawnAt(app.world, app.mouseWorld, 3); break;
        case 'G': {
            if (app.world.gravity().lengthSq() > 1e-9) {
                app.gravitySaved = app.world.gravity();
                app.world.setGravity(Vec2());
            } else {
                app.world.setGravity(app.gravitySaved);
            }
            for (RigidBody* b : app.world.bodies()) b->wake();
            break;
        }
        case 'I': {
            const Integrator next = (app.world.integrator() == Integrator::SymplecticEuler) ? Integrator::Verlet
                                  : (app.world.integrator() == Integrator::Verlet)          ? Integrator::RK4
                                                                                            : Integrator::SymplecticEuler;
            app.world.setIntegrator(next);
            break;
        }
        case 'A': app.showAABB = !app.showAABB; break;
        case 'N': app.showContacts = !app.showContacts; break;
        case 'T':
            app.showTrails = !app.showTrails;
            app.world.enableModule(MOD_TRAILS, app.showTrails);
            break;
        case 'M': app.world.enableModule(MOD_MULTITHREADING, !app.world.moduleEnabled(MOD_MULTITHREADING)); break;
        case 'F': app.world.enableModule(MOD_FRICTION, !app.world.moduleEnabled(MOD_FRICTION)); break;
        case 'E': app.world.enableModule(MOD_RESTITUTION, !app.world.moduleEnabled(MOD_RESTITUTION)); break;
        case 'P': app.world.enableModule(MOD_POSITION_CORRECTION, !app.world.moduleEnabled(MOD_POSITION_CORRECTION)); break;
        case 'J': app.world.saveToFile("scene.json"); break;
        case 'L': {
            std::string err;
            app.world.loadFromFile("scene.json", &err);
            break;
        }
        case VK_OEM_4: cfg.solver.velocityIterations = std::max(1, cfg.solver.velocityIterations - 1); break;
        case VK_OEM_6: cfg.solver.velocityIterations = std::min(cfg.solver.maxIterations, cfg.solver.velocityIterations + 1); break;
        case VK_OEM_COMMA:  cfg.substeps = std::max(1, cfg.substeps - 1); break;
        case VK_OEM_PERIOD: cfg.substeps = std::min(cfg.maxSubsteps, cfg.substeps + 1); break;
        case VK_LEFT:  app.cam.center.x -= 1.0; break;
        case VK_RIGHT: app.cam.center.x += 1.0; break;
        case VK_UP:    app.cam.center.y += 1.0; break;
        case VK_DOWN:  app.cam.center.y -= 1.0; break;
        default: break;
    }
}

LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    AppState* app = g_app;
    if (!app) return DefWindowProcA(hwnd, msg, wp, lp);

    switch (msg) {
        case WM_DESTROY:
        case WM_CLOSE:
            app->running = false;
            return 0;
        case WM_PAINT:
            render(hwnd, *app);
            return 0;
        case WM_ERASEBKGND:
            return 1;
        case WM_KEYDOWN:
            onKey(*app, wp);
            return 0;
        case WM_MOUSEMOVE:
            app->mouseWorld = app->cam.toWorld(GET_X_LPARAM_SAFE(lp), GET_Y_LPARAM_SAFE(lp));
            return 0;
        case WM_LBUTTONDOWN: {
            app->mouseWorld = app->cam.toWorld(GET_X_LPARAM_SAFE(lp), GET_Y_LPARAM_SAFE(lp));
            RigidBody* b = app->world.queryPoint(app->mouseWorld);
            if (b && b->isDynamic()) {
                app->dragged = b->id;
                app->dragLocal = b->transform().invApply(app->mouseWorld);
                b->wake();
            }
            return 0;
        }
        case WM_LBUTTONUP:
            app->dragged = INVALID_BODY;
            return 0;
        case WM_RBUTTONDOWN:
            app->mouseWorld = app->cam.toWorld(GET_X_LPARAM_SAFE(lp), GET_Y_LPARAM_SAFE(lp));
            explosion(app->world, app->mouseWorld, 7.0, 14.0);
            return 0;
        case WM_MOUSEWHEEL: {
            const int delta = GET_WHEEL_DELTA_WPARAM(wp);
            app->cam.scale = clampr(app->cam.scale * ((delta > 0) ? 1.12 : 1.0 / 1.12), 4.0, 160.0);
            return 0;
        }
        default: break;
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

void applyDrag(AppState& app, real dt) {
    if (app.dragged == INVALID_BODY) return;
    RigidBody* b = app.world.body(app.dragged);
    if (!b) { app.dragged = INVALID_BODY; return; }

    const Vec2 grab = b->transform().apply(app.dragLocal);
    const Vec2 delta = app.mouseWorld - grab;
    const real stiffness = 900.0 * b->mass;
    const real damping = 60.0 * b->mass;
    const Vec2 force = delta * stiffness - b->velocityAtPoint(grab) * damping;
    b->applyForceAtPoint(force, grab);
    (void)dt;
}

} // namespace

int main() {
    AppState app;
    g_app = &app;
    buildScene(app.world, 120, 60, 40, 20);

    app.world.setBeginContactCallback([&app](const CollisionEvent&) { ++app.beginEvents; });

    WNDCLASSA wc{};
    wc.lpfnWndProc = wndProc;
    wc.hInstance = GetModuleHandleA(nullptr);
    wc.lpszClassName = "phys2d_realtime";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowExA(0, wc.lpszClassName, "phys2d - real-time physics test",
                                WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                                CW_USEDEFAULT, CW_USEDEFAULT, 1280, 820,
                                nullptr, nullptr, wc.hInstance, nullptr);
    if (!hwnd) {
        std::printf("cannot create window\n");
        return 1;
    }

    LARGE_INTEGER freq, prev;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);
    double accumulator = 0.0;
    const double fixedDt = 1.0 / 60.0;

    while (app.running) {
        MSG msg;
        while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) app.running = false;
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }
        if (!app.running) break;

        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        const double elapsed = double(now.QuadPart - prev.QuadPart) / double(freq.QuadPart);
        prev = now;
        app.frameMs = elapsed * 1000.0;
        app.fps = (elapsed > 1e-9) ? (1.0 / elapsed) : 0.0;

        accumulator = std::min(accumulator + elapsed, 0.25);
        while (accumulator >= fixedDt) {
            accumulator -= fixedDt;
            if (!app.paused || app.stepOnce) {
                applyDrag(app, fixedDt);
                app.world.step(fixedDt);
                app.stepOnce = false;
            } else {
                break;
            }
        }

        InvalidateRect(hwnd, nullptr, FALSE);
        Sleep(1);
    }

    g_app = nullptr;
    return 0;
}

#else   // ============================================================ headless

int main(int argc, char** argv) {
    const int seconds = (argc > 1) ? std::atoi(argv[1]) : 5;

    WorldConfig cfg;
    cfg.modules = MOD_DEFAULT | MOD_TOI;
    cfg.substeps = 4;
    cfg.gridCellSize = 2.5;
    cfg.arenaBytes = 32u * 1024u * 1024u;
    World world(cfg);

    buildScene(world, 120, 60, 40, 20);

    int begins = 0;
    world.setBeginContactCallback([&begins](const CollisionEvent&) { ++begins; });

    const real dt = 1.0 / 60.0;
    const int steps = seconds * 60;
    std::printf("headless real-time scene: %zu bodies, %d steps\n", world.bodyCount(), steps);

    for (int i = 0; i < steps; ++i) {
        world.step(dt);
        if (i % 60 == 0)
            std::printf("t=%5.2f s  contacts %4zu  awake %4zu  step %6.2f ms  energy %.1f\n",
                        i * dt, world.contactCount(), world.awakeCount(),
                        world.profile().ms[(size_t)Stage::Total], world.totalEnergy());
    }

    std::printf("%s\n", world.profile().toString().c_str());
    std::printf("begin-events %d   arena %.1f MB\n", begins,
                (double)world.arenaBytes() / (1024.0 * 1024.0));

    world.debugBuffer().layers = DBG_SHAPES | DBG_CONTACTS | DBG_NORMALS | DBG_JOINTS;
    world.buildDebugGeometry();
    world.saveToFile("realtime_scene.json");
    std::printf("realtime_scene.json written\n");
    return 0;
}

#endif
