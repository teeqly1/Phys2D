// phys2d - простой спавнер человечков
// Нажимаешь ЛКМ - появляется человечек, падает и дрыгается

#include "phys2d/Extras.h"
#include <cstdio>
#include <cstdlib>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <windowsx.h>   // <-- ЭТО НУЖНО
#endif

using namespace phys2d;

// ============================================================
// Камера
// ============================================================
struct Camera {
    Vec2 center{0.0, 5.0};
    real scale = 28.0;
    int w = 1200, h = 750;

    void toScreen(const Vec2& world, int& x, int& y) const {
        x = (int)((world.x - center.x) * scale + w * 0.5);
        y = (int)(h * 0.5 - (world.y - center.y) * scale);
    }

    Vec2 toWorld(int sx, int sy) const {
        return Vec2((sx - w * 0.5) / scale + center.x,
                    (h * 0.5 - sy) / scale + center.y);
    }
};

// ============================================================
// Состояние
// ============================================================
struct App {
    Engine engine{[] {
        WorldConfig cfg;
        cfg.gravity = Vec2(0.0, -9.81);
        cfg.substeps = 4;
        cfg.arenaBytes = 32 * 1024 * 1024;
        return cfg;
    }()};

    Camera cam;
    bool running = true;
    bool paused = false;
    Vec2 mouse;

    int spawned = 0;
    int ragdolls = 0;
};

App* g_app = nullptr;

// ============================================================
// Построить арену
// ============================================================
void buildArena(App& app) {
    World& w = app.engine.world();
    w.clear();

    // Пол
    BodyDef floor;
    floor.type = BodyType::Static;
    floor.shape = Shape::box(40.0, 1.0);
    floor.position = Vec2(0.0, -0.5);
    floor.material.restitution = 0.05;
    w.createBody(floor);

    // Стены
    BodyDef wall;
    wall.type = BodyType::Static;
    wall.shape = Shape::box(1.0, 18.0);
    wall.position = Vec2(-20.0, 8.5);
    w.createBody(wall);
    wall.position = Vec2(20.0, 8.5);
    w.createBody(wall);

    // Потолок
    BodyDef ceil;
    ceil.type = BodyType::Static;
    ceil.shape = Shape::box(40.0, 1.0);
    ceil.position = Vec2(0.0, 18.0);
    w.createBody(ceil);
}

// ============================================================
// Спавн человечка
// ============================================================
void spawnDude(App& app, const Vec2& pos) {
    RagdollConfig cfg;
    cfg.position = pos;
    cfg.scale = 0.8 + (rand() % 50) * 0.005;
    cfg.density = 0.8 + (rand() % 40) * 0.01;

    createRagdoll(app.engine.world(), cfg);
    app.spawned++;
}

// ============================================================
// Win32 рендер
// ============================================================
#ifdef _WIN32

void drawBody(HDC dc, const Camera& cam, const RigidBody* b) {
    COLORREF col;
    if (b->isStatic())      col = RGB(120, 140, 160);
    else if (b->sleeping)   col = RGB(80, 100, 120);
    else                    col = RGB(200, 160, 140); // телесный

    HPEN pen = CreatePen(PS_SOLID, 1, col);
    HGDIOBJ old = SelectObject(dc, pen);

    if (b->shape.type == ShapeType::Circle) {
        int cx, cy;
        cam.toScreen(b->position, cx, cy);
        int r = (int)(b->shape.radius * cam.scale);
        Ellipse(dc, cx - r, cy - r, cx + r, cy + r);
    } else if (b->shape.type == ShapeType::Capsule) {
        Vec2 a, e;
        b->shape.capsuleSegment(b->transform(), a, e);
        int ax, ay, ex, ey;
        cam.toScreen(a, ax, ay);
        cam.toScreen(e, ex, ey);
        int r = (int)(b->shape.radius * cam.scale);
        Ellipse(dc, ax - r, ay - r, ax + r, ay + r);
        Ellipse(dc, ex - r, ey - r, ex + r, ey + r);
        MoveToEx(dc, ax, ay, nullptr);
        LineTo(dc, ex, ey);
    } else {
        const auto& vs = b->worldVertices;
        if (vs.size() >= 2) {
            std::vector<POINT> pts(vs.size());
            for (size_t i = 0; i < vs.size(); ++i) {
                int px, py;
                cam.toScreen(vs[i], px, py);
                pts[i].x = px;
                pts[i].y = py;
            }
            Polygon(dc, pts.data(), (int)pts.size());
        }
    }

    SelectObject(dc, old);
    DeleteObject(pen);
}

void drawHud(HDC dc, App& app) {
    char buf[512];
    snprintf(buf, sizeof(buf),
        "🧍 Спавн человечков  |  spawned: %d  ragdolls: %d  bodies: %zu\n"
        "ЛКМ - спавн  ПКМ - взрыв  SPACE - пауза  R - перестроить  ESC - выход",
        app.spawned, app.ragdolls, app.engine.world().bodyCount());

    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(200, 215, 230));
    RECT r{10, 10, 1180, 50};
    DrawTextA(dc, buf, -1, &r, DT_LEFT);
}

void render(HWND hwnd, App& app) {
    PAINTSTRUCT ps;
    HDC dc = BeginPaint(hwnd, &ps);

    RECT rc;
    GetClientRect(hwnd, &rc);
    app.cam.w = rc.right;
    app.cam.h = rc.bottom;

    HDC mem = CreateCompatibleDC(dc);
    HBITMAP bmp = CreateCompatibleBitmap(dc, app.cam.w, app.cam.h);
    HGDIOBJ oldBmp = SelectObject(mem, bmp);

    HBRUSH bg = CreateSolidBrush(RGB(24, 28, 34));
    FillRect(mem, &rc, bg);
    DeleteObject(bg);

    SelectObject(mem, GetStockObject(NULL_BRUSH));

    for (RigidBody* b : app.engine.world().bodies()) {
        drawBody(mem, app.cam, b);
    }

    drawHud(mem, app);

    BitBlt(dc, 0, 0, app.cam.w, app.cam.h, mem, 0, 0, SRCCOPY);
    SelectObject(mem, oldBmp);
    DeleteObject(bmp);
    DeleteDC(mem);

    EndPaint(hwnd, &ps);
}

LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    App* app = g_app;
    if (!app) return DefWindowProcA(hwnd, msg, wp, lp);

    switch (msg) {
        case WM_CLOSE:
        case WM_DESTROY:
            app->running = false;
            return 0;

        case WM_PAINT:
            render(hwnd, *app);
            return 0;

        case WM_ERASEBKGND:
            return 1;

        case WM_MOUSEMOVE: {
            int x = GET_X_LPARAM(lp);
            int y = GET_Y_LPARAM(lp);
            app->mouse = app->cam.toWorld(x, y);
            return 0;
        }

        case WM_LBUTTONDOWN: {
            spawnDude(*app, app->mouse);
            return 0;
        }

        case WM_RBUTTONDOWN: {
            // Взрыв - разбрасывает тела
            for (RigidBody* b : app->engine.world().bodies()) {
                if (!b->isDynamic()) continue;
                Vec2 d = b->position - app->mouse;
                real dist = d.length();
                if (dist < 0.1 || dist > 8.0) continue;
                real force = 12.0 / (dist + 0.5);
                b->applyImpulseAtPoint(d.normalized() * force * b->mass, b->position);
            }
            return 0;
        }

        case WM_KEYDOWN:
            if (wp == VK_ESCAPE) app->running = false;
            if (wp == VK_SPACE) app->paused = !app->paused;
            if (wp == 'R') buildArena(*app);
            return 0;

        default:
            return DefWindowProcA(hwnd, msg, wp, lp);
    }
}

#endif

// ============================================================
// Main
// ============================================================
int main() {
    App app;
    g_app = &app;

    buildArena(app);

    #ifdef _WIN32
    WNDCLASSA wc{};
    wc.lpfnWndProc = wndProc;
    wc.hInstance = GetModuleHandleA(nullptr);
    wc.lpszClassName = "phys2d_spawn_demo";
    wc.hCursor = LoadCursor(nullptr, IDC_CROSS);
    wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowExA(
        0, wc.lpszClassName, "🧍 Спавн человечков - phys2d",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT, 1200, 800,
        nullptr, nullptr, wc.hInstance, nullptr
    );

    if (!hwnd) {
        printf("Cannot create window\n");
        return 1;
    }

    LARGE_INTEGER freq, prev, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);

    while (app.running) {
        MSG msg;
        while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }

        QueryPerformanceCounter(&now);
        double dt = (double)(now.QuadPart - prev.QuadPart) / freq.QuadPart;
        prev = now;

        if (!app.paused && dt > 0.0) {
            app.engine.step(1.0 / 60.0);
        }

        InvalidateRect(hwnd, nullptr, FALSE);
        Sleep(1);
    }

    #endif

    return 0;
}