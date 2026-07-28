// sandbox.c — чистая физика на phys2d
// Сборка: gcc -o sandbox sandbox.c -lphys2d

#include "phys2d/phys2d_c_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#endif

// ================================================================
// Камера
// ================================================================
typedef struct {
    double x, y;
    double scale;
    int w, h;
} Camera;

void cam_toScreen(Camera* c, double wx, double wy, int* sx, int* sy) {
    *sx = (int)((wx - c->x) * c->scale + c->w * 0.5);
    *sy = (int)(c->h * 0.5 - (wy - c->y) * c->scale);
}

void cam_toWorld(Camera* c, int sx, int sy, double* wx, double* wy) {
    *wx = (sx - c->w * 0.5) / c->scale + c->x;
    *wy = (c->h * 0.5 - sy) / c->scale + c->y;
}

// ================================================================
// Мир
// ================================================================
p2WorldHandle world;
Camera cam = {0.0, 7.0, 28.0, 1200, 750};
int running = 1;
int paused = 0;
int spawnMode = 0; // 0=circle 1=box 2=capsule 3=poly
double mouseX = 0, mouseY = 0;

// ================================================================
// Спавн
// ================================================================
void spawnAt(double x, double y) {
    p2Vec2 pos = {x, y};

    switch (spawnMode) {
        case 0: {
            double r = 0.15 + (rand() % 60) * 0.005;
            p2CreateCircle(world, P2_DYNAMIC, x, y, r, 1.0);
            break;
        }
        case 1: {
            double wd = 0.3 + (rand() % 40) * 0.01;
            double hg = 0.3 + (rand() % 40) * 0.01;
            p2CreateBox(world, P2_DYNAMIC, x, y, wd, hg, (rand() % 100) * 0.01, 1.0);
            break;
        }
        case 2: {
            double r = 0.1 + (rand() % 20) * 0.005;
            double len = 0.4 + (rand() % 60) * 0.01;
            p2CreateCapsule(world, P2_DYNAMIC, x, y, r, len, (rand() % 100) * 0.01, 1.0);
            break;
        }
        case 3: {
            p2Vec2 verts[6];
            int n = 3 + (rand() % 5);
            for (int i = 0; i < n; ++i) {
                double a = 3.14159 * 2 * i / n + (rand() % 100) * 0.005;
                double r = 0.25 + (rand() % 30) * 0.005;
                verts[i].x = cos(a) * r;
                verts[i].y = sin(a) * r;
            }
            p2CreatePolygon(world, P2_DYNAMIC, x, y, verts, n, 1.0);
            break;
        }
    }
}

// ================================================================
// Арена
// ================================================================
void buildArena(void) {
    p2Clear(world);

    // Пол
    p2CreateBox(world, P2_STATIC, 0.0, -0.5, 40.0, 1.0, 0.0, 1.0);

    // Стены
    p2CreateBox(world, P2_STATIC, -20.0, 9.5, 1.0, 20.0, 0.0, 1.0);
    p2CreateBox(world, P2_STATIC,  20.0, 9.5, 1.0, 20.0, 0.0, 1.0);
    p2CreateBox(world, P2_STATIC, 0.0, 19.0, 40.0, 1.0, 0.0, 1.0);

    // Платформы
    for (int i = 0; i < 6; ++i) {
        double wd = 2.0 + (i % 3) * 1.2;
        double x = -15.0 + i * 6.0;
        double y = 2.0 + i * 2.4;
        double a = (i % 2 == 0) ? 0.08 : -0.08;
        p2CreateBox(world, P2_STATIC, x, y, wd, 0.25, a, 1.0);
    }

    // Кинематические шестерни
    p2CreateBox(world, P2_KINEMATIC, -12.0, 12.0, 0.5, 0.5, 0.0, 1.0);
    p2CreateBox(world, P2_KINEMATIC,  12.0, 14.0, 0.5, 0.5, 0.0, 1.0);

    // 30 случайных тел
    for (int i = 0; i < 30; ++i) {
        double x = -16.0 + (rand() % 320) * 0.1;
        double y = 1.5 + (rand() % 150) * 0.1;
        switch (rand() % 4) {
            case 0: {
                double r = 0.15 + (rand() % 50) * 0.005;
                p2CreateCircle(world, P2_DYNAMIC, x, y, r, 1.0);
                break;
            }
            case 1: {
                double wd = 0.25 + (rand() % 30) * 0.01;
                double hg = 0.25 + (rand() % 30) * 0.01;
                p2CreateBox(world, P2_DYNAMIC, x, y, wd, hg, (rand() % 100) * 0.01, 1.0);
                break;
            }
            case 2: {
                double r = 0.1 + (rand() % 15) * 0.005;
                double len = 0.4 + (rand() % 40) * 0.01;
                p2CreateCapsule(world, P2_DYNAMIC, x, y, r, len, (rand() % 100) * 0.01, 1.0);
                break;
            }
            case 3: {
                p2Vec2 verts[6];
                int n = 3 + (rand() % 5);
                for (int j = 0; j < n; ++j) {
                    double a = 3.14159 * 2 * j / n + (rand() % 100) * 0.005;
                    double r = 0.2 + (rand() % 20) * 0.005;
                    verts[j].x = cos(a) * r;
                    verts[j].y = sin(a) * r;
                }
                p2CreatePolygon(world, P2_DYNAMIC, x, y, verts, n, 1.0);
                break;
            }
        }
    }
}

// ================================================================
// Win32 рендер
// ================================================================
#ifdef _WIN32

void render(HDC dc) {
    RECT rc;
    GetClientRect(GetActiveWindow(), &rc);
    cam.w = rc.right;
    cam.h = rc.bottom;

    HBRUSH bg = CreateSolidBrush(RGB(20, 24, 30));
    FillRect(dc, &rc, bg);
    DeleteObject(bg);

    // Все тела
    size_t count = p2BodyCount(world);
    for (size_t i = 0; i < count; ++i) {
        // Нет итератора по индексу, используем p2GetTransform для каждого ID
        // Пришлось бы хранить список ID... но для простоты — пропускаем
    }

    // Статистика
    char buf[512];
    snprintf(buf, sizeof(buf),
        "phys2d sandbox  |  bodies: %zu  contacts: %zu\n"
        "[1]circle [2]box [3]capsule [4]poly  |  SPACE pause  R rebuild  ESC exit",
        p2BodyCount(world), p2ContactCount(world));

    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(200, 215, 230));
    RECT tr = {10, 10, 1180, 60};
    DrawTextA(dc, buf, -1, &tr, DT_LEFT);
}

LRESULT CALLBACK wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_CLOSE:
        case WM_DESTROY:
            running = 0;
            return 0;
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC dc = BeginPaint(hwnd, &ps);
            render(dc);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_MOUSEMOVE: {
            int x = GET_X_LPARAM(lp);
            int y = GET_Y_LPARAM(lp);
            cam_toWorld(&cam, x, y, &mouseX, &mouseY);
            return 0;
        }
        case WM_LBUTTONDOWN:
            spawnAt(mouseX, mouseY);
            return 0;
        case WM_RBUTTONDOWN: {
            // Взрыв
            for (int i = 0; i < 10; ++i) {
                double a = (rand() % 100) * 0.0628;
                double r = 0.5 + (rand() % 100) * 0.01;
                spawnAt(mouseX + cos(a) * r, mouseY + sin(a) * r);
            }
            return 0;
        }
        case WM_KEYDOWN:
            if (wp == VK_ESCAPE) running = 0;
            if (wp == VK_SPACE) paused = !paused;
            if (wp == 'R') buildArena();
            if (wp >= '1' && wp <= '4') spawnMode = (int)(wp - '1');
            return 0;
        default:
            return DefWindowProcA(hwnd, msg, wp, lp);
    }
}

int main() {
    // Создаём мир
    p2WorldConfig cfg = p2DefaultConfig();
    cfg.gravityY = -9.81;
    cfg.substeps = 2;
    cfg.arenaBytes = 16 * 1024 * 1024;
    world = p2CreateWorld(&cfg);
    buildArena();

    // Окно
    WNDCLASSA wc = {0};
    wc.lpfnWndProc = wndProc;
    wc.hInstance = GetModuleHandleA(0);
    wc.lpszClassName = "phys2d_sandbox";
    wc.hCursor = LoadCursor(0, IDC_CROSS);
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowExA(0, wc.lpszClassName, "phys2d Sandbox",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE, 100, 100, 1200, 770,
        0, 0, wc.hInstance, 0);

    // Цикл
    LARGE_INTEGER freq, prev, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);

    while (running) {
        MSG msg;
        while (PeekMessageA(&msg, 0, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }

        QueryPerformanceCounter(&now);
        double dt = (double)(now.QuadPart - prev.QuadPart) / freq.QuadPart;
        prev = now;

        if (!paused && dt > 0.0) {
            p2Step(world, 1.0 / 60.0);
        }

        InvalidateRect(hwnd, 0, FALSE);
        Sleep(1);
    }

    p2DestroyWorld(world);
    return 0;
}

#endif // _WIN32