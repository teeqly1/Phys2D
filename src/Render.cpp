// phys2d — реализация рендера. Геометрия, батчинг и шрифт общие для всех
// платформ; только создание устройства и отправка треугольников зависят от
// Direct3D 9. Так вся математика рендера компилируется и тестируется везде.
#include "phys2d/Render.h"
#include "phys2d/Body.h"
#include "phys2d/Shape.h"
#include "phys2d/World.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#if defined(_WIN32) && !defined(PHYS2D_NO_D3D9)
#  define PHYS2D_D3D9 1
#  include <windows.h>
#  include <d3d9.h>
#else
#  define PHYS2D_D3D9 0
#endif

namespace phys2d {
namespace {

// Формат вершины совпадает с D3DFVF_XYZRHW | D3DFVF_DIFFUSE, поэтому буфер
// уходит в DrawPrimitiveUP без переупаковки.
struct Vtx {
    float    x = 0.0f, y = 0.0f, z = 0.0f, rhw = 1.0f;
    uint32_t color = 0xFFFFFFFFu;
};

inline uint32_t packColor(RenderColor c) {
    return ((uint32_t)c.a << 24) | ((uint32_t)c.r << 16) | ((uint32_t)c.g << 8) | (uint32_t)c.b;
}

// Растровый шрифт 3x5: пять строк по три бита на символ.
struct Glyph { char c; unsigned char rows[5]; };
const Glyph FONT[] = {
    {'A',{7,5,7,5,5}}, {'B',{7,5,6,5,7}}, {'C',{7,4,4,4,7}}, {'D',{6,5,5,5,6}},
    {'E',{7,4,7,4,7}}, {'F',{7,4,7,4,4}}, {'G',{7,4,5,5,7}}, {'H',{5,5,7,5,5}},
    {'I',{7,2,2,2,7}}, {'J',{1,1,1,5,7}}, {'K',{5,5,6,5,5}}, {'L',{4,4,4,4,7}},
    {'M',{5,7,7,5,5}}, {'N',{6,5,5,5,5}}, {'O',{7,5,5,5,7}}, {'P',{7,5,7,4,4}},
    {'Q',{7,5,5,7,1}}, {'R',{7,5,7,6,5}}, {'S',{7,4,7,1,7}}, {'T',{7,2,2,2,2}},
    {'U',{5,5,5,5,7}}, {'V',{5,5,5,5,2}}, {'W',{5,5,7,7,5}}, {'X',{5,5,2,5,5}},
    {'Y',{5,5,7,2,2}}, {'Z',{7,1,2,4,7}},
    {'0',{7,5,5,5,7}}, {'1',{2,6,2,2,7}}, {'2',{7,1,7,4,7}}, {'3',{7,1,7,1,7}},
    {'4',{5,5,7,1,1}}, {'5',{7,4,7,1,7}}, {'6',{7,4,7,5,7}}, {'7',{7,1,1,1,1}},
    {'8',{7,5,7,5,7}}, {'9',{7,5,7,1,7}},
    {' ',{0,0,0,0,0}}, {'.',{0,0,0,0,2}}, {',',{0,0,0,2,2}}, {':',{0,2,0,2,0}},
    {'-',{0,0,7,0,0}}, {'_',{0,0,0,0,7}}, {'=',{0,7,0,7,0}}, {'/',{1,1,2,4,4}},
    {'+',{0,2,7,2,0}}, {'%',{5,1,2,4,5}}, {'*',{5,2,7,2,5}}, {'!',{2,2,2,0,2}},
    {'?',{7,1,3,0,2}}, {'[',{3,2,2,2,3}}, {']',{6,2,2,2,6}}, {'(',{3,2,2,2,3}},
    {')',{6,2,2,2,6}}, {'<',{1,2,4,2,1}}, {'>',{4,2,1,2,4}}, {'#',{5,7,5,7,5}},
};

const Glyph* findGlyph(char ch) {
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); ++i)
        if (FONT[i].c == ch) return &FONT[i];
    return nullptr;
}

inline real clampReal(real v, real lo, real hi) { return v < lo ? lo : (v > hi ? hi : v); }

} // namespace

// ---------------------------------------------------------------- impl
struct RendererImpl {
    RenderConfig cfg;
    RenderInput  in;
    std::vector<Vtx> tris;
    size_t lastTris = 0;
    int    offlineLeft = 0;
    std::chrono::steady_clock::time_point prevTick = std::chrono::steady_clock::now();
    real   dt = 1.0 / 60.0;
    RenderBackend backend = RenderBackend::None;

#if PHYS2D_D3D9
    HWND                  hwnd = nullptr;
    IDirect3D9*           d3d  = nullptr;
    IDirect3DDevice9*     dev  = nullptr;
    D3DPRESENT_PARAMETERS pp;
    bool                  needReset = false;
    bool                  quit = false;
#endif

    void pushVtx(float x, float y, uint32_t c) {
        Vtx v;
        v.x = x - 0.5f;          // выравнивание по центрам пикселей D3D9
        v.y = y - 0.5f;
        v.z = 0.0f;
        v.rhw = 1.0f;
        v.color = c;
        tris.push_back(v);
    }
    void pushTri(float x1, float y1, uint32_t c1, float x2, float y2, uint32_t c2,
                 float x3, float y3, uint32_t c3) {
        pushVtx(x1, y1, c1);
        pushVtx(x2, y2, c2);
        pushVtx(x3, y3, c3);
    }
    void screenOf(const Vec2& w, float& sx, float& sy) const {
        sx = (float)((w.x - cfg.camera.x) * cfg.pixelsPerMeter + cfg.width * 0.5);
        sy = (float)(cfg.height * 0.5 - (w.y - cfg.camera.y) * cfg.pixelsPerMeter);
    }
};

#if PHYS2D_D3D9
static RendererImpl* g_active = nullptr;   // одно окно на процесс

static LRESULT CALLBACK phys2dWndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    RendererImpl* m = g_active;
    if (!m) return DefWindowProcA(hwnd, msg, wp, lp);
    switch (msg) {
        case WM_CLOSE:
        case WM_DESTROY:
            m->quit = true;
            m->in.closeRequested = true;
            return 0;
        case WM_SIZE:
            if (LOWORD(lp) > 0 && HIWORD(lp) > 0) {
                m->cfg.width  = LOWORD(lp);
                m->cfg.height = HIWORD(lp);
                m->needReset  = true;
            }
            return 0;
        case WM_MOUSEMOVE:
            m->in.mouseX = (int)(short)LOWORD(lp);
            m->in.mouseY = (int)(short)HIWORD(lp);
            return 0;
        case WM_LBUTTONDOWN: m->in.leftDown = true;  m->in.leftPressed = true;  return 0;
        case WM_LBUTTONUP:   m->in.leftDown = false; return 0;
        case WM_RBUTTONDOWN: m->in.rightDown = true; m->in.rightPressed = true; return 0;
        case WM_RBUTTONUP:   m->in.rightDown = false; return 0;
        case WM_MOUSEWHEEL:
            m->in.wheel += (real)GET_WHEEL_DELTA_WPARAM(wp) / 120.0;
            return 0;
        case WM_KEYDOWN:
            if (wp < 256) {
                if (!m->in.keyDown[wp]) m->in.keyPressed[wp] = true;
                m->in.keyDown[wp] = true;
            }
            return 0;
        case WM_KEYUP:
            if (wp < 256) m->in.keyDown[wp] = false;
            return 0;
        default: break;
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

static void applyStates(IDirect3DDevice9* dev) {
    if (!dev) return;
    dev->SetRenderState(D3DRS_LIGHTING, FALSE);
    dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    dev->SetRenderState(D3DRS_ZENABLE, FALSE);
    dev->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    dev->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
    dev->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    dev->SetTexture(0, nullptr);
    dev->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1);
    dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    dev->SetTextureStageState(0, D3DTSS_ALPHAOP,   D3DTOP_SELECTARG1);
    dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
}
#endif // PHYS2D_D3D9

Renderer::Renderer() : m(new RendererImpl()) {}
Renderer::~Renderer() {
#if PHYS2D_D3D9
    if (m) {
        if (m->dev) m->dev->Release();
        if (m->d3d) m->d3d->Release();
        if (m->hwnd) DestroyWindow(m->hwnd);
        if (g_active == m.get()) g_active = nullptr;
    }
#endif
}

std::unique_ptr<Renderer> Renderer::create(const RenderConfig& config) {
    std::unique_ptr<Renderer> r(new Renderer());
    r->m->cfg = config;
    r->m->cfg.pixelsPerMeter = clampReal(config.pixelsPerMeter, 4.0, 400.0);
    r->m->offlineLeft = config.offlineFrames;
    r->m->tris.reserve(1 << 16);

#if PHYS2D_D3D9
    WNDCLASSA wc;
    ZeroMemory(&wc, sizeof(wc));
    wc.lpfnWndProc   = phys2dWndProc;
    wc.hInstance     = GetModuleHandleA(nullptr);
    wc.hCursor       = LoadCursorA(nullptr, IDC_CROSS);
    wc.lpszClassName = "phys2dRenderWindow";
    RegisterClassA(&wc);

    g_active = r->m.get();
    r->m->hwnd = CreateWindowA("phys2dRenderWindow", config.title.c_str(),
                              WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
                              config.width, config.height, nullptr, nullptr,
                              wc.hInstance, nullptr);
    if (!r->m->hwnd) { g_active = nullptr; return nullptr; }

    r->m->d3d = Direct3DCreate9(D3D_SDK_VERSION);
    if (!r->m->d3d) { g_active = nullptr; return nullptr; }

    ZeroMemory(&r->m->pp, sizeof(r->m->pp));
    r->m->pp.Windowed             = TRUE;
    r->m->pp.SwapEffect           = D3DSWAPEFFECT_DISCARD;
    r->m->pp.BackBufferFormat     = D3DFMT_UNKNOWN;
    r->m->pp.hDeviceWindow        = r->m->hwnd;
    r->m->pp.PresentationInterval = config.vsync ? D3DPRESENT_INTERVAL_ONE
                                                 : D3DPRESENT_INTERVAL_IMMEDIATE;
    HRESULT hr = r->m->d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, r->m->hwnd,
                                         D3DCREATE_HARDWARE_VERTEXPROCESSING,
                                         &r->m->pp, &r->m->dev);
    if (FAILED(hr))
        hr = r->m->d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, r->m->hwnd,
                                     D3DCREATE_SOFTWARE_VERTEXPROCESSING,
                                     &r->m->pp, &r->m->dev);
    if (FAILED(hr) || !r->m->dev) { g_active = nullptr; return nullptr; }

    applyStates(r->m->dev);
    r->m->backend = RenderBackend::Direct3D9;
#else
    r->m->backend = RenderBackend::None;
#endif
    return r;
}

RenderBackend Renderer::backend() const { return m->backend; }
const char* Renderer::backendName() const {
    return m->backend == RenderBackend::Direct3D9 ? "Direct3D 9" : "offline";
}

bool Renderer::pumpEvents() {
    std::memset(m->in.keyPressed, 0, sizeof(m->in.keyPressed));
    m->in.leftPressed = false;
    m->in.rightPressed = false;
    m->in.wheel = 0.0;

    const auto now = std::chrono::steady_clock::now();
    m->dt = clampReal(std::chrono::duration<real>(now - m->prevTick).count(),
                      1.0 / 600.0, 1.0 / 20.0);
    m->prevTick = now;

#if PHYS2D_D3D9
    MSG msg;
    while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) m->quit = true;
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
    if (m->needReset && m->dev) {                       // ресайз или потеря устройства
        const HRESULT coop = m->dev->TestCooperativeLevel();
        if (coop != D3DERR_DEVICELOST) {
            m->tris.clear();
            m->pp.BackBufferWidth  = m->cfg.width;
            m->pp.BackBufferHeight = m->cfg.height;
            if (SUCCEEDED(m->dev->Reset(&m->pp))) {
                applyStates(m->dev);
                m->needReset = false;
            }
        }
    }
    m->in.mouseWorld = toWorld(m->in.mouseX, m->in.mouseY);
    return !m->quit;
#else
    if (m->offlineLeft <= 0) { m->in.closeRequested = true; return false; }
    --m->offlineLeft;
    m->in.mouseWorld = toWorld(m->in.mouseX, m->in.mouseY);
    return true;
#endif
}

const RenderInput& Renderer::input() const { return m->in; }
real Renderer::frameSeconds() const { return m->dt; }

void Renderer::beginFrame() { m->tris.clear(); }

void Renderer::endFrame() {
    m->lastTris = m->tris.size() / 3;
#if PHYS2D_D3D9
    if (!m->dev) return;
    const RenderColor bg = m->cfg.clearColor;
    m->dev->Clear(0, nullptr, D3DCLEAR_TARGET,
                  D3DCOLOR_XRGB(bg.r, bg.g, bg.b), 1.0f, 0);
    if (SUCCEEDED(m->dev->BeginScene())) {
        m->dev->SetFVF(D3DFVF_XYZRHW | D3DFVF_DIFFUSE);
        const size_t total = m->tris.size() / 3;
        size_t done = 0;
        while (done < total) {                      // пачками, без вершинных буферов
            const size_t chunk = std::min<size_t>(2000, total - done);
            m->dev->DrawPrimitiveUP(D3DPT_TRIANGLELIST, (UINT)chunk,
                                    &m->tris[done * 3], sizeof(Vtx));
            done += chunk;
        }
        m->dev->EndScene();
    }
    if (m->dev->Present(nullptr, nullptr, nullptr, nullptr) == D3DERR_DEVICELOST)
        m->needReset = true;
#endif
}

Vec2 Renderer::camera() const { return m->cfg.camera; }
void Renderer::setCamera(const Vec2& c) { m->cfg.camera = c; }
real Renderer::pixelsPerMeter() const { return m->cfg.pixelsPerMeter; }
void Renderer::setPixelsPerMeter(real ppm) { m->cfg.pixelsPerMeter = clampReal(ppm, 4.0, 400.0); }
void Renderer::zoomBy(real factor) { setPixelsPerMeter(m->cfg.pixelsPerMeter * factor); }
int  Renderer::width() const { return m->cfg.width; }
int  Renderer::height() const { return m->cfg.height; }

Vec2 Renderer::toWorld(int sx, int sy) const {
    return Vec2((sx - m->cfg.width * 0.5) / m->cfg.pixelsPerMeter + m->cfg.camera.x,
                (m->cfg.height * 0.5 - sy) / m->cfg.pixelsPerMeter + m->cfg.camera.y);
}
void Renderer::toScreen(const Vec2& world, float& sx, float& sy) const {
    m->screenOf(world, sx, sy);
}

void Renderer::softDisc(const Vec2& center, real radius, RenderColor inner,
                        RenderColor outer, int segments) {
    if (segments < 3) segments = 3;
    float cx, cy;
    m->screenOf(center, cx, cy);
    float rad = (float)(radius * m->cfg.pixelsPerMeter);
    if (rad < 0.4f) rad = 0.4f;
    const uint32_t ci = packColor(inner), co = packColor(outer);
    const float step = 6.28318530718f / (float)segments;
    for (int i = 0; i < segments; ++i) {
        const float a0 = step * i, a1 = step * (i + 1);
        m->pushTri(cx, cy, ci,
                   cx + std::cos(a0) * rad, cy + std::sin(a0) * rad, co,
                   cx + std::cos(a1) * rad, cy + std::sin(a1) * rad, co);
    }
}

void Renderer::disc(const Vec2& center, real radius, RenderColor color, int segments) {
    softDisc(center, radius, color, color, segments);
}

void Renderer::ellipse(const Vec2& center, real halfWidth, real halfHeight,
                       RenderColor inner, RenderColor outer, int segments) {
    if (segments < 3) segments = 3;
    float cx, cy;
    m->screenOf(center, cx, cy);
    const float hw = std::max(1.0f, (float)(halfWidth * m->cfg.pixelsPerMeter));
    const float hh = std::max(1.0f, (float)(halfHeight * m->cfg.pixelsPerMeter));
    const uint32_t ci = packColor(inner), co = packColor(outer);
    const float step = 6.28318530718f / (float)segments;
    for (int i = 0; i < segments; ++i) {
        const float a0 = step * i, a1 = step * (i + 1);
        m->pushTri(cx, cy, ci,
                   cx + std::cos(a0) * hw, cy + std::sin(a0) * hh, co,
                   cx + std::cos(a1) * hw, cy + std::sin(a1) * hh, co);
    }
}

void Renderer::thickLine(const Vec2& a, const Vec2& b, real width,
                         RenderColor ca, RenderColor cb) {
    float ax, ay, bx, by;
    m->screenOf(a, ax, ay);
    m->screenOf(b, bx, by);
    const float dx = bx - ax, dy = by - ay;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.0001f) return;
    const float hw = std::max(0.6f, (float)(width * m->cfg.pixelsPerMeter * 0.5));
    const float nx = -dy / len * hw, ny = dx / len * hw;
    const uint32_t c1 = packColor(ca), c2 = packColor(cb);
    m->pushTri(ax + nx, ay + ny, c1, ax - nx, ay - ny, c1, bx - nx, by - ny, c2);
    m->pushTri(ax + nx, ay + ny, c1, bx - nx, by - ny, c2, bx + nx, by + ny, c2);
}

void Renderer::fillConvex(const std::vector<Vec2>& vs, RenderColor center, RenderColor edge) {
    if (vs.size() < 3) return;
    Vec2 c;
    for (const Vec2& v : vs) c += v;
    c *= (real)1.0 / (real)vs.size();
    float cx, cy;
    m->screenOf(c, cx, cy);
    const uint32_t ci = packColor(center), ce = packColor(edge);
    for (size_t i = 0; i < vs.size(); ++i) {
        float x1, y1, x2, y2;
        m->screenOf(vs[i], x1, y1);
        m->screenOf(vs[(i + 1) % vs.size()], x2, y2);
        m->pushTri(cx, cy, ci, x1, y1, ce, x2, y2, ce);
    }
}

void Renderer::polyOutline(const std::vector<Vec2>& vs, real width, RenderColor color) {
    if (vs.size() < 2) return;
    for (size_t i = 0; i < vs.size(); ++i)
        thickLine(vs[i], vs[(i + 1) % vs.size()], width, color, color);
}

void Renderer::rectPx(float x, float y, float w, float h, RenderColor color) {
    const uint32_t c = packColor(color);
    m->pushTri(x, y, c, x + w, y, c, x + w, y + h, c);
    m->pushTri(x, y, c, x + w, y + h, c, x, y + h, c);
}

void Renderer::triPx(float x1, float y1, float x2, float y2, float x3, float y3,
                     RenderColor color) {
    const uint32_t c = packColor(color);
    m->pushTri(x1, y1, c, x2, y2, c, x3, y3, c);
}

void Renderer::textPx(float x, float y, const std::string& text, float px, RenderColor color) {
    if (px <= 0.0f) px = 2.0f;
    float cx = x;
    for (char ch : text) {
        if (const Glyph* g = findGlyph(ch)) {
            for (int row = 0; row < 5; ++row)
                for (int bit = 0; bit < 3; ++bit)
                    if (g->rows[row] & (1 << (2 - bit)))
                        rectPx(cx + bit * px, y + row * px, px, px, color);
        }
        cx += 4.0f * px;
    }
}

float Renderer::textWidthPx(const std::string& text, float px) const {
    return (float)text.size() * 4.0f * (px <= 0.0f ? 2.0f : px);
}

void Renderer::drawBody(const RigidBody& body, RenderColor fill, RenderColor edge,
                        real outlineWidth) {
    if (body.shape.type == ShapeType::Circle) {
        softDisc(body.position, body.shape.radius, fill, edge, 16);
        // спица показывает вращение
        const Vec2 spoke = body.position +
            Vec2(std::cos(body.angle), std::sin(body.angle)) * body.shape.radius;
        thickLine(body.position, spoke, outlineWidth, edge, edge);
    } else if (body.shape.type == ShapeType::Capsule) {
        Vec2 a, b;
        body.shape.capsuleSegment(body.transform(), a, b);
        thickLine(a, b, body.shape.radius * 2.0, fill, fill);
        softDisc(a, body.shape.radius, fill, edge, 12);
        softDisc(b, body.shape.radius, fill, edge, 12);
    } else {
        fillConvex(body.worldVertices, fill, edge);
        const std::vector<Vec2>& vs = body.worldVertices;
        for (size_t i = 0; i < vs.size(); ++i)
            thickLine(vs[i], vs[(i + 1) % vs.size()], outlineWidth, edge, edge);
    }
}

void Renderer::drawWorld(World& world, BodyPalette palette, void* user) {
    for (RigidBody* body : world.bodies()) {
        if (!body) continue;
        RenderColor fill = body->isStatic() ? rgba(74, 70, 64) : rgba(150, 154, 162);
        RenderColor edge = rgba(18, 18, 22, 200);
        if (palette) palette(*body, user, fill, edge);
        drawBody(*body, fill, edge);
    }
}

void Renderer::drawWaterParticles(const Vec2* positions, const Vec2* velocities,
                                  size_t count, real radius, RenderColor tint) {
    if (!positions || count == 0) return;
    // 1. тело объёма: широкие перекрывающиеся пятна дают метабол-вид
    for (size_t i = 0; i < count; ++i)
        softDisc(positions[i], radius,
                 rgba(tint.r / 2, tint.g / 2, tint.b / 2, 150),
                 rgba(tint.r / 3, tint.g / 3, tint.b / 3, 0), 8);
    // 2. яркое ядро
    for (size_t i = 0; i < count; ++i)
        softDisc(positions[i], radius * 0.55,
                 rgba(tint.r + 30, tint.g + 40, tint.b + 20, 205),
                 rgba(tint.r, tint.g, tint.b, 0), 8);
    // 3. пена на быстрой воде
    if (!velocities) return;
    for (size_t i = 0; i < count; ++i) {
        const real sp = velocities[i].length();
        if (sp < 1.4) continue;
        const int a = (int)std::min((real)210.0, 60.0 + sp * 40.0);
        softDisc(positions[i] + velocities[i] * 0.02, radius * 0.45,
                 rgba(226, 240, 255, a), rgba(200, 225, 250, 0), 7);
    }
}

void Renderer::drawDebugBuffer(const DebugDrawBuffer& buf) {
    for (const DebugLine& l : buf.lines) {
        const RenderColor c = rgba(l.color.r, l.color.g, l.color.b, l.color.a);
        thickLine(l.a, l.b, 0.03, c, c);
    }
    for (const DebugCircleD& c0 : buf.circles) {
        const RenderColor c = rgba(c0.color.r, c0.color.g, c0.color.b, c0.color.a);
        const int seg = 18;
        for (int i = 0; i < seg; ++i) {
            const real a0 = 6.28318530718 * i / seg, a1 = 6.28318530718 * (i + 1) / seg;
            thickLine(c0.center + Vec2(std::cos(a0), std::sin(a0)) * c0.radius,
                      c0.center + Vec2(std::cos(a1), std::sin(a1)) * c0.radius,
                      0.03, c, c);
        }
    }
    for (const DebugPoint& p : buf.points)
        softDisc(p.p, p.size, rgba(p.color.r, p.color.g, p.color.b, p.color.a),
                 rgba(p.color.r, p.color.g, p.color.b, 0), 8);
    for (const DebugText& t : buf.texts) {
        float sx, sy;
        m->screenOf(t.p, sx, sy);
        textPx(sx, sy, t.text, 2.0f, rgba(t.color.r, t.color.g, t.color.b, t.color.a));
    }
}

size_t Renderer::triangleCount() const { return m->tris.size() / 3; }
size_t Renderer::lastFrameTriangles() const { return m->lastTris; }

} // namespace phys2d
