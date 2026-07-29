// phys2d — рендер-модуль движка. Основной бэкенд: Direct3D 9 (Windows).
// На платформах без DirectX собирается «пустой» бэкенд, поэтому код игры
// компилируется и прогоняется где угодно без правок.
#pragma once

#include "Vec2.h"
#include "DebugDraw.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace phys2d {

class World;
class RigidBody;
struct RendererImpl;

enum class RenderBackend : uint8_t {
    None      = 0,   // офлайн: треугольники считаются, но не выводятся
    Direct3D9 = 1    // DirectX
};

struct RenderColor {
    uint8_t r = 255, g = 255, b = 255, a = 255;
};
inline RenderColor rgba(int r, int g, int b, int a = 255) {
    RenderColor c;
    c.r = (uint8_t)(r < 0 ? 0 : (r > 255 ? 255 : r));
    c.g = (uint8_t)(g < 0 ? 0 : (g > 255 ? 255 : g));
    c.b = (uint8_t)(b < 0 ? 0 : (b > 255 ? 255 : b));
    c.a = (uint8_t)(a < 0 ? 0 : (a > 255 ? 255 : a));
    return c;
}
inline RenderColor fade(RenderColor c, real k) {
    c.a = (uint8_t)(c.a * (k < 0.0 ? 0.0 : (k > 1.0 ? 1.0 : k)));
    return c;
}

struct RenderConfig {
    int         width  = 1440;
    int         height = 860;
    std::string title  = "phys2d";
    real        pixelsPerMeter = 34.0;
    Vec2        camera;
    RenderColor clearColor = rgba(24, 26, 32);
    bool        vsync = true;
    // Только для бэкенда None: сколько кадров «прокрутить» перед выходом.
    int         offlineFrames = 600;
};

// Состояние ввода за кадр. keyDown — виртуальные коды клавиш Windows
// ('A'..'Z', '0'..'9', VK_*); pressed — только фронт нажатия.
struct RenderInput {
    bool keyDown[256]    = {};
    bool keyPressed[256] = {};
    int  mouseX = 0, mouseY = 0;
    Vec2 mouseWorld;
    bool leftDown = false, rightDown = false;
    bool leftPressed = false, rightPressed = false;
    real wheel = 0.0;      // шаги колеса за кадр (+вверх)
    bool closeRequested = false;
};

// Палитра для drawWorld: возвращает заливку и цвет контура для тела.
using BodyPalette = void (*)(const RigidBody& body, void* user,
                            RenderColor& fill, RenderColor& edge);

// Аппаратный рендер движка. Всё рисование пакуется в один буфер цветных
// треугольников и отправляется пачками через DrawPrimitiveUP — без шейдеров,
// без d3dx9, без COM-идентификаторов, поэтому собирается обычным MinGW.
class Renderer {
public:
    // Возвращает nullptr, если окно/устройство создать не удалось.
    static std::unique_ptr<Renderer> create(const RenderConfig& config);
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    RenderBackend backend() const;
    const char*   backendName() const;

    // ---- цикл кадра -------------------------------------------------------
    bool  pumpEvents();          // false — окно закрыто, пора выходить
    const RenderInput& input() const;
    void  beginFrame();          // очистить буфер треугольников
    void  endFrame();            // Clear + DrawPrimitiveUP + Present
    real  frameSeconds() const;  // дельта времени, уже ограниченная [1/600, 1/20]

    // ---- камера -----------------------------------------------------------
    Vec2  camera() const;
    void  setCamera(const Vec2& c);
    real  pixelsPerMeter() const;
    void  setPixelsPerMeter(real ppm);   // зажимается в [4, 400]
    void  zoomBy(real factor);
    int   width() const;
    int   height() const;
    Vec2  toWorld(int screenX, int screenY) const;
    void  toScreen(const Vec2& world, float& screenX, float& screenY) const;

    // ---- примитивы в мировых координатах ----------------------------------
    // Мягкая частица: яркий центр, прозрачный край. Из неё делаются вода,
    // кровь, искры, дым — то, что на GDI выглядело плоскими кружками.
    void softDisc(const Vec2& center, real radius, RenderColor inner,
                  RenderColor outer, int segments = 10);
    void disc(const Vec2& center, real radius, RenderColor color, int segments = 14);
    void thickLine(const Vec2& a, const Vec2& b, real width,
                   RenderColor ca, RenderColor cb);
    void fillConvex(const std::vector<Vec2>& worldVertices, RenderColor center,
                    RenderColor edge);
    void polyOutline(const std::vector<Vec2>& worldVertices, real width,
                     RenderColor color);
    void ellipse(const Vec2& center, real halfWidth, real halfHeight,
                 RenderColor inner, RenderColor outer, int segments = 14);

    // ---- примитивы в пикселях (HUD) ---------------------------------------
    void rectPx(float x, float y, float w, float h, RenderColor color);
    void triPx(float x1, float y1, float x2, float y2, float x3, float y3,
               RenderColor color);
    // Встроенный растровый шрифт 3x5, рисуется квадами (ID3DXFont не нужен).
    void textPx(float x, float y, const std::string& text, float pixelSize,
                RenderColor color);
    float textWidthPx(const std::string& text, float pixelSize) const;

    // ---- высокоуровневая отрисовка сцены движка ---------------------------
    void drawBody(const RigidBody& body, RenderColor fill, RenderColor edge,
                  real outlineWidth = 0.035);
    void drawWorld(World& world, BodyPalette palette = nullptr, void* user = nullptr);
    // Частицы SPH: три слоя (тело объёма, ядро, пена по скорости).
    void drawWaterParticles(const Vec2* positions, const Vec2* velocities,
                            size_t count, real radius = 0.20,
                            RenderColor tint = rgba(40, 120, 210));
    void drawDebugBuffer(const DebugDrawBuffer& buffer);

    size_t triangleCount() const;      // треугольников в текущем кадре
    size_t lastFrameTriangles() const; // сколько было отправлено в прошлом кадре

private:
    Renderer();
    std::unique_ptr<RendererImpl> m;
};

} // namespace phys2d
