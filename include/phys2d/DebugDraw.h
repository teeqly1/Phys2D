// phys2d — дебаг-визуализация: контакты, силы, траектории, AABB, оси SAT.
#pragma once

#include "Vec2.h"
#include <vector>
#include <string>
#include <cstdint>

namespace phys2d {

enum DebugLayer : uint32_t {
    DBG_SHAPES     = 1u << 0,
    DBG_CONTACTS   = 1u << 1,   // рисование контактов
    DBG_NORMALS    = 1u << 2,
    DBG_FORCES     = 1u << 3,   // рисование сил
    DBG_TRAILS     = 1u << 4,   // рисование траекторий
    DBG_AABB       = 1u << 5,   // отображение AABB
    DBG_SAT_AXES   = 1u << 6,   // отображение осей SAT
    DBG_JOINTS     = 1u << 7,
    DBG_SLEEP      = 1u << 8,
    DBG_GRID       = 1u << 9,
    DBG_ALL        = 0xFFFFFFFFu
};

struct DebugColor {
    uint8_t r = 255, g = 255, b = 255, a = 255;
};

struct DebugLine    { Vec2 a, b;       DebugColor color; uint32_t layer = DBG_SHAPES; };
struct DebugCircleD { Vec2 center; real radius = 0; DebugColor color; uint32_t layer = DBG_SHAPES; };
struct DebugPoint   { Vec2 p; real size = 0.05; DebugColor color; uint32_t layer = DBG_CONTACTS; };
struct DebugText    { Vec2 p; std::string text; DebugColor color; uint32_t layer = DBG_SHAPES; };

// Буфер команд рисования — заполняется движком, читается рендером.
struct DebugDrawBuffer {
    uint32_t layers = DBG_SHAPES | DBG_CONTACTS | DBG_AABB;
    real     forceScale = 0.02;
    real     normalScale = 0.3;
    int      trailLength = 32;

    std::vector<DebugLine>    lines;
    std::vector<DebugCircleD> circles;
    std::vector<DebugPoint>   points;
    std::vector<DebugText>    texts;

    bool has(uint32_t layer) const { return (layers & layer) != 0; }
    void clear() { lines.clear(); circles.clear(); points.clear(); texts.clear(); }

    void line(const Vec2& a, const Vec2& b, DebugColor c, uint32_t layer) {
        if (has(layer)) lines.push_back({a, b, c, layer});
    }
    void circle(const Vec2& c0, real r, DebugColor c, uint32_t layer) {
        if (has(layer)) circles.push_back({c0, r, c, layer});
    }
    void point(const Vec2& p, real s, DebugColor c, uint32_t layer) {
        if (has(layer)) points.push_back({p, s, c, layer});
    }
    void box(const AABB& bb, DebugColor c, uint32_t layer) {
        if (!has(layer)) return;
        const Vec2 p00(bb.min.x, bb.min.y), p10(bb.max.x, bb.min.y);
        const Vec2 p11(bb.max.x, bb.max.y), p01(bb.min.x, bb.max.y);
        lines.push_back({p00, p10, c, layer});
        lines.push_back({p10, p11, c, layer});
        lines.push_back({p11, p01, c, layer});
        lines.push_back({p01, p00, c, layer});
    }
    size_t itemCount() const { return lines.size() + circles.size() + points.size() + texts.size(); }
};

// Экспорт буфера в SVG — удобно для оффлайн-отладки без графического бэкенда.
std::string debugBufferToSvg(const DebugDrawBuffer& buf, const AABB& view, int pixelWidth = 1200);

} // namespace phys2d
