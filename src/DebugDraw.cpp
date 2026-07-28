#include "phys2d/DebugDraw.h"
#include <cstdio>
#include <sstream>

namespace phys2d {

static std::string colorToHex(const DebugColor& c) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%02x%02x%02x", c.r, c.g, c.b);
    return std::string(buf);
}

std::string debugBufferToSvg(const DebugDrawBuffer& buf, const AABB& view, int pixelWidth) {
    const real w = std::max(view.max.x - view.min.x, EPSILON);
    const real h = std::max(view.max.y - view.min.y, EPSILON);
    const real scale = (real)pixelWidth / w;
    const int  pixelHeight = (int)(h * scale);

    auto X = [&](real x) { return (x - view.min.x) * scale; };
    auto Y = [&](real y) { return (real)pixelHeight - (y - view.min.y) * scale; };

    std::ostringstream out;
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << pixelWidth
        << "\" height=\"" << pixelHeight << "\" viewBox=\"0 0 " << pixelWidth << ' ' << pixelHeight << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#101418\"/>\n";

    for (const DebugLine& l : buf.lines) {
        out << "<line x1=\"" << X(l.a.x) << "\" y1=\"" << Y(l.a.y)
            << "\" x2=\"" << X(l.b.x) << "\" y2=\"" << Y(l.b.y)
            << "\" stroke=\"" << colorToHex(l.color) << "\" stroke-width=\"1\" stroke-opacity=\""
            << (real)l.color.a / 255.0 << "\"/>\n";
    }
    for (const DebugCircleD& c : buf.circles) {
        out << "<circle cx=\"" << X(c.center.x) << "\" cy=\"" << Y(c.center.y)
            << "\" r=\"" << c.radius * scale << "\" fill=\"none\" stroke=\""
            << colorToHex(c.color) << "\" stroke-width=\"1\"/>\n";
    }
    for (const DebugPoint& p : buf.points) {
        out << "<circle cx=\"" << X(p.p.x) << "\" cy=\"" << Y(p.p.y)
            << "\" r=\"" << std::max(p.size * scale, 1.0) << "\" fill=\""
            << colorToHex(p.color) << "\"/>\n";
    }
    for (const DebugText& t : buf.texts) {
        out << "<text x=\"" << X(t.p.x) << "\" y=\"" << Y(t.p.y)
            << "\" fill=\"" << colorToHex(t.color) << "\" font-size=\"12\" font-family=\"monospace\">"
            << t.text << "</text>\n";
    }
    out << "</svg>\n";
    return out.str();
}

} // namespace phys2d
