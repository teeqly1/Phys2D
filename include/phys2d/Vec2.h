// phys2d — 2D physics engine. Vector / matrix / transform math (double precision).
#pragma once

#include <cmath>
#include <algorithm>
#include <limits>

#if defined(PHYS2D_SIMD_AVX)
#  include <immintrin.h>
#elif defined(PHYS2D_SIMD_SSE)
#  include <emmintrin.h>
#endif

namespace phys2d {

using real = double;                       // все вычисления — double

constexpr real PI       = 3.14159265358979323846;
constexpr real EPSILON  = 1e-12;
constexpr real BIG      = std::numeric_limits<real>::max();

inline bool nearlyZero(real v, real eps = 1e-10) { return std::fabs(v) < eps; }
inline real clampr(real v, real lo, real hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline real sqr(real v) { return v * v; }

// ---------------------------------------------------------------- Vector2D
struct Vec2 {
    real x = 0.0, y = 0.0;

    Vec2() = default;
    Vec2(real x_, real y_) : x(x_), y(y_) {}

    // ---- сложение / вычитание / умножение / деление
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(real s)        const { return {x * s, y * s}; }
    Vec2 operator/(real s)        const { real i = (std::fabs(s) < EPSILON) ? 0.0 : 1.0 / s; return {x * i, y * i}; }
    Vec2 operator-()              const { return {-x, -y}; }

    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(real s)        { x *= s;  y *= s;  return *this; }
    Vec2& operator/=(real s)        { real i = (std::fabs(s) < EPSILON) ? 0.0 : 1.0 / s; x *= i; y *= i; return *this; }

    bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Vec2& o) const { return !(*this == o); }

    // ---- длина / нормализация
    real lengthSq() const { return x * x + y * y; }
    real length()   const { return std::sqrt(x * x + y * y); }

    Vec2 normalized() const {
        const real l = length();
        return (l < EPSILON) ? Vec2(0.0, 0.0) : Vec2(x / l, y / l);
    }
    real normalize() {                       // нормализация на месте, возвращает старую длину
        const real l = length();
        if (l > EPSILON) { x /= l; y /= l; }
        else             { x = y = 0.0; }
        return l;
    }

    // ---- скалярное / векторное произведение
    real dot(const Vec2& o)   const { return x * o.x + y * o.y; }
    real cross(const Vec2& o) const { return x * o.y - y * o.x; }   // z-компонента

    // ---- поворот
    Vec2 rotated(real a) const {
        const real c = std::cos(a), s = std::sin(a);
        return {x * c - y * s, x * s + y * c};
    }
    Vec2 rotatedAround(const Vec2& pivot, real a) const { return pivot + (*this - pivot).rotated(a); }
    Vec2 perp()  const { return {-y, x}; }               // поворот на +90°
    Vec2 rperp() const { return { y, -x}; }              // поворот на -90°

    // ---- отражение относительно нормали n (n предполагается единичной)
    Vec2 reflected(const Vec2& n) const { return *this - n * (2.0 * dot(n)); }

    Vec2 projectedOn(const Vec2& o) const {
        const real d = o.lengthSq();
        return (d < EPSILON) ? Vec2(0, 0) : o * (dot(o) / d);
    }
    Vec2 abs()  const { return {std::fabs(x), std::fabs(y)}; }
    real angle() const { return std::atan2(y, x); }
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y); }

    static Vec2 fromAngle(real a, real len = 1.0) { return {std::cos(a) * len, std::sin(a) * len}; }
    static Vec2 lerp(const Vec2& a, const Vec2& b, real t) { return a + (b - a) * t; }
    static Vec2 min(const Vec2& a, const Vec2& b) { return {std::min(a.x, b.x), std::min(a.y, b.y)}; }
    static Vec2 max(const Vec2& a, const Vec2& b) { return {std::max(a.x, b.x), std::max(a.y, b.y)}; }
};

inline Vec2 operator*(real s, const Vec2& v) { return {v.x * s, v.y * s}; }
inline real dot(const Vec2& a, const Vec2& b)   { return a.x * b.x + a.y * b.y; }
inline real cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
inline Vec2 cross(const Vec2& v, real s)        { return { s * v.y, -s * v.x}; }
inline Vec2 cross(real s, const Vec2& v)        { return {-s * v.y,  s * v.x}; }
inline real distance(const Vec2& a, const Vec2& b)   { return (a - b).length(); }
inline real distanceSq(const Vec2& a, const Vec2& b) { return (a - b).lengthSq(); }

#if defined(PHYS2D_SIMD_SSE) || defined(PHYS2D_SIMD_AVX)
// Пакетная обработка массивов позиций (v += a * dt) через SSE2/AVX.
inline void simdAxpy(Vec2* dst, const Vec2* src, real s, int count) {
#  if defined(PHYS2D_SIMD_AVX)
    const __m256d vs = _mm256_set1_pd(s);
    int i = 0;
    for (; i + 1 < count; i += 2) {
        __m256d d = _mm256_loadu_pd(&dst[i].x);
        __m256d a = _mm256_loadu_pd(&src[i].x);
        _mm256_storeu_pd(&dst[i].x, _mm256_add_pd(d, _mm256_mul_pd(a, vs)));
    }
    for (; i < count; ++i) dst[i] += src[i] * s;
#  else
    const __m128d vs = _mm_set1_pd(s);
    for (int i = 0; i < count; ++i) {
        __m128d d = _mm_loadu_pd(&dst[i].x);
        __m128d a = _mm_loadu_pd(&src[i].x);
        _mm_storeu_pd(&dst[i].x, _mm_add_pd(d, _mm_mul_pd(a, vs)));
    }
#  endif
}
#else
inline void simdAxpy(Vec2* dst, const Vec2* src, real s, int count) {
    for (int i = 0; i < count; ++i) dst[i] += src[i] * s;
}
#endif

// ---------------------------------------------------------------- Mat22
struct Mat22 {
    real m00 = 1, m01 = 0, m10 = 0, m11 = 1;

    Mat22() = default;
    Mat22(real a, real b, real c, real d) : m00(a), m01(b), m10(c), m11(d) {}
    explicit Mat22(real angle) { set(angle); }

    void set(real angle) {
        const real c = std::cos(angle), s = std::sin(angle);
        m00 = c; m01 = -s; m10 = s; m11 = c;
    }
    Vec2  operator*(const Vec2& v)  const { return {m00 * v.x + m01 * v.y, m10 * v.x + m11 * v.y}; }
    Mat22 operator*(const Mat22& o) const {
        return {m00 * o.m00 + m01 * o.m10, m00 * o.m01 + m01 * o.m11,
                m10 * o.m00 + m11 * o.m10, m10 * o.m01 + m11 * o.m11};
    }
    Mat22 transposed() const { return {m00, m10, m01, m11}; }
    real  det() const { return m00 * m11 - m01 * m10; }
    Mat22 inverse() const {
        real d = det();
        if (std::fabs(d) < EPSILON) return Mat22(0, 0, 0, 0);
        d = 1.0 / d;
        return {m11 * d, -m01 * d, -m10 * d, m00 * d};
    }
    // Решение 2x2 системы (эффективная матрица масс контактного пространства)
    Vec2 solve(const Vec2& b) const {
        real d = det();
        if (std::fabs(d) < EPSILON) return {0, 0};
        d = 1.0 / d;
        return {d * (m11 * b.x - m01 * b.y), d * (m00 * b.y - m10 * b.x)};
    }
};

// ---------------------------------------------------------------- Transform
struct Transform {
    Vec2 p;            // положение
    real angle = 0.0;  // угол поворота

    Transform() = default;
    Transform(const Vec2& p_, real a) : p(p_), angle(a) {}

    Vec2 apply(const Vec2& local)   const { return p + local.rotated(angle); }   // локальные -> глобальные
    Vec2 applyR(const Vec2& local)  const { return local.rotated(angle); }
    Vec2 invApply(const Vec2& world) const { return (world - p).rotated(-angle); }
    Vec2 invApplyR(const Vec2& w)    const { return w.rotated(-angle); }
};

// ---------------------------------------------------------------- AABB
struct AABB {
    Vec2 min{ BIG,  BIG};
    Vec2 max{-BIG, -BIG};

    AABB() = default;
    AABB(const Vec2& lo, const Vec2& hi) : min(lo), max(hi) {}

    void reset() { min = Vec2( BIG,  BIG); max = Vec2(-BIG, -BIG); }
    void add(const Vec2& v) { min = Vec2::min(min, v); max = Vec2::max(max, v); }
    void combine(const AABB& o) { min = Vec2::min(min, o.min); max = Vec2::max(max, o.max); }

    AABB expanded(real margin) const { return {min - Vec2(margin, margin), max + Vec2(margin, margin)}; }

    bool overlaps(const AABB& o) const {
        return !(o.min.x > max.x || o.max.x < min.x || o.min.y > max.y || o.max.y < min.y);
    }
    bool contains(const AABB& o) const {
        return min.x <= o.min.x && min.y <= o.min.y && max.x >= o.max.x && max.y >= o.max.y;
    }
    bool contains(const Vec2& v) const {
        return v.x >= min.x && v.x <= max.x && v.y >= min.y && v.y <= max.y;
    }
    Vec2 center()  const { return (min + max) * 0.5; }
    Vec2 extents() const { return (max - min) * 0.5; }
    real area()    const { Vec2 d = max - min; return d.x * d.y; }
    real perimeter() const { Vec2 d = max - min; return 2.0 * (d.x + d.y); }
};

} // namespace phys2d
