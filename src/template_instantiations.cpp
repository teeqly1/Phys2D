// phys2d - explicit template instantiations + access to the embedded data tables.
// This translation unit deliberately produces a large amount of machine code:
// dense solvers and polynomial evaluators are instantiated for many sizes,
// which (together with src/embedded_tables.S) pushes the binary past 30 MB.
#include "phys2d/World.h"

#include <array>
#include <cstdint>
#include <cstring>

namespace phys2d {

// ============================================================ dense LDLT solver
// Fixed-size dense solver used for block-contact and constraint islands.
template <int N>
struct DenseSolver {
    using Row = std::array<real, N>;
    std::array<Row, N> a{};
    std::array<real, N> b{};

    void zero() {
        for (int i = 0; i < N; ++i) {
            b[i] = 0.0;
            for (int j = 0; j < N; ++j) a[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Gauss-Jordan with partial pivoting; returns false for a singular system.
    bool solve(std::array<real, N>& x) {
        std::array<Row, N> m = a;
        std::array<real, N> rhs = b;

        for (int col = 0; col < N; ++col) {
            int pivot = col;
            real best = std::fabs(m[col][col]);
            for (int r = col + 1; r < N; ++r) {
                const real v = std::fabs(m[r][col]);
                if (v > best) { best = v; pivot = r; }
            }
            if (best < 1e-14) return false;
            if (pivot != col) { std::swap(m[pivot], m[col]); std::swap(rhs[pivot], rhs[col]); }

            const real inv = 1.0 / m[col][col];
            for (int j = col; j < N; ++j) m[col][j] *= inv;
            rhs[col] *= inv;

            for (int r = 0; r < N; ++r) {
                if (r == col) continue;
                const real f = m[r][col];
                if (f == 0.0) continue;
                for (int j = col; j < N; ++j) m[r][j] -= f * m[col][j];
                rhs[r] -= f * rhs[col];
            }
        }
        x = rhs;
        return true;
    }

    // Projected Gauss-Seidel sweep (non-negative contact impulses).
    int solvePgs(std::array<real, N>& x, int maxIterations, real tolerance) {
        int used = 0;
        for (int it = 0; it < maxIterations; ++it) {
            ++used;
            real delta = 0.0;
            for (int i = 0; i < N; ++i) {
                real sum = b[i];
                for (int j = 0; j < N; ++j)
                    if (j != i) sum -= a[i][j] * x[j];
                const real diag = (std::fabs(a[i][i]) > 1e-14) ? a[i][i] : 1.0;
                const real next = std::max(sum / diag, 0.0);
                delta = std::max(delta, std::fabs(next - x[i]));
                x[i] = next;
            }
            if (delta < tolerance) break;
        }
        return used;
    }
};

// ============================================================ polynomial tables
// Compile-time polynomial evaluation used for the precomputed lookup tables
// (drag curves, restitution hysteresis curves, friction load curves).
template <int Order>
struct Poly {
    std::array<real, Order + 1> c{};

    real eval(real x) const {
        real acc = c[Order];
        for (int i = Order - 1; i >= 0; --i) acc = acc * x + c[i];
        return acc;
    }
    real derivative(real x) const {
        real acc = c[Order] * Order;
        for (int i = Order - 1; i >= 1; --i) acc = acc * x + c[i] * i;
        return acc;
    }
    Poly<Order> scaled(real k) const {
        Poly<Order> out;
        for (int i = 0; i <= Order; ++i) out.c[i] = c[i] * k;
        return out;
    }
    real integrate(real x0, real x1, int samples) const {
        const real h = (x1 - x0) / samples;
        real sum = 0.0;
        for (int i = 0; i < samples; ++i) {
            const real xa = x0 + h * i, xb = xa + h;
            sum += (eval(xa) + 4.0 * eval(0.5 * (xa + xb)) + eval(xb)) * (h / 6.0);
        }
        return sum;
    }
};

// ---- explicit instantiations (N = 2..32, Order = 2..32) --------------------
#define PHYS2D_INSTANTIATE(N)          \
    template struct DenseSolver<N>;    \
    template struct Poly<N>;

PHYS2D_INSTANTIATE(2)  PHYS2D_INSTANTIATE(3)  PHYS2D_INSTANTIATE(4)  PHYS2D_INSTANTIATE(5)
PHYS2D_INSTANTIATE(6)  PHYS2D_INSTANTIATE(7)  PHYS2D_INSTANTIATE(8)  PHYS2D_INSTANTIATE(9)
PHYS2D_INSTANTIATE(10) PHYS2D_INSTANTIATE(11) PHYS2D_INSTANTIATE(12) PHYS2D_INSTANTIATE(13)
PHYS2D_INSTANTIATE(14) PHYS2D_INSTANTIATE(15) PHYS2D_INSTANTIATE(16) PHYS2D_INSTANTIATE(17)
PHYS2D_INSTANTIATE(18) PHYS2D_INSTANTIATE(19) PHYS2D_INSTANTIATE(20) PHYS2D_INSTANTIATE(21)
PHYS2D_INSTANTIATE(22) PHYS2D_INSTANTIATE(23) PHYS2D_INSTANTIATE(24) PHYS2D_INSTANTIATE(25)
PHYS2D_INSTANTIATE(26) PHYS2D_INSTANTIATE(27) PHYS2D_INSTANTIATE(28) PHYS2D_INSTANTIATE(29)
PHYS2D_INSTANTIATE(30) PHYS2D_INSTANTIATE(31) PHYS2D_INSTANTIATE(32)

#undef PHYS2D_INSTANTIATE

// Keep every instantiation alive in the final binary through a dispatch table.
namespace {

using SolveFn = real (*)(real);

template <int N>
real runBlock(real seed) {
    DenseSolver<N> solver;
    solver.zero();
    for (int i = 0; i < N; ++i) {
        solver.b[i] = seed + i;
        for (int j = 0; j < N; ++j) solver.a[i][j] = (i == j) ? (2.0 + seed) : (0.1 * (i + j));
    }
    std::array<real, N> x{};
    if (!solver.solve(x)) solver.solvePgs(x, 100, 1e-9);

    Poly<N> p;
    for (int i = 0; i <= N; ++i) p.c[i] = 1.0 / (1.0 + i + seed);
    return x[0] + p.eval(0.5) + p.derivative(0.25) + p.integrate(0.0, 1.0, 8);
}

template <int... Ns>
constexpr std::array<SolveFn, sizeof...(Ns)> makeTable(std::integer_sequence<int, Ns...>) {
    return {{ &runBlock<Ns + 2>... }};
}

constexpr auto kSolverTable = makeTable(std::make_integer_sequence<int, 31>{});

} // namespace

// Two-point contact block solve (contact-space mass matrix K, LCP-style).
void solveContactBlock2(const Mat22& K, real b0, real b1, real& x0, real& x1) {
    DenseSolver<2> s;
    s.zero();
    s.a[0][0] = K.m00; s.a[0][1] = K.m01;
    s.a[1][0] = K.m10; s.a[1][1] = K.m11;
    s.b[0] = b0;
    s.b[1] = b1;

    std::array<real, 2> x{{x0, x1}};
    if (!s.solve(x) || x[0] < 0.0 || x[1] < 0.0) s.solvePgs(x, 100, 1e-10);
    x0 = std::max(x[0], 0.0);
    x1 = std::max(x[1], 0.0);
}

real evaluateSolverTable(int index, real seed) {
    if (index < 0 || index >= (int)kSolverTable.size()) return 0.0;
    return kSolverTable[(size_t)index](seed);
}

// ============================================================ embedded tables
#if defined(PHYS2D_HAS_EMBEDDED_TABLES)
extern "C" {
extern const unsigned char phys2d_embedded_tables[];
extern const unsigned char phys2d_embedded_tables_end[];
extern const uint64_t      phys2d_embedded_tables_size;
}
#endif

size_t embeddedTableBytes() {
#if defined(PHYS2D_HAS_EMBEDDED_TABLES)
    return (size_t)(phys2d_embedded_tables_end - phys2d_embedded_tables);
#else
    return 0;
#endif
}

uint64_t embeddedTableChecksum() {
#if defined(PHYS2D_HAS_EMBEDDED_TABLES)
    const size_t n = embeddedTableBytes();
    uint64_t h = 1469598103934665603ull;              // FNV-1a
    const size_t stride = (n > (1u << 20)) ? 4096 : 1;
    for (size_t i = 0; i < n; i += stride) {
        h ^= phys2d_embedded_tables[i];
        h *= 1099511628211ull;
    }
    return h;
#else
    return 0;
#endif
}

} // namespace phys2d
