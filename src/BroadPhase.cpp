#include "phys2d/BroadPhase.h"
#include <algorithm>
#include <cmath>

namespace phys2d {

// ---------------------------------------------------------------- сетка
void SpatialGrid::clear() {
    for (auto& kv : m_cells) kv.second.clear();
}

void SpatialGrid::insert(RigidBody* b) {
    const AABB& bb = b->fatAABB;
    const int32_t x0 = (int32_t)std::floor(bb.min.x / m_cellSize);
    const int32_t x1 = (int32_t)std::floor(bb.max.x / m_cellSize);
    const int32_t y0 = (int32_t)std::floor(bb.min.y / m_cellSize);
    const int32_t y1 = (int32_t)std::floor(bb.max.y / m_cellSize);

    // Защита от гигантских тел (стены): ограничиваем число ячеек.
    const int64_t cells = (int64_t)(x1 - x0 + 1) * (int64_t)(y1 - y0 + 1);
    if (cells > 4096) {
        m_cells[hashCell(0x7FFFFFFF, 0x7FFFFFFF)].push_back(b);   // список "больших" тел
        return;
    }
    for (int32_t y = y0; y <= y1; ++y)
        for (int32_t x = x0; x <= x1; ++x)
            m_cells[hashCell(x, y)].push_back(b);
}

void SpatialGrid::queryPairs(std::vector<BroadPair>& out) const {
    for (const auto& kv : m_cells) {
        const std::vector<RigidBody*>& cell = kv.second;
        for (size_t i = 0; i < cell.size(); ++i) {
            for (size_t j = i + 1; j < cell.size(); ++j) {
                RigidBody* a = cell[i];
                RigidBody* b = cell[j];
                if (!a->isDynamic() && !b->isDynamic()) continue;
                if (a->sleeping && b->sleeping) continue;
                if (!a->fatAABB.overlaps(b->fatAABB)) continue;
                out.push_back({a, b, makePairKey(a->id, b->id)});
            }
        }
    }
}

void SpatialGrid::queryRegion(const AABB& region, std::vector<RigidBody*>& out) const {
    for (const auto& kv : m_cells)
        for (RigidBody* b : kv.second)
            if (b->fatAABB.overlaps(region)) out.push_back(b);
}

// ---------------------------------------------------------------- SAP
void SweepAndPrune::build(const std::vector<RigidBody*>& bodies) {
    m_entries.clear();
    m_entries.reserve(bodies.size());
    for (RigidBody* b : bodies)
        m_entries.push_back({b, b->fatAABB.min.x, b->fatAABB.max.x});
    std::sort(m_entries.begin(), m_entries.end(),
              [](const Entry& l, const Entry& r) { return l.minX < r.minX; });
}

void SweepAndPrune::update() {
    // Обновляем границы и досортировываем вставками — почти O(n) при малых смещениях.
    for (Entry& e : m_entries) {
        e.minX = e.body->fatAABB.min.x;
        e.maxX = e.body->fatAABB.max.x;
    }
    for (size_t i = 1; i < m_entries.size(); ++i) {
        Entry key = m_entries[i];
        size_t j = i;
        while (j > 0 && m_entries[j - 1].minX > key.minX) {
            m_entries[j] = m_entries[j - 1];
            --j;
        }
        m_entries[j] = key;
    }
}

void SweepAndPrune::queryPairs(std::vector<BroadPair>& out) const {
    for (size_t i = 0; i < m_entries.size(); ++i) {
        const Entry& ei = m_entries[i];
        for (size_t j = i + 1; j < m_entries.size(); ++j) {
            const Entry& ej = m_entries[j];
            if (ej.minX > ei.maxX) break;                 // дальше пересечений нет
            RigidBody* a = ei.body;
            RigidBody* b = ej.body;
            if (!a->isDynamic() && !b->isDynamic()) continue;
            if (a->sleeping && b->sleeping) continue;
            if (!a->fatAABB.overlaps(b->fatAABB)) continue;
            out.push_back({a, b, makePairKey(a->id, b->id)});
        }
    }
}

// ---------------------------------------------------------------- BroadPhase
void BroadPhase::updateAABBs(std::vector<RigidBody*>& bodies, real dt) {
    m_moved = 0;
    for (RigidBody* b : bodies) {
        // Динамический запас: базовый margin + прогноз по скорости.
        const real speed  = b->velocity.length();
        const real margin = aabbMargin + velocityMarginScale * speed * dt;
        const AABB tight  = b->shape.computeAABB(b->transform());
        b->aabb = tight;
        if (!b->fatAABB.contains(tight)) {
            b->fatAABB = tight.expanded(margin);
            ++m_moved;
        }
    }
}

void BroadPhase::buildRegions(const std::vector<RigidBody*>& bodies, int count) {
    count = std::max(1, count);
    m_regions.assign((size_t)count, SceneRegion{});

    AABB world;
    for (RigidBody* b : bodies) world.combine(b->fatAABB);
    if (bodies.empty()) return;

    // Разбиение сцены на вертикальные полосы (регионы) для параллельной работы.
    const real span = std::max(world.max.x - world.min.x, EPSILON);
    const real step = span / (real)count;
    for (int i = 0; i < count; ++i) {
        m_regions[(size_t)i].bounds = AABB(Vec2(world.min.x + step * i, world.min.y),
                                           Vec2(world.min.x + step * (i + 1), world.max.y));
    }
    for (RigidBody* b : bodies) {
        int i0 = (int)std::floor((b->fatAABB.min.x - world.min.x) / step);
        int i1 = (int)std::floor((b->fatAABB.max.x - world.min.x) / step);
        i0 = std::max(0, std::min(count - 1, i0));
        i1 = std::max(0, std::min(count - 1, i1));
        for (int i = i0; i <= i1; ++i) m_regions[(size_t)i].bodies.push_back(b);
    }
}

void BroadPhase::computePairs(std::vector<RigidBody*>& bodies,
                              std::vector<BroadPair>& out,
                              bool multithreaded,
                              int  threadCount) {
    out.clear();
    (void)threadCount;

    if (mode == BroadPhaseMode::SweepAndPrune) {
        if (m_sap.size() != bodies.size()) m_sap.build(bodies);
        else                               m_sap.update();
        m_sap.queryPairs(out);
    } else if (mode == BroadPhaseMode::Grid) {
        m_grid.clear();
        for (RigidBody* b : bodies) m_grid.insert(b);
        m_grid.queryPairs(out);
    } else {
        // Hybrid: сетка как основной индекс, SAP для крупных/статических тел.
        m_grid.clear();
        std::vector<RigidBody*> large;
        for (RigidBody* b : bodies) {
            const Vec2 ext = b->fatAABB.extents();
            if (std::max(ext.x, ext.y) > m_grid.cellSize() * 4.0) large.push_back(b);
            else                                                  m_grid.insert(b);
        }
        m_grid.queryPairs(out);

        if (!large.empty()) {
            // Крупные тела проверяем против всех остальных (их мало).
            for (RigidBody* lb : large) {
                for (RigidBody* b : bodies) {
                    if (b == lb) continue;
                    if (!lb->isDynamic() && !b->isDynamic()) continue;
                    if (lb->sleeping && b->sleeping) continue;
                    if (!lb->fatAABB.overlaps(b->fatAABB)) continue;
                    if (lb->id > b->id && std::find(large.begin(), large.end(), b) != large.end())
                        continue;               // пара large-large только один раз
                    out.push_back({lb, b, makePairKey(lb->id, b->id)});
                }
            }
        }
    }

    if (multithreaded) buildRegions(bodies, regionCount);

    // Убираем дубликаты (тело может попасть в несколько ячеек).
    std::sort(out.begin(), out.end(),
              [](const BroadPair& l, const BroadPair& r) { return l.key < r.key; });
    out.erase(std::unique(out.begin(), out.end(),
                          [](const BroadPair& l, const BroadPair& r) { return l.key == r.key; }),
              out.end());
}

} // namespace phys2d
