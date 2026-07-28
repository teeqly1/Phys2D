// phys2d — широкая фаза: равномерная сетка + SAP по отсортированным AABB.
#pragma once

#include "Body.h"
#include "Collision.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>

namespace phys2d {

struct BroadPair {
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    uint64_t   key = 0;
};

enum class BroadPhaseMode : uint8_t { Grid = 0, SweepAndPrune = 1, Hybrid = 2 };

// Равномерная сетка с настраиваемым размером ячейки.
class SpatialGrid {
public:
    explicit SpatialGrid(real cellSize = 4.0) : m_cellSize(cellSize) {}

    void setCellSize(real s) { m_cellSize = (s > EPSILON) ? s : 1.0; }
    real cellSize() const { return m_cellSize; }

    void clear();
    void insert(RigidBody* b);
    void queryPairs(std::vector<BroadPair>& out) const;
    void queryRegion(const AABB& region, std::vector<RigidBody*>& out) const;
    size_t cellCount() const { return m_cells.size(); }

private:
    static uint64_t hashCell(int32_t x, int32_t y) {
        return ((uint64_t)(uint32_t)x << 32) ^ (uint32_t)y;
    }
    real m_cellSize;
    std::unordered_map<uint64_t, std::vector<RigidBody*>> m_cells;
};

// Sweep and prune по оси X с постоянно отсортированным списком (insertion sort — почти O(n)).
class SweepAndPrune {
public:
    void build(const std::vector<RigidBody*>& bodies);
    void update();                                  // инкрементальная пересортировка
    void queryPairs(std::vector<BroadPair>& out) const;
    size_t size() const { return m_entries.size(); }

private:
    struct Entry { RigidBody* body; real minX, maxX; };
    std::vector<Entry> m_entries;
};

// Разбиение сцены на регионы для параллельной широкой фазы.
struct SceneRegion {
    AABB bounds;
    std::vector<RigidBody*> bodies;
};

class BroadPhase {
public:
    BroadPhaseMode mode      = BroadPhaseMode::Hybrid;
    real           aabbMargin = 0.10;   // динамическое обновление AABB с запасом
    real           velocityMarginScale = 2.0;
    int            regionCount = 4;

    void  setCellSize(real s) { m_grid.setCellSize(s); }
    void  updateAABBs(std::vector<RigidBody*>& bodies, real dt);
    void  computePairs(std::vector<RigidBody*>& bodies,
                       std::vector<BroadPair>& out,
                       bool multithreaded,
                       int  threadCount);
    void  buildRegions(const std::vector<RigidBody*>& bodies, int count);
    const std::vector<SceneRegion>& regions() const { return m_regions; }
    size_t gridCells() const { return m_grid.cellCount(); }
    int    movedCount() const { return m_moved; }

private:
    SpatialGrid              m_grid{4.0};
    SweepAndPrune            m_sap;
    std::vector<SceneRegion> m_regions;
    int                      m_moved = 0;
};

} // namespace phys2d
