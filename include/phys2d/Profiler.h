// phys2d — профайлинг: замер времени каждого этапа шага.
#pragma once

#include "Vec2.h"
#include <chrono>
#include <string>
#include <array>
#include <cstdint>

namespace phys2d {

enum class Stage : int {
    Total = 0,
    IntegrateForces,
    BroadPhase,
    NarrowPhase,
    SolveVelocity,
    SolvePosition,
    Constraints,
    IntegratePositions,
    Sleeping,
    DebugDraw,
    Count
};

inline const char* stageName(Stage s) {
    switch (s) {
        case Stage::Total:              return "total";
        case Stage::IntegrateForces:    return "integrate_forces";
        case Stage::BroadPhase:         return "broad_phase";
        case Stage::NarrowPhase:        return "narrow_phase";
        case Stage::SolveVelocity:      return "solve_velocity";
        case Stage::SolvePosition:      return "solve_position";
        case Stage::Constraints:        return "constraints";
        case Stage::IntegratePositions: return "integrate_positions";
        case Stage::Sleeping:           return "sleeping";
        case Stage::DebugDraw:          return "debug_draw";
        default:                        return "unknown";
    }
}

struct ProfileData {
    std::array<double, (size_t)Stage::Count> ms{};          // последний шаг, мс
    std::array<double, (size_t)Stage::Count> msAverage{};    // скользящее среднее
    std::array<double, (size_t)Stage::Count> msMax{};
    uint64_t steps = 0;

    // Статистика шага
    int  bodies = 0, awakeBodies = 0, sleepingBodies = 0;
    int  broadPairs = 0, contacts = 0, contactPoints = 0;
    int  velocityIterationsUsed = 0, positionIterationsUsed = 0;
    int  substepsUsed = 0;
    int  gjkCalls = 0, epaCalls = 0, satCalls = 0, satCacheHits = 0, toiCalls = 0;
    real dtUsed = 0.0;

    void reset() {
        ms.fill(0.0);
        broadPairs = contacts = contactPoints = 0;
        gjkCalls = epaCalls = satCalls = satCacheHits = toiCalls = 0;
    }
    std::string toString() const;
};

class Profiler {
public:
    using Clock = std::chrono::steady_clock;

    void begin(Stage s) { m_start[(size_t)s] = Clock::now(); }
    void end(Stage s, ProfileData& data) {
        const auto dt = std::chrono::duration<double, std::milli>(Clock::now() - m_start[(size_t)s]).count();
        const size_t i = (size_t)s;
        data.ms[i] += dt;
    }
    static void commit(ProfileData& data) {
        const double alpha = 0.05;
        for (size_t i = 0; i < (size_t)Stage::Count; ++i) {
            data.msAverage[i] = (data.steps == 0) ? data.ms[i]
                                                  : data.msAverage[i] * (1.0 - alpha) + data.ms[i] * alpha;
            data.msMax[i] = std::max(data.msMax[i], data.ms[i]);
        }
        ++data.steps;
    }
    bool enabled = true;
private:
    std::array<Clock::time_point, (size_t)Stage::Count> m_start{};
};

// RAII-замер этапа.
struct ScopedStage {
    ScopedStage(Profiler& p, ProfileData& d, Stage s) : prof(p), data(d), stage(s) {
        if (prof.enabled) prof.begin(stage);
    }
    ~ScopedStage() { if (prof.enabled) prof.end(stage, data); }
    Profiler&    prof;
    ProfileData& data;
    Stage        stage;
};

} // namespace phys2d
