#include "phys2d/Profiler.h"
#include <cstdio>

namespace phys2d {

std::string ProfileData::toString() const {
    char buf[2048];
    int n = 0;
    n += std::snprintf(buf + n, sizeof(buf) - (size_t)n,
                       "step %llu | dt=%.5f s | тел: %d (активных %d, спят %d)\n",
                       (unsigned long long)steps, dtUsed, bodies, awakeBodies, sleepingBodies);
    n += std::snprintf(buf + n, sizeof(buf) - (size_t)n,
                       "пар broad: %d | контактов: %d (точек %d) | substeps: %d\n",
                       broadPairs, contacts, contactPoints, substepsUsed);
    n += std::snprintf(buf + n, sizeof(buf) - (size_t)n,
                       "GJK: %d | EPA: %d | SAT: %d (кэш %d) | TOI: %d\n",
                       gjkCalls, epaCalls, satCalls, satCacheHits, toiCalls);
    for (size_t i = 0; i < (size_t)Stage::Count; ++i) {
        n += std::snprintf(buf + n, sizeof(buf) - (size_t)n,
                           "  %-20s %8.3f мс  (ср %7.3f | макс %7.3f)\n",
                           stageName((Stage)i), ms[i], msAverage[i], msMax[i]);
        if ((size_t)n + 64 >= sizeof(buf)) break;
    }
    return std::string(buf, (size_t)n);
}

} // namespace phys2d
