#pragma once
// Shared timing/reporting helpers for the per-domain bench_*.cpp drivers
// (bench_hopscotch.cpp for pickplace, bench_stacking.cpp for stacking, ...).
#include "hop_bench_cxx.h"
#include <Core/util.h>
#include <Search/AStar.h>
#include <ostream>

namespace hopct {

struct TrialResult {
    bool solved = false;
    double elapsed_s = 0.;
    unsigned steps = 0;
    unsigned nodes = 0;
};

// Steps `astar` until a solution is found or one of the (wall-clock time /
// step count / node count) budgets is exhausted. See bench_hopscotch.cpp's
// original comments for why both a step cap and a node cap exist alongside
// the wall-clock timeout (RSS safety backstop for astar.mem, which never
// frees a node -- same design as rai's own LGP search).
inline TrialResult runSearch(rai::AStar &astar, double timeout_s, int stepCap, int nodeCap) {
    TrialResult r;
    double t0 = rai::realTime();
    while (true) {
        astar.step();
        r.elapsed_s = rai::realTime() - t0;
        if (astar.solutions.N > 0) {
            r.solved = true;
            break;
        }
        if (r.elapsed_s >= timeout_s) {
            break;
        }
        if ((int)astar.steps >= stepCap) {
            break;
        }
        if ((int)astar.mem.N >= nodeCap) {
            break;
        }
        if (!astar.queue.N) {
            break;
        }
    }
    r.steps = astar.steps;
    r.nodes = astar.mem.N;
    return r;
}

inline void writePose(std::ostream &os, CPose p) {
    os << "[" << p.x << "," << p.y << "," << p.z << "," << p.qx << "," << p.qy << "," << p.qz << ","
       << p.qw << "]";
}
inline void writeConfig(std::ostream &os, CConfig c) {
    os << "[";
    for (int i = 0; i < c.dim; i++) {
        if (i) {
            os << ",";
        }
        os << c.q[i];
    }
    os << "]";
}

} // namespace hopct
