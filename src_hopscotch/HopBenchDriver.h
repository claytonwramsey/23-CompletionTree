#pragma once
// Shared timing/reporting helpers for the per-domain bench_*.cpp drivers
// (bench_hopscotch.cpp for pickplace, bench_stacking.cpp for stacking, ...).
#include "hop_bench_cxx.h"
#include <Core/util.h>
#include <Search/AStar.h>
#include <chrono>
#include <ostream>

namespace hopct {

struct TrialResult {
    bool solved = false;
    double elapsed_s = 0.;
    unsigned steps = 0;
    unsigned nodes = 0;
    // Why the search stopped. One of "solved", "timeout", "step_cap", "node_cap", or
    // "exhausted". Only "timeout" means the wall-clock budget ran out; "exhausted" means the
    // queue emptied, so the search ruled out every node it could reach.
    const char *stop = "solved";
};

// The search mode named by the `solver` parameter.
// "ELS" runs effort level search, ordering the queue by each node's level.
// "RoundRobin" runs the FIFO baseline, which spends compute on every open node in turn.
// These are the two solvers the completion tree paper's own LGP experiment compares.
inline rai::AStar::SearchMode parseSearchMode() {
    std::string solver = rai::getParameter<rai::String>("solver", STRING("ELS")).p;
    if (solver == "ELS") {
        return rai::AStar::astar;
    }
    if (solver == "RoundRobin") {
        return rai::AStar::FIFO;
    }
    HALT("unknown solver '" << solver << "' (this driver supports ELS/RoundRobin)");
}

// Steps `astar` until a solution is found or one of the (wall-clock time /
// step count / node count) budgets is exhausted. See bench_hopscotch.cpp's
// original comments for why both a step cap and a node cap exist alongside
// the wall-clock timeout (RSS safety backstop for astar.mem, which never
// frees a node -- same design as rai's own LGP search).
inline TrialResult runSearch(rai::AStar &astar, double timeout_s, int stepCap, int nodeCap) {
    TrialResult r;
    // std::chrono::steady_clock instead of rai::realTime(): monotonic (immune
    // to system-clock adjustments, unlike rai::realTime()'s system_clock
    // basis), and this is exactly the elapsed-time-delta use case it's for.
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        astar.step();
        r.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (astar.solutions.N > 0) {
            r.solved = true;
            r.stop = "solved";
            break;
        }
        if (r.elapsed_s >= timeout_s) {
            r.stop = "timeout";
            break;
        }
        if (static_cast<int>(astar.steps) >= stepCap) {
            r.stop = "step_cap";
            break;
        }
        if (static_cast<int>(astar.mem.N) >= nodeCap) {
            r.stop = "node_cap";
            break;
        }
        if (!astar.queue.N) {
            r.stop = "exhausted";
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
