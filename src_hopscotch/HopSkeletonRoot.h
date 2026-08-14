#pragma once
// Task search for pick-place problems.
#include "HopCommon.h"
#include "HopPickPlaceNode.h"
#include <Logic/folWorld.h>
#include <Search/AStar.h>
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

struct HopSkeletonRoot : rai::ComputeNode {
    const PickPlaceScenario &scenario; // borrowed; owned by the driver
    RobotTag robot;

    std::shared_ptr<rai::FOL_World> L;
    std::shared_ptr<rai::AStar> folAstar;

    // List of plans already created.
    std::vector<std::unique_ptr<std::vector<Action>>> skeletonPlans;

    // The domain (see HopSkeletonRoot.cpp) models objects with an `on`
    // predicate rather than a monotonic "already placed, never touch again"
    // flag, so the same object can be picked and placed arbitrarily many
    // times (e.g. moved out of the way, then moved again) and onto any
    // surface, not just a single fixed goal surface -- the symbolic plan
    // space is therefore genuinely infinite (unlike the earlier
    // once-per-object-only version). -1 permanently in practice; kept as a
    // field (rather than removed) because rai::AStar::step()'s queue could
    // still empty in the degenerate case of zero objects/surfaces, and
    // getNumDecisions() must reflect that if it ever happens rather than
    // claiming infinite branching (Bpar(n)=∞) forever -- see .cpp's
    // createNewChild for why this alone isn't sufficient and a "dead child"
    // fallback is also needed.
    int totalSkeletonCount = -1;

    HopSkeletonRoot(const PickPlaceScenario &scenario, RobotTag robot);

    virtual void untimedCompute() { }
    virtual int getNumDecisions() { return totalSkeletonCount; } // -1 == still infinite
    virtual double effortHeuristic() { return 11. + 10.; }
    virtual double branchingPenalty_child(int i);

    virtual std::shared_ptr<ComputeNode> createNewChild(int i);

private:
    // nullptr means the symbolic search space is exhausted -- no skeleton at
    // this index (or beyond) exists.
    const std::vector<Action> *getOrBuildSkeleton(int i);
};

} // namespace hopct
