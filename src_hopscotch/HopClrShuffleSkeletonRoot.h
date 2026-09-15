#pragma once
// A rai::ComputeNode that gives the completion tree task-level search for the `clr_shuffle`
// problem.
#include "HopClrShuffleNode.h"
#include "HopCommon.h"
#include <Logic/folWorld.h>
#include <Search/AStar.h>
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

struct HopClrShuffleSkeletonRoot : rai::ComputeNode {
    const ClrShuffleScenario &scenario; // borrowed; owned by the driver

    std::shared_ptr<rai::FOL_World> L;
    std::shared_ptr<rai::AStar> folAstar;
    std::vector<std::unique_ptr<std::vector<ClrAction>>> skeletonPlans;
    int totalSkeletonCount = -1; // -1 while the symbolic search may still yield more
    // When non-negative, every child grounds this one skeleton, so a trial isolates whether a
    // known task plan can be grounded at all apart from the rest of the search.
    int forcedSkeleton = -1;

    explicit HopClrShuffleSkeletonRoot(const ClrShuffleScenario &scenario);

    void untimedCompute() override { }
    int getNumDecisions() override { return forcedSkeleton >= 0 ? -1 : totalSkeletonCount; }
    double effortHeuristic() override { return 11. + 10.; }
    double branchingPenalty_child(int i) override;

    std::shared_ptr<ComputeNode> createNewChild(int i) override;

private:
    const std::vector<ClrAction> *getOrBuildSkeleton(int i);
};

} // namespace hopct
