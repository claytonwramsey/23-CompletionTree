#pragma once
// A rai::ComputeNode that gives the completion tree task search for the mobile problem.
#include "HopCommon.h"
#include "HopMobileNode.h"
#include <Logic/folWorld.h>
#include <Search/AStar.h>
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

struct HopMobileSkeletonRoot : rai::ComputeNode {
    const MobileScenario &scenario; // borrowed; owned by the driver

    std::shared_ptr<rai::FOL_World> L;
    std::shared_ptr<rai::AStar> folAstar;
    std::vector<std::unique_ptr<std::vector<Action>>> skeletonPlans;
    int totalSkeletonCount = -1; // -1 == still possibly-infinite; see HopSkeletonRoot.h

    explicit HopMobileSkeletonRoot(const MobileScenario &scenario);

    void untimedCompute() override { }
    int getNumDecisions() override { return totalSkeletonCount; }
    double effortHeuristic() override { return 11. + 10.; }
    double branchingPenalty_child(int i) override;

    std::shared_ptr<ComputeNode> createNewChild(int i) override;

private:
    const std::vector<Action> *getOrBuildSkeleton(int i);
};

} // namespace hopct
