#pragma once
// A rai::ComputeNode that gives the completion tree task-level
// search for the `coffee` problem.
#include "HopCoffeeNode.h"
#include "HopCommon.h"
#include <Logic/folWorld.h>
#include <Search/AStar.h>
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

struct HopCoffeeSkeletonRoot : rai::ComputeNode {
    const CoffeeScenario &scenario; // borrowed; owned by the driver
    RobotTag robot;

    std::shared_ptr<rai::FOL_World> L;
    std::shared_ptr<rai::AStar> folAstar;
    std::vector<std::unique_ptr<std::vector<CoffeeAction>>> skeletonPlans;
    int totalSkeletonCount = -1; // -1 == still possibly-infinite; see HopSkeletonRoot.h

    HopCoffeeSkeletonRoot(const CoffeeScenario &scenario, RobotTag robot);

    void untimedCompute() override { }
    int getNumDecisions() override { return totalSkeletonCount; }
    double effortHeuristic() override { return 11. + 10.; }
    double branchingPenalty_child(int i) override;

    std::shared_ptr<ComputeNode> createNewChild(int i) override;

private:
    const std::vector<CoffeeAction> *getOrBuildSkeleton(int i);
};

} // namespace hopct
