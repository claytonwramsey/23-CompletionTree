#pragma once
// A rai::ComputeNode that gives the completion tree task-level search for the `stacking` problem.
#include "HopCommon.h"
#include "HopStackingNode.h"
#include <Logic/folWorld.h>
#include <Search/AStar.h>
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

struct HopStackingSkeletonRoot : rai::ComputeNode {
    const StackingScenario &scenario; // borrowed; owned by the driver
    RobotTag robot;

    std::shared_ptr<rai::FOL_World> L;
    std::shared_ptr<rai::AStar> folAstar;
    std::vector<std::unique_ptr<std::vector<Action>>> skeletonPlans;
    int totalSkeletonCount = -1; // -1 == still possibly-infinite; see HopSkeletonRoot.h
    // placeOrder[k] = scenario object index goal_order wants at tower
    // position k -- only used to build the terminal-rule chain in the ctor
    std::vector<size_t> placeOrder;

    HopStackingSkeletonRoot(const StackingScenario &scenario, RobotTag robot);

    virtual void untimedCompute() { }
    virtual int getNumDecisions() { return totalSkeletonCount; }
    virtual double effortHeuristic() { return 11. + 10.; }
    virtual double branchingPenalty_child(int i);

    virtual std::shared_ptr<ComputeNode> createNewChild(int i);

private:
    const std::vector<Action> *getOrBuildSkeleton(int i);
};

} // namespace hopct
