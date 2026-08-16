#pragma once
// A rai::ComputeNode implementation of hopscotch's stacking domain.
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

struct HopStackingNode : rai::ComputeNode {
    const StackingScenario &scenario; // borrowed; owned by the root/driver
    const std::vector<Action> &plan; // borrowed
    RobotTag robot;
    int action_index; // -1 at the root

    CConfig q_arm;
    int64_t held_object = -1;
    CPose grasp_offset = CPose { 0, 0, 0, 0, 0, 0, 1 };
    std::shared_ptr<std::vector<CPose>> poses; // per scenario object index

    bool motionStarted = false;
    MotionPlanState motionState;
    CConfig pendingQEnd {};
    CPose pendingHeldRel {};
    bool pendingHasHeld = false;
    CConfig nextQArm {};
    int64_t nextHeldObject = -1;
    CPose nextGraspOffset {};
    CPose nextPlacedPose {};

    std::vector<CConfig> trajectory;

    HopStackingNode(const StackingScenario &scenario, const std::vector<Action> &plan,
        RobotTag robot, rai::ComputeNode *parent = nullptr);
    HopStackingNode(HopStackingNode &parent, int childIndex);

    void untimedCompute() override;
    int getNumDecisions() override { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
    double branchingPenalty_child(int i) override;
    std::shared_ptr<ComputeNode> createNewChild(int i) override;
    void write(std::ostream &os) const override;
};

} // namespace hopct
