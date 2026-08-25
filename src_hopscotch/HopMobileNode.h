#pragma once
// A rai::ComputeNode implementation of hopscotch's `mobile` domain (PR2,
// see hop-bench-py's MobileScenario) -- same completion-tree structure as
// HopPickPlaceNode/HopStackingNode (including the resumable-motion-planning
// state machine, see HopCommon.h).
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

struct HopMobileNode : rai::ComputeNode {
    const MobileScenario &scenario; // borrowed; owned by the root/driver
    const std::vector<Action> &plan; // borrowed
    int action_index; // -1 at the root

    CConfig q_arm;
    CPose base_pose;
    int64_t held_object = -1;
    CPose grasp_offset = CPose { 0, 0, 0, 0, 0, 0, 1 };
    std::shared_ptr<std::vector<CPose>> poses; // per scenario object index, world frame

    bool motionStarted = false;
    MotionPlanState motionState;
    CConfig pendingQEnd {};
    CPose pendingHeldRel {};
    bool pendingHasHeld = false;
    CPose pendingBasePose {}; // the freshly-sampled base pose this attempt plans in
    CConfig nextQArm {};
    int64_t nextHeldObject = -1;
    CPose nextGraspOffset {};
    CPose nextPlacedPose {}; // world frame

    std::vector<CConfig> trajectory;

    HopMobileNode(const MobileScenario &scenario, const std::vector<Action> &plan,
        rai::ComputeNode *parent = nullptr);
    HopMobileNode(HopMobileNode &parent, int childIndex);

    void untimedCompute() override;
    int getNumDecisions() override { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
    double branchingPenalty_child(int i) override;
    std::shared_ptr<ComputeNode> createNewChild(int i) override;
    void write(std::ostream &os) const override;
};

} // namespace hopct
