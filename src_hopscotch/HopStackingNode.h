#pragma once
// A rai::ComputeNode implementation of hopscotch's `stacking` domain
// (see hop-bench-py's StackingScenario) -- same completion-tree structure as
// HopPickPlaceNode (see that file's docstring for the design rationale,
// including the resumable-motion-planning state machine both share), but
// the plan is always Pick/Place pairs in `goal_order` (bottom-to-top), and a
// Place's target pose is either a free table pose (bottom block) or directly
// on top of the block placed immediately before it in the tower (matching
// `common/streams_stacking.py`'s `sample_stack_pose`: `p_below *
// Pose.from_xyz_yaw(0, 0, 2*block_r, random_yaw)`).
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

// Pick/Place pairs for every object in `goal_order` (bottom-to-top).
std::vector<Action> makeStackingPlan(const StackingScenario *scenario);

struct HopStackingNode : rai::ComputeNode {
    const StackingScenario *scenario; // borrowed; owned by the root/driver
    const std::vector<Action> *plan; // borrowed
    RobotTag robot;
    int action_index; // -1 at the root

    CConfig q_arm;
    int64_t held_object = -1;
    CPose grasp_offset = CPose { 0, 0, 0, 0, 0, 0, 1 };
    std::shared_ptr<std::vector<CPose>> poses; // per scenario object index

    bool motionStarted = false;
    MotionPlanState motionState;
    CConfig pendingQEnd { };
    CPose pendingHeldRel { };
    bool pendingHasHeld = false;
    CConfig nextQArm { };
    int64_t nextHeldObject = -1;
    CPose nextGraspOffset { };
    CPose nextPlacedPose { };

    std::vector<CConfig> trajectory;

    HopStackingNode(
        const StackingScenario *scenario, const std::vector<Action> *plan, RobotTag robot);
    HopStackingNode(HopStackingNode *parent, int childIndex);

    virtual void untimedCompute();
    virtual int getNumDecisions() { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
    virtual double branchingPenalty_child(int i);
    virtual std::shared_ptr<ComputeNode> createNewChild(int i);
    virtual void write(std::ostream &os) const;
};

} // namespace hopct
