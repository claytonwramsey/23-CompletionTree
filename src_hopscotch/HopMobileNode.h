#pragma once
// A rai::ComputeNode implementation of hopscotch's `mobile` domain (PR2,
// see hop-bench-py's MobileScenario) -- same completion-tree structure as
// HopPickPlaceNode/HopStackingNode (including the resumable-motion-planning
// state machine, see HopCommon.h), except for Move (see below).
//
// Base repositioning is its own first-class action (ActionType::Move,
// HopMobileSkeletonRoot's `move` rule), not folded into every Pick/Place
// attempt as a free teleport -- matching the PDDLStream port's PDDL domain,
// which models `move-base` as its own cost-1 action too (previously this
// file did the opposite, on the grounds that hop-bench never plans a real
// geometric base path -- MobilePr2::ik() itself panics, and hopscotch's own
// native mobile solver only ever does arm-only RRT (fixed base) or an
// instantaneous "teleport" -- see project memory
// `project_pddlstream_port_architecture`, point 2 -- but a free teleport
// undercounts the real search cost of relocating, so Move now does real,
// accounted geometric work: sample a point on the target surface, sample a
// reachable base pose for it, and confirm the arm's current configuration
// (attached object included, if any) is still collision-free from there).
// Move has no trajectory to plan (a base teleport, not a driven path), so
// it's handled entirely separately from the resumable-motion-planning
// machinery below, completing in one shot. Pick/Place, in turn, no longer
// sample their own reachable base -- they operate from whatever base_pose
// the most recent Move committed, and simply fail IK naturally if that
// base position turns out not to reach their particular target.
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
    CConfig pendingQEnd { };
    CPose pendingHeldRel { };
    bool pendingHasHeld = false;
    CPose pendingBasePose { }; // the (possibly new) base pose this attempt plans in
    CConfig nextQArm { };
    int64_t nextHeldObject = -1;
    CPose nextGraspOffset { };
    CPose nextPlacedPose { }; // world frame

    std::vector<CConfig> trajectory;

    HopMobileNode(const MobileScenario &scenario, const std::vector<Action> &plan,
        rai::ComputeNode *parent = nullptr);
    HopMobileNode(HopMobileNode &parent, int childIndex);

    virtual void untimedCompute();
    virtual int getNumDecisions() { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
    virtual double branchingPenalty_child(int i);
    virtual std::shared_ptr<ComputeNode> createNewChild(int i);
    virtual void write(std::ostream &os) const;
};

} // namespace hopct
