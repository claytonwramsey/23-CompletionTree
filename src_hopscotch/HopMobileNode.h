#pragma once
// A rai::ComputeNode implementation of hopscotch's `mobile` domain (PR2,
// see hop-bench-py's MobileScenario) -- same completion-tree structure as
// HopPickPlaceNode/HopStackingNode (including the resumable-motion-planning
// state machine, see HopCommon.h), but every Pick/Place attempt also draws
// a fresh reachable base pose for its target and does IK/validate/motion
// planning in *that base's local frame* (`hopcxx_env_transformed`).
//
// hop-bench never plans a real geometric base path -- MobilePr2::ik()
// itself panics, and hopscotch's own native mobile solver only ever does
// arm-only RRT (fixed base) or an instantaneous "teleport" to a new base
// pose (see project memory `project_pddlstream_port_architecture`, point
// 2) -- so base repositioning here is likewise a zero-cost teleport folded
// into the Pick/Place attempt itself, not a separate move-base
// ComputeNode/action (unlike the PDDLStream port's PDDL domain, which
// *does* model `move-base` as its own cost-1 action -- that's a modeling
// choice specific to PDDL's action-cost bookkeeping, not a difference in
// what's physically being planned).
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"

namespace hopct {

// Pick/Place pairs for every object in scenario order, all targeting
// `goal_surface` (mirrors makePickPlacePlan's `packing` case).
std::vector<Action> makeMobilePlan(const MobileScenario* scenario);

struct HopMobileNode : rai::ComputeNode {
  const MobileScenario* scenario;  // borrowed; owned by the root/driver
  const std::vector<Action>* plan;  // borrowed
  int action_index;  // -1 at the root

  CConfig q_arm;
  CPose base_pose;
  int64_t held_object = -1;
  CPose grasp_offset = CPose{0, 0, 0, 0, 0, 0, 1};
  std::shared_ptr<std::vector<CPose>> poses;  // per scenario object index, world frame

  bool motionStarted = false;
  MotionPlanState motionState;
  CConfig pendingQEnd{};
  CPose pendingHeldRel{};
  bool pendingHasHeld = false;
  CPose pendingBasePose{};  // the (possibly new) base pose this attempt plans in
  CConfig nextQArm{};
  int64_t nextHeldObject = -1;
  CPose nextGraspOffset{};
  CPose nextPlacedPose{};  // world frame

  std::vector<CConfig> trajectory;

  HopMobileNode(const MobileScenario* scenario, const std::vector<Action>* plan);
  HopMobileNode(HopMobileNode* parent, int childIndex);

  virtual void untimedCompute();
  virtual int getNumDecisions() { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
  virtual double branchingPenalty_child(int i);
  virtual std::shared_ptr<ComputeNode> createNewChild(int i);
  virtual void write(std::ostream& os) const;
};

}  // namespace hopct
