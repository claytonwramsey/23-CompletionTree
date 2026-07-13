#pragma once
// A rai::ComputeNode implementation of hopscotch's `pickplace` domain family
// (cabinet, packing -- see hop-bench-py's PickPlaceScenario) for the
// completion-tree planner. Each node represents ONE sampled attempt at the
// next action in a fixed-order pick/place plan; hopscotch's own "resample
// until you get a hit" coroutine philosophy maps directly onto
// rai::ComputeNode's infinite-branching (`getNumDecisions() == -1`) +
// widening machinery, so no new search algorithm is needed -- this reuses
// the exact same rai::AStar the paper's own LGPcomp_Skeleton/Waypoints nodes
// run on (see LGP_computers.h for the analogous real pattern this mirrors).
//
// Motion planning within one action attempt is itself resumable (see
// HopCommon.h's MotionPlanState): a node samples its grasp/target and does
// IK exactly once (a failure there is a dead attempt, same as any other),
// but once past that it may need several ComputeNode::compute() calls
// (i.e. several rai::AStar steps, `isComplete` left false in between) to
// finish planning the actual motion -- each such call is its own honestly
// accounted search-tree step, not hidden inside one opaque hop-bench call.
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

// Builds the fixed-order pick/place plan for a PickPlaceScenario: a single
// Pick(target) for `cabinet` (target_block set), or Pick/Place pairs for
// every object in scenario order for `packing` (goal_surface set).
std::vector<Action> makePickPlacePlan(const PickPlaceScenario &scenario);

struct HopPickPlaceNode : rai::ComputeNode {
    const PickPlaceScenario &scenario; // borrowed; owned by the root/driver
    const std::vector<Action> &plan; // borrowed
    RobotTag robot;
    int action_index; // -1 at the root (no action attempted yet)

    // State inherited from the parent (this node's *starting* state); becomes
    // this node's own resulting state only once its action attempt succeeds.
    CConfig q_arm;
    int64_t held_object = -1; // index into scenario's object list, or -1
    CPose grasp_offset = CPose { 0, 0, 0, 0, 0, 0, 1 };
    std::shared_ptr<std::vector<CPose>> poses; // per scenario object index

    // Motion-planning-in-progress state (only meaningful once past the
    // grasp/IK draw -- see untimedCompute()).
    bool motionStarted = false;
    MotionPlanState motionState;
    CConfig pendingQEnd { };
    CPose pendingHeldRel { };
    bool pendingHasHeld = false;
    // What this node's fields should become once motion planning succeeds:
    CConfig nextQArm { };
    int64_t nextHeldObject = -1;
    CPose nextGraspOffset { };
    CPose nextPlacedPose { }; // only meaningful for Place actions

    // The trajectory this node's action took (empty at the root), kept around
    // purely so a solved plan can be dumped for visualization -- see
    // dump_solution.cpp. `dim` waypoint size matches the robot's DOF.
    std::vector<CConfig> trajectory;

    // Root constructor.
    HopPickPlaceNode(
        const PickPlaceScenario &scenario, const std::vector<Action> &plan, RobotTag robot);
    // Child constructor: attempts `plan.at(parent.action_index + 1)` once.
    HopPickPlaceNode(HopPickPlaceNode &parent, int childIndex);

    virtual void untimedCompute();
    virtual int getNumDecisions() { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
    virtual double branchingPenalty_child(int i);
    virtual std::shared_ptr<ComputeNode> createNewChild(int i);
    virtual void write(std::ostream &os) const;
};

} // namespace hopct
