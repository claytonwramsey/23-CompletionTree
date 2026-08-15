#pragma once
// A rai::ComputeNode implementation of hopscotch's `coffee` domain.
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"
#include <Search/ComputeNode.h>
#include <memory>
#include <vector>

namespace hopct {

enum class CoffeeActionType { Pick, Place, Fill, Pour, Scoop, Dump, Stir };

struct CoffeeAction {
    CoffeeActionType type;
    size_t item_index; // held/moved item (into scenario's object list)
    size_t target_index; // reference item for Pour/Scoop/Dump/Stir's "over ___" pose (unused
                         // otherwise)
};

// Per-cup logical state, mirroring hop_problem::coffee::CupState -- either
// Unmixed with some subset of {coffee, sugar, cream}, or Mixed.
struct CupLogic {
    bool coffee = false, sugar = false, cream = false, mixed = false;
};

struct HopCoffeeNode : rai::ComputeNode {
    const CoffeeScenario &scenario; // borrowed; owned by the root/driver
    const std::vector<CoffeeAction> &plan; // borrowed
    int action_index; // -1 at the root

    CConfig q_arm;
    int64_t held_object = -1;
    CPose grasp_offset = CPose { 0, 0, 0, 0, 0, 0, 1 };
    std::shared_ptr<std::vector<CPose>>
        poses; // per scenario object index (only Pick/Place move these)
    std::shared_ptr<std::vector<CupLogic>>
        cupLogic; // per scenario object index (only meaningful for cups)
    bool spoonScooped = false;

    bool motionStarted = false;
    MotionPlanState motionState;
    CConfig pendingQEnd {};
    CPose pendingHeldRel {};
    bool pendingHasHeld = false;
    CConfig nextQArm {};
    int64_t nextHeldObject = -1;
    CPose nextGraspOffset {};
    CPose nextPlacedPose {}; // only meaningful for Place
    CupLogic nextCupLogic {}; // only meaningful for Fill/Pour/Dump/Stir (on the relevant cup)
    bool nextSpoonScooped = false;

    std::vector<CConfig> trajectory;

    HopCoffeeNode(const CoffeeScenario &scenario, const std::vector<CoffeeAction> &plan,
        RobotTag robot, rai::ComputeNode *parent = nullptr);
    HopCoffeeNode(HopCoffeeNode &parent, int childIndex);

    RobotTag robot;

    virtual void untimedCompute();
    virtual int getNumDecisions() { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
    virtual double branchingPenalty_child(int i);
    virtual std::shared_ptr<ComputeNode> createNewChild(int i);
    virtual void write(std::ostream &os) const;
};

} // namespace hopct
