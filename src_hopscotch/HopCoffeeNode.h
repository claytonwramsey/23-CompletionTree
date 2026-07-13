#pragma once
// A rai::ComputeNode implementation of hopscotch's `coffee` domain (see
// hop-bench-py's CoffeeScenario / common/coffee_domain.pddl) -- same
// resumable-motion-planning completion-tree structure as the other domains
// (see HopPickPlaceNode.h), but the plan is a *fixed* hand-coded action
// recipe (not a loop over N objects) matching the single fixed goal
// ("cup0 held and mixed", see hop-bench's coffee.rs): cup0 starts EMPTY,
// cream_cup starts CREAMY, both sugar cups start SWEET, so the only path to
// Mixed is fill(cup0) + pour(cream_cup->cup0) + scoop+dump(spoon,sugar
// cup->cup0) + stir(cup0), each interleaved with the pick/place needed to
// free up the hand for the next held item:
//   pick(cup0), fill(cup0), place(cup0),
//   pick(cream_cup), pour(cream_cup, cup0), place(cream_cup),
//   pick(spoon), scoop(spoon, sugar_cup), dump(spoon, cup0), stir(spoon, cup0), place(spoon),
//   pick(cup0)
// Fill/Pour/Scoop/Dump/Stir don't change any object's *position* (unlike
// Place) -- they just move the held item to a computed pose (spigot pose,
// "over" a target, or a flipped "dump" pose over a target -- see
// `common/streams_coffee.py`) and flip a logical cup/spoon flag on success.
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

// The fixed 12-action recipe described above, resolved to this scenario's
// actual object indices (cup0/cream_cup/sugar_cup/spoon0, looked up by
// (kind, index) since array position isn't guaranteed -- see
// CoffeeScenario's docstring in hop-bench-cxx/src/lib.rs).
std::vector<CoffeeAction> makeCoffeePlan(const CoffeeScenario *scenario);

// Per-cup logical state, mirroring hop_problem::coffee::CupState -- either
// Unmixed with some subset of {coffee, sugar, cream}, or Mixed.
struct CupLogic {
    bool coffee = false, sugar = false, cream = false, mixed = false;
};

struct HopCoffeeNode : rai::ComputeNode {
    const CoffeeScenario *scenario; // borrowed; owned by the root/driver
    const std::vector<CoffeeAction> *plan; // borrowed
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
    CConfig pendingQEnd { };
    CPose pendingHeldRel { };
    bool pendingHasHeld = false;
    CConfig nextQArm { };
    int64_t nextHeldObject = -1;
    CPose nextGraspOffset { };
    CPose nextPlacedPose { }; // only meaningful for Place
    CupLogic nextCupLogic { }; // only meaningful for Fill/Pour/Dump/Stir (on the relevant cup)
    bool nextSpoonScooped = false;

    std::vector<CConfig> trajectory;

    HopCoffeeNode(
        const CoffeeScenario *scenario, const std::vector<CoffeeAction> *plan, RobotTag robot);
    HopCoffeeNode(HopCoffeeNode *parent, int childIndex);

    RobotTag robot;

    virtual void untimedCompute();
    virtual int getNumDecisions() { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
    virtual double branchingPenalty_child(int i);
    virtual std::shared_ptr<ComputeNode> createNewChild(int i);
    virtual void write(std::ostream &os) const;
};

} // namespace hopct
