#pragma once
// Hopscotch's coffee domain for the completion tree in HopChain.h.
#include "HopChain.h"
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"
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

struct CoffeeDomain {
    struct Context {
        const CoffeeScenario *scenario;
        RobotTag robot;
    };
    using Action = CoffeeAction;
    struct State : ArmState {
        std::shared_ptr<const std::vector<CupLogic>> cupLogic; // per scenario object index
        bool spoonScooped = false;
    };
    struct Keyframe {
        State after;
        bool hasHeld = false;
        CPose heldRel = CPose { 0, 0, 0, 0, 0, 0, 1 };
    };
    using MotionState = MotionPlanState;

    static State initialState(const Context &ctx);
    static bool drawKeyframe(const Context &ctx, const State &before, const Action &act,
        const Action *next, size_t k, Keyframe &out);
    static int motionStep(const Context &ctx, const State &before, const Action &act,
        const Keyframe &kf, size_t k, MotionState &motion, float *buf, size_t cap, size_t *len);
};

} // namespace hopct
