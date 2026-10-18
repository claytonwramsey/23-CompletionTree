#pragma once
// Hopscotch's stacking domain for the completion tree in HopChain.h.
#include "HopChain.h"
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"
#include <memory>
#include <vector>

namespace hopct {

struct StackingDomain {
    struct Context {
        const StackingScenario *scenario;
        RobotTag robot;
    };
    using Action = hopct::Action;
    using State = ArmState;
    using Keyframe = ArmKeyframe;
    using MotionState = MotionPlanState;

    static State initialState(const Context &ctx);
    static bool drawKeyframe(const Context &ctx, const State &before, const Action &act,
        const Action *next, size_t k, Keyframe &out);
    static int motionStep(const Context &ctx, const State &before, const Action &act,
        const Keyframe &kf, size_t k, MotionState &motion, float *buf, size_t cap, size_t *len);
};

} // namespace hopct
