#pragma once
// Hopscotch's mobile domain, a PR2 that moves between tables, for the completion tree in HopChain.h.
#include "HopChain.h"
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"
#include <memory>
#include <vector>

namespace hopct {

// Each keyframe draws a fresh base pose along with its arm configuration.
// The robot jumps between keyframes, as hopscotch's mobile robot does, so no motion is planned.
struct MobileDomain {
    struct Context {
        const MobileScenario *scenario;
    };
    using Action = hopct::Action;
    struct State : ArmState {
        CPose base_pose;
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
