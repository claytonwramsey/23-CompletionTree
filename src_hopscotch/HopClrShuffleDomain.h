#pragma once
// Hopscotch's CLR shuffle domain, a UR10e moving CTBs in a lidded rig, for the completion tree in HopChain.h.
#include "HopChain.h"
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include <memory>
#include <vector>

namespace hopct {

enum class ClrActionType { Pick, Place, OpenLid, CloseLid };

struct ClrAction {
    ClrActionType type;
    size_t ctb_index; // index into the scenario's CTB list; unused by the lid actions
    size_t surface_index; // which surface to pick from or place onto; unused by the lid actions
};

// A resumable motion-planning search for the CLR rig's arm, carrying a CTB's three half-extents
// where MotionPlanState carries a block radius.
struct ClrMotionPlanState {
    ClrMotionPlanState() = default;
    ClrMotionPlanState(const ClrMotionPlanState &) = delete;
    ClrMotionPlanState &operator=(const ClrMotionPlanState &) = delete;
    ~ClrMotionPlanState() { reset(); }

    // Returns 1 once a trajectory lands in out_buf/out_len, 0 while the search is still running,
    // and -1 when it gives up, which covers a failed construction, a path longer than out_cap,
    // and an exhausted sample budget. The handle is freed on both 1 and -1.
    int step(CConfig q_start, CConfig q_end, const Environment *env, const float *half_extents,
        bool has_held, CPose held_rel_pose, float *out_buf, size_t out_cap, size_t *out_len);
    void reset();

private:
    hopcxx_clr_ur10e_Planning *handle = nullptr;
    int cumulativeSamples = 0;
};

// `held_object` in its State indexes the scenario's CTB list.
struct ClrDomain {
    struct Context {
        const ClrShuffleScenario *scenario;
    };
    using Action = ClrAction;
    struct State : ArmState {
        bool lid_open = false;
    };
    struct Keyframe {
        State after;
        bool hasHeld = false;
        CPose heldRel = CPose { 0, 0, 0, 0, 0, 0, 1 };
    };
    using MotionState = ClrMotionPlanState;

    static State initialState(const Context &ctx);
    static bool drawKeyframe(const Context &ctx, const State &before, const Action &act,
        const Action *next, size_t k, Keyframe &out);
    static int motionStep(const Context &ctx, const State &before, const Action &act,
        const Keyframe &kf, size_t k, MotionState &motion, float *buf, size_t cap, size_t *len);
};

} // namespace hopct
