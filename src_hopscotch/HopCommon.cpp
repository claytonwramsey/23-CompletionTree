#include "HopCommon.h"
#include <cmath>

namespace hopct {

HopGlobalInfo &hopInfo() {
    static HopGlobalInfo singleton;
    return singleton;
}

double hopBranchingPenalty(int i) { return ::pow(double(i) / hopInfo().w0, hopInfo().wP); }

HopDiagStats &diagStats() {
    static HopDiagStats singleton;
    return singleton;
}

int MotionPlanState::step(const RobotVtable &rvArg, CConfig q_start, CConfig q_end,
    const Environment *env, float block_r, bool has_held, CPose held_rel_pose, float *out_buf,
    size_t out_cap, size_t *out_len, int coffee_item_kind) {
    rv = &rvArg;
    if (!handle) {
        handle = has_held && coffee_item_kind >= 0
            ? rv->motion_plan_start_coffee_item(q_start, q_end, env,
                  static_cast<uint8_t>(coffee_item_kind), block_r, held_rel_pose)
            : rv->motion_plan_start(q_start, q_end, env, block_r, has_held, held_rel_pose);
        if (!handle) {
            return -1;
        }
    }
    int samplesPerResume = hopInfo().motionSamplesPerResume;
    int r = rv->motion_plan_resume(
        handle, static_cast<size_t>(samplesPerResume), out_buf, out_cap, out_len);
    if (r == 1) {
        reset();
        return 1;
    }
    if (r == -1) {
        reset();
        return -1;
    }
    cumulativeSamples += samplesPerResume;
    if (cumulativeSamples >= hopInfo().motionMaxTotalSamples) {
        reset();
        return -1;
    }
    return 0;
}

void MotionPlanState::reset() {
    if (handle) {
        rv->motion_plan_free(handle);
        handle = nullptr;
    }
}

} // namespace hopct
