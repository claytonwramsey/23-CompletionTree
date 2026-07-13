#include "HopCommon.h"

namespace hopct {

HopGlobalInfo& hopInfo() {
  static HopGlobalInfo singleton;
  return singleton;
}

HopDiagStats& diagStats() {
  static HopDiagStats singleton;
  return singleton;
}

int MotionPlanState::step(const RobotVtable& rvArg, CConfig q_start, CConfig q_end, const Environment* env,
                           float block_r, bool has_held, CPose held_rel_pose, float* out_buf, size_t out_cap,
                           size_t* out_len) {
  rv = &rvArg;
  if (!handle) {
    handle = rv->motion_plan_start(q_start, q_end, env, block_r, has_held, held_rel_pose);
    if (!handle) return -1;
  }
  int samplesPerResume = hopInfo().motionSamplesPerResume;
  int r = rv->motion_plan_resume(handle, (size_t)samplesPerResume, out_buf, out_cap, out_len);
  if (r == 1) {
    free();
    return 1;
  }
  if (r == -1) {
    free();
    return -1;
  }
  cumulativeSamples += samplesPerResume;
  if (cumulativeSamples >= hopInfo().motionMaxTotalSamples) {
    free();
    return -1;
  }
  return 0;
}

void MotionPlanState::free() {
  if (handle) {
    rv->motion_plan_free(handle);
    handle = nullptr;
  }
}

MotionPlanState::~MotionPlanState() { free(); }

}  // namespace hopct
