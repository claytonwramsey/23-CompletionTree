#pragma once
// Shared types across all hopscotch-ported completion-tree domains
// (HopPickPlaceNode, HopStackingNode, ...).
#include <Core/util.h>
#include "hop_bench_cxx.h"
#include "hop_robot_vtable.h"

namespace hopct {

struct HopGlobalInfo {
  RAI_PARAM("Hop/", double, w0, 4.)
  RAI_PARAM("Hop/", double, wP, 2.)
  RAI_PARAM("Hop/", int, maxTrajWaypoints, 512)
  // Motion planning is a genuinely incremental search (see MotionPlanState),
  // not a single-shot call -- these two control how it's chunked into
  // completion-tree-accounted steps. `samplesPerResume` is how much budget
  // one ComputeNode::compute() call spends; `maxTotalSamples` is the
  // give-up threshold for one action attempt's total planning effort
  // (matches the sample budget hop-bench-py/PDDLStream use for a single
  // motion-planning call, so per-attempt effort is comparable).
  RAI_PARAM("Hop/", int, motionSamplesPerResume, 100)
  RAI_PARAM("Hop/", int, motionMaxTotalSamples, 3000)
};
HopGlobalInfo& hopInfo();

enum class ActionType { Pick, Place };
struct Action {
  ActionType type;
  size_t object_index;  // index into the scenario's object list
};

// Lightweight, always-on per-action-index diagnostic counters -- cheap
// (a few integer increments per attempt) and invaluable for answering "is a
// low solve rate a bug or a real sampling-difficulty wall, and if so at
// which step": `attempts[i]` = how many times action `i` in a plan was
// attempted, `ikOk`/`validateOk` = how many got past IK / collision
// validation, `motionOk` = how many fully completed (found a path). A domain
// where `ikOk` is already near-zero relative to `attempts` at some index
// means resampling more won't help there without changing the geometry.
struct HopDiagStats {
  static const int MAX_ACTIONS = 32;
  long long attempts[MAX_ACTIONS] = {0};
  long long ikOk[MAX_ACTIONS] = {0};
  long long validateOk[MAX_ACTIONS] = {0};
  long long motionOk[MAX_ACTIONS] = {0};
};
HopDiagStats& diagStats();

// A resumable motion-planning search, chunked across possibly many
// `step()` calls (one per ComputeNode::compute() invocation) instead of one
// call that internally exhausts a whole sample budget -- see
// hop-bench-cxx/src/lib.rs's `Planning` docstring for the fairness
// rationale (every unit of real planning effort should be its own
// accounted-for search-tree node, the same principle as
// `hopcxx_*_ik` being a single draw rather than an internal retry loop).
struct MotionPlanState {
  void* handle = nullptr;
  const RobotVtable* rv = nullptr;  // set on first `step()`, needed by `free`/~dtor
  int cumulativeSamples = 0;

  // Returns 1 (solved -- trajectory written to out_buf/out_len, handle
  // freed), -1 (gave up -- either construction failed, the path exceeded
  // out_cap, or the total sample budget was exhausted; handle freed), or 0
  // (still searching -- caller should leave its ComputeNode `isComplete =
  // false` so rai::AStar re-queues it and calls compute() again later,
  // which will resume this same search rather than restarting it).
  int step(const RobotVtable& rv, CConfig q_start, CConfig q_end, const Environment* env, float block_r,
           bool has_held, CPose held_rel_pose, float* out_buf, size_t out_cap, size_t* out_len);

  void free();
  ~MotionPlanState();
};

}  // namespace hopct
