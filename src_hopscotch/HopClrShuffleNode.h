#pragma once
// A rai::ComputeNode implementation of hopscotch's `clr_shuffle` domain.
#include "HopCommon.h"
#include "hop_bench_cxx.h"
#include <Search/ComputeNode.h>
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

struct HopClrShuffleNode : rai::ComputeNode {
    const ClrShuffleScenario &scenario; // borrowed; owned by the root/driver
    const std::vector<ClrAction> &plan; // borrowed
    int action_index; // -1 at the root

    CConfig q_arm;
    int64_t held_ctb = -1;
    CPose grasp_offset = CPose { 0, 0, 0, 0, 0, 0, 1 };
    std::shared_ptr<std::vector<CPose>> poses; // per CTB index
    bool lid_open = false;

    bool motionStarted = false;
    ClrMotionPlanState motionState;
    CConfig pendingQEnd {};
    CPose pendingHeldRel {};
    bool pendingHasHeld = false;
    CConfig nextQArm {};
    CPose nextGraspOffset {};
    CPose nextPlacedPose {};

    std::vector<CConfig> trajectory;

    HopClrShuffleNode(const ClrShuffleScenario &scenario, const std::vector<ClrAction> &plan,
        rai::ComputeNode *parent = nullptr);
    HopClrShuffleNode(HopClrShuffleNode &parent, int childIndex);

    void untimedCompute() override;
    int getNumDecisions() override { return isComplete && isFeasible && !isTerminal ? -1 : 0; }
    double branchingPenalty_child(int i) override;
    std::shared_ptr<ComputeNode> createNewChild(int i) override;
    void write(std::ostream &os) const override;
};

} // namespace hopct
