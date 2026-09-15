#include "HopClrShuffleNode.h"
#include "hop_pose.h"
#include <algorithm>
#include <random>

namespace hopct {

int ClrMotionPlanState::step(CConfig q_start, CConfig q_end, const Environment *env,
    const float *half_extents, bool has_held, CPose held_rel_pose, float *out_buf, size_t out_cap,
    size_t *out_len) {
    if (!handle) {
        handle = hopcxx_clr_ur10e_motion_plan_start(
            q_start, q_end, env, half_extents, has_held, held_rel_pose);
        if (!handle) {
            return -1;
        }
    }
    int samplesPerResume = hopInfo().motionSamplesPerResume;
    int r = hopcxx_clr_ur10e_motion_plan_resume(
        handle, static_cast<size_t>(samplesPerResume), out_buf, out_cap, out_len);
    if (r != 0) {
        reset();
        return r;
    }
    cumulativeSamples += samplesPerResume;
    if (cumulativeSamples >= hopInfo().motionMaxTotalSamples) {
        reset();
        return -1;
    }
    return 0;
}

void ClrMotionPlanState::reset() {
    if (handle) {
        hopcxx_clr_ur10e_motion_plan_free(handle);
        handle = nullptr;
    }
}

namespace {

    // The rig's own geometry, every CTB the arm isn't holding, and the lid where this state
    // leaves it. A held CTB rides along as an attachment instead of sitting in the world.
    EnvPtr buildAttemptEnv(const ClrShuffleScenario &scenario, const std::vector<CPose> &poses,
        int64_t heldIndex, bool lidOpen) {
        EnvPtr env = wrapEnv(hopcxx_env_clone(hopcxx_clr_shuffle_env(&scenario)));
        float half[3];
        hopcxx_clr_shuffle_ctb_half_extents(&scenario, half);
        for (size_t i = 0; i < poses.size(); i++) {
            if (static_cast<int64_t>(i) == heldIndex) {
                continue;
            }
            hopcxx_env_add_box(env.get(), poses[i], half);
        }
        float lidHalf[3];
        hopcxx_clr_shuffle_lid_half_extents(&scenario, lidHalf);
        hopcxx_env_add_box(env.get(), hopcxx_clr_shuffle_lid_pose(&scenario, lidOpen), lidHalf);
        return env;
    }

    size_t sampleGraspChoice() {
        static thread_local std::mt19937 rng { std::random_device {}() };
        return std::uniform_int_distribution<size_t>(0, 1)(rng);
    }

} // namespace

HopClrShuffleNode::HopClrShuffleNode(
    const ClrShuffleScenario &scenario, const std::vector<ClrAction> &plan, rai::ComputeNode *parent)
    : ComputeNode(parent)
    , scenario(scenario)
    , plan(plan)
    , action_index(-1) {
    q_arm = hopcxx_clr_shuffle_robot_q_start(&scenario);
    size_t n = hopcxx_clr_shuffle_num_ctbs(&scenario);
    poses = std::make_shared<std::vector<CPose>>();
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_clr_shuffle_ctb_pose(&scenario, i));
    }
    isComplete = true;
    isFeasible = true;
    isTerminal = false;
    name = "root";
}

HopClrShuffleNode::HopClrShuffleNode(HopClrShuffleNode &parent, int childIndex)
    : ComputeNode(&parent)
    , scenario(parent.scenario)
    , plan(parent.plan)
    , action_index(parent.action_index + 1)
    , q_arm(parent.q_arm)
    , held_ctb(parent.held_ctb)
    , grasp_offset(parent.grasp_offset)
    , poses(parent.poses)
    , lid_open(parent.lid_open) {
    name << "a" << action_index << "#" << childIndex;
}

void HopClrShuffleNode::write(std::ostream &os) const { os << name; }

double HopClrShuffleNode::branchingPenalty_child(int i) { return hopBranchingPenalty(i); }

std::shared_ptr<rai::ComputeNode> HopClrShuffleNode::createNewChild(int i) {
    return std::make_shared<HopClrShuffleNode>(*this, i);
}

void HopClrShuffleNode::untimedCompute() {
    CHECK_GE(action_index, 0, "root should never be (re-)computed");
    const ClrAction &act = plan.at(action_index);
    float half[3];
    hopcxx_clr_shuffle_ctb_half_extents(&scenario, half);
    bool isLidAction = act.type == ClrActionType::OpenLid || act.type == ClrActionType::CloseLid;

    if (!motionStarted) {
        diagStats().attempts[action_index]++;
        EnvPtr env = buildAttemptEnv(scenario, *poses, held_ctb, lid_open);
        CPose heldRel = pose_identity();
        bool hasHeld = act.type == ClrActionType::Place;

        // Every end-effector target here is a fixed function of a pose, and this arm's IK is
        // deterministic per pose, so drawing a single branch would retry one arm posture forever.
        // Every branch gets a try instead.
        CPose eeTarget;
        if (isLidAction) {
            eeTarget = hopcxx_clr_shuffle_lid_grasp_pose(&scenario, lid_open);
        } else if (act.type == ClrActionType::Pick) {
            eeTarget = hopcxx_clr_shuffle_handle_grasp(
                (*poses)[act.ctb_index], half, sampleGraspChoice());
            nextGraspOffset = pose_mul(pose_inverse((*poses)[act.ctb_index]), eeTarget);
        } else {
            nextPlacedPose = hopcxx_clr_shuffle_sample_place_pose(&scenario, act.surface_index);
            eeTarget = pose_mul(nextPlacedPose, grasp_offset);
            heldRel = pose_inverse(grasp_offset);
        }

        CConfig solns[HOPCXX_CLR_UR10E_MAX_IK_SOLNS];
        size_t n = hopcxx_clr_ur10e_ik_all(eeTarget, solns, HOPCXX_CLR_UR10E_MAX_IK_SOLNS);
        if (n > 0) {
            diagStats().ikOk[action_index]++;
        }
        CConfig qTarget {};
        bool ok = false;
        for (size_t i = 0; i < n; i++) {
            bool valid = hasHeld
                ? hopcxx_clr_ur10e_validate_attached_ctb(solns[i], env.get(), half, heldRel)
                : hopcxx_clr_ur10e_validate(solns[i], env.get());
            if (valid) {
                qTarget = solns[i];
                ok = true;
                break;
            }
        }

        if (ok) {
            diagStats().validateOk[action_index]++;
        } else {
            isFeasible = false;
            isComplete = true;
            return;
        }
        nextQArm = qTarget;
        pendingQEnd = qTarget;
        pendingHasHeld = hasHeld;
        pendingHeldRel = heldRel;
        motionStarted = true;
    }

    EnvPtr env = buildAttemptEnv(scenario, *poses, held_ctb, lid_open);
    int maxWp = hopInfo().maxTrajWaypoints;
    std::vector<float> buf(maxWp * HOPCXX_MAX_DIM);
    size_t len = 0;
    int r = motionState.step(q_arm, pendingQEnd, env.get(), half, pendingHasHeld, pendingHeldRel,
        buf.data(), maxWp, &len);

    if (r == 0) {
        return;
    }
    if (r < 0) {
        isFeasible = false;
        isComplete = true;
        return;
    }
    diagStats().motionOk[action_index]++;

    for (size_t w = 0; w < len; w++) {
        CConfig c {};
        c.dim = nextQArm.dim;
        std::copy_n(buf.begin() + w * c.dim, c.dim, c.q);
        trajectory.push_back(c);
    }
    q_arm = nextQArm;

    switch (act.type) {
    case ClrActionType::Pick:
        held_ctb = static_cast<int64_t>(act.ctb_index);
        grasp_offset = nextGraspOffset;
        break;
    case ClrActionType::Place: {
        held_ctb = -1;
        grasp_offset = pose_identity();
        auto newPoses = std::make_shared<std::vector<CPose>>(*poses);
        (*newPoses)[act.ctb_index] = nextPlacedPose;
        poses = newPoses;
        break;
    }
    case ClrActionType::OpenLid:
        lid_open = true;
        break;
    case ClrActionType::CloseLid:
        lid_open = false;
        break;
    }

    isFeasible = true;
    isComplete = true;
    l = 1.;
    if (action_index + 1 == static_cast<int>(plan.size())) {
        isTerminal = true;
    }
}

} // namespace hopct
