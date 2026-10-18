#include "HopClrShuffleDomain.h"
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
        for (size_t i = 0; i < hopcxx_clr_shuffle_num_lid_boxes(&scenario); i++) {
            float lidHalf[3];
            hopcxx_clr_shuffle_lid_box_half_extents(&scenario, i, lidHalf);
            hopcxx_env_add_box(
                env.get(), hopcxx_clr_shuffle_lid_box_pose(&scenario, i, lidOpen), lidHalf);
        }
        return env;
    }

    size_t sampleGraspChoice() {
        static thread_local std::mt19937 rng { std::random_device {}() };
        return std::uniform_int_distribution<size_t>(0, 1)(rng);
    }

} // namespace

ClrDomain::State ClrDomain::initialState(const Context &ctx) {
    State s;
    s.q_arm = hopcxx_clr_shuffle_robot_q_start(ctx.scenario);
    size_t n = hopcxx_clr_shuffle_num_ctbs(ctx.scenario);
    auto poses = std::make_shared<std::vector<CPose>>();
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_clr_shuffle_ctb_pose(ctx.scenario, i));
    }
    s.poses = poses;
    return s;
}

bool ClrDomain::drawKeyframe(const Context &ctx, const State &before, const Action &act,
    const Action *next, size_t k, Keyframe &out) {
    if (k < HopDiagStats::MAX_ACTIONS) {
        diagStats().attempts[k]++;
    }
    float half[3];
    hopcxx_clr_shuffle_ctb_half_extents(ctx.scenario, half);
    EnvPtr env = buildAttemptEnv(*ctx.scenario, *before.poses, before.held_object, before.lid_open);
    out.after = before;
    CPose heldRel = pose_identity();
    bool hasHeld = act.type == ClrActionType::Place;
    CPose eeTarget;
    CPose placed {};
    if (act.type == ClrActionType::OpenLid || act.type == ClrActionType::CloseLid) {
        eeTarget = hopcxx_clr_shuffle_lid_grasp_pose(ctx.scenario, before.lid_open);
    } else if (act.type == ClrActionType::Pick) {
        CPose ctbPose = (*before.poses)[act.ctb_index];
        eeTarget = hopcxx_clr_shuffle_handle_grasp(ctbPose, half, sampleGraspChoice());
        out.after.grasp_offset = pose_mul(pose_inverse(ctbPose), eeTarget);
    } else {
        placed = hopcxx_clr_shuffle_sample_place_pose(ctx.scenario, act.surface_index);
        eeTarget = pose_mul(placed, before.grasp_offset);
        heldRel = pose_inverse(before.grasp_offset);
    }
    // This arm's IK is deterministic per pose, so every branch gets a try.
    CConfig solns[HOPCXX_CLR_UR10E_MAX_IK_SOLNS];
    size_t n = hopcxx_clr_ur10e_ik_all(eeTarget, solns, HOPCXX_CLR_UR10E_MAX_IK_SOLNS);
    if (n > 0 && k < HopDiagStats::MAX_ACTIONS) {
        diagStats().ikOk[k]++;
    }
    // The keyframe also starts the next action's motion, in the world this action leaves
    // behind, so it has to be valid there too.
    std::vector<CPose> after = *before.poses;
    int64_t heldAfter = before.held_object;
    bool lidAfter = before.lid_open;
    CPose heldRelAfter = heldRel;
    switch (act.type) {
    case ClrActionType::Pick:
        heldAfter = static_cast<int64_t>(act.ctb_index);
        heldRelAfter = pose_inverse(out.after.grasp_offset);
        break;
    case ClrActionType::Place:
        heldAfter = -1;
        after[act.ctb_index] = placed;
        break;
    case ClrActionType::OpenLid:
        lidAfter = true;
        break;
    case ClrActionType::CloseLid:
        lidAfter = false;
        break;
    }
    EnvPtr envAfter = next ? buildAttemptEnv(*ctx.scenario, after, heldAfter, lidAfter)
                           : EnvPtr(nullptr, hopcxx_env_free);
    bool ok = false;
    for (size_t i = 0; i < n && !ok; i++) {
        ok = hasHeld ? hopcxx_clr_ur10e_validate_attached_ctb(solns[i], env.get(), half, heldRel)
                     : hopcxx_clr_ur10e_validate(solns[i], env.get());
        if (ok && next) {
            ok = heldAfter >= 0 ? hopcxx_clr_ur10e_validate_attached_ctb(
                                      solns[i], envAfter.get(), half, heldRelAfter)
                                : hopcxx_clr_ur10e_validate(solns[i], envAfter.get());
        }
        if (ok) {
            out.after.q_arm = solns[i];
        }
    }
    if (!ok) {
        return false;
    }
    if (k < HopDiagStats::MAX_ACTIONS) {
        diagStats().validateOk[k]++;
    }
    out.hasHeld = hasHeld;
    out.heldRel = heldRel;
    switch (act.type) {
    case ClrActionType::Pick:
        out.after.held_object = static_cast<int64_t>(act.ctb_index);
        break;
    case ClrActionType::Place: {
        out.after.held_object = -1;
        out.after.grasp_offset = pose_identity();
        auto poses = std::make_shared<std::vector<CPose>>(*before.poses);
        (*poses)[act.ctb_index] = placed;
        out.after.poses = poses;
        break;
    }
    case ClrActionType::OpenLid:
        out.after.lid_open = true;
        break;
    case ClrActionType::CloseLid:
        out.after.lid_open = false;
        break;
    }
    return true;
}

int ClrDomain::motionStep(const Context &ctx, const State &before, const Action &,
    const Keyframe &kf, size_t k, MotionState &motion, float *buf, size_t cap, size_t *len) {
    float half[3];
    hopcxx_clr_shuffle_ctb_half_extents(ctx.scenario, half);
    EnvPtr env = buildAttemptEnv(*ctx.scenario, *before.poses, before.held_object, before.lid_open);
    int r = motion.step(
        before.q_arm, kf.after.q_arm, env.get(), half, kf.hasHeld, kf.heldRel, buf, cap, len);
    if (r > 0 && k < HopDiagStats::MAX_ACTIONS) {
        diagStats().motionOk[k]++;
    }
    return r;
}

} // namespace hopct
