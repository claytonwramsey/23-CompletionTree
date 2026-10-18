#include "HopMobileDomain.h"
#include "hop_pose.h"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <numbers>
#include <random>

namespace hopct {

// Builds the env for one action attempt: `scenario`'s static world
// re-expressed in `bq`'s local frame, plus a rotated cuboid per other
// object.
static EnvPtr buildAttemptEnv(
    const MobileScenario &scenario, const std::vector<CPose> &poses, size_t movingIndex, CPose bq) {
    EnvPtr env = wrapEnv(hopcxx_env_transformed(hopcxx_mobile_env(&scenario), bq));
    float block_r = hopcxx_mobile_block_r(&scenario);
    CPose bqInv = pose_inverse(bq);
    for (size_t i = 0; i < poses.size(); i++) {
        if (i == movingIndex) {
            continue;
        }
        CPose rel = pose_mul(bqInv, poses[i]);
        hopcxx_env_add_cuboid(env.get(), rel, block_r);
    }
    return env;
}

MobileDomain::State MobileDomain::initialState(const Context &ctx) {
    State s;
    s.q_arm = hopcxx_mobile_robot_q_start_arm(ctx.scenario);
    s.base_pose = hopcxx_mobile_base_start_pose(ctx.scenario);
    auto poses = std::make_shared<std::vector<CPose>>();
    size_t n = hopcxx_mobile_num_objects(ctx.scenario);
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_mobile_object_pose(ctx.scenario, i));
    }
    s.poses = poses;
    return s;
}

bool MobileDomain::drawKeyframe(const Context &ctx, const State &before, const Action &act,
    const Action *, size_t, Keyframe &out) {
    const RobotVtable &rv = robot_vtable(RobotTag::Pr2);
    std::array<float, 4> baseBounds;
    hopcxx_mobile_base_bounds(ctx.scenario, baseBounds.data());
    out.after = before;
    CPose bq;
    if (act.type == ActionType::Pick) {
        CPose objPose = (*before.poses)[act.object_index];
        bool ok = sample_reachable_base(objPose, baseBounds.data(), &bq);
        CPose g = rv.sample_rel_pose();
        CPose eeTarget = pose_mul(pose_mul(pose_inverse(bq), objPose), g);
        if (!ok || !rv.ik(eeTarget, &out.after.q_arm)) {
            return false;
        }
        EnvPtr env = buildAttemptEnv(*ctx.scenario, *before.poses, act.object_index, bq);
        if (!rv.validate(out.after.q_arm, env.get())) {
            return false;
        }
        out.after.held_object = static_cast<int64_t>(act.object_index);
        out.after.grasp_offset = g;
        out.after.base_pose = bq;
        out.hasHeld = false;
        out.heldRel = pose_identity();
        return true;
    }
    CHECK_EQ(before.held_object, static_cast<int64_t>(act.object_index),
        "place must follow pick of the same object");
    CTable surface = hopcxx_mobile_surface(ctx.scenario, act.surface_index);
    CPose target = rv.sample_table_pose(surface); // world frame
    float block_r = hopcxx_mobile_block_r(ctx.scenario);
    bool ok = sample_reachable_base(target, baseBounds.data(), &bq);
    CPose eeTarget = pose_mul(pose_mul(pose_inverse(bq), target), before.grasp_offset);
    CPose heldRel = pose_inverse(before.grasp_offset);
    if (!ok || !rv.ik(eeTarget, &out.after.q_arm)) {
        return false;
    }
    EnvPtr env = buildAttemptEnv(*ctx.scenario, *before.poses, act.object_index, bq);
    if (!rv.validate_attached(out.after.q_arm, env.get(), block_r, heldRel)) {
        return false;
    }
    auto poses = std::make_shared<std::vector<CPose>>(*before.poses);
    (*poses)[act.object_index] = target;
    out.after.poses = poses;
    out.after.held_object = -1;
    out.after.grasp_offset = pose_identity();
    out.after.base_pose = bq;
    out.hasHeld = true;
    out.heldRel = heldRel;
    return true;
}

int MobileDomain::motionStep(const Context &, const State &before, const Action &,
    const Keyframe &kf, size_t, MotionState &, float *buf, size_t cap, size_t *len) {
    // Hopscotch's mobile robot moves between keyframes whose base poses differ by a jump of its
    // whole configuration, without planning a motion (`MobilePr2`'s teleport in hop-mobile), and
    // every keyframe here samples a new base pose, so every motion is such a jump.
    CHECK_GE(cap, 2u, "a jump needs two waypoints");
    int dim = kf.after.q_arm.dim;
    std::copy_n(before.q_arm.q, dim, buf);
    std::copy_n(kf.after.q_arm.q, dim, buf + dim);
    *len = 2;
    return 1;
}

} // namespace hopct
