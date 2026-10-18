#include "HopPickPlaceDomain.h"
#include "hop_pose.h"
#include <algorithm>
#include <cstdlib>

namespace hopct {

// Build the static-geometry + other-objects env for one action attempt on
// `movingIndex`. Other objects from `movingIndex` are modeled as rotated
// cuboids.
static EnvPtr buildAttemptEnv(
    const PickPlaceScenario &scenario, const std::vector<CPose> &poses, size_t movingIndex) {
    EnvPtr env = wrapEnv(hopcxx_env_clone(hopcxx_pickplace_env(&scenario)));
    float block_r = hopcxx_pickplace_block_r(&scenario);
    for (size_t i = 0; i < poses.size(); i++) {
        if (i == movingIndex) {
            continue;
        }
        hopcxx_env_add_cuboid(env.get(), poses[i], block_r);
    }
    return env;
}

PickPlaceDomain::State PickPlaceDomain::initialState(const Context &ctx) {
    State s;
    s.q_arm = hopcxx_pickplace_robot_q_start(ctx.scenario);
    auto poses = std::make_shared<std::vector<CPose>>();
    size_t n = hopcxx_pickplace_num_objects(ctx.scenario);
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_pickplace_object_pose(ctx.scenario, i));
    }
    s.poses = poses;
    return s;
}

bool PickPlaceDomain::drawKeyframe(const Context &ctx, const State &before, const Action &act,
    const Action *next, size_t, Keyframe &out) {
    const RobotVtable &rv = robot_vtable(ctx.robot);
    EnvPtr env = buildAttemptEnv(*ctx.scenario, *before.poses, act.object_index);
    out.after = before;
    if (act.type == ActionType::Pick) {
        CPose g = rv.sample_rel_pose();
        CPose eeTarget = pose_mul((*before.poses)[act.object_index], g);
        if (!rv.ik(eeTarget, &out.after.q_arm) || !rv.validate(out.after.q_arm, env.get())) {
            return false;
        }
        // The keyframe also starts the next action's motion, in the world this action leaves
        // behind, so it has to be valid there too.
        if (next
            && !rv.validate_attached(
                out.after.q_arm, env.get(), hopcxx_pickplace_block_r(ctx.scenario), pose_inverse(g))) {
            return false;
        }
        out.after.held_object = static_cast<int64_t>(act.object_index);
        out.after.grasp_offset = g;
        out.hasHeld = false;
        out.heldRel = pose_identity();
        return true;
    }
    CHECK_EQ(before.held_object, static_cast<int64_t>(act.object_index),
        "place must follow pick of the same object");
    float block_r = hopcxx_pickplace_block_r(ctx.scenario);
    CTable surface = hopcxx_pickplace_surface(ctx.scenario, act.surface_index);
    CPose target = pose_mul(rv.sample_table_pose(surface), sample_placing_rotation());
    CPose eeTarget = pose_mul(target, before.grasp_offset);
    CPose heldRel = pose_inverse(before.grasp_offset);
    if (!rv.ik(eeTarget, &out.after.q_arm)
        || !rv.validate_attached(out.after.q_arm, env.get(), block_r, heldRel)) {
        return false;
    }
    auto poses = std::make_shared<std::vector<CPose>>(*before.poses);
    (*poses)[act.object_index] = target;
    if (next) {
        EnvPtr envAfter = buildAttemptEnv(*ctx.scenario, *poses, next->object_index);
        if (!rv.validate(out.after.q_arm, envAfter.get())) {
            return false;
        }
    }
    out.after.poses = poses;
    out.after.held_object = -1;
    out.after.grasp_offset = pose_identity();
    out.hasHeld = true;
    out.heldRel = heldRel;
    return true;
}

int PickPlaceDomain::motionStep(const Context &ctx, const State &before, const Action &act,
    const Keyframe &kf, size_t, MotionState &motion, float *buf, size_t cap, size_t *len) {
    EnvPtr env = buildAttemptEnv(*ctx.scenario, *before.poses, act.object_index);
    return motion.step(robot_vtable(ctx.robot), before.q_arm, kf.after.q_arm, env.get(),
        hopcxx_pickplace_block_r(ctx.scenario), kf.hasHeld, kf.heldRel, buf, cap, len);
}

} // namespace hopct
