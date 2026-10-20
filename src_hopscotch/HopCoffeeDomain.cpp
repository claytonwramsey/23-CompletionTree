#include "HopCoffeeDomain.h"
#include "hop_pose.h"
#include <algorithm>
#include <cstdlib>
#include <numbers>
#include <random>

namespace hopct {

// The world with every item except the held one, or every item when `heldIndex` is negative.
// Each item is the convex decomposition of its mesh, as in hopscotch.
static EnvPtr buildAttemptEnv(
    const CoffeeScenario &scenario, const std::vector<CPose> &poses, int64_t heldIndex) {
    EnvPtr env = wrapEnv(hopcxx_env_clone(hopcxx_coffee_env(&scenario)));
    float object_r = hopcxx_coffee_object_r(&scenario);
    for (size_t i = 0; i < poses.size(); i++) {
        if (static_cast<int64_t>(i) == heldIndex) {
            continue;
        }
        hopcxx_env_add_coffee_item(
            env.get(), hopcxx_coffee_object_kind(&scenario, i), poses[i], object_r);
    }
    return env;
}

// The item that `act` holds, or -1 if it holds none.
static int64_t heldBy(const CoffeeAction &act) {
    return act.type == CoffeeActionType::Pick ? -1 : static_cast<int64_t>(act.item_index);
}

// Matches common/streams_coffee.py's `_zoff`/`_zrot` helpers.
static CPose sampleZOff(float object_r) {
    static thread_local std::mt19937 rng { std::random_device {}() };
    std::uniform_real_distribution<float> mult(2.0f, 4.0f);
    return pose_from_xyz_yaw(0, 0, object_r * mult(rng), 0);
}
static CPose sampleZRot() {
    static thread_local std::mt19937 rng { std::random_device {}() };
    std::uniform_real_distribution<float> unit(0.0f, 2.0f * std::numbers::pi_v<float>);
    return pose_from_xyz_yaw(0, 0, 0, unit(rng));
}

CoffeeDomain::State CoffeeDomain::initialState(const Context &ctx) {
    State s;
    s.q_arm = hopcxx_coffee_robot_q_start(ctx.scenario);
    size_t n = hopcxx_coffee_num_objects(ctx.scenario);
    auto poses = std::make_shared<std::vector<CPose>>();
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_coffee_object_pose(ctx.scenario, i));
    }
    s.poses = poses;
    auto logic = std::make_shared<std::vector<CupLogic>>(n);
    // Each cup starts with the ingredients that the scenario's contents bitmask lists.
    for (size_t i = 0; i < n; i++) {
        uint8_t contents = hopcxx_coffee_object_contents(ctx.scenario, i);
        (*logic)[i].coffee = contents & 1;
        (*logic)[i].sugar = contents & 2;
        (*logic)[i].cream = contents & 4;
    }
    s.cupLogic = logic;
    return s;
}

bool CoffeeDomain::drawKeyframe(const Context &ctx, const State &before, const Action &act,
    const Action *next, size_t k, Keyframe &out) {
    const RobotVtable &rv = robot_vtable(ctx.robot);
    float object_r = hopcxx_coffee_object_r(ctx.scenario);
    if (k < HopDiagStats::MAX_ACTIONS) {
        diagStats().attempts[k]++;
    }
    EnvPtr env = buildAttemptEnv(*ctx.scenario, *before.poses, heldBy(act));
    uint8_t kind = hopcxx_coffee_object_kind(ctx.scenario, act.item_index);
    out.after = before;
    CPose eeTarget, heldRel = pose_identity();
    CPose placed {};
    bool hasHeld = act.type != CoffeeActionType::Pick;
    if (act.type == CoffeeActionType::Pick) {
        CPose g = hopcxx_coffee_sample_grasp(kind, object_r);
        eeTarget = pose_mul((*before.poses)[act.item_index], g);
        out.after.grasp_offset = g;
    } else {
        CPose target;
        switch (act.type) {
        case CoffeeActionType::Place:
            target = rv.sample_table_pose(hopcxx_coffee_table(ctx.scenario));
            placed = target;
            break;
        case CoffeeActionType::Fill: {
            static thread_local std::mt19937 rng { std::random_device {}() };
            std::uniform_real_distribution<float> unit(0.0f, 2.0f * std::numbers::pi_v<float>);
            target = pose_mul(
                hopcxx_coffee_fill_pose(ctx.scenario), pose_from_xyz_yaw(0, 0, 0, unit(rng)));
            break;
        }
        case CoffeeActionType::Pour:
        case CoffeeActionType::Scoop:
        case CoffeeActionType::Stir:
            target = pose_mul(
                pose_mul(sampleZOff(object_r), (*before.poses)[act.target_index]), sampleZRot());
            break;
        case CoffeeActionType::Dump: {
            CPose flip = pose_from_xyz_rpy(0, 0, 0, std::numbers::pi_v<float>, 0, 0);
            target = pose_mul(
                pose_mul(pose_mul(sampleZOff(object_r), (*before.poses)[act.target_index]),
                    sampleZRot()),
                flip);
            break;
        }
        default:
            HALT("unreachable");
        }
        eeTarget = pose_mul(target, before.grasp_offset);
        heldRel = pose_inverse(before.grasp_offset);
    }
    if (!rv.ik(eeTarget, &out.after.q_arm)) {
        return false;
    }
    if (k < HopDiagStats::MAX_ACTIONS) {
        diagStats().ikOk[k]++;
    }
    bool valid = hasHeld
        ? rv.validate_attached_coffee_item(out.after.q_arm, env.get(), kind, object_r, heldRel)
        : rv.validate(out.after.q_arm, env.get());
    // The keyframe also starts the next action's motion, in the world this action leaves
    // behind, so it has to be valid there too.
    if (valid && next) {
        std::vector<CPose> after = *before.poses;
        if (act.type == CoffeeActionType::Place) {
            after[act.item_index] = placed;
        }
        // A place leaves the hand empty, and every other action keeps holding the same item.
        int64_t heldAfter
            = act.type == CoffeeActionType::Place ? -1 : static_cast<int64_t>(act.item_index);
        EnvPtr envAfter = buildAttemptEnv(*ctx.scenario, after, heldAfter);
        if (heldAfter < 0) {
            valid = rv.validate(out.after.q_arm, envAfter.get());
        } else {
            valid = rv.validate_attached_coffee_item(out.after.q_arm, envAfter.get(), kind,
                object_r, pose_inverse(out.after.grasp_offset));
        }
    }
    if (!valid) {
        return false;
    }
    if (k < HopDiagStats::MAX_ACTIONS) {
        diagStats().validateOk[k]++;
    }
    out.hasHeld = hasHeld;
    out.heldRel = heldRel;

    switch (act.type) {
    case CoffeeActionType::Pick:
        out.after.held_object = static_cast<int64_t>(act.item_index);
        break;
    case CoffeeActionType::Place: {
        out.after.held_object = -1;
        out.after.grasp_offset = pose_identity();
        auto poses = std::make_shared<std::vector<CPose>>(*before.poses);
        (*poses)[act.item_index] = placed;
        out.after.poses = poses;
        break;
    }
    case CoffeeActionType::Fill: {
        auto nl = std::make_shared<std::vector<CupLogic>>(*before.cupLogic);
        CHECK(!(*nl)[act.item_index].coffee && !(*nl)[act.item_index].mixed,
            "fill precondition violated");
        (*nl)[act.item_index].coffee = true;
        out.after.cupLogic = nl;
        break;
    }
    case CoffeeActionType::Pour: {
        auto nl = std::make_shared<std::vector<CupLogic>>(*before.cupLogic);
        CHECK((*nl)[act.item_index].cream, "pour precondition violated (src has no cream)");
        CHECK(!(*nl)[act.target_index].mixed, "pour precondition violated (dest already mixed)");
        (*nl)[act.item_index] = CupLogic {};
        (*nl)[act.target_index].cream = true;
        out.after.cupLogic = nl;
        break;
    }
    case CoffeeActionType::Scoop:
        CHECK(!before.spoonScooped, "scoop precondition violated (spoon already scooped)");
        CHECK((*before.cupLogic)[act.target_index].sugar,
            "scoop precondition violated (cup has no sugar)");
        out.after.spoonScooped = true;
        break;
    case CoffeeActionType::Dump: {
        CHECK(before.spoonScooped, "dump precondition violated (spoon not scooped)");
        auto nl = std::make_shared<std::vector<CupLogic>>(*before.cupLogic);
        CHECK(!(*nl)[act.target_index].sugar, "dump precondition violated (cup already has sugar)");
        (*nl)[act.target_index].sugar = true;
        out.after.cupLogic = nl;
        out.after.spoonScooped = false;
        break;
    }
    case CoffeeActionType::Stir: {
        auto nl = std::make_shared<std::vector<CupLogic>>(*before.cupLogic);
        CupLogic &c = (*nl)[act.target_index];
        CHECK(c.coffee && c.sugar && c.cream && !c.mixed,
            "stir precondition violated (cup not ready to mix)");
        c.mixed = true;
        out.after.cupLogic = nl;
        break;
    }
    }
    return true;
}

int CoffeeDomain::motionStep(const Context &ctx, const State &before, const Action &act,
    const Keyframe &kf, size_t k, MotionState &motion, float *buf, size_t cap, size_t *len) {
    EnvPtr env = buildAttemptEnv(*ctx.scenario, *before.poses, heldBy(act));
    int r = motion.step(robot_vtable(ctx.robot), before.q_arm, kf.after.q_arm, env.get(),
        hopcxx_coffee_object_r(ctx.scenario), kf.hasHeld, kf.heldRel, buf, cap, len,
        hopcxx_coffee_object_kind(ctx.scenario, act.item_index));
    if (r > 0 && k < HopDiagStats::MAX_ACTIONS) {
        diagStats().motionOk[k]++;
    }
    return r;
}

} // namespace hopct
