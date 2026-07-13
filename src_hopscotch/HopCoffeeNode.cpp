#include "HopCoffeeNode.h"
#include "hop_pose.h"
#include <algorithm>
#include <cstdlib>
#include <random>

namespace hopct {

static size_t findItem(const CoffeeScenario &scenario, uint8_t kind, uint64_t index) {
    size_t n = hopcxx_coffee_num_objects(&scenario);
    for (size_t i = 0; i < n; i++) {
        if (hopcxx_coffee_object_kind(&scenario, i) == kind
            && hopcxx_coffee_object_index(&scenario, i) == index) {
            return i;
        }
    }
    HALT("coffee item (kind=" << (int)kind << ", index=" << index << ") not found in scenario");
}

std::vector<CoffeeAction> makeCoffeePlan(const CoffeeScenario &scenario) {
    size_t cup0 = findItem(scenario, 0, 0);
    size_t creamCup = findItem(scenario, 0, 1);
    size_t sugarCup = findItem(scenario, 0, 2);
    size_t spoon = findItem(scenario, 1, 0);
    using T = CoffeeActionType;
    return {
        { T::Pick, cup0, 0 },
        { T::Fill, cup0, 0 },
        { T::Place, cup0, 0 },
        { T::Pick, creamCup, 0 },
        { T::Pour, creamCup, cup0 },
        { T::Place, creamCup, 0 },
        { T::Pick, spoon, 0 },
        { T::Scoop, spoon, sugarCup },
        { T::Dump, spoon, cup0 },
        { T::Stir, spoon, cup0 },
        { T::Place, spoon, 0 },
        { T::Pick, cup0, 0 },
    };
}

static EnvPtr buildAttemptEnv(
    const CoffeeScenario &scenario, const std::vector<CPose> &poses, size_t movingIndex) {
    EnvPtr env = wrapEnv(hopcxx_env_clone(hopcxx_coffee_env(&scenario)));
    float object_r = hopcxx_coffee_object_r(&scenario);
    for (size_t i = 0; i < poses.size(); i++) {
        if (i == movingIndex) {
            continue;
        }
        hopcxx_env_add_ball(env.get(), poses[i].x, poses[i].y, poses[i].z, object_r);
    }
    return env;
}

// Matches common/streams_coffee.py's `_zoff`/`_zrot` helpers.
static CPose sampleZOff(float object_r) {
    static thread_local std::mt19937 rng { std::random_device { }() };
    std::uniform_real_distribution<float> mult(2.0f, 4.0f);
    return pose_from_xyz_yaw(0, 0, object_r * mult(rng), 0);
}
static CPose sampleZRot() {
    static thread_local std::mt19937 rng { std::random_device { }() };
    std::uniform_real_distribution<float> unit(0.0f, 2.0f * (float)M_PI);
    return pose_from_xyz_yaw(0, 0, 0, unit(rng));
}

HopCoffeeNode::HopCoffeeNode(
    const CoffeeScenario &scenario, const std::vector<CoffeeAction> &plan, RobotTag robot)
    : ComputeNode(nullptr)
    , scenario(scenario)
    , plan(plan)
    , action_index(-1)
    , robot(robot) {
    q_arm = hopcxx_coffee_robot_q_start(&scenario);
    size_t n = hopcxx_coffee_num_objects(&scenario);
    poses = std::make_shared<std::vector<CPose>>();
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_coffee_object_pose(&scenario, i));
    }
    cupLogic = std::make_shared<std::vector<CupLogic>>(n);
    // cup0=EMPTY, cream_cup=CREAMY, both sugar cups=SWEET (see hop-bench's coffee.rs).
    (*cupLogic)[findItem(scenario, 0, 1)].cream = true;
    (*cupLogic)[findItem(scenario, 0, 2)].sugar = true;
    (*cupLogic)[findItem(scenario, 0, 3)].sugar = true;
    isComplete = true;
    isFeasible = true;
    isTerminal = false;
    name = "root";
}

HopCoffeeNode::HopCoffeeNode(HopCoffeeNode &parent, int childIndex)
    : ComputeNode(&parent)
    , scenario(parent.scenario)
    , plan(parent.plan)
    , action_index(parent.action_index + 1)
    , q_arm(parent.q_arm)
    , held_object(parent.held_object)
    , grasp_offset(parent.grasp_offset)
    , poses(parent.poses)
    , cupLogic(parent.cupLogic)
    , spoonScooped(parent.spoonScooped)
    , robot(parent.robot) {
    name << "a" << action_index << "#" << childIndex;
}

void HopCoffeeNode::write(std::ostream &os) const { os << name; }

double HopCoffeeNode::branchingPenalty_child(int i) {
    return ::pow(double(i) / hopInfo().w0, hopInfo().wP);
}

std::shared_ptr<rai::ComputeNode> HopCoffeeNode::createNewChild(int i) {
    return std::make_shared<HopCoffeeNode>(*this, i);
}

void HopCoffeeNode::untimedCompute() {
    CHECK_GE(action_index, 0, "root should never be (re-)computed");
    const CoffeeAction &act = plan.at(action_index);
    const RobotVtable &rv = robot_vtable(robot);
    float object_r = hopcxx_coffee_object_r(&scenario);

    if (!motionStarted) {
        diagStats().attempts[action_index]++;
        EnvPtr env = buildAttemptEnv(scenario, *poses, act.item_index);
        CConfig qTarget;
        bool ok = true;
        CPose eeTarget, heldRel = pose_identity();
        bool hasHeld = (act.type != CoffeeActionType::Pick);

        if (act.type == CoffeeActionType::Pick) {
            CPose g = rv.sample_rel_pose();
            eeTarget = pose_mul((*poses)[act.item_index], g);
            nextGraspOffset = g;
        } else {
            CPose target;
            switch (act.type) {
            case CoffeeActionType::Place:
                target = rv.sample_table_pose(hopcxx_coffee_table(&scenario));
                nextPlacedPose = target;
                break;
            case CoffeeActionType::Fill: {
                static thread_local std::mt19937 rng { std::random_device { }() };
                std::uniform_real_distribution<float> unit(0.0f, 2.0f * (float)M_PI);
                target = pose_mul(
                    hopcxx_coffee_fill_pose(&scenario), pose_from_xyz_yaw(0, 0, 0, unit(rng)));
                break;
            }
            case CoffeeActionType::Pour:
            case CoffeeActionType::Scoop:
            case CoffeeActionType::Stir:
                target = pose_mul(
                    pose_mul(sampleZOff(object_r), (*poses)[act.target_index]), sampleZRot());
                break;
            case CoffeeActionType::Dump: {
                CPose flip = pose_from_xyz_rpy(0, 0, 0, (float)M_PI, 0, 0);
                target
                    = pose_mul(pose_mul(pose_mul(sampleZOff(object_r), (*poses)[act.target_index]),
                                   sampleZRot()),
                        flip);
                break;
            }
            default:
                HALT("unreachable");
            }
            eeTarget = pose_mul(target, grasp_offset);
            heldRel = pose_inverse(grasp_offset);
        }

        ok = rv.ik(eeTarget, &qTarget);
        if (ok) {
            diagStats().ikOk[action_index]++;
            ok = hasHeld ? rv.validate_attached(qTarget, env.get(), object_r, heldRel)
                         : rv.validate(qTarget, env.get());
        }
        if (ok) {
            diagStats().validateOk[action_index]++;
        }
        if (!ok) {
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

    EnvPtr env = buildAttemptEnv(scenario, *poses, act.item_index);
    int maxWp = hopInfo().maxTrajWaypoints;
    std::vector<float> buf(maxWp * HOPCXX_MAX_DIM);
    size_t len = 0;
    int r = motionState.step(rv, q_arm, pendingQEnd, env.get(), object_r, pendingHasHeld,
        pendingHeldRel, buf.data(), maxWp, &len);

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
        CConfig c { };
        c.dim = nextQArm.dim;
        std::copy_n(buf.begin() + w * c.dim, c.dim, c.q);
        trajectory.push_back(c);
    }
    q_arm = nextQArm;

    switch (act.type) {
    case CoffeeActionType::Pick:
        held_object = (int64_t)act.item_index;
        grasp_offset = nextGraspOffset;
        break;
    case CoffeeActionType::Place: {
        held_object = -1;
        grasp_offset = pose_identity();
        auto newPoses = std::make_shared<std::vector<CPose>>(*poses);
        (*newPoses)[act.item_index] = nextPlacedPose;
        poses = newPoses;
        break;
    }
    case CoffeeActionType::Fill: {
        auto nl = std::make_shared<std::vector<CupLogic>>(*cupLogic);
        CHECK(!(*nl)[act.item_index].coffee && !(*nl)[act.item_index].mixed,
            "fill precondition violated");
        (*nl)[act.item_index].coffee = true;
        cupLogic = nl;
        break;
    }
    case CoffeeActionType::Pour: {
        auto nl = std::make_shared<std::vector<CupLogic>>(*cupLogic);
        CHECK((*nl)[act.item_index].cream, "pour precondition violated (src has no cream)");
        CHECK(!(*nl)[act.target_index].mixed, "pour precondition violated (dest already mixed)");
        (*nl)[act.item_index] = CupLogic { }; // pouring empties the source
        (*nl)[act.target_index].cream = true;
        cupLogic = nl;
        break;
    }
    case CoffeeActionType::Scoop: {
        CHECK(!spoonScooped, "scoop precondition violated (spoon already scooped)");
        CHECK(
            (*cupLogic)[act.target_index].sugar, "scoop precondition violated (cup has no sugar)");
        spoonScooped = true;
        break;
    }
    case CoffeeActionType::Dump: {
        CHECK(spoonScooped, "dump precondition violated (spoon not scooped)");
        auto nl = std::make_shared<std::vector<CupLogic>>(*cupLogic);
        CHECK(!(*nl)[act.target_index].sugar, "dump precondition violated (cup already has sugar)");
        (*nl)[act.target_index].sugar = true;
        cupLogic = nl;
        spoonScooped = false;
        break;
    }
    case CoffeeActionType::Stir: {
        auto nl = std::make_shared<std::vector<CupLogic>>(*cupLogic);
        CupLogic &c = (*nl)[act.target_index];
        CHECK(c.coffee && c.sugar && c.cream, "stir precondition violated (cup not ready to mix)");
        c = CupLogic { };
        c.mixed = true;
        cupLogic = nl;
        break;
    }
    }

    isFeasible = true;
    isComplete = true;
    l = 1.;
    if (action_index + 1 == (int)plan.size()) {
        isTerminal = true;
    }
}

} // namespace hopct
