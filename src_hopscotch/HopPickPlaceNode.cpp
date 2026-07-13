#include "HopPickPlaceNode.h"
#include "hop_pose.h"
#include <algorithm>
#include <cstdlib>

namespace hopct {

std::vector<Action> makePickPlacePlan(const PickPlaceScenario &scenario) {
    std::vector<Action> plan;
    int64_t target = hopcxx_pickplace_target_block(&scenario);
    int64_t goalSurface = hopcxx_pickplace_goal_surface(&scenario);
    size_t n = hopcxx_pickplace_num_objects(&scenario);
    if (target >= 0) {
        // cabinet: goal is "holding target_block" -- a single Pick action.
        for (size_t i = 0; i < n; i++) {
            if ((int64_t)hopcxx_pickplace_object_id(&scenario, i) == target) {
                plan.push_back({ ActionType::Pick, i });
                break;
            }
        }
    } else {
        CHECK_GE(goalSurface, 0, "PickPlaceScenario must set target_block xor goal_surface");
        // packing: goal is "every object on goal_surface" -- Pick/Place per object.
        for (size_t i = 0; i < n; i++) {
            plan.push_back({ ActionType::Pick, i });
            plan.push_back({ ActionType::Place, i });
        }
    }
    return plan;
}

// Build the static-geometry + other-objects env for one action attempt on
// `movingIndex` (excluded, since it's either about to be picked or is the
// held/attached object being placed -- see HopPickPlaceNode.h's design note
// on why other objects are modeled as balls of radius block_r rather than
// via full trajectory-vs-trajectory checking).
static EnvPtr buildAttemptEnv(
    const PickPlaceScenario &scenario, const std::vector<CPose> &poses, size_t movingIndex) {
    EnvPtr env = wrapEnv(hopcxx_env_clone(hopcxx_pickplace_env(&scenario)));
    float block_r = hopcxx_pickplace_block_r(&scenario);
    for (size_t i = 0; i < poses.size(); i++) {
        if (i == movingIndex) {
            continue;
        }
        hopcxx_env_add_ball(env.get(), poses[i].x, poses[i].y, poses[i].z, block_r);
    }
    return env;
}

HopPickPlaceNode::HopPickPlaceNode(
    const PickPlaceScenario &scenario, const std::vector<Action> &plan, RobotTag robot)
    : ComputeNode(nullptr)
    , scenario(scenario)
    , plan(plan)
    , robot(robot)
    , action_index(-1) {
    q_arm = hopcxx_pickplace_robot_q_start(&scenario);
    poses = std::make_shared<std::vector<CPose>>();
    size_t n = hopcxx_pickplace_num_objects(&scenario);
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_pickplace_object_pose(&scenario, i));
    }
    isComplete = true;
    isFeasible = true;
    isTerminal = false;
    name = "root";
}

HopPickPlaceNode::HopPickPlaceNode(HopPickPlaceNode &parent, int childIndex)
    : ComputeNode(&parent)
    , scenario(parent.scenario)
    , plan(parent.plan)
    , robot(parent.robot)
    , action_index(parent.action_index + 1)
    , q_arm(parent.q_arm)
    , held_object(parent.held_object)
    , grasp_offset(parent.grasp_offset)
    , poses(parent.poses) {
    name << "a" << action_index << "#" << childIndex;
}

void HopPickPlaceNode::write(std::ostream &os) const { os << name; }

double HopPickPlaceNode::branchingPenalty_child(int i) {
    return ::pow(double(i) / hopInfo().w0, hopInfo().wP);
}

std::shared_ptr<rai::ComputeNode> HopPickPlaceNode::createNewChild(int i) {
    return std::make_shared<HopPickPlaceNode>(*this, i);
}

void HopPickPlaceNode::untimedCompute() {
    CHECK_GE(action_index, 0, "root should never be (re-)computed");
    const Action &act = plan.at(action_index);
    const RobotVtable &rv = robot_vtable(robot);

    if (!motionStarted) {
        // ---- One-shot grasp/target sample + IK + validate. A failure here is
        // a dead attempt (same as before); on success we only *stage* the
        // outcome (next*/pending* fields) -- q_arm/held_object/grasp_offset/
        // poses aren't updated until motion planning actually succeeds below.
        EnvPtr env = buildAttemptEnv(scenario, *poses, act.object_index);
        CConfig qTarget;
        bool ok;
        if (act.type == ActionType::Pick) {
            CPose g = rv.sample_rel_pose();
            CPose objPose = (*poses)[act.object_index];
            CPose eeTarget = pose_mul(objPose, g);
            ok = rv.ik(eeTarget, &qTarget);
            if (ok) {
                ok = rv.validate(qTarget, env.get());
            }
            if (ok) {
                nextHeldObject = (int64_t)act.object_index;
                nextGraspOffset = g;
                pendingHasHeld = false;
                pendingHeldRel = pose_identity();
            }
        } else {
            CHECK_EQ(held_object, (int64_t)act.object_index,
                "place must follow pick of the same object");
            float block_r = hopcxx_pickplace_block_r(&scenario);
            int64_t goalSurface = hopcxx_pickplace_goal_surface(&scenario);
            CTable surface = hopcxx_pickplace_surface(&scenario, (size_t)goalSurface);
            CPose target = rv.sample_table_pose(surface);
            CPose eeTarget = pose_mul(target, grasp_offset);
            CPose heldRel = pose_inverse(grasp_offset);
            ok = rv.ik(eeTarget, &qTarget);
            if (ok) {
                ok = rv.validate_attached(qTarget, env.get(), block_r, heldRel);
            }
            if (ok) {
                nextHeldObject = -1;
                nextGraspOffset = pose_identity();
                nextPlacedPose = target;
                pendingHasHeld = true;
                pendingHeldRel = heldRel;
            }
        }
        if (!ok) {
            isFeasible = false;
            isComplete = true;
            return;
        }
        nextQArm = qTarget;
        pendingQEnd = qTarget;
        motionStarted = true;
        // fall through: attempt the first motion-planning resume immediately,
        // rather than wasting a full extra AStar step just to have started.
    }

    // ---- Resumable motion planning (see HopCommon.h's MotionPlanState). ----
    EnvPtr env = buildAttemptEnv(scenario, *poses, act.object_index);
    int maxWp = hopInfo().maxTrajWaypoints;
    std::vector<float> buf(maxWp * HOPCXX_MAX_DIM);
    size_t len = 0;
    float block_r = hopcxx_pickplace_block_r(&scenario);
    int r = motionState.step(rv, q_arm, pendingQEnd, env.get(), block_r, pendingHasHeld,
        pendingHeldRel, buf.data(), maxWp, &len);

    if (r == 0) {
        return; // still searching -- leave isComplete false to be resumed later
    }
    if (r < 0) {
        isFeasible = false;
        isComplete = true;
        return;
    }

    for (size_t w = 0; w < len; w++) {
        CConfig c { };
        c.dim = nextQArm.dim;
        std::copy_n(buf.begin() + w * c.dim, c.dim, c.q);
        trajectory.push_back(c);
    }
    q_arm = nextQArm;
    held_object = nextHeldObject;
    grasp_offset = nextGraspOffset;
    if (act.type == ActionType::Place) {
        auto newPoses = std::make_shared<std::vector<CPose>>(*poses);
        (*newPoses)[act.object_index] = nextPlacedPose;
        poses = newPoses;
    }

    isFeasible = true;
    isComplete = true;
    l = 1.;
    if (action_index + 1 == (int)plan.size()) {
        isTerminal = true;
    }
}

} // namespace hopct
