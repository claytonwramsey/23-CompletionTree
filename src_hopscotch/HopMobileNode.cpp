#include "HopMobileNode.h"
#include "hop_pose.h"
#include <algorithm>
#include <cstdlib>
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

HopMobileNode::HopMobileNode(
    const MobileScenario &scenario, const std::vector<Action> &plan, rai::ComputeNode *parent)
    : ComputeNode(parent)
    , scenario(scenario)
    , plan(plan)
    , action_index(-1) {
    q_arm = hopcxx_mobile_robot_q_start_arm(&scenario);
    base_pose = hopcxx_mobile_base_start_pose(&scenario);
    poses = std::make_shared<std::vector<CPose>>();
    size_t n = hopcxx_mobile_num_objects(&scenario);
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_mobile_object_pose(&scenario, i));
    }
    isComplete = true;
    isFeasible = true;
    isTerminal = false;
    name = "root";
}

HopMobileNode::HopMobileNode(HopMobileNode &parent, int childIndex)
    : ComputeNode(&parent)
    , scenario(parent.scenario)
    , plan(parent.plan)
    , action_index(parent.action_index + 1)
    , q_arm(parent.q_arm)
    , base_pose(parent.base_pose)
    , held_object(parent.held_object)
    , grasp_offset(parent.grasp_offset)
    , poses(parent.poses) {
    name << "a" << action_index << "#" << childIndex;
}

void HopMobileNode::write(std::ostream &os) const { os << name; }

double HopMobileNode::branchingPenalty_child(int i) { return hopBranchingPenalty(i); }

std::shared_ptr<rai::ComputeNode> HopMobileNode::createNewChild(int i) {
    return std::make_shared<HopMobileNode>(*this, i);
}

void HopMobileNode::untimedCompute() {
    CHECK_GE(action_index, 0, "root should never be (re-)computed");
    const Action &act = plan.at(action_index);
    const RobotVtable &rv = robot_vtable(RobotTag::Pr2);

    // single-shot geometric move.
    // Selects target poses close to the next action.
    if (act.type == ActionType::Move) {
        float baseBounds[4];
        hopcxx_mobile_base_bounds(&scenario, baseBounds);
        const Action *nextPickTarget = nullptr;
        for (size_t k = (size_t)action_index + 1;
            k < plan.size() && plan[k].type != ActionType::Move; k++) {
            if (plan[k].type == ActionType::Pick) {
                nextPickTarget = &plan[k];
                break;
            }
        }
        CPose bq;
        bool ok;
        if (nextPickTarget) {
            CPose targetPoint = (*poses)[nextPickTarget->object_index];
            ok = sample_reachable_base(targetPoint, baseBounds, &bq);
        } else {
            static thread_local std::mt19937 rng { std::random_device {}() };
            std::uniform_real_distribution<float> xDist(baseBounds[0], baseBounds[2]);
            std::uniform_real_distribution<float> yDist(baseBounds[1], baseBounds[3]);
            std::uniform_real_distribution<float> yawDist(0.0f, 2.0f * (float)M_PI);
            bq = pose_from_xyz_yaw(xDist(rng), yDist(rng), 0.0f, yawDist(rng));
            ok = true;
        }
        if (ok) {
            size_t excludeIndex = held_object >= 0 ? (size_t)held_object : poses->size();
            EnvPtr env = buildAttemptEnv(scenario, *poses, excludeIndex, bq);
            if (held_object >= 0) {
                float block_r = hopcxx_mobile_block_r(&scenario);
                CPose heldRel = pose_inverse(grasp_offset);
                ok = rv.validate_attached(q_arm, env.get(), block_r, heldRel);
            } else {
                ok = rv.validate(q_arm, env.get());
            }
        }
        if (!ok) {
            isFeasible = false;
            isComplete = true;
            return;
        }
        base_pose = bq;
        isFeasible = true;
        isComplete = true;
        l = 1.;
        if (action_index + 1 == (int)plan.size()) {
            isTerminal = true;
        }
        return;
    }

    if (!motionStarted) {
        CConfig qTarget;
        bool ok;

        if (act.type == ActionType::Pick) {
            CPose objPose = (*poses)[act.object_index];
            CPose g = rv.sample_rel_pose();
            CPose targetLocal = pose_mul(pose_inverse(base_pose), objPose);
            CPose eeTarget = pose_mul(targetLocal, g);
            ok = rv.ik(eeTarget, &qTarget);
            if (ok) {
                EnvPtr env = buildAttemptEnv(scenario, *poses, act.object_index, base_pose);
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

            CTable surface = hopcxx_mobile_surface(&scenario, act.surface_index);
            CPose target = rv.sample_table_pose(surface); // world frame
            float block_r = hopcxx_mobile_block_r(&scenario);
            CPose targetLocal = pose_mul(pose_inverse(base_pose), target);
            CPose eeTarget = pose_mul(targetLocal, grasp_offset);
            CPose heldRel = pose_inverse(grasp_offset);
            ok = rv.ik(eeTarget, &qTarget);
            if (ok) {
                EnvPtr env = buildAttemptEnv(scenario, *poses, act.object_index, base_pose);
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
        pendingBasePose = base_pose; // Pick/Place never change the base
        motionStarted = true;
    }

    EnvPtr env = buildAttemptEnv(scenario, *poses, act.object_index, pendingBasePose);
    int maxWp = hopInfo().maxTrajWaypoints;
    std::vector<float> buf(maxWp * HOPCXX_MAX_DIM);
    size_t len = 0;
    float block_r = hopcxx_mobile_block_r(&scenario);
    int r = motionState.step(rv, q_arm, pendingQEnd, env.get(), block_r, pendingHasHeld,
        pendingHeldRel, buf.data(), maxWp, &len);

    if (r == 0) {
        return;
    }
    if (r < 0) {
        isFeasible = false;
        isComplete = true;
        return;
    }

    for (size_t w = 0; w < len; w++) {
        CConfig c {};
        c.dim = nextQArm.dim;
        std::copy_n(buf.begin() + w * c.dim, c.dim, c.q);
        trajectory.push_back(c);
    }
    q_arm = nextQArm;
    base_pose = pendingBasePose;
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
