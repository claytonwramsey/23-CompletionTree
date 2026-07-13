#include "HopMobileNode.h"
#include "hop_pose.h"
#include <algorithm>
#include <cstdlib>

namespace hopct {

std::vector<Action> makeMobilePlan(const MobileScenario &scenario) {
    size_t n = hopcxx_mobile_num_objects(&scenario);
    std::vector<Action> plan;
    for (size_t i = 0; i < n; i++) {
        plan.push_back({ ActionType::Pick, i });
        plan.push_back({ ActionType::Place, i });
    }
    return plan;
}

// Builds the env for one action attempt: `scenario`'s static world
// re-expressed in `bq`'s local frame, plus a ball per *other* object (also
// re-expressed in that frame) -- mirrors `common/streams_mobile.py`'s
// `cfree_traj_pose`/`cfree_traj_holding_pose` (`rel = bq.inverse() * p2`).
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
        hopcxx_env_add_ball(env.get(), rel.x, rel.y, rel.z, block_r);
    }
    return env;
}

HopMobileNode::HopMobileNode(const MobileScenario &scenario, const std::vector<Action> &plan)
    : ComputeNode(nullptr)
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

double HopMobileNode::branchingPenalty_child(int i) {
    return ::pow(double(i) / hopInfo().w0, hopInfo().wP);
}

std::shared_ptr<rai::ComputeNode> HopMobileNode::createNewChild(int i) {
    return std::make_shared<HopMobileNode>(*this, i);
}

void HopMobileNode::untimedCompute() {
    CHECK_GE(action_index, 0, "root should never be (re-)computed");
    const Action &act = plan.at(action_index);
    const RobotVtable &rv = robot_vtable(RobotTag::Pr2);

    if (!motionStarted) {
        float baseBounds[4];
        hopcxx_mobile_base_bounds(&scenario, baseBounds);
        CConfig qTarget;
        bool ok;
        CPose bq;
        if (act.type == ActionType::Pick) {
            CPose objPose = (*poses)[act.object_index];
            ok = sample_reachable_base(objPose, baseBounds, &bq);
            if (ok) {
                CPose g = rv.sample_rel_pose();
                CPose targetLocal = pose_mul(pose_inverse(bq), objPose);
                CPose eeTarget = pose_mul(targetLocal, g);
                ok = rv.ik(eeTarget, &qTarget);
                if (ok) {
                    EnvPtr env = buildAttemptEnv(scenario, *poses, act.object_index, bq);
                    ok = rv.validate(qTarget, env.get());
                }
                if (ok) {
                    nextHeldObject = (int64_t)act.object_index;
                    nextGraspOffset = g;
                    pendingHasHeld = false;
                    pendingHeldRel = pose_identity();
                }
            }
        } else {
            CHECK_EQ(held_object, (int64_t)act.object_index,
                "place must follow pick of the same object");
            size_t goalSurface = hopcxx_mobile_goal_surface(&scenario);
            CTable surface = hopcxx_mobile_surface(&scenario, goalSurface);
            CPose target = rv.sample_table_pose(surface); // world frame
            ok = sample_reachable_base(target, baseBounds, &bq);
            if (ok) {
                float block_r = hopcxx_mobile_block_r(&scenario);
                CPose targetLocal = pose_mul(pose_inverse(bq), target);
                CPose eeTarget = pose_mul(targetLocal, grasp_offset);
                CPose heldRel = pose_inverse(grasp_offset);
                ok = rv.ik(eeTarget, &qTarget);
                if (ok) {
                    EnvPtr env = buildAttemptEnv(scenario, *poses, act.object_index, bq);
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
        }
        if (!ok) {
            isFeasible = false;
            isComplete = true;
            return;
        }
        nextQArm = qTarget;
        pendingQEnd = qTarget;
        pendingBasePose = bq;
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
        CConfig c { };
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
