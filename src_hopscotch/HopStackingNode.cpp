#include "HopStackingNode.h"
#include "hop_pose.h"
#include <algorithm>
#include <cstdlib>
#include <random>

namespace hopct {

std::vector<Action> makeStackingPlan(const StackingScenario &scenario) {
    size_t n = hopcxx_stacking_num_objects(&scenario);
    std::vector<Action> plan;
    for (size_t k = 0; k < n; k++) {
        uint64_t goalId = hopcxx_stacking_goal_order(&scenario, k);
        size_t idx = n; // object_ids aren't necessarily 0..n-1 in order (HashMap
                        // iteration order on the Rust side) -- look it up.
        for (size_t i = 0; i < n; i++) {
            if (hopcxx_stacking_object_id(&scenario, i) == goalId) {
                idx = i;
                break;
            }
        }
        CHECK(idx < n, "goal_order object id not found among scenario objects");
        plan.push_back({ ActionType::Pick, idx });
        plan.push_back({ ActionType::Place, idx });
    }
    return plan;
}

static EnvPtr buildAttemptEnv(
    const StackingScenario &scenario, const std::vector<CPose> &poses, size_t movingIndex) {
    EnvPtr env = wrapEnv(hopcxx_env_clone(hopcxx_stacking_env(&scenario)));
    float block_r = hopcxx_stacking_block_r(&scenario);
    for (size_t i = 0; i < poses.size(); i++) {
        if (i == movingIndex) {
            continue;
        }
        hopcxx_env_add_ball(env.get(), poses[i].x, poses[i].y, poses[i].z, block_r);
    }
    return env;
}

HopStackingNode::HopStackingNode(
    const StackingScenario &scenario, const std::vector<Action> &plan, RobotTag robot)
    : ComputeNode(nullptr)
    , scenario(scenario)
    , plan(plan)
    , robot(robot)
    , action_index(-1) {
    q_arm = hopcxx_stacking_robot_q_start(&scenario);
    poses = std::make_shared<std::vector<CPose>>();
    size_t n = hopcxx_stacking_num_objects(&scenario);
    poses->reserve(n);
    for (size_t i = 0; i < n; i++) {
        poses->push_back(hopcxx_stacking_object_pose(&scenario, i));
    }
    isComplete = true;
    isFeasible = true;
    isTerminal = false;
    name = "root";
}

HopStackingNode::HopStackingNode(HopStackingNode &parent, int childIndex)
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

void HopStackingNode::write(std::ostream &os) const { os << name; }

double HopStackingNode::branchingPenalty_child(int i) {
    return ::pow(double(i) / hopInfo().w0, hopInfo().wP);
}

std::shared_ptr<rai::ComputeNode> HopStackingNode::createNewChild(int i) {
    return std::make_shared<HopStackingNode>(*this, i);
}

void HopStackingNode::untimedCompute() {
    CHECK_GE(action_index, 0, "root should never be (re-)computed");
    const Action &act = plan.at(action_index);
    const RobotVtable &rv = robot_vtable(robot);

    if (!motionStarted) {
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
            float block_r = hopcxx_stacking_block_r(&scenario);

            // Tower position of this Place = (action_index - 1) / 2 (each tower
            // slot is a Pick then a Place); 0 -> free table pose, otherwise stack
            // directly on top of the block placed at the previous tower position
            // (matches `common/streams_stacking.py`'s `sample_stack_pose`).
            int towerPos = (action_index - 1) / 2;
            CPose target;
            if (towerPos == 0) {
                target = rv.sample_table_pose(hopcxx_stacking_table(&scenario));
            } else {
                uint64_t belowId = hopcxx_stacking_goal_order(&scenario, towerPos - 1);
                size_t belowIdx = poses->size();
                for (size_t i = 0; i < hopcxx_stacking_num_objects(&scenario); i++) {
                    if (hopcxx_stacking_object_id(&scenario, i) == belowId) {
                        belowIdx = i;
                        break;
                    }
                }
                CHECK(belowIdx < poses->size(), "below-block id not found");
                CPose belowPose = (*poses)[belowIdx];
                static thread_local std::mt19937 rng { std::random_device { }() };
                std::uniform_real_distribution<float> yawDist(0.0f, 2.0f * (float)M_PI);
                target = pose_mul(belowPose, pose_from_xyz_yaw(0, 0, 2 * block_r, yawDist(rng)));
            }

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
    }

    EnvPtr env = buildAttemptEnv(scenario, *poses, act.object_index);
    int maxWp = hopInfo().maxTrajWaypoints;
    std::vector<float> buf(maxWp * HOPCXX_MAX_DIM);
    size_t len = 0;
    float block_r = hopcxx_stacking_block_r(&scenario);
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
