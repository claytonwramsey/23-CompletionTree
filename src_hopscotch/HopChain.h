#pragma once
// The completion tree of the ELS paper's TAMP experiments, built on hopscotch's primitives.
//
// A skeleton node holds one task plan and branches infinitely over waypoint seeds.
// A waypoint node solves every keyframe of that plan in plan order, each conditioned on the
// keyframes before it.
// This mirrors the reference's sequential waypoint solver, which solves one keyframe at a time.
// Each compute step makes one draw, so a node stays incomplete while it works, like the
// reference's stepped NLP solver.
// A keyframe whose draw fails is drawn again, and after Hop/waypointMaxDraws failed draws in a
// row the whole seed is infeasible, like an NLP that runs out of evaluations.
// Once a keyframe is drawn it never changes, so the only way to recover from a bad keyframe or
// a failed motion is a fresh seed.
// Below a complete waypoint node hangs a chain of motion nodes, one per action, each with
// exactly one child.
// A motion node runs a resumable RRT between two keyframes, and its failure kills the seed.
// The last motion node is the terminal.
// The reference ends with a trajectory optimization node, which hopscotch has no counterpart for.
//
// A domain type D supplies the following.
//   Context, a cheap copyable handle on the scenario.
//   Action, one step of a task plan.
//   State, the world before an action, with `q_arm` and `poses`.
//   Keyframe, with `after` holding the State after the action and its `q_arm` as the keyframe.
//   MotionState, a resumable motion search.
//   static State initialState(const Context &).
//   static bool drawKeyframe(const Context &, const State &before, const Action &,
//       const Action *next, size_t k, Keyframe &out), which draws once and reports whether the
//       draw is valid. `next` is the following action, or null for the last.
//   static int motionStep(const Context &, const State &before, const Action &,
//       const Keyframe &, size_t k, MotionState &, float *buf, size_t cap, size_t *len), which
//       returns 1 when solved, 0 while searching, and -1 when it gives up.
// A keyframe also starts the next action's motion, so drawKeyframe checks it in the world after
// its action as well as before.
#include "HopCommon.h"
#include <Search/ComputeNode.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace hopct {

template <class D> struct HopWaypointsNode;
template <class D> struct HopMotionNode;

// The State and Keyframe of the domains whose world is an arm plus movable objects.
struct ArmState {
    CConfig q_arm;
    int64_t held_object = -1; // index into the scenario's object list
    CPose grasp_offset = CPose { 0, 0, 0, 0, 0, 0, 1 };
    std::shared_ptr<const std::vector<CPose>> poses; // per scenario object index
};
struct ArmKeyframe {
    ArmState after;
    bool hasHeld = false; // whether the motion into this keyframe carries an object
    CPose heldRel = CPose { 0, 0, 0, 0, 0, 0, 1 };
};

template <class D> struct HopSkeletonNode : rai::ComputeNode {
    typename D::Context ctx;
    const std::vector<typename D::Action> &plan; // borrowed from the skeleton root

    HopSkeletonNode(typename D::Context ctx, const std::vector<typename D::Action> &plan,
        rai::ComputeNode *parent, int index)
        : ComputeNode(parent)
        , ctx(ctx)
        , plan(plan) {
        name << "skeleton#" << index;
        isComplete = true;
        isFeasible = !plan.empty();
    }

    void untimedCompute() override { }
    int getNumDecisions() override { return -1; }
    double branchingPenalty_child(int i) override {
        return ::pow(double(i) / hopInfo().waypoint_w0, hopInfo().wP);
    }
    std::shared_ptr<ComputeNode> createNewChild(int i) override {
        return std::make_shared<HopWaypointsNode<D>>(*this, i);
    }
};

template <class D> struct HopWaypointsNode : rai::ComputeNode {
    const HopSkeletonNode<D> &skeleton;
    typename D::State initial;
    std::vector<typename D::Keyframe> keyframes; // keyframes[k] ends action k
    int failedDraws = 0; // failed draws in a row on the next keyframe

    HopWaypointsNode(HopSkeletonNode<D> &parent, int seed)
        : ComputeNode(&parent)
        , skeleton(parent)
        , initial(D::initialState(parent.ctx)) {
        name << "waypoints#" << seed;
        keyframes.reserve(parent.plan.size());
    }

    const std::vector<typename D::Action> &plan() const { return skeleton.plan; }
    const typename D::State &stateBefore(size_t k) const {
        return k == 0 ? initial : keyframes[k - 1].after;
    }

    void untimedCompute() override {
        size_t k = keyframes.size();
        typename D::Keyframe kf;
        const typename D::Action *next = k + 1 < plan().size() ? &plan()[k + 1] : nullptr;
        if (!D::drawKeyframe(skeleton.ctx, stateBefore(k), plan()[k], next, k, kf)) {
            if (++failedDraws >= hopInfo().waypointMaxDraws) {
                isFeasible = false;
                isComplete = true;
            }
            return;
        }
        failedDraws = 0;
        keyframes.push_back(std::move(kf));
        if (keyframes.size() == plan().size()) {
            isComplete = true;
            l = 0.;
        }
    }
    int getNumDecisions() override { return 1; }
    std::shared_ptr<ComputeNode> createNewChild(int i) override {
        CHECK(!i, "a waypoint node has a single child");
        return std::make_shared<HopMotionNode<D>>(*this, *this, 0);
    }
};

template <class D> struct HopMotionNode : rai::ComputeNode {
    const HopWaypointsNode<D> &ways;
    size_t t; // the action this motion executes
    typename D::MotionState motion;
    std::vector<CConfig> trajectory;

    HopMotionNode(rai::ComputeNode &parent, const HopWaypointsNode<D> &ways, size_t t)
        : ComputeNode(&parent)
        , ways(ways)
        , t(t) {
        name << "motion" << t;
    }

    const typename D::Keyframe &keyframe() const { return ways.keyframes[t]; }

    void untimedCompute() override {
        int maxWp = hopInfo().maxTrajWaypoints;
        std::vector<float> buf(maxWp * HOPCXX_MAX_DIM);
        size_t len = 0;
        int r = D::motionStep(ways.skeleton.ctx, ways.stateBefore(t), ways.plan()[t], keyframe(), t, motion,
            buf.data(), maxWp, &len);
        if (r == 0) {
            return;
        }
        isComplete = true;
        if (r < 0) {
            isFeasible = false;
            return;
        }
        int dim = keyframe().after.q_arm.dim;
        trajectory.reserve(len);
        for (size_t w = 0; w < len; w++) {
            CConfig c {};
            c.dim = dim;
            std::copy_n(buf.begin() + w * dim, dim, c.q);
            trajectory.push_back(c);
        }
        l = 0.;
        isTerminal = t + 1 == ways.plan().size();
    }
    int getNumDecisions() override { return isTerminal ? 0 : 1; }
    std::shared_ptr<ComputeNode> createNewChild(int i) override {
        CHECK(!i, "a motion node has a single child");
        return std::make_shared<HopMotionNode<D>>(*this, ways, t + 1);
    }
};

// The motion nodes from the first action to `terminal`, in plan order.
template <class D>
std::vector<const HopMotionNode<D> *> motionChain(const HopMotionNode<D> *terminal) {
    std::vector<const HopMotionNode<D> *> chain;
    for (const rai::TreeSearchNode *n = terminal; n;) {
        auto *m = dynamic_cast<const HopMotionNode<D> *>(n);
        if (!m) {
            break;
        }
        chain.push_back(m);
        n = m->parent;
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
}

} // namespace hopct
