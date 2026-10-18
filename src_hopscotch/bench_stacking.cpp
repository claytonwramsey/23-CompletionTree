// Benchmark driver for the hopscotch-ported `stacking` domain running on
// rai's own completion-tree search (rai::AStar over the tree in HopChain.h).
// See bench_hopscotch.cpp (the `pickplace`-domain analog) for the CLI/CSV
// conventions this mirrors.
#include "HopBenchDriver.h"
#include "HopStackingDomain.h"
#include "HopStackingSkeletonRoot.h"
#include <Core/util.h>
#include <Search/AStar.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

using namespace hopct;

static RobotTag parseRobot(const std::string &s) {
    if (s == "panda") {
        return RobotTag::Panda;
    }
    if (s == "ur5") {
        return RobotTag::Ur5;
    }
    if (s == "pr2") {
        return RobotTag::Pr2;
    }
    if (s == "ur10e") {
        return RobotTag::Ur10e;
    }
    HALT("unknown robot '" << s << "'");
}

static void dumpSolution(const std::string &path, const std::string &robotName, uint64_t seed,
    const StackingScenario &scenario, const std::vector<Action> &plan,
    const std::vector<const HopMotionNode<StackingDomain> *> &chain) {

    std::ofstream os(path);
    os << "{\n";
    os << "  \"problem\": \"stacking\",\n";
    os << "  \"robot\": \"" << robotName << "\",\n";
    os << "  \"seed\": " << seed << ",\n";
    size_t n = hopcxx_stacking_num_objects(&scenario);
    os << "  \"objects\": [";
    for (size_t i = 0; i < n; i++) {
        if (i) {
            os << ", ";
        }
        os << "{\"id\": " << hopcxx_stacking_object_id(&scenario, i) << ", \"start_pose\": ";
        writePose(os, hopcxx_stacking_object_pose(&scenario, i));
        os << "}";
    }
    os << "],\n";
    CTable t = hopcxx_stacking_table(&scenario);
    os << "  \"surfaces\": [{\"height\": " << t.height << ", \"aabb\": [" << t.x0 << "," << t.y0
       << "," << t.x1 << "," << t.y1 << "]}],\n";
    os << "  \"block_r\": " << hopcxx_stacking_block_r(&scenario) << ",\n";
    os << "  \"q_start\": ";
    writeConfig(os, hopcxx_stacking_robot_q_start(&scenario));
    os << ",\n";
    os << "  \"actions\": [\n";
    for (size_t i = 0; i < chain.size(); i++) {
        const HopMotionNode<StackingDomain> *node = chain[i];
        const Action &act = plan[node->t];
        os << "    {\"type\": \"" << (act.type == ActionType::Pick ? "pick" : "place")
           << "\", \"object_index\": " << act.object_index << ", \"trajectory\": [";
        for (size_t w = 0; w < node->trajectory.size(); w++) {
            if (w) {
                os << ", ";
            }
            writeConfig(os, node->trajectory[w]);
        }
        os << "]";
        if (act.type == ActionType::Place) {
            os << ", \"placed_pose\": ";
            writePose(os, (*node->keyframe().after.poses)[act.object_index]);
        }
        os << "}";
        if (i + 1 < chain.size()) {
            os << ",";
        }
        os << "\n";
    }
    os << "  ]\n";
    os << "}\n";
}

int main(int argc, char **argv) {
    rai::initCmdLine(argc, argv);

    std::string robotName = rai::getParameter<rai::String>("robot", STRING("panda")).p;
    uint64_t seed = static_cast<uint64_t>(rai::getParameter<double>("seed", 0.));
    double timeout_s = rai::getParameter<double>("timeout", 30.);
    int stepCap = rai::getParameter<int>("stepCap", 200000000);
    int nodeCap = rai::getParameter<int>("nodeCap", 4000000);

    RobotTag robot = parseRobot(robotName);

    StackingScenario *rawScenario = nullptr;
    if (robot == RobotTag::Panda) {
        rawScenario = hopcxx_make_stacking_scenario_panda(seed);
    } else if (robot == RobotTag::Ur5) {
        rawScenario = hopcxx_make_stacking_scenario_ur5(seed);
    } else if (robot == RobotTag::Ur10e) {
        rawScenario = hopcxx_make_stacking_scenario_ur10e(seed);
    } else {
        rawScenario = hopcxx_make_stacking_scenario_pr2(seed);
    }
    ScenarioPtr<StackingScenario> scenario(rawScenario, hopcxx_stacking_free);
    std::string dumpPath = rai::getParameter<rai::String>("dumpSolution", STRING("")).p;

    auto root = std::make_shared<HopStackingSkeletonRoot>(*scenario, robot);
    rai::AStar astar(root, hopct::parseSearchMode());
    astar.verbose = 0;

    TrialResult result = runSearch(astar, timeout_s, stepCap, nodeCap);
    auto *solution
        = result.solved ? dynamic_cast<HopMotionNode<StackingDomain> *>(astar.solutions(0)) : nullptr;
    const std::vector<Action> *solutionPlan = solution ? &solution->ways.plan() : nullptr;
    size_t planLen = solutionPlan
        ? solutionPlan->size()
        : (root->skeletonPlans.empty() ? 0 : root->skeletonPlans[0]->size());

    printf("problem=stacking robot=%s seed=%llu solved=%d elapsed_s=%.4f steps=%u nodes=%u "
           "plan_len=%zu stop=%s skeleton_s=%.4f\n",
        robotName.c_str(), static_cast<unsigned long long>(seed), result.solved ? 1 : 0,
        result.elapsed_s, result.steps, result.nodes, planLen, result.stop, hopct::skeletonSeconds);

    if (solutionPlan && !dumpPath.empty()) {
        dumpSolution(dumpPath, robotName, seed, *scenario, *solutionPlan, motionChain(solution));
    }

    return 0;
}
