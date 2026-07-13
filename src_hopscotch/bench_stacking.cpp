// Benchmark driver for the hopscotch-ported `stacking` domain running on
// rai's own completion-tree search (rai::AStar over hopct::HopStackingNode).
// See bench_hopscotch.cpp (the `pickplace`-domain analog) for the CLI/CSV
// conventions this mirrors.
#include "HopBenchDriver.h"
#include "HopStackingNode.h"
#include <Core/util.h>
#include <Search/AStar.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

using namespace hopct;

static RobotTag parseRobot(const char *s) {
    if (!strcmp(s, "panda")) {
        return RobotTag::Panda;
    }
    if (!strcmp(s, "ur5")) {
        return RobotTag::Ur5;
    }
    if (!strcmp(s, "pr2")) {
        return RobotTag::Pr2;
    }
    HALT("unknown robot '" << s << "'");
}

static void dumpSolution(const std::string &path, const std::string &robotName, uint64_t seed,
    const StackingScenario &scenario, const std::vector<Action> &plan,
    HopStackingNode *solutionNode) {
    std::vector<HopStackingNode *> chain;
    for (HopStackingNode *n = solutionNode; n->action_index >= 0;
        n = dynamic_cast<HopStackingNode *>(n->parent)) {
        chain.push_back(n);
    }
    std::reverse(chain.begin(), chain.end());

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
        HopStackingNode *node = chain[i];
        const Action &act = plan[node->action_index];
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
            writePose(os, (*node->poses)[act.object_index]);
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
    uint64_t seed = (uint64_t)rai::getParameter<double>("seed", 0.);
    double timeout_s = rai::getParameter<double>("timeout", 30.);
    int stepCap = rai::getParameter<int>("stepCap", 200000000);
    int nodeCap = rai::getParameter<int>("nodeCap", 4000000);

    RobotTag robot = parseRobot(robotName.c_str());

    StackingScenario *rawScenario = nullptr;
    if (robot == RobotTag::Panda) {
        rawScenario = hopcxx_make_stacking_scenario_panda(seed);
    } else if (robot == RobotTag::Ur5) {
        rawScenario = hopcxx_make_stacking_scenario_ur5(seed);
    } else {
        rawScenario = hopcxx_make_stacking_scenario_pr2(seed);
    }
    ScenarioPtr<StackingScenario> scenario(rawScenario, hopcxx_stacking_free);

    std::vector<Action> plan = makeStackingPlan(*scenario);

    auto root = std::make_shared<HopStackingNode>(*scenario, plan, robot);
    rai::AStar astar(root, rai::AStar::astar);
    astar.verbose = 0;

    TrialResult result = runSearch(astar, timeout_s, stepCap, nodeCap);

    printf("problem=stacking robot=%s seed=%llu solved=%d elapsed_s=%.4f steps=%u nodes=%u "
           "plan_len=%zu\n",
        robotName.c_str(), (unsigned long long)seed, result.solved ? 1 : 0, result.elapsed_s,
        result.steps, result.nodes, plan.size());

    std::string dumpPath = rai::getParameter<rai::String>("dumpSolution", STRING("")).p;
    if (result.solved && !dumpPath.empty()) {
        dumpSolution(dumpPath, robotName, seed, *scenario, plan,
            dynamic_cast<HopStackingNode *>(astar.solutions(0)));
    }

    return 0;
}
