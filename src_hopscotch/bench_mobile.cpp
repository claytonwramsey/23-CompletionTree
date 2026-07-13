// Benchmark driver for the hopscotch-ported `mobile` domain (PR2) running
// on rai's own completion-tree search (rai::AStar over hopct::HopMobileNode).
// See bench_hopscotch.cpp (the `pickplace`-domain analog) for the CLI/CSV
// conventions this mirrors. No `-robot` flag: mobile is always PR2.
#include "HopBenchDriver.h"
#include "HopMobileNode.h"
#include <Core/util.h>
#include <Search/AStar.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

using namespace hopct;

static void dumpSolution(const std::string &path, uint64_t seed, const MobileScenario &scenario,
    const std::vector<Action> &plan, HopMobileNode *solutionNode) {
    std::vector<HopMobileNode *> chain;
    for (HopMobileNode *n = solutionNode; n->action_index >= 0;
        n = dynamic_cast<HopMobileNode *>(n->parent)) {
        chain.push_back(n);
    }
    std::reverse(chain.begin(), chain.end());

    std::ofstream os(path);
    os << "{\n";
    os << "  \"problem\": \"mobile\",\n";
    os << "  \"robot\": \"pr2\",\n";
    os << "  \"seed\": " << seed << ",\n";
    size_t n = hopcxx_mobile_num_objects(&scenario);
    os << "  \"objects\": [";
    for (size_t i = 0; i < n; i++) {
        if (i) {
            os << ", ";
        }
        os << "{\"id\": " << hopcxx_mobile_object_id(&scenario, i) << ", \"start_pose\": ";
        writePose(os, hopcxx_mobile_object_pose(&scenario, i));
        os << "}";
    }
    os << "],\n";
    size_t ns = hopcxx_mobile_num_surfaces(&scenario);
    os << "  \"surfaces\": [";
    for (size_t i = 0; i < ns; i++) {
        if (i) {
            os << ", ";
        }
        CTable t = hopcxx_mobile_surface(&scenario, i);
        os << "{\"height\": " << t.height << ", \"aabb\": [" << t.x0 << "," << t.y0 << "," << t.x1
           << "," << t.y1 << "]}";
    }
    os << "],\n";
    os << "  \"block_r\": " << hopcxx_mobile_block_r(&scenario) << ",\n";
    os << "  \"q_start\": ";
    writeConfig(os, hopcxx_mobile_robot_q_start_arm(&scenario));
    os << ",\n";
    os << "  \"base_start_pose\": ";
    writePose(os, hopcxx_mobile_base_start_pose(&scenario));
    os << ",\n";
    os << "  \"actions\": [\n";
    for (size_t i = 0; i < chain.size(); i++) {
        HopMobileNode *node = chain[i];
        const Action &act = plan[node->action_index];
        os << "    {\"type\": \"" << (act.type == ActionType::Pick ? "pick" : "place")
           << "\", \"object_index\": " << act.object_index << ", \"base_pose\": ";
        writePose(os, node->base_pose);
        os << ", \"trajectory\": [";
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

    uint64_t seed = (uint64_t)rai::getParameter<double>("seed", 0.);
    double timeout_s = rai::getParameter<double>("timeout", 30.);
    int stepCap = rai::getParameter<int>("stepCap", 200000000);
    int nodeCap = rai::getParameter<int>("nodeCap", 4000000);

    ScenarioPtr<MobileScenario> scenario(hopcxx_make_mobile_scenario(seed), hopcxx_mobile_free);
    std::vector<Action> plan = makeMobilePlan(*scenario);

    auto root = std::make_shared<HopMobileNode>(*scenario, plan);
    rai::AStar astar(root, rai::AStar::astar);
    astar.verbose = 0;

    TrialResult result = runSearch(astar, timeout_s, stepCap, nodeCap);

    printf("problem=mobile robot=pr2 seed=%llu solved=%d elapsed_s=%.4f steps=%u nodes=%u "
           "plan_len=%zu\n",
        (unsigned long long)seed, result.solved ? 1 : 0, result.elapsed_s, result.steps,
        result.nodes, plan.size());

    std::string dumpPath = rai::getParameter<rai::String>("dumpSolution", STRING("")).p;
    if (result.solved && !dumpPath.empty()) {
        dumpSolution(
            dumpPath, seed, *scenario, plan, dynamic_cast<HopMobileNode *>(astar.solutions(0)));
    }

    return 0;
}
