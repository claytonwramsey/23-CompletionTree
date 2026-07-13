// Benchmark driver for the hopscotch-ported `pickplace` domain (cabinet,
// packing) running on rai's own completion-tree search (rai::AStar over
// hopct::HopPickPlaceNode). Mirrors the CLI/CSV conventions of hopscotch's
// and the PDDLStream port's own benchmark harnesses (one trial per process,
// wall-clock timeout, solved/elapsed_s/steps/nodes columns) so the output
// merges into the same comparison pipeline.
#include "HopBenchDriver.h"
#include "HopPickPlaceNode.h"
#include <Core/util.h>
#include <Search/AStar.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>

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

// Writes the solved plan (root -> ... -> terminal solution node) to a JSON
// file for offline visualization (see dump_solution's Python consumer,
// common/replay_completiontree.py) -- not a general-purpose serializer, just
// enough structure to replay the trajectory + which object moved where.
static void dumpSolution(const char *path, const rai::String &problem, const rai::String &robotName,
    uint64_t seed, const PickPlaceScenario *scenario, const std::vector<Action> &plan,
    HopPickPlaceNode *solutionNode) {
    std::vector<HopPickPlaceNode *> chain;
    for (HopPickPlaceNode *n = solutionNode; n->action_index >= 0;
        n = dynamic_cast<HopPickPlaceNode *>(n->parent)) {
        chain.push_back(n);
    }
    std::reverse(chain.begin(), chain.end());

    std::ofstream os(path);
    os << "{\n";
    os << "  \"problem\": \"" << problem.p << "\",\n";
    os << "  \"robot\": \"" << robotName.p << "\",\n";
    os << "  \"seed\": " << seed << ",\n";
    size_t n = hopcxx_pickplace_num_objects(scenario);
    os << "  \"objects\": [";
    for (size_t i = 0; i < n; i++) {
        if (i) {
            os << ", ";
        }
        os << "{\"id\": " << hopcxx_pickplace_object_id(scenario, i) << ", \"start_pose\": ";
        writePose(os, hopcxx_pickplace_object_pose(scenario, i));
        os << "}";
    }
    os << "],\n";
    size_t ns = hopcxx_pickplace_num_surfaces(scenario);
    os << "  \"surfaces\": [";
    for (size_t i = 0; i < ns; i++) {
        if (i) {
            os << ", ";
        }
        CTable t = hopcxx_pickplace_surface(scenario, i);
        os << "{\"height\": " << t.height << ", \"aabb\": [" << t.x0 << "," << t.y0 << "," << t.x1
           << "," << t.y1 << "]}";
    }
    os << "],\n";
    os << "  \"block_r\": " << hopcxx_pickplace_block_r(scenario) << ",\n";
    os << "  \"q_start\": ";
    writeConfig(os, hopcxx_pickplace_robot_q_start(scenario));
    os << ",\n";
    os << "  \"actions\": [\n";
    for (size_t i = 0; i < chain.size(); i++) {
        HopPickPlaceNode *node = chain[i];
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

    rai::String problem = rai::getParameter<rai::String>("problem", STRING("cabinet"));
    rai::String robotName = rai::getParameter<rai::String>("robot", STRING("panda"));
    uint64_t seed = (uint64_t)rai::getParameter<double>("seed", 0.);
    double timeout_s = rai::getParameter<double>("timeout", 30.);
    // A step cap is still useful as a hard backstop, but the intended gate is
    // wall-clock time (matching the 30s-timeout convention used for both
    // hopscotch's own and the PDDLStream port's benchmark sweeps) -- set high
    // enough that fast scenarios never hit it before the timeout does.
    int stepCap = rai::getParameter<int>("stepCap", 200000000);
    // astar.mem never frees a node (same design as rai's own LGP search), so
    // an unsolved trial's RSS grows without bound over a 30s timeout (observed
    // ~10GB for one packing/ur5 DNF). Cap node count too, so a sweep with
    // several trials in flight can't exhaust memory; hitting this cap is
    // recorded identically to a wall-clock timeout (both just mean "did not
    // solve in the allotted search").
    int nodeCap = rai::getParameter<int>("nodeCap", 4000000);

    RobotTag robot = parseRobot(robotName.p);

    PickPlaceScenario *scenario = nullptr;
    if (problem == "cabinet") {
        if (robot == RobotTag::Panda) {
            scenario = hopcxx_make_cabinet_scenario_panda(seed);
        } else if (robot == RobotTag::Ur5) {
            scenario = hopcxx_make_cabinet_scenario_ur5(seed);
        } else {
            scenario = hopcxx_make_cabinet_scenario_pr2(seed);
        }
    } else if (problem == "packing") {
        if (robot == RobotTag::Panda) {
            scenario = hopcxx_make_packing_scenario_panda(seed);
        } else if (robot == RobotTag::Ur5) {
            scenario = hopcxx_make_packing_scenario_ur5(seed);
        } else {
            scenario = hopcxx_make_packing_scenario_pr2(seed);
        }
    } else {
        HALT("unknown problem '" << problem << "' (this driver supports cabinet/packing)");
    }

    std::vector<Action> plan = makePickPlacePlan(scenario);

    auto root = std::make_shared<HopPickPlaceNode>(scenario, &plan, robot);
    rai::AStar astar(root, rai::AStar::astar);
    astar.verbose = 0;

    TrialResult result = runSearch(astar, timeout_s, stepCap, nodeCap);

    printf(
        "problem=%s robot=%s seed=%llu solved=%d elapsed_s=%.4f steps=%u nodes=%u plan_len=%zu\n",
        problem.p, robotName.p, (unsigned long long)seed, result.solved ? 1 : 0, result.elapsed_s,
        result.steps, result.nodes, plan.size());

    rai::String dumpPath = rai::getParameter<rai::String>("dumpSolution", STRING(""));
    if (result.solved && dumpPath.N) {
        dumpSolution(dumpPath.p, problem, robotName, seed, scenario, plan,
            dynamic_cast<HopPickPlaceNode *>(astar.solutions(0)));
    }

    hopcxx_pickplace_free(scenario);
    return 0;
}
