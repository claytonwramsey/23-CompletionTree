// Benchmark driver for the hopscotch-ported `coffee` domain running on
// rai's own completion-tree search (rai::AStar over hopct::HopCoffeeNode).
// See bench_hopscotch.cpp (the `pickplace`-domain analog) for the CLI/CSV
// conventions this mirrors.
#include "HopBenchDriver.h"
#include "HopCoffeeNode.h"
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

static const char *actionName(CoffeeActionType t) {
    switch (t) {
    case CoffeeActionType::Pick:
        return "pick";
    case CoffeeActionType::Place:
        return "place";
    case CoffeeActionType::Fill:
        return "fill";
    case CoffeeActionType::Pour:
        return "pour";
    case CoffeeActionType::Scoop:
        return "scoop";
    case CoffeeActionType::Dump:
        return "dump";
    case CoffeeActionType::Stir:
        return "stir";
    }
    return "?";
}

static void dumpSolution(const std::string &path, const std::string &robotName, uint64_t seed,
    const CoffeeScenario &scenario, const std::vector<CoffeeAction> &plan,
    HopCoffeeNode *solutionNode) {
    std::vector<HopCoffeeNode *> chain;
    for (HopCoffeeNode *n = solutionNode; n->action_index >= 0;
        n = dynamic_cast<HopCoffeeNode *>(n->parent)) {
        chain.push_back(n);
    }
    std::reverse(chain.begin(), chain.end());

    std::ofstream os(path);
    os << "{\n";
    os << "  \"problem\": \"coffee\",\n";
    os << "  \"robot\": \"" << robotName << "\",\n";
    os << "  \"seed\": " << seed << ",\n";
    size_t n = hopcxx_coffee_num_objects(&scenario);
    os << "  \"objects\": [";
    for (size_t i = 0; i < n; i++) {
        if (i) {
            os << ", ";
        }
        os << "{\"kind\": " << (int)hopcxx_coffee_object_kind(&scenario, i)
           << ", \"index\": " << hopcxx_coffee_object_index(&scenario, i) << ", \"start_pose\": ";
        writePose(os, hopcxx_coffee_object_pose(&scenario, i));
        os << "}";
    }
    os << "],\n";
    CTable t = hopcxx_coffee_table(&scenario);
    os << "  \"surfaces\": [{\"height\": " << t.height << ", \"aabb\": [" << t.x0 << "," << t.y0
       << "," << t.x1 << "," << t.y1 << "]}],\n";
    os << "  \"object_r\": " << hopcxx_coffee_object_r(&scenario) << ",\n";
    os << "  \"q_start\": ";
    writeConfig(os, hopcxx_coffee_robot_q_start(&scenario));
    os << ",\n";
    os << "  \"actions\": [\n";
    for (size_t i = 0; i < chain.size(); i++) {
        HopCoffeeNode *node = chain[i];
        const CoffeeAction &act = plan[node->action_index];
        os << "    {\"type\": \"" << actionName(act.type)
           << "\", \"item_index\": " << act.item_index << ", \"trajectory\": [";
        for (size_t w = 0; w < node->trajectory.size(); w++) {
            if (w) {
                os << ", ";
            }
            writeConfig(os, node->trajectory[w]);
        }
        os << "]";
        if (act.type == CoffeeActionType::Place) {
            os << ", \"placed_pose\": ";
            writePose(os, (*node->poses)[act.item_index]);
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

    CoffeeScenario *rawScenario = nullptr;
    if (robot == RobotTag::Panda) {
        rawScenario = hopcxx_make_coffee_scenario_panda(seed);
    } else if (robot == RobotTag::Ur5) {
        rawScenario = hopcxx_make_coffee_scenario_ur5(seed);
    } else {
        rawScenario = hopcxx_make_coffee_scenario_pr2(seed);
    }
    ScenarioPtr<CoffeeScenario> scenario(rawScenario, hopcxx_coffee_free);

    std::vector<CoffeeAction> plan = makeCoffeePlan(*scenario);

    auto root = std::make_shared<HopCoffeeNode>(*scenario, plan, robot);
    rai::AStar astar(root, rai::AStar::astar);
    astar.verbose = 0;

    TrialResult result = runSearch(astar, timeout_s, stepCap, nodeCap);

    printf("problem=coffee robot=%s seed=%llu solved=%d elapsed_s=%.4f steps=%u nodes=%u "
           "plan_len=%zu\n",
        robotName.c_str(), (unsigned long long)seed, result.solved ? 1 : 0, result.elapsed_s,
        result.steps, result.nodes, plan.size());

    std::string dumpPath = rai::getParameter<rai::String>("dumpSolution", STRING("")).p;
    if (result.solved && !dumpPath.empty()) {
        dumpSolution(dumpPath, robotName, seed, *scenario, plan,
            dynamic_cast<HopCoffeeNode *>(astar.solutions(0)));
    }

    if (rai::getParameter<bool>("diagStats", false)) {
        printf("-- per-action diagnostic (attempts / ikOk / validateOk / motionOk):\n");
        for (size_t i = 0; i < plan.size(); i++) {
            printf("  [%zu] %-6s item=%zu: attempts=%lld ikOk=%lld validateOk=%lld motionOk=%lld\n",
                i, actionName(plan[i].type), plan[i].item_index, diagStats().attempts[i],
                diagStats().ikOk[i], diagStats().validateOk[i], diagStats().motionOk[i]);
        }
    }

    return 0;
}
