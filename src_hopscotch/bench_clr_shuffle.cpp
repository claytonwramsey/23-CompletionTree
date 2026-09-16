// Benchmark driver for the hopscotch-ported `clr_shuffle` domain running on rai's own
// completion-tree search, rai::AStar over hopct::HopClrShuffleNode. Mirrors the CLI/CSV
// conventions of hopscotch's and the PDDLStream port's benchmark harnesses, one trial per
// process with a wall-clock timeout and solved/elapsed_s/steps/nodes columns, so the output
// merges into the same comparison pipeline.
#include "HopBenchDriver.h"
#include "HopClrShuffleNode.h"
#include "HopClrShuffleSkeletonRoot.h"
#include <Core/util.h>
#include <Search/AStar.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

using namespace hopct;

// Writes the solved plan to a JSON file for offline visualization, enough structure to replay
// the trajectory plus which CTB moved where.
static void dumpSolution(const std::string &path, uint64_t seed,
    const ClrShuffleScenario &scenario, const std::vector<ClrAction> &plan,
    HopClrShuffleNode *solutionNode) {
    std::vector<HopClrShuffleNode *> chain;
    for (HopClrShuffleNode *n = solutionNode; n->action_index >= 0;
        n = dynamic_cast<HopClrShuffleNode *>(n->parent)) {
        chain.push_back(n);
    }
    std::reverse(chain.begin(), chain.end());

    std::ofstream os(path);
    os << "{\n";
    os << "  \"problem\": \"clr_shuffle\",\n";
    os << "  \"robot\": \"ur10e\",\n";
    os << "  \"seed\": " << seed << ",\n";
    size_t n = hopcxx_clr_shuffle_num_ctbs(&scenario);
    os << "  \"objects\": [";
    for (size_t i = 0; i < n; i++) {
        if (i) {
            os << ", ";
        }
        os << "{\"id\": " << i << ", \"start_pose\": ";
        writePose(os, hopcxx_clr_shuffle_ctb_pose(&scenario, i));
        os << "}";
    }
    os << "],\n";
    float half[3];
    hopcxx_clr_shuffle_ctb_half_extents(&scenario, half);
    os << "  \"ctb_half_extents\": [" << half[0] << "," << half[1] << "," << half[2] << "],\n";
    os << "  \"q_start\": ";
    writeConfig(os, hopcxx_clr_shuffle_robot_q_start(&scenario));
    os << ",\n";
    os << "  \"actions\": [\n";
    static const char *NAMES[] = { "pick", "place", "open_lid", "close_lid" };
    for (size_t i = 0; i < chain.size(); i++) {
        HopClrShuffleNode *node = chain[i];
        const ClrAction &act = plan[node->action_index];
        os << "    {\"type\": \"" << NAMES[static_cast<int>(act.type)]
           << "\", \"ctb_index\": " << act.ctb_index
           << ", \"surface_index\": " << act.surface_index << ", \"trajectory\": [";
        for (size_t w = 0; w < node->trajectory.size(); w++) {
            if (w) {
                os << ", ";
            }
            writeConfig(os, node->trajectory[w]);
        }
        os << "]";
        if (act.type == ClrActionType::Place) {
            os << ", \"placed_pose\": ";
            writePose(os, (*node->poses)[act.ctb_index]);
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

    std::string robotName = rai::getParameter<rai::String>("robot", STRING("ur10e")).p;
    if (robotName != "ur10e") {
        HALT("the CLR shuffle scenario only supports the ur10e, got '" << robotName << "'");
    }
    uint64_t seed = static_cast<uint64_t>(rai::getParameter<double>("seed", 0.));
    double timeout_s = rai::getParameter<double>("timeout", 30.);
    int stepCap = rai::getParameter<int>("stepCap", 200000000);
    int nodeCap = rai::getParameter<int>("nodeCap", 4000000);
    std::string dumpPath = rai::getParameter<rai::String>("dumpSolution", STRING("")).p;

    ScenarioPtr<ClrShuffleScenario> scenario(
        hopcxx_make_clr_shuffle_scenario_ur10e(seed), hopcxx_clr_shuffle_free);

    auto root = std::make_shared<HopClrShuffleSkeletonRoot>(*scenario);
    root->forcedSkeleton = rai::getParameter<int>("skeleton", -1);
    rai::AStar astar(root, rai::AStar::astar);
    astar.verbose = 0;

    TrialResult result = runSearch(astar, timeout_s, stepCap, nodeCap);
    HopClrShuffleNode *solutionNode
        = result.solved ? dynamic_cast<HopClrShuffleNode *>(astar.solutions(0)) : nullptr;
    size_t planLen = solutionNode
        ? solutionNode->plan.size()
        : (root->skeletonPlans.empty() ? 0 : root->skeletonPlans[0]->size());

    printf("problem=clr_shuffle robot=ur10e seed=%llu solved=%d elapsed_s=%.4f steps=%u nodes=%u "
           "plan_len=%zu stop=%s\n",
        static_cast<unsigned long long>(seed), result.solved ? 1 : 0, result.elapsed_s,
        result.steps, result.nodes, planLen, result.stop);

    if (solutionNode && !dumpPath.empty()) {
        dumpSolution(dumpPath, seed, *scenario, solutionNode->plan, solutionNode);
    }

    if (rai::getParameter<bool>("diagStats", false)) {
        static const char *NAMES[] = { "pick", "place", "open", "close" };
        const std::vector<ClrAction> *diagPlan = solutionNode ? &solutionNode->plan
            : (root->skeletonPlans.empty() ? nullptr : root->skeletonPlans[0].get());
        printf("-- per-action diagnostic (attempts / ikOk / validateOk / motionOk):\n");
        for (size_t i = 0; i < HopDiagStats::MAX_ACTIONS && diagStats().attempts[i] > 0; i++) {
            const char *label = diagPlan && i < diagPlan->size()
                ? NAMES[static_cast<int>((*diagPlan)[i].type)]
                : "?";
            printf("  [%zu] %-5s attempts=%lld ikOk=%lld validateOk=%lld motionOk=%lld\n", i,
                label, diagStats().attempts[i], diagStats().ikOk[i], diagStats().validateOk[i],
                diagStats().motionOk[i]);
        }
    }

    return 0;
}
