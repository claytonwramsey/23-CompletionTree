#include "HopClrShuffleSkeletonRoot.h"
#include <Core/graph.h>
#include <cstdlib>
#include <sstream>

namespace hopct {

namespace {

    // Domain text for clr_shuffle. Picking from or placing onto a surface the lid covers needs
    // the lid open, so each of pick and place splits into a gated and an ungated rule.
    const char *CLR_SHUFFLE_DOMAIN_TEXT = R"FOL(
QUIT
WAIT
ANY
Terminate

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

agent
object
surface
lid
free
held
on
gated
open

START_STATE {}

REWARD {
}

DecisionRule pickGated {
  Obj, Surf, Lid, Hand,
  { (agent Hand) (object Obj) (surface Surf) (lid Lid) (free Hand) (on Obj Surf) (gated Surf) (open Lid) }
  { (on Obj Surf)!
    (free Hand)!
    (held Obj)
    }
}

DecisionRule pickOpen {
  Obj, Surf, Hand,
  { (agent Hand) (object Obj) (surface Surf) (free Hand) (on Obj Surf) (gated Surf)! }
  { (on Obj Surf)!
    (free Hand)!
    (held Obj)
    }
}

DecisionRule placeGated {
  Obj, Surf, Lid, Hand,
  { (agent Hand) (object Obj) (surface Surf) (lid Lid) (held Obj) (gated Surf) (open Lid) }
  { (held Obj)!
    (free Hand)
    (on Obj Surf)
    }
}

DecisionRule placeOpen {
  Obj, Surf, Hand,
  { (agent Hand) (object Obj) (surface Surf) (held Obj) (gated Surf)! }
  { (held Obj)!
    (free Hand)
    (on Obj Surf)
    }
}

DecisionRule openLid {
  Lid, Hand,
  { (agent Hand) (lid Lid) (free Hand) (open Lid)! }
  { (open Lid)
    }
}

DecisionRule closeLid {
  Lid, Hand,
  { (agent Hand) (lid Lid) (free Hand) (open Lid) }
  { (open Lid)!
    }
}
)FOL";

    size_t indexAfterPrefix(rai::Node *node, size_t prefixLen) {
        std::string symbol(node->key.p);
        return static_cast<size_t>(std::atoi(symbol.c_str() + prefixLen));
    }

} // namespace

HopClrShuffleSkeletonRoot::HopClrShuffleSkeletonRoot(const ClrShuffleScenario &scenario)
    : ComputeNode(nullptr)
    , scenario(scenario) {
    name << "HopClrShuffleSkeletonRoot#0";
    isComplete = true;
    isFeasible = true;
    isTerminal = false;

    std::istringstream domainStream(CLR_SHUFFLE_DOMAIN_TEXT);
    rai::Graph kb(domainStream);
    L = std::make_shared<rai::FOL_World>();
    L->init(kb);

    L->addAgent("hand0");
    size_t nCtbs = hopcxx_clr_shuffle_num_ctbs(&scenario);
    for (size_t i = 0; i < nCtbs; i++) {
        L->addObject(("obj" + std::to_string(i)).c_str());
    }
    size_t nSurfaces = hopcxx_clr_shuffle_num_surfaces(&scenario);
    for (size_t i = 0; i < nSurfaces; i++) {
        std::string surf = "surf" + std::to_string(i);
        L->addObject(surf.c_str());
        L->addFact({ "surface", surf.c_str() });
        if (hopcxx_clr_shuffle_surface_lid_gated(&scenario, i)) {
            L->addFact({ "gated", surf.c_str() });
        }
    }
    L->addObject("lid0");
    L->addFact({ "lid", "lid0" });
    for (size_t i = 0; i < nCtbs; i++) {
        std::string surf = "surf" + std::to_string(hopcxx_clr_shuffle_ctb_surface(&scenario, i));
        L->addFact({ "on", ("obj" + std::to_string(i)).c_str(), surf.c_str() });
    }
    L->addTerminalRule(("(on obj" + std::to_string(hopcxx_clr_shuffle_goal_ctb(&scenario)) + " surf"
        + std::to_string(hopcxx_clr_shuffle_goal_surface(&scenario)) + ")")
                           .c_str());
    L->reset_state();

    folAstar
        = std::make_shared<rai::AStar>(std::make_shared<rai::FOL_World_State>(*L, nullptr, false));
    folAstar->verbose = 0;
}

double HopClrShuffleSkeletonRoot::branchingPenalty_child(int i) { return hopBranchingPenalty(i); }

const std::vector<ClrAction> *HopClrShuffleSkeletonRoot::getOrBuildSkeleton(int i) {
    if (totalSkeletonCount >= 0 && i >= totalSkeletonCount) {
        return nullptr;
    }
    while (static_cast<int>(skeletonPlans.size()) <= i) {
        skeletonPlans.push_back(std::make_unique<std::vector<ClrAction>>());
    }
    std::vector<ClrAction> &plan = *skeletonPlans[i];
    if (!plan.empty()) {
        return &plan;
    }

    while (static_cast<int>(folAstar->solutions.N) <= i && folAstar->queue.N > 0) {
        folAstar->step();
    }
    if (static_cast<int>(folAstar->solutions.N) <= i) {
        totalSkeletonCount = static_cast<int>(folAstar->solutions.N);
        return nullptr;
    }
    auto *sol = dynamic_cast<rai::FOL_World_State *>(folAstar->solutions(i));
    CHECK(sol, "expected a FOL_World_State solution from the symbolic search");
    rai::String dummy;
    rai::NodeL decisions = sol->getDecisionSequence(dummy);
    for (rai::Node *d : decisions) {
        std::string ruleName(d->parents(0)->key.p);
        if (ruleName == "openLid") {
            plan.push_back({ ClrActionType::OpenLid, 0, 0 });
        } else if (ruleName == "closeLid") {
            plan.push_back({ ClrActionType::CloseLid, 0, 0 });
        } else if (ruleName == "pickGated" || ruleName == "pickOpen") {
            plan.push_back({ ClrActionType::Pick, indexAfterPrefix(d->parents(1), 3),
                indexAfterPrefix(d->parents(2), 4) });
        } else if (ruleName == "placeGated" || ruleName == "placeOpen") {
            plan.push_back({ ClrActionType::Place, indexAfterPrefix(d->parents(1), 3),
                indexAfterPrefix(d->parents(2), 4) });
        } else {
            HALT("unexpected clr_shuffle rule '" << ruleName << "'");
        }
    }
    CHECK(!plan.empty(), "FOL_World produced an empty skeleton");
    if (getenv("HOP_SKELETON_DEBUG")) {
        static const char *NAMES[] = { "pick", "place", "openLid", "closeLid" };
        fprintf(stderr, "clr_shuffle skeleton[%d] (len %zu):", i, plan.size());
        for (const ClrAction &a : plan) {
            fprintf(stderr, " %s(%zu,%zu)", NAMES[static_cast<int>(a.type)], a.ctb_index,
                a.surface_index);
        }
        fprintf(stderr, "\n");
    }
    return &plan;
}

std::shared_ptr<rai::ComputeNode> HopClrShuffleSkeletonRoot::createNewChild(int i) {
    const std::vector<ClrAction> *plan = getOrBuildSkeleton(forcedSkeleton >= 0 ? forcedSkeleton : i);
    if (!plan) {
        static const std::vector<ClrAction> EMPTY_PLAN;
        auto dead = std::make_shared<HopClrShuffleNode>(scenario, EMPTY_PLAN, this);
        dead->isFeasible = false;
        return dead;
    }
    return std::make_shared<HopClrShuffleNode>(scenario, *plan, this);
}

} // namespace hopct
