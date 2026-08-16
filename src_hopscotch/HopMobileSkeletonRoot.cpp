#include "HopMobileSkeletonRoot.h"
#include <Core/graph.h>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace hopct {

namespace {

    const char *MOBILE_DOMAIN_TEXT = R"FOL(
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
free
held
on

START_STATE {}

REWARD {
}

DecisionRule pick {
  Obj, Hand, Surf,
  { (agent Hand) (object Obj) (surface Surf) (free Hand) (held Obj)! (on Obj Surf) }
  { (free Hand)!
    (held Obj)
    (on Obj Surf)!
    }
}

DecisionRule place {
  Obj, Hand, Surf,
  { (agent Hand) (object Obj) (surface Surf) (held Obj) }
  { (held Obj)!
    (free Hand)
    (on Obj Surf)
    }
}

DecisionRule move {
  Hand
  { (agent Hand) }
  { }
}
)FOL";

} // namespace

HopMobileSkeletonRoot::HopMobileSkeletonRoot(const MobileScenario &scenario)
    : ComputeNode(nullptr)
    , scenario(scenario) {
    name << "HopMobileSkeletonRoot#0";
    isComplete = true;
    isFeasible = true;
    isTerminal = false;

    size_t n = hopcxx_mobile_num_objects(&scenario);
    size_t numSurfaces = hopcxx_mobile_num_surfaces(&scenario);
    float block_r = hopcxx_mobile_block_r(&scenario);
    size_t goalSurface = hopcxx_mobile_goal_surface(&scenario);

    std::istringstream domainStream(MOBILE_DOMAIN_TEXT);
    rai::Graph kb(domainStream);
    L = std::make_shared<rai::FOL_World>();
    L->init(kb);

    L->addAgent("hand0");
    for (size_t i = 0; i < n; i++) {
        L->addObject(("obj" + std::to_string(i)).c_str());
    }
    for (size_t j = 0; j < numSurfaces; j++) {
        L->addFact({ "surface", ("surf" + std::to_string(j)).c_str() });
    }
    // Initial `on` facts. Estimate surfaces based on their height.
    for (size_t i = 0; i < n; i++) {
        CPose p = hopcxx_mobile_object_pose(&scenario, i);
        size_t bestSurf = 0;
        float bestDist = INFINITY;
        for (size_t j = 0; j < numSurfaces; j++) {
            CTable t = hopcxx_mobile_surface(&scenario, j);
            float dist = std::fabs(p.z - (t.height + block_r));
            if (dist < bestDist) {
                bestDist = dist;
                bestSurf = j;
            }
        }
        L->addFact({ "on", ("obj" + std::to_string(i)).c_str(),
            ("surf" + std::to_string(bestSurf)).c_str() });
    }

    // goal: every object on goal_surface.
    std::string literals;
    for (size_t i = 0; i < n; i++) {
        literals += "(on obj" + std::to_string(i) + " surf" + std::to_string(goalSurface) + ") ";
    }
    L->addTerminalRule(literals.c_str());
    L->reset_state();

    folAstar
        = std::make_shared<rai::AStar>(std::make_shared<rai::FOL_World_State>(*L, nullptr, false));
    folAstar->verbose = 0;
}

double HopMobileSkeletonRoot::branchingPenalty_child(int i) { return hopBranchingPenalty(i); }

const std::vector<Action> *HopMobileSkeletonRoot::getOrBuildSkeleton(int i) {
    if (totalSkeletonCount >= 0 && i >= totalSkeletonCount) {
        return nullptr;
    }
    while (static_cast<int>(skeletonPlans.size()) <= i) {
        skeletonPlans.push_back(std::make_unique<std::vector<Action>>());
    }
    std::vector<Action> &plan = *skeletonPlans[i];
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
        if (ruleName == "move") {
            plan.push_back({ ActionType::Move, 0, 0 });
            continue;
        }
        std::string objName(d->parents(1)->key.p);
        size_t objIndex = static_cast<size_t>(std::atoi(objName.c_str() + 3));
        std::string surfName(d->parents(3)->key.p);
        size_t surfIndex = static_cast<size_t>(std::atoi(surfName.c_str() + 4));
        ActionType type = (ruleName == "pick") ? ActionType::Pick : ActionType::Place;

        if (type == ActionType::Place) {
            plan.push_back({ type, objIndex, surfIndex });
        } else {
            plan.push_back({ type, objIndex });
        }
    }
    CHECK(!plan.empty(), "FOL_World produced an empty skeleton");
    if (getenv("HOP_SKELETON_DEBUG")) {
        fprintf(stderr, "mobile skeleton[%d] (len %zu):", i, plan.size());
        for (const Action &a : plan) {
            if (a.type == ActionType::Pick) {
                fprintf(stderr, " pick(%zu)", a.object_index);
            } else if (a.type == ActionType::Place) {
                fprintf(stderr, " place(%zu,surf%zu)", a.object_index, a.surface_index);
            } else {
                fprintf(stderr, " move()");
            }
        }
        fprintf(stderr, "\n");
    }
    return &plan;
}

std::shared_ptr<rai::ComputeNode> HopMobileSkeletonRoot::createNewChild(int i) {
    const std::vector<Action> *plan = getOrBuildSkeleton(i);
    if (!plan) {
        static const std::vector<Action> EMPTY_PLAN;
        auto dead = std::make_shared<HopMobileNode>(scenario, EMPTY_PLAN, this);
        dead->isFeasible = false;
        return dead;
    }
    return std::make_shared<HopMobileNode>(scenario, *plan, this);
}

} // namespace hopct
