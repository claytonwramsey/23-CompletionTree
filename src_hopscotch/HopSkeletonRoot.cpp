#include "HopSkeletonRoot.h"
#include <Core/graph.h>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace hopct {

namespace {

    // Shared STRIPS-style domain for pick-and-place problems.
    const char *PICKPLACE_DOMAIN_TEXT = R"FOL(
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
)FOL";

} // namespace

HopSkeletonRoot::HopSkeletonRoot(const PickPlaceScenario &scenario, RobotTag robot)
    : ComputeNode(nullptr)
    , scenario(scenario)
    , robot(robot) {
    name << "HopSkeletonRoot#0";
    isComplete = true;
    isFeasible = true;
    isTerminal = false;

    size_t n = hopcxx_pickplace_num_objects(&scenario);
    size_t numSurfaces = hopcxx_pickplace_num_surfaces(&scenario);
    float block_r = hopcxx_pickplace_block_r(&scenario);
    int64_t targetId = hopcxx_pickplace_target_block(&scenario);
    int64_t goalSurface = hopcxx_pickplace_goal_surface(&scenario);
    CHECK(targetId >= 0 || goalSurface >= 0,
        "PickPlaceScenario must set target_block xor goal_surface");

    std::istringstream domainStream(PICKPLACE_DOMAIN_TEXT);
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
        CPose p = hopcxx_pickplace_object_pose(&scenario, i);
        size_t bestSurf = 0;
        float bestDist = INFINITY;
        for (size_t j = 0; j < numSurfaces; j++) {
            CTable t = hopcxx_pickplace_surface(&scenario, j);
            float dist = std::fabs(p.z - (t.height + block_r));
            if (dist < bestDist) {
                bestDist = dist;
                bestSurf = j;
            }
        }
        L->addFact({ "on", ("obj" + std::to_string(i)).c_str(),
            ("surf" + std::to_string(bestSurf)).c_str() });
    }

    if (targetId >= 0) {
        // cabinet: goal is "holding target_block".
        size_t targetIndex = n;
        for (size_t i = 0; i < n; i++) {
            if (static_cast<int64_t>(hopcxx_pickplace_object_id(&scenario, i)) == targetId) {
                targetIndex = i;
                break;
            }
        }
        CHECK(targetIndex < n, "target_block id not found among scenario objects");
        L->addTerminalRule(("(held obj" + std::to_string(targetIndex) + ")").c_str());
    } else {
        // packing: goal is "every object on goal_surface" -- a conjunction
        // of one `on` literal per object, all required by the same rule.
        std::string literals;
        for (size_t i = 0; i < n; i++) {
            literals
                += "(on obj" + std::to_string(i) + " surf" + std::to_string(goalSurface) + ") ";
        }
        L->addTerminalRule(literals.c_str());
    }
    L->reset_state();

    folAstar
        = std::make_shared<rai::AStar>(std::make_shared<rai::FOL_World_State>(*L, nullptr, false));
    folAstar->verbose = 0;
}

double HopSkeletonRoot::branchingPenalty_child(int i) { return hopBranchingPenalty(i); }

const std::vector<Action> *HopSkeletonRoot::getOrBuildSkeleton(int i) {
    if (totalSkeletonCount >= 0 && i >= totalSkeletonCount) {
        return nullptr; // already known to be beyond the finite symbolic space
    }

    while (static_cast<int>(skeletonPlans.size()) <= i) {
        skeletonPlans.push_back(std::make_unique<std::vector<Action>>());
    }
    std::vector<Action> &plan = *skeletonPlans[i];
    if (!plan.empty()) {
        return &plan;
    }

    // Step the symbolic search by hand instead of calling folAstar->run()
    // to avoid diverging.
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
        std::string objName(d->parents(1)->key.p);
        // strip the "obj" prefix added in the constructor above.
        size_t objIndex = static_cast<size_t>(std::atoi(objName.c_str() + 3));
        ActionType type = (ruleName == "pick") ? ActionType::Pick : ActionType::Place;
        if (type == ActionType::Place) {
            std::string surfName(d->parents(3)->key.p);
            size_t surfIndex = static_cast<size_t>(std::atoi(surfName.c_str() + 4));
            plan.push_back({ type, objIndex, surfIndex });
        } else {
            plan.push_back({ type, objIndex });
        }
    }
    CHECK(!plan.empty(), "FOL_World produced an empty skeleton");
    if (getenv("HOP_SKELETON_DEBUG")) {
        fprintf(stderr, "skeleton[%d] (len %zu):", i, plan.size());
        for (const Action &a : plan) {
            if (a.type == ActionType::Pick) {
                fprintf(stderr, " pick(%zu)", a.object_index);
            } else {
                fprintf(stderr, " place(%zu,surf%zu)", a.object_index, a.surface_index);
            }
        }
        fprintf(stderr, "\n");
    }
    return &plan;
}

std::shared_ptr<rai::ComputeNode> HopSkeletonRoot::createNewChild(int i) {
    const std::vector<Action> *plan = getOrBuildSkeleton(i);
    if (!plan) {
        // create fake child if asked to make a plan that doesn't exist
        static const std::vector<Action> EMPTY_PLAN;
        auto dead = std::make_shared<HopPickPlaceNode>(scenario, EMPTY_PLAN, robot, this);
        dead->isFeasible = false;
        return dead;
    }
    return std::make_shared<HopPickPlaceNode>(scenario, *plan, robot, this);
}

} // namespace hopct
