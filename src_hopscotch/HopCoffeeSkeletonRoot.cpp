#include "HopCoffeeSkeletonRoot.h"
#include <Core/graph.h>
#include <cstdlib>
#include <sstream>

namespace hopct {

namespace {

    size_t findItem(const CoffeeScenario &scenario, uint8_t kind, uint64_t index) {
        size_t n = hopcxx_coffee_num_objects(&scenario);
        for (size_t i = 0; i < n; i++) {
            if (hopcxx_coffee_object_kind(&scenario, i) == kind
                && hopcxx_coffee_object_index(&scenario, i) == index) {
                return i;
            }
        }
        HALT("coffee item (kind=" << (int)kind << ", index=" << index << ") not found in scenario");
    }

    // Domain text for coffee.
    const char *COFFEE_DOMAIN_TEXT = R"FOL(
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
cup
spoon
free
held
hasCoffee
hasSugar
hasCream
mixed
scooped

START_STATE {}

REWARD {
}

DecisionRule pick {
  Obj, Hand,
  { (agent Hand) (object Obj) (free Hand) (held Obj)! }
  { (free Hand)!
    (held Obj)
    }
}

DecisionRule place {
  Obj, Hand,
  { (agent Hand) (held Obj) }
  { (held Obj)!
    (free Hand)
    }
}

DecisionRule fill {
  Cup, Hand,
  { (agent Hand) (cup Cup) (held Cup) (hasCoffee Cup)! (mixed Cup)! }
  { (hasCoffee Cup)
    }
}

DecisionRule pour {
  Src, Dest, Hand,
  { (agent Hand) (cup Src) (cup Dest) (held Src) (hasCream Src) (mixed Dest)! }
  { (hasCoffee Src)!
    (hasSugar Src)!
    (hasCream Src)!
    (hasCream Dest)
    }
}

DecisionRule scoop {
  Spoon, Cup, Hand,
  { (agent Hand) (spoon Spoon) (cup Cup) (held Spoon) (hasSugar Cup) (scooped Spoon)! }
  { (scooped Spoon)
    }
}

DecisionRule dump {
  Spoon, Cup, Hand,
  { (agent Hand) (spoon Spoon) (cup Cup) (held Spoon) (hasSugar Cup)! (scooped Spoon) }
  { (scooped Spoon)!
    (hasSugar Cup)
    }
}

DecisionRule stir {
  Spoon, Cup, Hand,
  { (agent Hand) (spoon Spoon) (cup Cup) (held Spoon) (hasCoffee Cup) (hasSugar Cup) (hasCream Cup) }
  { (hasCoffee Cup)!
    (hasSugar Cup)!
    (hasCream Cup)!
    (mixed Cup)
    }
}
)FOL";

    CoffeeActionType ruleNameToType(const std::string &name) {
        if (name == "pick") {
            return CoffeeActionType::Pick;
        }
        if (name == "place") {
            return CoffeeActionType::Place;
        }
        if (name == "fill") {
            return CoffeeActionType::Fill;
        }
        if (name == "pour") {
            return CoffeeActionType::Pour;
        }
        if (name == "scoop") {
            return CoffeeActionType::Scoop;
        }
        if (name == "dump") {
            return CoffeeActionType::Dump;
        }
        if (name == "stir") {
            return CoffeeActionType::Stir;
        }
        HALT("unexpected coffee rule '" << name << "'");
    }

} // namespace

HopCoffeeSkeletonRoot::HopCoffeeSkeletonRoot(const CoffeeScenario &scenario, RobotTag robot)
    : ComputeNode(nullptr)
    , scenario(scenario)
    , robot(robot) {
    name << "HopCoffeeSkeletonRoot#0";
    isComplete = true;
    isFeasible = true;
    isTerminal = false;

    size_t n = hopcxx_coffee_num_objects(&scenario);
    size_t cup0 = findItem(scenario, 0, 0);
    size_t creamCup = findItem(scenario, 0, 1);
    size_t sugarCup0 = findItem(scenario, 0, 2);
    size_t sugarCup1 = findItem(scenario, 0, 3);
    size_t spoon = findItem(scenario, 1, 0);

    std::istringstream domainStream(COFFEE_DOMAIN_TEXT);
    rai::Graph kb(domainStream);
    L = std::make_shared<rai::FOL_World>();
    L->init(kb);

    L->addAgent("hand0");
    for (size_t i = 0; i < n; i++) {
        L->addObject(("obj" + std::to_string(i)).c_str());
    }
    L->addFact({ "cup", ("obj" + std::to_string(cup0)).c_str() });
    L->addFact({ "cup", ("obj" + std::to_string(creamCup)).c_str() });
    L->addFact({ "cup", ("obj" + std::to_string(sugarCup0)).c_str() });
    L->addFact({ "cup", ("obj" + std::to_string(sugarCup1)).c_str() });
    L->addFact({ "spoon", ("obj" + std::to_string(spoon)).c_str() });
    L->addFact({ "hasCream", ("obj" + std::to_string(creamCup)).c_str() });
    L->addFact({ "hasSugar", ("obj" + std::to_string(sugarCup0)).c_str() });
    L->addFact({ "hasSugar", ("obj" + std::to_string(sugarCup1)).c_str() });
    L->addTerminalRule(("(mixed obj" + std::to_string(cup0) + ")").c_str());
    L->reset_state();

    folAstar
        = std::make_shared<rai::AStar>(std::make_shared<rai::FOL_World_State>(*L, nullptr, false));
    folAstar->verbose = 0;
}

double HopCoffeeSkeletonRoot::branchingPenalty_child(int i) { return hopBranchingPenalty(i); }

const std::vector<CoffeeAction> *HopCoffeeSkeletonRoot::getOrBuildSkeleton(int i) {
    if (totalSkeletonCount >= 0 && i >= totalSkeletonCount) {
        return nullptr;
    }
    while ((int)skeletonPlans.size() <= i) {
        skeletonPlans.push_back(std::make_unique<std::vector<CoffeeAction>>());
    }
    std::vector<CoffeeAction> &plan = *skeletonPlans[i];
    if (!plan.empty()) {
        return &plan;
    }

    while ((int)folAstar->solutions.N <= i && folAstar->queue.N > 0) {
        folAstar->step();
    }
    if ((int)folAstar->solutions.N <= i) {
        totalSkeletonCount = (int)folAstar->solutions.N;
        return nullptr;
    }
    auto *sol = dynamic_cast<rai::FOL_World_State *>(folAstar->solutions(i));
    CHECK(sol, "expected a FOL_World_State solution from the symbolic search");
    rai::String dummy;
    rai::NodeL decisions = sol->getDecisionSequence(dummy);
    for (rai::Node *d : decisions) {
        std::string ruleName(d->parents(0)->key.p);
        CoffeeActionType type = ruleNameToType(ruleName);
        auto objIndexOf = [&](int subIdx) -> size_t {
            std::string objName(d->parents(1 + subIdx)->key.p);
            return (size_t)std::atoi(objName.c_str() + 3);
        };
        size_t itemIndex = objIndexOf(0);
        size_t targetIndex = 0;
        if (type == CoffeeActionType::Pour || type == CoffeeActionType::Scoop
            || type == CoffeeActionType::Dump || type == CoffeeActionType::Stir) {
            targetIndex = objIndexOf(1);
        }
        plan.push_back({ type, itemIndex, targetIndex });
    }
    CHECK(!plan.empty(), "FOL_World produced an empty skeleton");
    if (getenv("HOP_SKELETON_DEBUG")) {
        static const char *NAMES[] = { "pick", "place", "fill", "pour", "scoop", "dump", "stir" };
        fprintf(stderr, "coffee skeleton[%d] (len %zu):", i, plan.size());
        for (const CoffeeAction &a : plan) {
            fprintf(stderr, " %s(%zu,%zu)", NAMES[(int)a.type], a.item_index, a.target_index);
        }
        fprintf(stderr, "\n");
    }
    return &plan;
}

std::shared_ptr<rai::ComputeNode> HopCoffeeSkeletonRoot::createNewChild(int i) {
    const std::vector<CoffeeAction> *plan = getOrBuildSkeleton(i);
    if (!plan) {
        static const std::vector<CoffeeAction> EMPTY_PLAN;
        auto dead = std::make_shared<HopCoffeeNode>(scenario, EMPTY_PLAN, robot, this);
        dead->isFeasible = false;
        return dead;
    }
    return std::make_shared<HopCoffeeNode>(scenario, *plan, robot, this);
}

} // namespace hopct
