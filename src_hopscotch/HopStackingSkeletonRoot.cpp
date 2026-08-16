#include "HopStackingSkeletonRoot.h"
#include <Core/graph.h>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace hopct {

namespace {

    size_t findObjectIndex(const StackingScenario &scenario, uint64_t id) {
        size_t n = hopcxx_stacking_num_objects(&scenario);
        for (size_t i = 0; i < n; i++) {
            if (hopcxx_stacking_object_id(&scenario, i) == id) {
                return i;
            }
        }
        HALT("goal_order object id not found among scenario objects");
    }

    // Static domain text.
    const char *STACKING_DOMAIN_TEXT = R"FOL(
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
  Obj, Hand, Below,
  { (agent Hand) (object Obj) (free Hand) (held Obj)! (on Obj Below) }
  { (free Hand)!
    (held Obj)
    (on Obj Below)!
    }
}

DecisionRule place {
  Obj, Hand, Below,
  { (agent Hand) (object Obj) (surface Below) (held Obj) }
  { (held Obj)!
    (free Hand)
    (on Obj Below)
    }
}
)FOL";

    // A FOL world with a search heuristic based on the number of satisfied goal propositions.
    struct GoalCountFolState : rai::FOL_World_State {
        std::shared_ptr<const std::vector<std::pair<size_t, size_t>>>
            goalChain; // (obj, below) pairs
        std::shared_ptr<const std::vector<bool>> satisfied; // per goalChain entry, copy-on-write

        GoalCountFolState(rai::FOL_World &L, TreeSearchNode *parent, bool isTerminal,
            std::shared_ptr<const std::vector<std::pair<size_t, size_t>>> goalChain,
            std::shared_ptr<const std::vector<bool>> satisfied)
            : FOL_World_State(L, parent, isTerminal)
            , goalChain(goalChain)
            , satisfied(satisfied) { }

        std::shared_ptr<rai::TreeSearchNode> transition(int action) override {
            CHECK_GE(action, 0, "");
            if (static_cast<uint>(action) < children.N && children(action)) {
                HALT("duplicate transition call");
            }
            if (L.state != state) {
                L.setState(state, T_step);
            }
            L.T_real = T_real;
            CHECK_LE(1 + static_cast<uint>(action), actions.N, "that action doesn't exist");

            // Figure out this decision's effect on goalChain .
            auto newSatisfied = std::make_shared<std::vector<bool>>(*satisfied);
            const auto *d = dynamic_cast<const rai::FOL_World::Decision *>(actions(action).get());
            if (d && !d->waitDecision && d->substitution.N >= 3) {
                std::string ruleName(d->rule->key.p);
                const char *objName = d->substitution(0)->key.p;
                const char *belowName = d->substitution(2)->key.p;
                if ((ruleName == "pick" || ruleName == "place") && !strncmp(objName, "obj", 3)
                    && !strncmp(belowName, "obj", 3)) {
                    size_t objIndex = static_cast<size_t>(std::atoi(objName + 3));
                    size_t belowIndex = static_cast<size_t>(std::atoi(belowName + 3));
                    for (size_t k = 0; k < goalChain->size(); k++) {
                        if ((*goalChain)[k].first == objIndex
                            && (*goalChain)[k].second == belowIndex) {
                            (*newSatisfied)[k] = (ruleName == "place");
                        }
                    }
                }
            }

            L.transition(actions(action));
            CHECK(L.state != state, "");
            size_t unsatisfied = 0;
            for (bool b : *newSatisfied) {
                if (!b) {
                    unsatisfied++;
                }
            }
            auto s = std::make_shared<GoalCountFolState>(
                L, this, L.is_terminal_state(), goalChain, newSatisfied);
            s->folDecision = s->state->getNode("decision");
            // No non-terminal tiebreak bias.
            s->f_prio = L.T_step + static_cast<double>(unsatisfied);
            s->name << L.T_step << '.' << action << ' ' << *actions(action);
            while (static_cast<uint>(action) >= children.N) {
                children.append(0);
            }
            children(action) = s.get();
            return s;
        }
    };

} // namespace

HopStackingSkeletonRoot::HopStackingSkeletonRoot(const StackingScenario &scenario, RobotTag robot)
    : ComputeNode(nullptr)
    , scenario(scenario)
    , robot(robot) {
    name << "HopStackingSkeletonRoot#0";
    isComplete = true;
    isFeasible = true;
    isTerminal = false;

    size_t n = hopcxx_stacking_num_objects(&scenario);
    placeOrder.reserve(n);
    for (size_t k = 0; k < n; k++) {
        placeOrder.push_back(findObjectIndex(scenario, hopcxx_stacking_goal_order(&scenario, k)));
    }

    std::istringstream domainStream(STACKING_DOMAIN_TEXT);
    rai::Graph kb(domainStream);
    L = std::make_shared<rai::FOL_World>();
    L->init(kb);

    L->addAgent("hand0");
    for (size_t i = 0; i < n; i++) {
        L->addObject(("obj" + std::to_string(i)).c_str());
    }
    // Every object, and the table, is a valid thing to place onto.
    L->addFact({ "surface", "table" });
    for (size_t i = 0; i < n; i++) {
        L->addFact({ "surface", ("obj" + std::to_string(i)).c_str() });
    }
    // Every object starts resting on the table.
    for (size_t i = 0; i < n; i++) {
        L->addFact({ "on", ("obj" + std::to_string(i)).c_str(), "table" });
    }
    // Goal: the whole goal_order chain holds simultaneously.
    auto goalChain = std::make_shared<std::vector<std::pair<size_t, size_t>>>();
    std::string literals;
    for (size_t k = 1; k < n; k++) {
        goalChain->push_back({ placeOrder[k], placeOrder[k - 1] });
        literals += "(on obj" + std::to_string(placeOrder[k]) + " obj"
            + std::to_string(placeOrder[k - 1]) + ") ";
    }
    L->addTerminalRule(literals.c_str());
    L->reset_state();

    auto initialSatisfied = std::make_shared<std::vector<bool>>(goalChain->size(), false);
    folAstar = std::make_shared<rai::AStar>(
        std::make_shared<GoalCountFolState>(*L, nullptr, false, goalChain, initialSatisfied));
    folAstar->verbose = 0;
}

double HopStackingSkeletonRoot::branchingPenalty_child(int i) { return hopBranchingPenalty(i); }

const std::vector<Action> *HopStackingSkeletonRoot::getOrBuildSkeleton(int i) {
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
        std::string objName(d->parents(1)->key.p);
        size_t objIndex = static_cast<size_t>(std::atoi(objName.c_str() + 3));
        if (ruleName == "pick") {
            plan.push_back({ ActionType::Pick, objIndex });
        } else {
            CHECK(ruleName == "place", "unexpected stacking rule '" << ruleName << "'");
            std::string belowName(d->parents(3)->key.p);
            int64_t belowIndex = belowName == "table"
                ? -1
                : static_cast<int64_t>(std::atoi(belowName.c_str() + 3));
            plan.push_back({ ActionType::Place, objIndex, 0, belowIndex });
        }
    }
    CHECK(!plan.empty(), "FOL_World produced an empty skeleton");
    if (getenv("HOP_SKELETON_DEBUG")) {
        fprintf(stderr, "stacking skeleton[%d] (len %zu):", i, plan.size());
        for (const Action &a : plan) {
            if (a.type == ActionType::Pick) {
                fprintf(stderr, " pick(%zu)", a.object_index);
            } else {
                fprintf(stderr, " place(%zu,on=%lld)", a.object_index,
                    static_cast<long long>(a.below_object_index));
            }
        }
        fprintf(stderr, "\n");
    }
    return &plan;
}

std::shared_ptr<rai::ComputeNode> HopStackingSkeletonRoot::createNewChild(int i) {
    const std::vector<Action> *plan = getOrBuildSkeleton(i);
    if (!plan) {
        static const std::vector<Action> EMPTY_PLAN;
        auto dead = std::make_shared<HopStackingNode>(scenario, EMPTY_PLAN, robot, this);
        dead->isFeasible = false;
        return dead;
    }
    return std::make_shared<HopStackingNode>(scenario, *plan, robot, this);
}

} // namespace hopct
