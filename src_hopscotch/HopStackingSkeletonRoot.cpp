#include "HopBenchDriver.h"
#include "HopStackingSkeletonRoot.h"
#include <Core/graph.h>
#include <algorithm>
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
table
free
held
on
clear

START_STATE {}

REWARD {
}

DecisionRule pick {
  Obj, Hand, Below,
  { (agent Hand) (object Obj) (free Hand) (held Obj)! (on Obj Below) (clear Obj) }
  { (free Hand)!
    (held Obj)
    (on Obj Below)!
    (clear Obj)!
    (clear Below)
    }
}

DecisionRule place {
  Obj, Hand, Below,
  { (agent Hand) (object Obj) (object Below) (held Obj) (clear Below) }
  { (held Obj)!
    (free Hand)
    (on Obj Below)
    (clear Below)!
    (clear Obj)
    }
}

DecisionRule placeTable {
  Obj, Hand, Below,
  { (agent Hand) (object Obj) (table Below) (held Obj) }
  { (held Obj)!
    (free Hand)
    (on Obj Below)
    (clear Obj)
    }
}
)FOL";

    // Where a block rests in the symbolic state.
    constexpr int64_t ON_TABLE = -1;
    constexpr int64_t IN_HAND = -2;

    // A FOL world searched by A* with a misplaced-block heuristic.
    // A block is in final position when it rests on its goal support and that support is the
    // table or a block in final position.
    // Every other block must move at least once.
    // A block also can't reach its final position until every block below it in the goal tower
    // is in final position.
    // So a block that rests above a lower goal-tower block that isn't in final position must move
    // twice, once off that block and once into place.
    // The heuristic sums each block's action lower bound, which makes it admissible.
    struct MisplacedBlocksFolState : rai::FOL_World_State {
        // The goal tower from the bottom up.
        std::shared_ptr<const std::vector<size_t>> placeOrder;
        // Each block's support, indexed by object, as an object index, ON_TABLE, or IN_HAND.
        std::vector<int64_t> support;

        MisplacedBlocksFolState(rai::FOL_World &L, TreeSearchNode *parent, bool isTerminal,
            std::shared_ptr<const std::vector<size_t>> placeOrder, std::vector<int64_t> support)
            : FOL_World_State(L, parent, isTerminal)
            , placeOrder(std::move(placeOrder))
            , support(std::move(support)) { }

        double heuristic() const {
            size_t n = support.size();
            // rank[obj] is obj's height in the goal tower.
            std::vector<size_t> rank(n);
            std::vector<bool> inFinal(n, false);
            bool belowFinal = true;
            int64_t goalBelow = ON_TABLE;
            for (size_t k = 0; k < n; k++) {
                size_t obj = (*placeOrder)[k];
                rank[obj] = k;
                belowFinal = belowFinal && support[obj] == goalBelow;
                inFinal[obj] = belowFinal;
                goalBelow = static_cast<int64_t>(obj);
            }
            double h = 0.;
            bool lowerAllFinal = true;
            goalBelow = ON_TABLE;
            for (size_t obj : *placeOrder) {
                if (!inFinal[obj] && support[obj] == IN_HAND) {
                    // A held block goes straight into place only if its goal support is final
                    // and clear, and otherwise it must be set down, picked, and placed.
                    bool supportClear = goalBelow == ON_TABLE
                        || std::find(support.begin(), support.end(), goalBelow) == support.end();
                    h += lowerAllFinal && supportClear ? 1. : 3.;
                } else if (!inFinal[obj]) {
                    bool moveTwice = false;
                    for (int64_t c = support[obj]; c >= 0; c = support[c]) {
                        if (!inFinal[c] && rank[c] < rank[obj]) {
                            moveTwice = true;
                            break;
                        }
                    }
                    h += moveTwice ? 4. : 2.;
                }
                lowerAllFinal = lowerAllFinal && inFinal[obj];
                goalBelow = static_cast<int64_t>(obj);
            }
            return h;
        }

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

            // Track this decision's effect on block supports.
            std::vector<int64_t> newSupport = support;
            const auto *d = dynamic_cast<const rai::FOL_World::Decision *>(actions(action).get());
            if (d && !d->waitDecision && d->substitution.N >= 3) {
                std::string ruleName(d->rule->key.p);
                const char *objName = d->substitution(0)->key.p;
                const char *belowName = d->substitution(2)->key.p;
                CHECK(
                    !strncmp(objName, "obj", 3), "unexpected stacking object '" << objName << "'");
                size_t objIndex = static_cast<size_t>(std::atoi(objName + 3));
                if (ruleName == "pick") {
                    newSupport[objIndex] = IN_HAND;
                } else if (ruleName == "place") {
                    newSupport[objIndex] = std::atoi(belowName + 3);
                } else if (ruleName == "placeTable") {
                    newSupport[objIndex] = ON_TABLE;
                }
            }

            L.transition(actions(action));
            CHECK(L.state != state, "");
            auto s = std::make_shared<MisplacedBlocksFolState>(
                L, this, L.is_terminal_state(), placeOrder, std::move(newSupport));
            s->folDecision = s->state->getNode("decision");
            // Taking the max with the parent's priority keeps priorities monotone along every
            // path, which rai's A* checks, even where the heuristic isn't consistent.
            s->f_prio = std::max(f_prio, L.T_step + s->heuristic());
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
    L->addFact({ "table", "table" });
    // Each object starts on the table or on another object, and it's clear if nothing starts on
    // it.
    // startBelow holds the index of the object below each object, or -1 for the table.
    std::vector<int64_t> startBelow(n);
    std::vector<bool> covered(n, false);
    for (size_t i = 0; i < n; i++) {
        int64_t belowId = hopcxx_stacking_object_below(&scenario, i);
        if (belowId >= 0) {
            size_t below = findObjectIndex(scenario, static_cast<uint64_t>(belowId));
            startBelow[i] = static_cast<int64_t>(below);
            covered[below] = true;
        } else {
            startBelow[i] = -1;
        }
    }
    for (size_t i = 0; i < n; i++) {
        std::string obj = "obj" + std::to_string(i);
        std::string below = startBelow[i] < 0 ? "table" : "obj" + std::to_string(startBelow[i]);
        L->addFact({ "on", obj.c_str(), below.c_str() });
        if (!covered[i]) {
            L->addFact({ "clear", obj.c_str() });
        }
    }
    // Goal: the whole goal_order chain holds simultaneously.
    std::string literals;
    for (size_t k = 1; k < n; k++) {
        literals += "(on obj" + std::to_string(placeOrder[k]) + " obj"
            + std::to_string(placeOrder[k - 1]) + ") ";
    }
    L->addTerminalRule(literals.c_str());
    L->reset_state();

    folAstar = std::make_shared<rai::AStar>(std::make_shared<MisplacedBlocksFolState>(
        *L, nullptr, false, std::make_shared<const std::vector<size_t>>(placeOrder), startBelow));
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

    {
        SkeletonTimer timer;
        while (static_cast<int>(folAstar->solutions.N) <= i && folAstar->queue.N > 0) {
            folAstar->step();
        }
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
            CHECK(ruleName == "place" || ruleName == "placeTable",
                "unexpected stacking rule '" << ruleName << "'");
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
    double searchStart = rai::cpuTime();
    const std::vector<Action> *plan = getOrBuildSkeleton(i);
    double searched = rai::cpuTime() - searchStart;
    // A skeleton index past the end of the symbolic search space yields an infeasible node.
    static const std::vector<Action> EMPTY_PLAN;
    auto node = std::make_shared<HopSkeletonNode<StackingDomain>>(
        StackingDomain::Context { &scenario, robot }, plan ? *plan : EMPTY_PLAN, this, i);
    // The reference times the symbolic search inside its skeleton node's computation, so the
    // search's CPU time counts toward the effort the skeleton's descendants inherit.
    node->c += searched;
    return node;
}

} // namespace hopct
