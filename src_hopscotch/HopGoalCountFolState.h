#pragma once
#include <Core/graph.h>
#include <Logic/fol.h>
#include <Logic/folWorld.h>
#include <memory>

namespace hopct {

// A FOL world searched by A* with a goal-count heuristic.
// The heuristic counts the terminal rule's literals that don't hold in the state.
// Each action makes at most one goal literal true, so the heuristic is consistent.
struct GoalCountFolState : rai::FOL_World_State {
    // The terminal rule's ground literals.
    const rai::Graph &goal;

    GoalCountFolState(
        rai::FOL_World &L, TreeSearchNode *parent, bool isTerminal, const rai::Graph &goal)
        : FOL_World_State(L, parent, isTerminal)
        , goal(goal) { }

    double heuristic() const {
        double h = 0.;
        for (rai::Node *literal : goal) {
            if (!rai::getEqualFactInKB(*state, literal)) {
                h += 1.;
            }
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

        L.transition(actions(action));
        CHECK(L.state != state, "");
        auto s = std::make_shared<GoalCountFolState>(L, this, L.is_terminal_state(), goal);
        s->folDecision = s->state->getNode("decision");
        s->f_prio = L.T_step + s->heuristic();
        s->name << L.T_step << '.' << action << ' ' << *actions(action);
        while (static_cast<uint>(action) >= children.N) {
            children.append(0);
        }
        children(action) = s.get();
        return s;
    }
};

// The ground literals of the terminal rule that `L` added last.
inline const rai::Graph &terminalGoal(rai::FOL_World &L) {
    return rai::getFirstNonSymbolOfScope(L.worldRules.last()->graph())->graph();
}

} // namespace hopct
