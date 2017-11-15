#ifndef HEURISTICS_FF_HEURISTIC_H
#define HEURISTICS_FF_HEURISTIC_H

#include "additive_heuristic.h"

#include <vector>

namespace ff_heuristic {
using Proposition = relaxation_heuristic::Proposition;
using UnaryOperator = relaxation_heuristic::UnaryOperator;

/*
  TODO: In a better world, this should not derive from
        AdditiveHeuristic. Rather, the common parts should be
        implemented in a common base class. That refactoring could be
        made at the same time at which we also unify this with the
        other relaxation heuristics and the additional FF heuristic
        implementation in the landmark code.
*/
template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class FFHeuristic : public additive_heuristic::AdditiveHeuristic<StateType, OperatorType> {
    // Relaxed plans are represented as a set of operators implemented
    // as a bit vector.
    typedef std::vector<bool> RelaxedPlan;
    RelaxedPlan relaxed_plan;

	template<class InternalStateType>
    void mark_preferred_operators_and_relaxed_plan(
        const InternalStateType &state, Proposition *goal);

	template<class InternalStateType>
	auto compute_heuristic_internal(const InternalStateType &state) -> int;

protected:
    virtual int compute_heuristic(const StateType &global_state);
public:
    FFHeuristic(const options::Options &options);
    ~FFHeuristic();
};

// construction and destruction
template<class StateType, class OperatorType>
FFHeuristic<StateType, OperatorType>::FFHeuristic(const options::Options &opts)
    : additive_heuristic::AdditiveHeuristic<StateType, OperatorType>(opts),
      relaxed_plan(this->task_proxy.get_operators().size(), false) {
    std::cout << "Initializing FF heuristic..." << std::endl;
}

template<class StateType, class OperatorType>
FFHeuristic<StateType, OperatorType>::~FFHeuristic() {
}

template<class StateType, class OperatorType>
template<class InternalStateType>
void FFHeuristic<StateType, OperatorType>::mark_preferred_operators_and_relaxed_plan(
    const InternalStateType &state, Proposition *goal) {
    if (!goal->marked) { // Only consider each subgoal once.
        goal->marked = true;
        UnaryOperator *unary_op = goal->reached_by;
        if (unary_op) { // We have not yet chained back to a start node.
            for (size_t i = 0; i < unary_op->precondition.size(); ++i)
                mark_preferred_operators_and_relaxed_plan(
                    state, unary_op->precondition[i]);
            int operator_no = unary_op->operator_no;
            if (operator_no != -1) {
                // This is not an axiom.
                relaxed_plan[operator_no] = true;

                if (unary_op->cost == unary_op->base_cost) {
                    // This test is implied by the next but cheaper,
                    // so we perform it to save work.
                    // If we had no 0-cost operators and axioms to worry
                    // about, it would also imply applicability.
                    if (this->is_operator_applicable(state, operator_no))
                        this->set_preferred(this->task_proxy.get_operators()[operator_no]);
                }
            }
        }
    }
}

template<class StateType, class OperatorType>
int FFHeuristic<StateType, OperatorType>::compute_heuristic(const StateType &global_state) {
	return compute_heuristic_internal(global_state);
}

template<class StateType, class OperatorType>
template<class InternalStateType>
auto FFHeuristic<StateType, OperatorType>::compute_heuristic_internal(const InternalStateType &state) -> int {
    int h_add = this->compute_add_and_ff(state);
    if (h_add == this->DEAD_END)
        return h_add;

    // Collecting the relaxed plan also sets the preferred operators.
    for (size_t i = 0; i < this->goal_propositions.size(); ++i)
        mark_preferred_operators_and_relaxed_plan(state, this->goal_propositions[i]);

    int h_ff = 0;
    for (size_t op_no = 0; op_no < relaxed_plan.size(); ++op_no) {
        if (relaxed_plan[op_no]) {
            relaxed_plan[op_no] = false; // Clean up for next computation.
            h_ff += this->task_proxy.get_operators()[op_no].get_cost();
        }
    }
    return h_ff;
}


template<>
int FFHeuristic<GlobalState, GlobalOperator>::compute_heuristic(const GlobalState &global_state);

}

#endif
