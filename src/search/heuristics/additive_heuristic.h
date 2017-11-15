#ifndef HEURISTICS_ADDITIVE_HEURISTIC_H
#define HEURISTICS_ADDITIVE_HEURISTIC_H

#include "relaxation_heuristic.h"

#include "../algorithms/priority_queues.h"
#include "../utils/collections.h"

#include <cassert>

class State;

namespace additive_heuristic {
using relaxation_heuristic::Proposition;
using relaxation_heuristic::UnaryOperator;

template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class AdditiveHeuristic : public relaxation_heuristic::RelaxationHeuristic<StateType, OperatorType> {
    /* Costs larger than MAX_COST_VALUE are clamped to max_value. The
       precise value (100M) is a bit of a hack, since other parts of
       the code don't reliably check against overflow as of this
       writing. With a value of 100M, we want to ensure that even
       weighted A* with a weight of 10 will have f values comfortably
       below the signed 32-bit int upper bound.
     */
    static const int MAX_COST_VALUE = 100000000;

    priority_queues::AdaptiveQueue<Proposition *> queue;
    bool did_write_overflow_warning;

	template<class InternalStateType>
	auto compute_heuristic_internal(const InternalStateType &state) -> int;

    void setup_exploration_queue();
	template<class InternalStateType>
    void setup_exploration_queue_state(const InternalStateType &state);
    void relaxed_exploration();
	template<class InternalStateType>
    void mark_preferred_operators(const InternalStateType &state, Proposition *goal);

    void enqueue_if_necessary(Proposition *prop, int cost, UnaryOperator *op) {
        assert(cost >= 0);
        if (prop->cost == -1 || prop->cost > cost) {
            prop->cost = cost;
            prop->reached_by = op;
            queue.push(cost, prop);
        }
        assert(prop->cost != -1 && prop->cost <= cost);
    }

    void increase_cost(int &cost, int amount) {
        assert(cost >= 0);
        assert(amount >= 0);
        cost += amount;
        if (cost > MAX_COST_VALUE) {
            write_overflow_warning();
            cost = MAX_COST_VALUE;
        }
    }

    void write_overflow_warning();

    int compute_heuristic(const State &state);
protected:
	template<class InternalStateType>
	auto convert_state(const StateType &state) -> InternalStateType;

	template<class InternalStateType>
	auto is_operator_applicable(const InternalStateType &state, int operator_no) -> bool;

    virtual int compute_heuristic(const StateType &global_state);

    // Common part of h^add and h^ff computation.
	template<class InternalStateType>
    int compute_add_and_ff(const InternalStateType &state);
public:
    explicit AdditiveHeuristic(const options::Options &options);
    ~AdditiveHeuristic();

    /*
      TODO: The two methods below are temporarily needed for the CEGAR
      heuristic. In the long run it might be better to split the
      computation from the heuristic class. Then the CEGAR code could
      use the computation object instead of the heuristic.
    */
    void compute_heuristic_for_cegar(const State &state);

    int get_cost_for_cegar(int var, int value) const {
        assert(utils::in_bounds(var, this->propositions));
        assert(utils::in_bounds(value, this->propositions[var]));
        return this->propositions[var][value].cost;
    }
};


// construction and destruction
template<class StateType, class OperatorType>
AdditiveHeuristic<StateType, OperatorType>::AdditiveHeuristic(const options::Options &opts)
    : relaxation_heuristic::RelaxationHeuristic<StateType, OperatorType>(opts),
      did_write_overflow_warning(false) {
    std::cout << "Initializing additive heuristic..." << std::endl;
}

template<class StateType, class OperatorType>
AdditiveHeuristic<StateType, OperatorType>::~AdditiveHeuristic() {
}

template<class StateType, class OperatorType>
void AdditiveHeuristic<StateType, OperatorType>::write_overflow_warning() {
    if (!did_write_overflow_warning) {
        // TODO: Should have a planner-wide warning mechanism to handle
        // things like this.
        std::cout << "WARNING: overflow on h^add! Costs clamped to "
             << MAX_COST_VALUE << std::endl;
        std::cerr << "WARNING: overflow on h^add! Costs clamped to "
             << MAX_COST_VALUE << std::endl;
        did_write_overflow_warning = true;
    }
}

template<class StateType, class OperatorType>
template<class InternalStateType>
auto AdditiveHeuristic<StateType, OperatorType>::convert_state(const StateType &) -> InternalStateType {
	assert(false && "must be specialized or overridden");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

// heuristic computation
template<class StateType, class OperatorType>
void AdditiveHeuristic<StateType, OperatorType>::setup_exploration_queue() {
    queue.clear();

    for (std::size_t var = 0; var < this->propositions.size(); ++var) {
        for (std::size_t value = 0; value < this->propositions[var].size(); ++value) {
            Proposition &prop = this->propositions[var][value];
            prop.cost = -1;
            prop.marked = false;
        }
    }

    // Deal with operators and axioms without preconditions.
    for (std::size_t i = 0; i < this->unary_operators.size(); ++i) {
        UnaryOperator &op = this->unary_operators[i];
        op.unsatisfied_preconditions = op.precondition.size();
        op.cost = op.base_cost; // will be increased by precondition costs

        if (op.unsatisfied_preconditions == 0)
            enqueue_if_necessary(op.effect, op.base_cost, &op);
    }
}

template<class StateType, class OperatorType>
template<class InternalStateType>
void AdditiveHeuristic<StateType, OperatorType>::setup_exploration_queue_state(const InternalStateType &) {
	assert(false && "must be specialized or overridden");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

template<class StateType, class OperatorType>
void AdditiveHeuristic<StateType, OperatorType>::relaxed_exploration() {
    int unsolved_goals = this->goal_propositions.size();
    while (!queue.empty()) {
        std::pair<int, Proposition *> top_pair = queue.pop();
        int distance = top_pair.first;
        Proposition *prop = top_pair.second;
        int prop_cost = prop->cost;
        assert(prop_cost >= 0);
        assert(prop_cost <= distance);
        if (prop_cost < distance)
            continue;
        if (prop->is_goal && --unsolved_goals == 0)
            return;
        const std::vector<UnaryOperator *> &triggered_operators =
            prop->precondition_of;
        for (std::size_t i = 0; i < triggered_operators.size(); ++i) {
            UnaryOperator *unary_op = triggered_operators[i];
            increase_cost(unary_op->cost, prop_cost);
            --unary_op->unsatisfied_preconditions;
            assert(unary_op->unsatisfied_preconditions >= 0);
            if (unary_op->unsatisfied_preconditions == 0)
                enqueue_if_necessary(unary_op->effect,
                                     unary_op->cost, unary_op);
        }
    }
}

template<class StateType, class OperatorType>
template<class InternalStateType>
void AdditiveHeuristic<StateType, OperatorType>::mark_preferred_operators(
    const InternalStateType &state, Proposition *goal) {
    if (!goal->marked) { // Only consider each subgoal once.
        goal->marked = true;
        UnaryOperator *unary_op = goal->reached_by;
        if (unary_op) { // We have not yet chained back to a start node.
            for (size_t i = 0; i < unary_op->precondition.size(); ++i)
                mark_preferred_operators(state, unary_op->precondition[i]);
            int operator_no = unary_op->operator_no;
            if (unary_op->cost == unary_op->base_cost && operator_no != -1) {
                // Necessary condition for this being a preferred
                // operator, which we use as a quick test before the
                // more expensive applicability test.
                // If we had no 0-cost operators and axioms to worry
                // about, this would also be a sufficient condition.
				if (is_operator_applicable(state, operator_no))
					this->set_preferred(this->task_proxy.get_operators()[operator_no]);
            }
        }
    }
}

template<class StateType, class OperatorType>
template<class InternalStateType>
auto AdditiveHeuristic<StateType, OperatorType>::is_operator_applicable(const InternalStateType &, int) -> bool {
	assert(false && "must be specialized or overridden");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

template<class StateType, class OperatorType>
template<class InternalStateType>
int AdditiveHeuristic<StateType, OperatorType>::compute_add_and_ff(const InternalStateType &state) {
    setup_exploration_queue();
    setup_exploration_queue_state(state);
    relaxed_exploration();

    int total_cost = 0;
    for (size_t i = 0; i < this->goal_propositions.size(); ++i) {
        int prop_cost = this->goal_propositions[i]->cost;
        if (prop_cost == -1)
            return this->DEAD_END;
        increase_cost(total_cost, prop_cost);
    }
    return total_cost;
}

template<class StateType, class OperatorType>
int AdditiveHeuristic<StateType, OperatorType>::compute_heuristic(const State &state) {
	return compute_heuristic_internal(state);
}

template<class StateType, class OperatorType>
template<class InternalStateType>
int AdditiveHeuristic<StateType, OperatorType>::compute_heuristic_internal(const InternalStateType &state) {
	int h = compute_add_and_ff(state);
	if (h != this->DEAD_END) {
		for (size_t i = 0; i < this->goal_propositions.size(); ++i)
			mark_preferred_operators(state, this->goal_propositions[i]);
	}
	return h;
}

template<class StateType, class OperatorType>
int AdditiveHeuristic<StateType, OperatorType>::compute_heuristic(const StateType &global_state) {
    return compute_heuristic_internal(global_state);
}

template<class StateType, class OperatorType>
void AdditiveHeuristic<StateType, OperatorType>::compute_heuristic_for_cegar(const State &state) {
    compute_heuristic(state);
}


template<>
template<>
auto AdditiveHeuristic<GlobalState, GlobalOperator>::convert_state(const GlobalState &state) -> State;

template<>
int AdditiveHeuristic<GlobalState, GlobalOperator>::compute_heuristic(const GlobalState &global_state);

template<>
template<>
void AdditiveHeuristic<GlobalState, GlobalOperator>::setup_exploration_queue_state(const State &state);

template<>
template<>
auto AdditiveHeuristic<GlobalState, GlobalOperator>::is_operator_applicable(const State &state, int operator_no) -> bool;

}

#endif
