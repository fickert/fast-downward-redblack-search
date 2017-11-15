#ifndef SEARCH_ENGINES_LAZY_SEARCH_H
#define SEARCH_ENGINES_LAZY_SEARCH_H

#include "../evaluation_context.h"
#include "../global_state.h"
#include "../open_list.h"
#include "../search_engine.h"
#include "../search_space.h"
#include "../task_utils/successor_generator.h"

#include <memory>
#include <vector>
#include "../open_list_factory.h"

class GlobalOperator;

namespace options {
class Options;
}

namespace lazy_search {
template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class LazySearch : public SearchEngine<StateType, OperatorType> {
protected:
    std::unique_ptr<EdgeOpenList<StateType, OperatorType>> open_list;

    // Search behavior parameters
    bool reopen_closed_nodes; // whether to reopen closed nodes upon finding lower g paths
    bool randomize_successors;
    bool preferred_successors_first;
    std::shared_ptr<utils::RandomNumberGenerator> rng;

    std::vector<Heuristic<StateType, OperatorType> *> heuristics;
    std::vector<Heuristic<StateType, OperatorType> *> preferred_operator_heuristics;
    std::vector<Heuristic<StateType, OperatorType> *> estimate_heuristics;

    StateType current_state;
    StateID current_predecessor_id;
    const OperatorType *current_operator;
    int current_g;
    int current_real_g;
    EvaluationContext<StateType, OperatorType> current_eval_context;

    virtual void initialize() override;
    virtual SearchStatus step() override;

	auto get_operator(int op_id) const -> const OperatorType *;

    void generate_successors();
    SearchStatus fetch_next_state();

    void reward_progress();

    std::vector<OperatorID> get_successor_operators(
        const ordered_set::OrderedSet<OperatorID> &preferred_operators) const;

    // TODO: Move into SearchEngine?
    void print_checkpoint_line(int g) const;

public:
	LazySearch(const options::Options &opts, std::unique_ptr<StateRegistryBase<StateType, OperatorType>> state_registry);

    explicit LazySearch(const options::Options &opts);
    virtual ~LazySearch() = default;

    void set_pref_operator_heuristics(std::vector<Heuristic<StateType, OperatorType> *> &heur);

    virtual void print_statistics() const override;
};


template<class StateType, class OperatorType>
LazySearch<StateType, OperatorType>::LazySearch(const options::Options &opts)
    : SearchEngine<StateType, OperatorType>(opts),
      open_list(opts.get<std::shared_ptr<OpenListFactory<StateType, OperatorType>>>("open")->
                create_edge_open_list()),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      randomize_successors(opts.get<bool>("randomize_successors")),
      preferred_successors_first(opts.get<bool>("preferred_successors_first")),
      rng(utils::parse_rng_from_options(opts)),
      current_state(state_registry->get_initial_state()),
      current_predecessor_id(StateID::no_state),
      current_operator(nullptr),
      current_g(0),
      current_real_g(0),
      current_eval_context(current_state, 0, true, &statistics) {
    /*
      We initialize current_eval_context in such a way that the initial node
      counts as "preferred".
    */
}

template<class StateType, class OperatorType>
void LazySearch<StateType, OperatorType>::set_pref_operator_heuristics(
    std::vector<Heuristic<StateType, OperatorType> *> &heur) {
    preferred_operator_heuristics = heur;
}

template <class StateType, class OperatorType>
LazySearch<StateType, OperatorType>::LazySearch(const options::Options &opts,
	std::unique_ptr<StateRegistryBase<StateType, OperatorType>> state_registry)
	: SearchEngine<StateType, OperatorType>(opts, std::move(state_registry)),
      open_list(opts.get<std::shared_ptr<OpenListFactory<StateType, OperatorType>>>("open")->create_edge_open_list()),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      randomize_successors(opts.get<bool>("randomize_successors")),
      preferred_successors_first(opts.get<bool>("preferred_successors_first")),
      rng(utils::parse_rng_from_options(opts)),
      current_state(this->state_registry->get_initial_state()),
      current_predecessor_id(StateID::no_state),
      current_operator(nullptr),
      current_g(0),
      current_real_g(0),
      current_eval_context(current_state, 0, true, &statistics) {
    /*
      We initialize current_eval_context in such a way that the initial node
      counts as "preferred".
    */
}

template<class StateType, class OperatorType>
void LazySearch<StateType, OperatorType>::initialize() {
    std::cout << "Conducting lazy best first search, (real) bound = " << bound << std::endl;

    assert(open_list);
    std::set<Heuristic<StateType, OperatorType> *> hset;
    open_list->get_involved_heuristics(hset);

    // Add heuristics that are used for preferred operators (in case they are
    // not also used in the open list).
    hset.insert(preferred_operator_heuristics.begin(),
                preferred_operator_heuristics.end());

    heuristics.assign(hset.begin(), hset.end());
    assert(!heuristics.empty());
    const auto &initial_state = state_registry->get_initial_state();
    for (Heuristic<StateType, OperatorType> *heuristic : heuristics) {
        heuristic->notify_initial_state(initial_state);
    }
}

template<class StateType, class OperatorType>
std::vector<OperatorID> LazySearch<StateType, OperatorType>::get_successor_operators(
    const ordered_set::OrderedSet<OperatorID> &preferred_operators) const {
    std::vector<OperatorID> applicable_operators;
    g_successor_generator->generate_applicable_ops(
        current_state, applicable_operators);

    if (randomize_successors) {
        rng->shuffle(applicable_operators);
    }

    if (preferred_successors_first) {
        ordered_set::OrderedSet<OperatorID> successor_operators;
        for (OperatorID op_id : preferred_operators) {
            successor_operators.insert(op_id);
        }
        for (OperatorID op_id : applicable_operators) {
            successor_operators.insert(op_id);
        }
        return successor_operators.pop_as_vector();
    } else {
        return applicable_operators;
    }
}

template<class StateType, class OperatorType>
void LazySearch<StateType, OperatorType>::generate_successors() {
    ordered_set::OrderedSet<OperatorID> preferred_operators =
        collect_preferred_operators(
            current_eval_context, preferred_operator_heuristics);
    if (randomize_successors) {
        preferred_operators.shuffle(*rng);
    }

    std::vector<OperatorID> successor_operators =
        get_successor_operators(preferred_operators);

    statistics.inc_generated(successor_operators.size());

    for (OperatorID op_id : successor_operators) {
        const auto *op = get_operator(op_id.get_index());
        int new_g = current_g + get_adjusted_cost(*op);
        int new_real_g = current_real_g + op->get_cost();
        bool is_preferred = preferred_operators.contains(op_id);
        if (new_real_g < bound) {
            EvaluationContext<StateType, OperatorType> new_eval_context(
                current_eval_context.get_cache(), new_g, is_preferred, nullptr);
            open_list->insert(new_eval_context, std::make_pair(current_state.get_id(), get_op_index_hacked(op)));
        }
    }
}

template<class StateType, class OperatorType>
SearchStatus LazySearch<StateType, OperatorType>::fetch_next_state() {
    if (open_list->empty()) {
        std::cout << "Completely explored state space -- no solution!" << std::endl;
        return FAILED;
    }

    EdgeOpenListEntry next = open_list->remove_min();

    current_predecessor_id = next.first;
    current_operator = get_operator(next.second);
    StateType current_predecessor = state_registry->lookup_state(current_predecessor_id);
    assert(current_operator->is_applicable(current_predecessor));
    current_state = state_registry->get_successor_state(current_predecessor, *current_operator);

    SearchNode<StateType, OperatorType> pred_node = search_space.get_node(current_predecessor);
    current_g = pred_node.get_g() + get_adjusted_cost(*current_operator);
    current_real_g = pred_node.get_real_g() + current_operator->get_cost();

    /*
      Note: We mark the node in current_eval_context as "preferred"
      here. This probably doesn't matter much either way because the
      node has already been selected for expansion, but eventually we
      should think more deeply about which path information to
      associate with the expanded vs. evaluated nodes in lazy search
      and where to obtain it from.
    */
    current_eval_context = EvaluationContext<StateType, OperatorType>(current_state, current_g, true, &statistics);

    return IN_PROGRESS;
}

template<class StateType, class OperatorType>
SearchStatus LazySearch<StateType, OperatorType>::step() {
    // Invariants:
    // - current_state is the next state for which we want to compute the heuristic.
    // - current_predecessor is a permanent pointer to the predecessor of that state.
    // - current_operator is the operator which leads to current_state from predecessor.
    // - current_g is the g value of the current state according to the cost_type
    // - current_real_g is the g value of the current state (using real costs)


    auto node = search_space.get_node(current_state);
    bool reopen = reopen_closed_nodes && !node.is_new() &&
                  !node.is_dead_end() && (current_g < node.get_g());

    if (node.is_new() || reopen) {
        StateID dummy_id = current_predecessor_id;
        // HACK! HACK! we do this because SearchNode has no default/copy constructor
        if (dummy_id == StateID::no_state) {
            const auto &initial_state = state_registry->get_initial_state();
            dummy_id = initial_state.get_id();
        }
        auto parent_state = state_registry->lookup_state(dummy_id);
        auto parent_node = search_space.get_node(parent_state);

        if (current_operator) {
            for (Heuristic<StateType, OperatorType> *heuristic : heuristics)
                heuristic->notify_state_transition(
                    parent_state, *current_operator, current_state);
        }
        statistics.inc_evaluated_states();
        if (!open_list->is_dead_end(current_eval_context)) {
            // TODO: Generalize code for using multiple heuristics.
            if (reopen) {
                node.reopen(parent_node, current_operator);
                statistics.inc_reopened();
            } else if (current_predecessor_id == StateID::no_state) {
                node.open_initial();
                if (search_progress.check_progress(current_eval_context))
                    print_checkpoint_line(current_g);
            } else {
                node.open(parent_node, current_operator);
            }
            node.close();
            if (check_goal_and_set_plan(current_state))
                return SOLVED;
            if (search_progress.check_progress(current_eval_context)) {
                print_checkpoint_line(current_g);
                reward_progress();
            }
            generate_successors();
            statistics.inc_expanded();
        } else {
            node.mark_as_dead_end();
            statistics.inc_dead_ends();
        }
        if (current_predecessor_id == StateID::no_state) {
            print_initial_h_values<StateType, OperatorType>(current_eval_context);
        }
    }
    return fetch_next_state();
}

template<class StateType, class OperatorType>
void LazySearch<StateType, OperatorType>::reward_progress() {
    open_list->boost_preferred();
}

template<class StateType, class OperatorType>
void LazySearch<StateType, OperatorType>::print_checkpoint_line(int g) const {
    std::cout << "[g=" << g << ", ";
    statistics.print_basic_statistics();
    std::cout << "]" << std::endl;
}

template<class StateType, class OperatorType>
void LazySearch<StateType, OperatorType>::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
}

}

#endif
