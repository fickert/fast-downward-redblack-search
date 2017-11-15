#ifndef SEARCH_SPACE_H
#define SEARCH_SPACE_H

#include "globals.h"
#include "global_operator.h"
#include "operator_cost.h"
#include "per_state_information.h"
#include "search_node_info.h"
#include "../utils/system.h"

#include <vector>

class GlobalState;


template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class SearchNode {
    using StateRegistryType = StateRegistryBase<StateType, OperatorType>;
    const StateRegistryType &state_registry;
    StateID state_id;
    SearchNodeInfo &info;
    OperatorCost cost_type;
public:
    SearchNode(const StateRegistryType &state_registry,
               StateID state_id,
               SearchNodeInfo &info,
               OperatorCost cost_type);

    StateID get_state_id() const {
        return state_id;
    }
    StateType get_state() const;

    bool is_new() const;
    bool is_open() const;
    bool is_closed() const;
    bool is_dead_end() const;

    int get_g() const;
    int get_real_g() const;

    void open_initial();
    void open(const SearchNode<StateType, OperatorType> &parent_node,
              const OperatorType *parent_op);
    void reopen(const SearchNode<StateType, OperatorType> &parent_node,
                const OperatorType *parent_op);
    void update_parent(const SearchNode<StateType, OperatorType> &parent_node,
                       const OperatorType *parent_op);
    void close();
    void mark_as_dead_end();

    void dump() const;
};

template<class StateType, class OperatorType>
SearchNode<StateType, OperatorType>::SearchNode(const StateRegistryType &state_registry,
                                                StateID state_id,
                                                SearchNodeInfo &info,
                                                OperatorCost cost_type)
    : state_registry(state_registry),
      state_id(state_id),
      info(info),
      cost_type(cost_type) {
    assert(state_id != StateID::no_state);
}

template<class StateType, class OperatorType>
StateType SearchNode<StateType, OperatorType>::get_state() const {
    return state_registry.lookup_state(state_id);
}

template<class StateType, class OperatorType>
bool SearchNode<StateType, OperatorType>::is_open() const {
    return info.status == SearchNodeInfo::OPEN;
}

template<class StateType, class OperatorType>
bool SearchNode<StateType, OperatorType>::is_closed() const {
    return info.status == SearchNodeInfo::CLOSED;
}

template<class StateType, class OperatorType>
bool SearchNode<StateType, OperatorType>::is_dead_end() const {
    return info.status == SearchNodeInfo::DEAD_END;
}

template<class StateType, class OperatorType>
bool SearchNode<StateType, OperatorType>::is_new() const {
    return info.status == SearchNodeInfo::NEW;
}

template<class StateType, class OperatorType>
int SearchNode<StateType, OperatorType>::get_g() const {
    assert(info.g >= 0);
    return info.g;
}

template<class StateType, class OperatorType>
int SearchNode<StateType, OperatorType>::get_real_g() const {
    return info.real_g;
}

template<class StateType, class OperatorType>
void SearchNode<StateType, OperatorType>::open_initial() {
    assert(info.status == SearchNodeInfo::NEW);
    info.status = SearchNodeInfo::OPEN;
    info.g = 0;
    info.real_g = 0;
    info.parent_state_id = StateID::no_state;
    info.creating_operator = -1;
}

template<class StateType, class OperatorType>
void SearchNode<StateType, OperatorType>::open(const SearchNode<StateType, OperatorType> &parent_node,
                      const OperatorType *parent_op) {
    assert(info.status == SearchNodeInfo::NEW);
    info.status = SearchNodeInfo::OPEN;
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = get_op_index_hacked(parent_op);
}

template<class StateType, class OperatorType>
void SearchNode<StateType, OperatorType>::reopen(const SearchNode &parent_node,
                        const OperatorType *parent_op) {
    assert(info.status == SearchNodeInfo::OPEN ||
           info.status == SearchNodeInfo::CLOSED);

    // The latter possibility is for inconsistent heuristics, which
    // may require reopening closed nodes.
    info.status = SearchNodeInfo::OPEN;
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = get_op_index_hacked(parent_op);
}

// like reopen, except doesn't change status
template<class StateType, class OperatorType>
void SearchNode<StateType, OperatorType>::update_parent(const SearchNode<StateType, OperatorType> &parent_node,
                               const OperatorType *parent_op) {
    assert(info.status == SearchNodeInfo::OPEN ||
           info.status == SearchNodeInfo::CLOSED);
    // The latter possibility is for inconsistent heuristics, which
    // may require reopening closed nodes.
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = get_op_index_hacked(parent_op);
}

template<class StateType, class OperatorType>
void SearchNode<StateType, OperatorType>::close() {
    assert(info.status == SearchNodeInfo::OPEN);
    info.status = SearchNodeInfo::CLOSED;
}

template<class StateType, class OperatorType>
void SearchNode<StateType, OperatorType>::mark_as_dead_end() {
    info.status = SearchNodeInfo::DEAD_END;
}

template<class StateType, class OperatorType>
void SearchNode<StateType, OperatorType>::dump() const {
    std::cout << state_id << ": ";
    get_state().dump_fdr();
    if (info.creating_operator != -1) {
        std::cout << " created by " << g_operators[info.creating_operator].get_name()
             << " from " << info.parent_state_id << std::endl;
    } else {
        std::cout << " no parent" << std::endl;
    }
}


template<
	class StateType = GlobalState,
	class OperatorType = GlobalOperator,
	class StateRegistryType = StateRegistryBase<StateType, OperatorType>
>
class SearchSpace {
protected:
    PerStateInformation<SearchNodeInfo, StateType, OperatorType> search_node_infos;

    //using StateRegistryType = StateRegistryBase<StateType, OperatorType>;
    StateRegistryType &state_registry;
    OperatorCost cost_type;
public:
    SearchSpace(StateRegistryType &state_registry, OperatorCost cost_type);
    virtual ~SearchSpace() = default;

    SearchNode<StateType, OperatorType> get_node(const StateType &state);
    virtual void trace_path(const StateType &goal_state,
                            std::vector<const OperatorType *> &path) const;

    void dump() const;
    void print_statistics() const;
};

template<class StateType, class OperatorType, class StateRegistryType>
SearchSpace<StateType, OperatorType, StateRegistryType>::SearchSpace(StateRegistryType &state_registry, OperatorCost cost_type)
    : state_registry(state_registry),
      cost_type(cost_type) {
}

template<class StateType, class OperatorType, class StateRegistryType>
SearchNode<StateType, OperatorType> SearchSpace<StateType, OperatorType, StateRegistryType>::get_node(const StateType &state) {
    return SearchNode<StateType, OperatorType>(
        state_registry, state.get_id(), search_node_infos[state], cost_type);
}

template<class StateType, class OperatorType, class StateRegistryType>
void SearchSpace<StateType, OperatorType, StateRegistryType>::dump() const {
    for (StateID id : state_registry) {
        auto state = state_registry.lookup_state(id);
        const SearchNodeInfo &node_info = search_node_infos[state];
        std::cout << id << ": ";
        state.dump_fdr();
        if (node_info.creating_operator != -1 &&
            node_info.parent_state_id != StateID::no_state) {
            std::cout << " created by " << g_operators[node_info.creating_operator].get_name()
                 << " from " << node_info.parent_state_id << std::endl;
        } else {
            std::cout << "has no parent" << std::endl;
        }
    }
}

template<class StateType, class OperatorType, class StateRegistryType>
void SearchSpace<StateType, OperatorType, StateRegistryType>::print_statistics() const {
	std::cout << "Number of registered states: "
         << state_registry.size() << std::endl;
}


template<>
void SearchSpace<GlobalState, GlobalOperator>::trace_path(const GlobalState &goal_state,
	std::vector<const GlobalOperator *> &path) const;

#endif
