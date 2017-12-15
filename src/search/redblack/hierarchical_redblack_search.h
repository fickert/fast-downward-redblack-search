#ifndef REDBLACK_HIERARCHICAL_RED_BLACK_SEARCH_H
#define REDBLACK_HIERARCHICAL_RED_BLACK_SEARCH_H

#include "painting.h"
#include "util.h"
#include "../search_engines/lazy_search.h"
#include "operator.h"
#include "rb_data.h"


#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace options {
class Options;
}

namespace redblack {
class IncrementalPaintingStrategy;

class HierarchicalRedBlackSearch : public lazy_search::LazySearch<RBState, RBOperator> {
public:
	//explicit HierarchicalRedBlackSearch(const options::Options &opts);
	HierarchicalRedBlackSearch(const options::Options &opts,
	                           std::shared_ptr<RBStateRegistry> state_registry,
	                           std::shared_ptr<SearchSpace<RBState, RBOperator>> search_space,
	                           GlobalState current_initial_state,
	                           StateRegistryBase<GlobalState, GlobalOperator> &global_state_registry,
	                           SearchSpace<GlobalState, GlobalOperator> &global_search_space,
	                           std::map<InternalPaintingType, std::tuple<std::shared_ptr<RBData>, std::shared_ptr<RBStateRegistry>, std::shared_ptr<SearchSpace<RBState, RBOperator>>>> &rb_search_spaces,
	                           int num_black,
	                           bool initial_state_is_preferred = true,
	                           int initial_state_h_value = 0);

	SearchStatus step() override;

	auto get_goal_state() const -> StateID;

protected:
	void generate_successors() override;
	SearchStatus fetch_next_state() override;

	auto update_search_space_and_check_plan(const GlobalState &state, const std::vector<OperatorID> &plan, const std::vector<FactPair> &goal_facts) -> std::pair<bool, GlobalState>;

	void enqueue_new_search(const Painting &new_painting, const GlobalState &initial_state, int key, bool preferred, EvaluationContext<RBState, RBOperator> &new_eval_context);

	auto get_hacked_cache_for_key(int key) const -> HeuristicCache<RBState, RBOperator>;

	bool is_current_preferred;
	int current_key;

	auto get_current_key() const -> int;

	std::unordered_map<StateID, std::vector<std::unique_ptr<HierarchicalRedBlackSearch>>> child_searches;
	//std::unordered_map<StateID, std::vector<std::unique_ptr<RBData>>> child_searches_rb_data;
	std::unique_ptr<HierarchicalRedBlackSearch> *current_child_search;
	int current_child_search_index;

	std::vector<std::vector<OperatorID>> current_best_supporters;

	std::unordered_map<StateID, StateID> corresponding_global_state;
	std::unordered_map<std::pair<StateID, int>, StateID> pending_corresponding_global_states;

	StateID global_goal_state;

	const options::Options search_options;
	std::shared_ptr<IncrementalPaintingStrategy> incremental_painting_strategy;
	GlobalState current_initial_state;
	StateRegistryBase<GlobalState, GlobalOperator> &global_state_registry;
	SearchSpace<GlobalState, GlobalOperator> &global_search_space;
	std::map<InternalPaintingType, std::tuple<std::shared_ptr<RBData>, std::shared_ptr<RBStateRegistry>, std::shared_ptr<SearchSpace<RBState, RBOperator>>>> &rb_search_spaces;
	const int num_black;

	struct IncrementalRedBlackSearchStatistics {
		IncrementalRedBlackSearchStatistics() {}
	} hierarchical_redblack_search_statistics;
};


class HierarchicalRedBlackSearchWrapper : public SearchEngine<GlobalState, GlobalOperator> {
public:
	explicit HierarchicalRedBlackSearchWrapper(const options::Options &opts);

	static void add_options_to_parser(options::OptionParser &);

	SearchStatus step() override;
	void print_statistics() const override;

protected:

	static auto get_rb_search_options(const options::Options &opts) -> options::Options;
	void update_statistics();

	std::unique_ptr<HierarchicalRedBlackSearch> root_search_engine;
	std::map<InternalPaintingType, std::tuple<std::shared_ptr<RBData>, std::shared_ptr<RBStateRegistry>, std::shared_ptr<SearchSpace<RBState, RBOperator>>>> rb_search_spaces;
	const int num_black;

};
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
