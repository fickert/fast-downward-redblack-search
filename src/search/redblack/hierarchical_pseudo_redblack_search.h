#ifndef REDBLACK_HIERARCHICAL_PSEUDO_RED_BLACK_SEARCH_H
#define REDBLACK_HIERARCHICAL_PSEUDO_RED_BLACK_SEARCH_H

#include "painting.h"
#include "util.h"
#include "../search_engines/lazy_search.h"
#include "operator.h"
#include "rb_data.h"
#include "red_actions_manager.h"
#include "mercury/red_black_DAG_fact_following_heuristic.h"


#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace options {
class Options;
}

namespace redblack {

struct HierarchicalPseudoRedBlackSearchStatistics {
	HierarchicalPseudoRedBlackSearchStatistics() :
		num_openend_searches(0),
		num_distinct_paintings(0),
		num_failed_incomplete_searches(0),
		max_num_black(0),
		total_num_evaluations(0) {}

	int num_openend_searches;
	int num_distinct_paintings;
	int num_failed_incomplete_searches;
	int max_num_black;
	int total_num_evaluations;
};

class IncrementalPaintingStrategy;

class HierarchicalPseudoRedBlackSearch : public lazy_search::LazySearch<RBState, RBOperator> {
public:
	//explicit HierarchicalPseudoRedBlackSearch(const options::Options &opts);
	HierarchicalPseudoRedBlackSearch(const options::Options &opts,
	                           std::shared_ptr<RBStateRegistry> state_registry,
	                           std::shared_ptr<SearchSpace<RBState, RBOperator>> search_space,
	                           GlobalState current_initial_state,
	                           StateRegistryBase<GlobalState, GlobalOperator> &global_state_registry,
	                           SearchSpace<GlobalState, GlobalOperator> &global_search_space,
	                           std::map<InternalPaintingType, std::tuple<std::shared_ptr<RBData>, std::shared_ptr<RBStateRegistry>, std::shared_ptr<RedActionsManager>, std::shared_ptr<SearchSpace<RBState, RBOperator>>>> &rb_search_spaces,
	                           std::shared_ptr<RedBlackDAGFactFollowingHeuristic> plan_repair_heuristic,
	                           std::shared_ptr<RedActionsManager> red_actions_manager,
	                           HierarchicalPseudoRedBlackSearchStatistics &hierarchical_red_black_search_statistics,
	                           int num_black,
	                           bool initial_state_is_preferred = true,
	                           int initial_state_h_value = 0);

	SearchStatus step() override;

	auto get_goal_state() const -> StateID;

protected:
	void generate_successors() override;
	SearchStatus fetch_next_state() override;

	static auto check_plan(const GlobalState &state, const std::vector<OperatorID> &plan, const std::vector<FactPair> &goal_facts) -> bool;
	auto get_repaired_plan(const GlobalState &state, const std::vector<OperatorID> &plan, const std::vector<FactPair> &goal_facts) const -> std::vector<OperatorID>;
	auto update_search_space_and_check_plan(const GlobalState &state, const std::vector<OperatorID> &plan, const std::vector<FactPair> &goal_facts) -> std::pair<bool, GlobalState>;

	void enqueue_new_search(const Painting &new_painting, const GlobalState &initial_state, int key, bool preferred, EvaluationContext<RBState, RBOperator> &new_eval_context);

	auto get_hacked_cache_for_key(int key) const -> HeuristicCache<RBState, RBOperator>;

	std::shared_ptr<RedBlackDAGFactFollowingHeuristic> plan_repair_heuristic;
	std::shared_ptr<RedActionsManager> red_actions_manager;

	bool is_current_preferred;
	int current_key;

	auto get_current_key() const -> int;

	std::unordered_map<StateID, std::vector<std::unique_ptr<HierarchicalPseudoRedBlackSearch>>> child_searches;
	//std::unordered_map<StateID, std::vector<std::unique_ptr<RBData>>> child_searches_rb_data;
	std::unique_ptr<HierarchicalPseudoRedBlackSearch> *current_child_search;
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
	std::map<InternalPaintingType, std::tuple<std::shared_ptr<RBData>, std::shared_ptr<RBStateRegistry>, std::shared_ptr<RedActionsManager>, std::shared_ptr<SearchSpace<RBState, RBOperator>>>> &rb_search_spaces;
	const int num_black;

	const bool force_completeness;

	HierarchicalPseudoRedBlackSearchStatistics &hierarchical_red_black_search_statistics;
};


class HierarchicalPseudoRedBlackSearchWrapper : public SearchEngine<GlobalState, GlobalOperator> {
public:
	explicit HierarchicalPseudoRedBlackSearchWrapper(const options::Options &opts);

	static void add_options_to_parser(options::OptionParser &);

	SearchStatus step() override;
	void print_rb_search_statistics() const;
	void print_statistics() const override;

protected:
	static auto get_rb_plan_repair_heuristic(const options::Options &opts) -> std::shared_ptr<RedBlackDAGFactFollowingHeuristic>;
	static auto get_rb_search_options(const options::Options &opts) -> options::Options;
	void update_statistics();

	std::unique_ptr<HierarchicalPseudoRedBlackSearch> root_search_engine;
	std::map<InternalPaintingType, std::tuple<std::shared_ptr<RBData>, std::shared_ptr<RBStateRegistry>, std::shared_ptr<RedActionsManager>, std::shared_ptr<SearchSpace<RBState, RBOperator>>>> rb_search_spaces;
	const int num_black;

	HierarchicalPseudoRedBlackSearchStatistics hierarchical_red_black_search_statistics;

	// for periodic statistics output
	utils::Timer search_timer;
	const int statistics_interval;
	double next_print_time;
};
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
