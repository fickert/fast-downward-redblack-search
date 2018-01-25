#ifndef REDBLACK_INCREMENTAL_RED_BLACK_SEARCH_H
#define REDBLACK_INCREMENTAL_RED_BLACK_SEARCH_H

#include "painting.h"
#include "util.h"
#include "../search_engines/lazy_search.h"
#include "operator.h"
#include "rb_data.h"
#include "mercury/red_black_DAG_fact_following_heuristic.h"
#include "red_actions_manager.h"


#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace options {
class Options;
}

namespace redblack {
class IncrementalPaintingStrategy;

class IncrementalRedBlackSearch : public SearchEngine<GlobalState, GlobalOperator> {
public:
	explicit IncrementalRedBlackSearch(const options::Options &opts);

	static void add_options_to_parser(options::OptionParser &);

	SearchStatus step() override;

protected:
	using RBPlan = std::vector<const RBOperator *>;
	using InternalRBSearchEngine = lazy_search::LazySearch<RBState, RBOperator>;
	const options::Options rb_search_engine_options;

	void initialize_rb_search_engine();
	void update_statistics();
	auto get_successor_and_update_search_space(const GlobalState &current_state, const GlobalOperator &op) -> GlobalState;
	auto check_plan_and_update_search_space(const GlobalState &state, const std::vector<OperatorID> &plan, const std::vector<FactPair> &goal_facts) -> std::pair<bool, GlobalState>;
	auto check_plan_and_update_search_space(const RBPlan &plan) -> std::pair<bool, GlobalState>;
	static auto is_valid_relaxed_plan(const std::vector<boost::dynamic_bitset<>> &achieved_facts, const std::vector<FactPair> &goal_facts, const std::vector<OperatorID> &relaxed_plan) -> bool;
	static auto is_valid_relaxed_plan(const GlobalState &state, const std::vector<FactPair> &goal_facts, const std::vector<OperatorID> &relaxed_plan) -> bool;
	auto repair_plan_and_update_search_space(const GlobalState &state,
	                                         const std::vector<FactPair> &goal_facts,
	                                         const std::vector<OperatorID> &partial_plan,
	                                         const boost::dynamic_bitset<> &red_actions) -> std::pair<bool, GlobalState>;
	auto repair_plan_and_update_search_space(const RBPlan &plan, const std::vector<std::set<FactPair>> &
	                                         marked_facts) -> std::pair<bool, GlobalState>;
	auto relaxed_repair_plan(const RBPlan &plan, const std::vector<std::set<FactPair>> &marked_facts) -> std::vector<OperatorID>;

	GlobalState current_initial_state;

	void set_solution(const Plan &partial_plan, const GlobalState &state);

	void print_statistics() const override;

	struct IncrementalRedBlackSearchStatistics {
		IncrementalRedBlackSearchStatistics()
			: num_episodes(0),
			  num_restarts(0),
			  num_broken_red_plans(0) {}

		int num_episodes;
		int num_restarts;
		int num_broken_red_plans;
	} incremental_redblack_search_statistics;

	std::unique_ptr<RBData> rb_data;
	std::unique_ptr<InternalRBSearchEngine> rb_search_engine;
	std::shared_ptr<IncrementalPaintingStrategy> incremental_painting_strategy;

	// options
	const bool continue_from_first_conflict;
	std::shared_ptr<RedBlackDAGFactFollowingHeuristic> plan_repair_heuristic;
	std::unique_ptr<RedActionsManager> red_actions_manager;
	const bool always_recompute_red_plans;

	std::vector<bool> never_black_variables;

	static auto get_rb_search_options(const options::Options &options) -> options::Options;
	static auto get_rb_plan_repair_heuristic(const options::Options &options) -> std::shared_ptr<RedBlackDAGFactFollowingHeuristic>;
};
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
