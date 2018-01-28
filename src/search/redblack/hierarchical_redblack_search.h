#ifndef REDBLACK_HIERARCHICAL_RED_BLACK_SEARCH_H
#define REDBLACK_HIERARCHICAL_RED_BLACK_SEARCH_H

#include "painting.h"
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
class HierarchicalRedBlackSearchWrapper;
class IncrementalPaintingStrategy;

class HierarchicalRedBlackSearch : public lazy_search::LazySearch<RBState, RBOperator> {
public:
	//explicit HierarchicalRedBlackSearch(const options::Options &opts);
	HierarchicalRedBlackSearch(const options::Options &opts,
	                           std::shared_ptr<RBStateRegistry> state_registry,
	                           std::shared_ptr<SearchSpace<RBState, RBOperator>> search_space,
	                           std::map<InternalPaintingType, std::pair<std::unique_ptr<RBData>, std::unique_ptr<HierarchicalRedBlackSearch>>> &rb_searches,
	                           std::shared_ptr<RedBlackDAGFactFollowingHeuristic> plan_repair_heuristic,
	                           std::shared_ptr<RedActionsManager> red_actions_manager,
	                           HierarchicalRedBlackSearchWrapper &wrapper);

	SearchStatus step() override;

	void enqueue_initial();

protected:
	static constexpr auto NO_PARENT = std::numeric_limits<int>::max();

	const Painting &painting;

	void recursive_split(const RBState &state, const std::vector<int> &split_vars, std::vector<int>::size_type current_pos, std::vector<boost::dynamic_bitset<>> &values, std::vector<RBState> &new_states);
	auto split_state(const RBState &state, const std::vector<int> &split_vars) -> std::vector<RBState>;
	void enqueue_states_from_split(const RBState &state, const std::vector<int> &split_vars, HierarchicalRedBlackSearch &parent_search,
		int parent_h, bool preferred, int parent_g);


	using RBPathSegment = decltype(search_space->trace_rb_path(current_state, {}));
	using RBPath = std::vector<std::pair<RBPathSegment, HierarchicalRedBlackSearch *>>;

	auto get_path_to_current_state(bool require_goal) -> RBPath;
	static auto get_rb_op_sequence(const RBPath &path) -> std::vector<const RBOperator *>;
	static auto get_op_sequence(const RBPath &path) -> std::vector<const GlobalOperator *>;
	static auto get_op_id_sequence(const RBPath &path) -> std::vector<OperatorID>;

	static auto check_path(const RBPath &path, bool require_goal) -> bool;

	using MarkedFacts = std::vector<std::set<FactPair>>;

	auto collect_marked_facts(const RBPath &path) -> std::vector<MarkedFacts>;

	template<bool relaxed>
	auto repair_path(const std::conditional_t<relaxed, std::vector<boost::dynamic_bitset<>>, std::vector<int>> &state_values,
		const std::vector<OperatorID> &relaxed_plan, const std::vector<std::vector<OperatorID>> &current_supporters,
		const std::vector<FactPair> &goal_facts, const InternalPaintingType &painting, const boost::dynamic_bitset<> &red_actions) const -> std::pair<bool, std::vector<OperatorID>>;

	template<bool relaxed>
	auto repair_path(const RBPath &path, const std::vector<MarkedFacts> &marked_facts) -> std::pair<bool, RBPath>;

	void adjust_plan_repair_painting();

	static auto get_split_vars_max_conflicts(const std::vector<FactPair> &goal_facts, const std::vector<OperatorID> &relaxed_plan, const Painting &painting) -> std::vector<int>;
	static auto get_split_vars_immediate_conflict(const std::vector<int> &state_values, const std::vector<FactPair> &expected_facts) -> std::vector<int>;
	static auto get_split_vars_immediate_conflict_expected_goal(const std::vector<int> &state_values) -> std::vector<int>;
	static auto get_split_vars_immediate_conflict_expected_operator(const std::vector<int> &state_values, const GlobalOperator &failed) -> std::vector<int>;

	void perform_split(const RBState &state, const std::vector<int> &split_vars);
	void perform_split_at_first_conflict(const RBPath &path, const std::vector<FactPair> &goal_facts);

	SearchStatus fetch_next_state() override;

	auto get_hacked_cache_for_key(int key, const RBState& state) const -> HeuristicCache<RBState, RBOperator>;

	// map from states in this search to the parent search from which search was started, the corresponding state
	// NOTE: we don't store the variables that were painted black here... we only need them during the solution reconstruction and it's cheap to reconstruct anyway
	std::unordered_map<StateID, std::tuple<HierarchicalRedBlackSearch *, StateID>> parents;

	std::shared_ptr<RedBlackDAGFactFollowingHeuristic> plan_repair_heuristic;
	std::shared_ptr<RedActionsManager> red_actions_manager;

	const bool always_recompute_red_plans;
	const bool split_on_immediate_conflict_variables;

	bool is_current_preferred;

	auto get_current_key() const -> int;

	std::unordered_map<StateID, std::vector<HierarchicalRedBlackSearch *>> child_searches;
	HierarchicalRedBlackSearch *current_child_search;
	int current_child_search_index;

	//std::vector<std::vector<OperatorID>> current_best_supporters;

	const options::Options search_options;
	std::shared_ptr<IncrementalPaintingStrategy> incremental_painting_strategy;
	std::map<InternalPaintingType, std::pair<std::unique_ptr<RBData>, std::unique_ptr<HierarchicalRedBlackSearch>>> &rb_searches;

	HierarchicalRedBlackSearchWrapper &wrapper;
};


class HierarchicalRedBlackSearchWrapper : public SearchEngine<GlobalState, GlobalOperator> {
public:
	explicit HierarchicalRedBlackSearchWrapper(const options::Options &opts);

	static void add_options_to_parser(options::OptionParser &);

	SearchStatus step() override;
	void print_rb_search_statistics() const;
	void print_statistics() const override;

protected:
	static auto get_rb_plan_repair_heuristic(const options::Options &opts) -> std::shared_ptr<RedBlackDAGFactFollowingHeuristic>;
	static auto get_rb_search_options(const options::Options &opts) -> options::Options;

	HierarchicalRedBlackSearch *root_search_engine;
	std::map<InternalPaintingType, std::pair<std::unique_ptr<RBData>, std::unique_ptr<HierarchicalRedBlackSearch>>> rb_searches;
	const int num_black;

	struct HierarchicalRedBlackSearchStatistics {
		HierarchicalRedBlackSearchStatistics() :
			num_openend_searches(0),
			num_distinct_paintings(0),
			max_num_black(0) {}

		int num_openend_searches;
		int num_distinct_paintings;
		int max_num_black;
	};

	HierarchicalRedBlackSearchStatistics hierarchical_red_black_search_statistics;

	// for periodic statistics output
	utils::Timer search_timer;
	const int statistics_interval;
	double next_print_time;

	friend class HierarchicalRedBlackSearch;
};
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
