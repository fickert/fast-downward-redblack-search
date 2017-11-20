#ifndef REDBLACK_INCREMENTAL_RED_BLACK_SEARCH_H
#define REDBLACK_INCREMENTAL_RED_BLACK_SEARCH_H

#include "painting.h"
#include "util.h"
#include "../search_engines/lazy_search.h"
#include "operator.h"


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

	auto update_search_space_and_check_plan(const RBPlan &plan) -> std::pair<bool, GlobalState>;

	GlobalState current_initial_state;

	void set_solution(const Plan &partial_plan, const GlobalState &state);

	void print_final_statistics() const;

	struct RBData {
		Painting painting;
		RBIntPacker int_packer;
		std::vector<RBOperator> operators;

		static auto construct_redblack_operators(const Painting &painting) -> std::vector<RBOperator>;

		RBData(const Painting &painting)
			: painting(painting),
			  int_packer(this->painting),
			  operators(construct_redblack_operators(this->painting)) {
			int_packer.initialize(g_variable_domain);
		}

		auto construct_state_registry(const std::vector<int> &initial_state_data) const -> std::unique_ptr<RBStateRegistry> {
			return std::make_unique<RBStateRegistry>(*g_root_task(), int_packer, *g_axiom_evaluator, initial_state_data, operators);
		}
	};

	struct IncrementalRedBlackSearchStatistics {
		IncrementalRedBlackSearchStatistics()
			: num_episodes(0),
			  num_restarts(0) {}

		int num_episodes;
		int num_restarts;
	} incremental_redblack_search_statistics;

	std::unique_ptr<RBData> rb_data;
	std::unique_ptr<SearchEngine<RBState, RBOperator>> rb_search_engine;
	std::shared_ptr<IncrementalPaintingStrategy> incremental_painting_strategy;

	// options
	const bool continue_from_first_conflict;

	static auto get_rb_search_options(const options::Options &options) -> options::Options;
};
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
