#include "incremental_red_black_search.h"

#include "util.h"
#include "../search_engine.h"
#include "../options/option_parser.h"
#include "../search_engines/lazy_search.h"
#include "../search_engines/search_common.h"
#include "incremental_painting_strategy.h"


namespace redblack {

IncrementalRedBlackSearch::IncrementalRedBlackSearch(const options::Options &opts)
	: SearchEngine<>(opts),
	  rb_search_engine_options(get_rb_search_options(opts)),
	  current_initial_state(state_registry->get_initial_state()),
	  rb_data(std::make_unique<RBData>(*opts.get<std::shared_ptr<Painting>>("base_painting"))),
	  rb_search_engine(std::make_unique<InternalRBSearchEngine>(rb_search_engine_options, rb_data->construct_state_registry(g_initial_state_data))),
	  incremental_painting_strategy(opts.get<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy")) {
	auto num_black = std::count_if(std::begin(rb_data->painting.get_painting()), std::end(rb_data->painting.get_painting()),
		[](auto b) { return !b; });
	std::cout << "Starting incremental red-black search, initial painting has " << num_black << " black variables ("
		<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)" << std::endl;
}

auto IncrementalRedBlackSearch::is_real_plan(const GlobalState& initial_state, const RBPlan &plan) -> std::tuple<bool, GlobalState> {
	auto current_state = initial_state;
	for (auto plan_it = std::begin(plan); plan_it != std::end(plan); ++plan_it) {
		const auto &op = (**plan_it).get_base_operator();
		if (!op.is_applicable(current_state))
			return {false, current_state};
		current_state = state_registry->get_successor_state(current_state, op);
	}
	return {test_goal(current_state), current_state};
}

void IncrementalRedBlackSearch::set_solution(const Plan &partial_plan, const GlobalState &state) {
	assert(!search_space.get_node(state).is_new());
	auto solution = Plan();
	search_space.trace_path(state, solution);
	solution.insert(std::end(solution), std::begin(partial_plan), std::end(partial_plan));
	set_plan(solution);
}

auto IncrementalRedBlackSearch::RBData::construct_redblack_operators(const Painting &painting) -> std::vector<RBOperator> {
	auto rb_operators = std::vector<RBOperator>();
	rb_operators.reserve(g_operators.size());
	for (const auto &op : g_operators) {
		rb_operators.emplace_back(op);
		rb_operators.back().apply_painting(painting);
	}
	return rb_operators;
}

auto IncrementalRedBlackSearch::get_rb_search_options(const options::Options &opts) -> options::Options {
	auto rb_search_options = opts;
	rb_search_options.set("evals", std::vector<Evaluator<RBState, RBOperator> *>{ opts.get<Heuristic<RBState, RBOperator> *>("heuristic") });
	rb_search_options.set("preferred", std::vector<Heuristic<RBState, RBOperator> *>{ opts.get<Heuristic<RBState, RBOperator> *>("heuristic") });
	rb_search_options.set("boost", 1000);
	rb_search_options.set("open", search_common::create_greedy_open_list_factory<RBState, RBOperator>(rb_search_options));
	rb_search_options.set("reopen_closed", false);
	//rb_search_options.set("randomize_successors", false);
	//rb_search_options.set("random_seed", -1);
	rb_search_options.set("bound", std::numeric_limits<int>::max());
	rb_search_options.set("max_time", std::numeric_limits<double>::infinity());
	return rb_search_options;
}

void IncrementalRedBlackSearch::add_options_to_parser(options::OptionParser &parser) {
	parser.add_option<std::shared_ptr<Painting>>("base_painting", "painting to be used in the initial red-black search", "cg_top_first()");
	parser.add_option<Heuristic<RBState, RBOperator> *>("heuristic", "red-black heuristic that will be passed to the underlying red-black search engine", "ff_rb(transform=adapt_costs(cost_type=1))");
	parser.add_option<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy", "strategy for painting more variables black after finding a red-black solution with conflicts", "least_conflicts()");
	add_succ_order_options(parser);
}

SearchStatus IncrementalRedBlackSearch::step() {
	assert(rb_search_engine->get_status() == IN_PROGRESS);
	auto status = rb_search_engine->step();
	if (status == IN_PROGRESS || status == FAILED)
		return status;
	if (status == FAILED)
		utils::exit_with(utils::ExitCode::UNSOLVABLE);
	assert(status == SOLVED);
	const auto &rb_plan = rb_search_engine->get_plan();
	auto [is_plan, resulting_state] = is_real_plan(current_initial_state, rb_plan);
	if (is_plan) {
		auto plan = std::vector<const GlobalOperator *>();
		plan.reserve(rb_plan.size());
		std::transform(std::begin(rb_plan), std::end(rb_plan), std::back_inserter(plan),
			[](const auto rb_operator) { return &g_operators[get_op_index_hacked(rb_operator)]; });
		set_plan(plan);
		return SOLVED;
	}
	std::cout << "Red-black plan is not a real plan. Search continues with a new painting..." << std::endl;
	auto plan = std::vector<OperatorID>();
	plan.reserve(rb_plan.size());
	std::transform(std::begin(rb_plan), std::end(rb_plan), std::back_inserter(plan),
		[](const auto rb_operator) { return OperatorID(get_op_index_hacked(rb_operator)); });
	rb_data = std::make_unique<RBData>(incremental_painting_strategy->generate_next_painting(rb_data->painting, plan));
	current_initial_state = resulting_state;
	rb_search_engine = std::make_unique<InternalRBSearchEngine>(rb_search_engine_options, rb_data->construct_state_registry(current_initial_state.get_values()));
	rb_search_engine->initialize();
	assert(rb_search_engine->get_status() == IN_PROGRESS);
	return IN_PROGRESS;
}


static std::shared_ptr<SearchEngine<GlobalState, GlobalOperator>> _parse(options::OptionParser &parser) {
	SearchEngine<GlobalState, GlobalOperator>::add_options_to_parser(parser);
	IncrementalRedBlackSearch::add_options_to_parser(parser);
	auto opts = parser.parse();
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<IncrementalRedBlackSearch>(opts);
}

static options::PluginShared<SearchEngine<GlobalState, GlobalOperator>> _plugin("incremental_rb", _parse);

}
