#include "incremental_redblack_search.h"

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
	  incremental_redblack_search_statistics(),
	  rb_data(std::make_unique<RBData>(*opts.get<std::shared_ptr<Painting>>("base_painting"))),
	  rb_search_engine(std::make_unique<InternalRBSearchEngine>(rb_search_engine_options, rb_data->construct_state_registry(g_initial_state_data))),
	  incremental_painting_strategy(opts.get<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy")),
	  continue_from_first_conflict(opts.get<bool>("continue_from_first_conflict")) {
	auto num_black = std::count_if(std::begin(rb_data->painting.get_painting()), std::end(rb_data->painting.get_painting()),
		[](auto b) { return !b; });
	std::cout << "Starting incremental red-black search, initial painting has " << num_black << " black variables ("
		<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)" << std::endl;
	auto initial_node = search_space.get_node(current_initial_state);
	initial_node.open_initial();
	initial_node.close();
	incremental_redblack_search_statistics.num_episodes = 1;
	initialize_rb_search_engine();
}

void IncrementalRedBlackSearch::initialize_rb_search_engine() {
	auto pref_operator_heuristics = rb_search_engine_options.get_list<Heuristic<RBState, RBOperator> *>("preferred");
	rb_search_engine->set_pref_operator_heuristics(pref_operator_heuristics);
	rb_search_engine->initialize();
}

void IncrementalRedBlackSearch::update_statistics() {
	statistics.inc_dead_ends(rb_search_engine->statistics.get_dead_ends());
	statistics.inc_evaluated_states(rb_search_engine->statistics.get_evaluated_states());
	statistics.inc_evaluations(rb_search_engine->statistics.get_evaluations());
	statistics.inc_expanded(rb_search_engine->statistics.get_expanded());
	statistics.inc_generated(rb_search_engine->statistics.get_generated());
	statistics.inc_generated_ops(rb_search_engine->statistics.get_generated_ops());
	statistics.inc_reopened(rb_search_engine->statistics.get_reopened());
}

auto IncrementalRedBlackSearch::update_search_space_and_check_plan(const RBPlan &plan) -> std::pair<bool, GlobalState> {
	auto current_state = current_initial_state;
	for (const auto rb_op : plan) {
		const auto &op = rb_op->get_base_operator();
		if (!op.is_applicable(current_state))
			return {false, current_state};
		auto current_parent_node = search_space.get_node(current_state);
		assert(current_parent_node.is_closed());
		current_state = state_registry->get_successor_state(current_state, op);
		auto successor_node = search_space.get_node(current_state);
		if (successor_node.is_new()) {
			successor_node.open(current_parent_node, &op);
			successor_node.close();
		} else if (successor_node.is_closed() && current_parent_node.get_g() + get_adjusted_cost(op) < successor_node.get_g()) {
			successor_node.reopen(current_parent_node, &op);
			successor_node.close();
		}
		assert(successor_node.is_closed());
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
	parser.add_option<std::shared_ptr<Painting>>("base_painting", "painting to be used in the initial red-black search", "all_red()");
	parser.add_option<Heuristic<RBState, RBOperator> *>("heuristic", "red-black heuristic that will be passed to the underlying red-black search engine", "ff_rb(transform=adapt_costs(cost_type=1))");
	parser.add_option<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy", "strategy for painting more variables black after finding a red-black solution with conflicts", "least_conflicts()");
	parser.add_option<bool>("continue_from_first_conflict", "Continue next iteration of red-black search from the first conflicting state in the previous red-black plan.", "true");
	add_succ_order_options(parser);
}

SearchStatus IncrementalRedBlackSearch::step() {
	assert(rb_search_engine->get_status() == IN_PROGRESS);
	auto status = rb_search_engine->step();
	if (status != IN_PROGRESS)
		update_statistics();
	if (status == IN_PROGRESS || status == TIMEOUT)
		return status;
	if (status == FAILED) {
		if (current_initial_state.get_id() == state_registry->get_initial_state().get_id()) {
			print_statistics();
			std::cout << "Proved task unsolvable." << std::endl;
			utils::exit_with(utils::ExitCode::UNSOLVABLE);
		} else {
			std::cout << "Red-black search failed to find a solution, restarting from the initial state..." << std::endl;
			current_initial_state = state_registry->get_initial_state();
			/*
			  NOTE: ideally, we would be able to reuse information about the
			  part of the search space that we already explored in the previous
			  search (from a different initial state), but this is VERY
			  difficult to do with FD's data structures.
			*/
			rb_search_engine = std::make_unique<InternalRBSearchEngine>(rb_search_engine_options, rb_data->construct_state_registry(current_initial_state.get_values()));
			initialize_rb_search_engine();
			assert(rb_search_engine->get_status() == IN_PROGRESS);
			++incremental_redblack_search_statistics.num_restarts;
			return IN_PROGRESS;
		}
	}
	assert(status == SOLVED);
	const auto &rb_plan = rb_search_engine->get_plan();
	auto [is_plan, resulting_state] = update_search_space_and_check_plan(rb_plan);
	if (is_plan) {
		auto plan = std::vector<const GlobalOperator *>();
		search_space.trace_path(resulting_state, plan);
		set_plan(plan);
		return SOLVED;
	}
	auto plan = std::vector<OperatorID>();
	plan.reserve(rb_plan.size());
	std::transform(std::begin(rb_plan), std::end(rb_plan), std::back_inserter(plan),
		[](const auto rb_operator) { return OperatorID(get_op_index_hacked(rb_operator)); });
	rb_data = std::make_unique<RBData>(incremental_painting_strategy->generate_next_painting(rb_data->painting, plan));
	auto num_black = std::count_if(std::begin(rb_data->painting.get_painting()), std::end(rb_data->painting.get_painting()),
		[](auto b) { return !b; });
	std::cout << "Red-black plan is not a real plan. Search continues with a new painting, "
		<< num_black << " black variables ("
		<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)..." << std::endl;
	if (continue_from_first_conflict)
		current_initial_state = resulting_state;
	rb_search_engine = std::make_unique<InternalRBSearchEngine>(rb_search_engine_options, rb_data->construct_state_registry(current_initial_state.get_values()));
	initialize_rb_search_engine();
	assert(rb_search_engine->get_status() == IN_PROGRESS);
	++incremental_redblack_search_statistics.num_episodes;
	return IN_PROGRESS;
}

void IncrementalRedBlackSearch::print_statistics() const {
	auto num_black = std::count_if(std::begin(rb_data->painting.get_painting()), std::end(rb_data->painting.get_painting()),
		[](auto b) { return !b; });
	std::cout << "Final painting has " << num_black << " black variables ("
		<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)" << std::endl;
	std::cout << "Performed " << incremental_redblack_search_statistics.num_episodes << " episodes of red-black search." << std::endl;
	std::cout << "Search was restarted " << incremental_redblack_search_statistics.num_restarts << " times after red-black search failed to find a solution." << std::endl;
	statistics.print_detailed_statistics();
	search_space.print_statistics();
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
