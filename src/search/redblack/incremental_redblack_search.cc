#include "incremental_redblack_search.h"

#include "util.h"
#include "../search_engine.h"
#include "../options/option_parser.h"
#include "../search_engines/lazy_search.h"
#include "../search_engines/search_common.h"
#include "incremental_painting_strategy.h"


namespace redblack {

auto paint_red_conditional_effect_conditions_black(const Painting &base_painting) -> Painting {
	if (!any_conditional_effect_condition_is_red(base_painting))
		return base_painting;
	auto num_black = std::count_if(std::begin(base_painting.get_painting()), std::end(base_painting.get_painting()),
		[](auto b) { return !b; });
	std::cout << "Base painting has operators with red conditional effect conditions! Original painting had " << num_black << " black variables("
		<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%), updating..." << std::endl;
	return get_no_red_conditional_effect_conditions_painting(base_painting);
}

IncrementalRedBlackSearch::IncrementalRedBlackSearch(const options::Options &opts)
	: SearchEngine<>(opts),
	  rb_search_engine_options(get_rb_search_options(opts)),
	  current_initial_state(state_registry->get_initial_state()),
	  incremental_redblack_search_statistics(),
	  rb_data(std::make_unique<RBData>(paint_red_conditional_effect_conditions_black(*opts.get<std::shared_ptr<Painting>>("base_painting")))),
	  rb_search_engine(),
	  incremental_painting_strategy(opts.get<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy")),
	  continue_from_first_conflict(opts.get<bool>("continue_from_first_conflict")),
	  plan_repair_heuristic(get_rb_plan_repair_heuristic(opts)),
	  red_actions_manager() {
	auto rb_state_registry = rb_data->construct_state_registry(g_initial_state_data);
	if (plan_repair_heuristic)
		red_actions_manager = std::make_unique<RedActionsManager>(rb_state_registry->get_operators());
	rb_search_engine = std::make_unique<InternalRBSearchEngine>(rb_search_engine_options, std::move(rb_state_registry));
	auto num_black = std::count_if(std::begin(rb_data->painting.get_painting()), std::end(rb_data->painting.get_painting()),
		[](auto b) { return !b; });
	std::cout << "Starting incremental red-black search, initial painting has " << num_black << " black variables ("
		<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)" << std::endl;
	auto initial_node = search_space->get_node(current_initial_state);
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

auto IncrementalRedBlackSearch::get_successor_and_update_search_space(const GlobalState &state, const GlobalOperator& op) -> GlobalState {
	assert(op.is_applicable(state));
	auto node = search_space->get_node(state);
	assert(node.is_closed());
	auto successor_state = state_registry->get_successor_state(state, op);
	auto successor_node = search_space->get_node(successor_state);
	if (successor_node.is_new()) {
		successor_node.open(node, &op);
		successor_node.close();
	} else if (successor_node.is_closed() && node.get_g() + get_adjusted_cost(op) < successor_node.get_g()) {
		successor_node.reopen(node, &op);
		successor_node.close();
	}
	assert(successor_node.is_closed());
	return successor_state;
}

auto IncrementalRedBlackSearch::check_plan_and_update_search_space(const GlobalState &state, const std::vector<OperatorID> &plan, const std::vector<FactPair> &goal_facts) -> std::pair<bool, GlobalState> {
	auto current_state = state;
	for (const auto op_id : plan) {
		const auto &op = g_operators[op_id.get_index()];
		if (!op.is_applicable(current_state))
			return {false, current_state};
		current_state = get_successor_and_update_search_space(current_state, op);
	}
	return {std::all_of(std::begin(goal_facts), std::end(goal_facts), [&current_state](const auto &fact) { return current_state[fact.var] == fact.value; }), current_state};
}

auto IncrementalRedBlackSearch::check_plan_and_update_search_space(const RBPlan &plan) -> std::pair<bool, GlobalState> {
	auto current_state = current_initial_state;
	for (const auto rb_op : plan) {
		const auto &op = rb_op->get_base_operator();
		if (!op.is_applicable(current_state))
			return {false, current_state};
		current_state = get_successor_and_update_search_space(current_state, op);
	}
	return {test_goal(current_state), current_state};
}

auto IncrementalRedBlackSearch::repair_plan_and_update_search_space(const GlobalState &state, const std::vector<FactPair> &goal_facts, const std::vector<OperatorID> &partial_plan, const boost::dynamic_bitset<> &red_actions) -> std::pair<bool, GlobalState> {
	// first check if the relaxed plan is actually a relaxed plan
	auto achieved_facts = std::vector<boost::dynamic_bitset<>>();
	achieved_facts.resize(g_root_task()->get_num_variables());
	for (auto i = 0u; i < achieved_facts.size(); ++i) {
		achieved_facts[i].resize(g_root_task()->get_variable_domain_size(i));
		achieved_facts[i].set(state[i]);
	}
	// TODO: this assumes that the relaxed plan is applicable in the given order which may not be guaranteed
	for (const auto &op_id : partial_plan) {
		const auto &op = g_operators[op_id.get_index()];
		if (!std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), [&achieved_facts](const auto &precondition) {
			return achieved_facts[precondition.var][precondition.val];
		}))
			return check_plan_and_update_search_space(state, partial_plan, goal_facts);
		for (const auto &effect : op.get_effects())
			if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), [&achieved_facts](const auto &condition) {
				return achieved_facts[condition.var][condition.val];
			}))
				achieved_facts[effect.var].set(effect.val);
	}
	if (!std::all_of(std::begin(goal_facts), std::end(goal_facts), [&achieved_facts](const auto &goal_fact) {
		return achieved_facts[goal_fact.var][goal_fact.value];
	}))
		return check_plan_and_update_search_space(state, partial_plan, goal_facts);
	// now that we made sure the relaxed plan is valid relaxed plan, attempt to repair it
	auto [repaired, repaired_partial_plan] = plan_repair_heuristic->compute_semi_relaxed_plan(state, goal_facts, partial_plan, red_actions);
	return repaired ?
		check_plan_and_update_search_space(state, repaired_partial_plan, goal_facts) :
		check_plan_and_update_search_space(state, partial_plan, goal_facts);
}

auto IncrementalRedBlackSearch::repair_plan_and_update_search_space(const RBPlan &plan) -> std::pair<bool, GlobalState> {
	assert(plan_repair_heuristic);
	auto current_state = current_initial_state;
	auto current_partial_plan = std::vector<OperatorID>();
	auto current_red_actions = red_actions_manager->get_red_actions_for_state(current_state);
	for (const auto rb_op : plan) {
		const auto op_index = get_op_index_hacked(rb_op);
		if (current_red_actions[op_index]) {
			current_partial_plan.emplace_back(op_index);
		} else {
			// TODO: test non-repaired partial plan before repairing?
			auto precondition_facts = std::vector<FactPair>();
			precondition_facts.reserve(rb_op->get_red_preconditions().size());
			std::transform(std::begin(rb_op->get_red_preconditions()), std::end(rb_op->get_red_preconditions()), std::back_inserter(precondition_facts),
				[](const auto &precondition) { return FactPair{precondition->var, precondition->val}; });
			auto [is_plan, resulting_state] = repair_plan_and_update_search_space(current_state, precondition_facts, current_partial_plan, current_red_actions);
			if (!is_plan)
				return {false, resulting_state};
			current_state = get_successor_and_update_search_space(resulting_state, rb_op->get_base_operator());
			current_partial_plan.clear();
			current_red_actions = red_actions_manager->get_red_actions_for_state(current_state);
		}
	}
	auto goal_facts = std::vector<FactPair>();
	goal_facts.reserve(g_goal.size());
	std::transform(std::begin(g_goal), std::end(g_goal), std::back_inserter(goal_facts), [](const auto &goal) { return FactPair{goal.first, goal.second}; });
	return repair_plan_and_update_search_space(current_state, goal_facts, current_partial_plan, current_red_actions);
}

void IncrementalRedBlackSearch::set_solution(const Plan &partial_plan, const GlobalState &state) {
	assert(!search_space->get_node(state).is_new());
	auto solution = Plan();
	search_space->trace_path(state, solution);
	solution.insert(std::end(solution), std::begin(partial_plan), std::end(partial_plan));
	set_plan(solution);
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

auto IncrementalRedBlackSearch::get_rb_plan_repair_heuristic(const options::Options &opts) -> std::shared_ptr<RedBlackDAGFactFollowingHeuristic> {
	if (!opts.get<bool>("repair_red_plans"))
		return nullptr;
	auto plan_repair_options = options::Options();
	plan_repair_options.set<std::shared_ptr<AbstractTask>>("transform", g_root_task());
	plan_repair_options.set<bool>("cache_estimates", false);
	plan_repair_options.set<bool>("extract_plan", true);
	plan_repair_options.set<bool>("paint_roots_black", false);
	plan_repair_options.set<bool>("ignore_invertibility", false);
	plan_repair_options.set<int>("prefs", 0);
	plan_repair_options.set<bool>("applicable_paths_first", true);
	plan_repair_options.set<bool>("next_red_action_test", true);
	plan_repair_options.set<bool>("use_connected", true);
	plan_repair_options.set<bool>("extract_plan_no_blacks", false);
	auto mercury_heuristic = std::make_shared<RedBlackDAGFactFollowingHeuristic>(plan_repair_options);
	if (mercury_heuristic->get_num_black() == 0)
		return nullptr;
	return mercury_heuristic;
}

void IncrementalRedBlackSearch::add_options_to_parser(options::OptionParser &parser) {
	parser.add_option<std::shared_ptr<Painting>>("base_painting", "painting to be used in the initial red-black search", "all_red()");
	parser.add_option<Heuristic<RBState, RBOperator> *>("heuristic", "red-black heuristic that will be passed to the underlying red-black search engine", "ff_rb(transform=adapt_costs(cost_type=1))");
	parser.add_option<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy", "strategy for painting more variables black after finding a red-black solution with conflicts", "least_conflicts()");
	parser.add_option<bool>("continue_from_first_conflict", "Continue next iteration of red-black search from the first conflicting state in the previous red-black plan.", "true");
	parser.add_option<bool>("repair_red_plans", "attempt to repair red plans using Mercury", "true");
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
	auto [is_plan, resulting_state] = check_plan_and_update_search_space(rb_plan);
	if (!is_plan && plan_repair_heuristic)
		std::tie(is_plan, resulting_state) = repair_plan_and_update_search_space(rb_plan);
	if (is_plan) {
		auto plan = std::vector<const GlobalOperator *>();
		search_space->trace_path(resulting_state, plan);
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
	auto rb_state_registry = rb_data->construct_state_registry(current_initial_state.get_values());
	if (plan_repair_heuristic)
		red_actions_manager = std::make_unique<RedActionsManager>(rb_state_registry->get_operators());
	rb_search_engine = std::make_unique<InternalRBSearchEngine>(rb_search_engine_options, std::move(rb_state_registry));
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
	search_space->print_statistics();
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
