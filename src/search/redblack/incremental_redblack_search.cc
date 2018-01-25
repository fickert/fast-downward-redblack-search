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
	  red_actions_manager(),
	  always_recompute_red_plans(opts.get<bool>("always_recompute_red_plans")),
	  never_black_variables(PaintingFactory::get_cg_leaves_painting()) {
	auto rb_state_registry = rb_data->construct_state_registry(g_initial_state_data);
	if (plan_repair_heuristic) {
		red_actions_manager = std::make_unique<RedActionsManager>(rb_state_registry->get_operators());
		for (auto black_index : plan_repair_heuristic->get_black_indices())
			never_black_variables[black_index] = true;
	}
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

auto IncrementalRedBlackSearch::is_valid_relaxed_plan(const GlobalState& state, const std::vector<FactPair>& goal_facts, const std::vector<OperatorID>& relaxed_plan) -> bool {
	auto achieved_facts = std::vector<boost::dynamic_bitset<>>();
	achieved_facts.resize(g_root_task()->get_num_variables());
	for (auto i = 0u; i < achieved_facts.size(); ++i) {
		achieved_facts[i].resize(g_root_task()->get_variable_domain_size(i));
		achieved_facts[i].set(state[i]);
	}
	// NOTE: this assumes that the relaxed plan is applicable in the given order which may not be guaranteed
	for (const auto &op_id : relaxed_plan) {
		const auto &op = g_operators[op_id.get_index()];
		if (!std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), [&achieved_facts](const auto &precondition) {
			return achieved_facts[precondition.var][precondition.val];
		}))
			return false;
		for (const auto &effect : op.get_effects())
			if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), [&achieved_facts](const auto &condition) {
				return achieved_facts[condition.var][condition.val];
			}))
				achieved_facts[effect.var].set(effect.val);
	}
	return std::all_of(std::begin(goal_facts), std::end(goal_facts), [&achieved_facts](const auto &goal_fact) {
		return achieved_facts[goal_fact.var][goal_fact.value];
	});
}

auto IncrementalRedBlackSearch::repair_plan_and_update_search_space(const GlobalState &state, const std::vector<FactPair> &goal_facts, const std::vector<OperatorID> &partial_plan, const boost::dynamic_bitset<> &red_actions) -> std::pair<bool, GlobalState> {
	// first check if the relaxed plan is actually a relaxed plan
	//if (!is_valid_relaxed_plan(state, goal_facts, partial_plan)) {
	//	++incremental_redblack_search_statistics.num_broken_red_plans;
	//	return check_plan_and_update_search_space(state, partial_plan, goal_facts);
	//}
	assert(is_valid_relaxed_plan(state, goal_facts, partial_plan));
	// now that we made sure the relaxed plan is valid relaxed plan, attempt to repair it
	auto [repaired, repaired_partial_plan] = plan_repair_heuristic->compute_semi_relaxed_plan(state, goal_facts, partial_plan, red_actions);
	return repaired ?
		check_plan_and_update_search_space(state, repaired_partial_plan, goal_facts) :
		check_plan_and_update_search_space(state, partial_plan, goal_facts);
}

auto IncrementalRedBlackSearch::repair_plan_and_update_search_space(const RBPlan &plan, const std::vector<std::set<FactPair>> &marked_facts) -> std::pair<bool, GlobalState> {
	assert(plan_repair_heuristic);
	auto current_state = current_initial_state;
	auto current_partial_plan = std::vector<OperatorID>();
	auto current_red_actions = red_actions_manager->get_red_actions_for_state(current_state);
	auto current_marked_facts_it = std::begin(marked_facts);
	auto deferred_goal_facts = std::vector<FactPair>();
	auto [current_redblack_state, current_supporters] = static_cast<RBStateRegistry *>(&rb_search_engine->get_state_registry())->get_state_and_best_supporters(current_state.get_values());
	for (const auto rb_op : plan) {
		const auto op_index = get_op_index_hacked(rb_op);
		if (current_red_actions[op_index]) {
			current_partial_plan.emplace_back(op_index);
			continue;
		}
		// don't just pass the red preconditions of the next variable as facts that need to be achieved by the relaxed plan,
		// but also all the marked fact at that point, except the michael-black ones, except the ones directly needed as precondition
		auto current_goal_facts = std::vector<FactPair>();
		current_goal_facts.insert(std::end(current_goal_facts), std::begin(*current_marked_facts_it), std::end(*current_marked_facts_it));
		// remove all mercury-black facts from the current goal facts
		current_goal_facts.erase(std::remove_if(std::begin(current_goal_facts), std::end(current_goal_facts), [this](const auto &fact) {
			return std::find(std::begin(plan_repair_heuristic->get_black_indices()),
				                std::end(plan_repair_heuristic->get_black_indices()), fact.var)
				!= std::end(plan_repair_heuristic->get_black_indices());
		}), std::end(current_goal_facts));
		assert(std::is_sorted(std::begin(current_goal_facts), std::end(current_goal_facts)));
		// insert previously deferred goal facts
		assert(std::none_of(std::begin(deferred_goal_facts), std::end(deferred_goal_facts), [&current_goal_facts](const auto &deferred_goal_fact) {
			return std::binary_search(std::begin(current_goal_facts), std::end(current_goal_facts), deferred_goal_fact);
		}));
		const auto before_deferred_size = current_goal_facts.size();
		current_goal_facts.insert(std::end(current_goal_facts), std::begin(deferred_goal_facts), std::end(deferred_goal_facts));
		std::sort(std::begin(current_goal_facts) + before_deferred_size, std::end(current_goal_facts));
		std::inplace_merge(std::begin(current_goal_facts), std::begin(current_goal_facts) + before_deferred_size, std::end(current_goal_facts));
		// defer all the marked facts that cannot be reached from this state to a later step
		const auto begin_deferred = std::remove_if(std::begin(current_goal_facts), std::end(current_goal_facts), [&current_redblack_state](const auto &fact) {
			return !current_redblack_state.has_fact(fact.var, fact.value);
		});
		deferred_goal_facts.clear();
		deferred_goal_facts.insert(std::begin(deferred_goal_facts), begin_deferred, std::end(current_goal_facts));
		current_goal_facts.erase(begin_deferred, std::end(current_goal_facts));
		// check if all the preconditions of the next black variable can actually be reached
		auto unreachable_preconditions = false;
		const auto size = current_goal_facts.size();
		for (const auto precondition : rb_op->get_red_preconditions()) {
			if (!current_redblack_state.has_fact(precondition->var, precondition->val))
				unreachable_preconditions = true;
			// NOTE: in theory, we only need to insert the mercury-black preconditions, because the red preconditions should already be achieved the red plan,
			// or in any previous red plan
			// however, they may have been deleted again, so it doesn't hurt to add them (the worst that can happen is that mercury can't repair the plan, but otherwise the plan would be broken anyway)
			auto precondition_fact = FactPair(precondition->var, precondition->val);
			if (!std::binary_search(std::begin(current_goal_facts), std::begin(current_goal_facts) + size, precondition_fact))
				current_goal_facts.emplace_back(precondition_fact);
		}
		// NOTE: current_goal_facts no longer sorted
		if (unreachable_preconditions)
			// the next action has preconditions that are not relaxed reachable from the current real state
			return check_plan_and_update_search_space(current_state, current_partial_plan, current_goal_facts);
		if (always_recompute_red_plans || !is_valid_relaxed_plan(current_state, current_goal_facts, current_partial_plan))
			current_partial_plan = get_red_plan(current_supporters, current_state, current_goal_facts, true);
		assert(is_valid_relaxed_plan(current_state, current_goal_facts, current_partial_plan));
		auto [is_plan, resulting_state] = repair_plan_and_update_search_space(current_state, current_goal_facts, current_partial_plan, current_red_actions);
		if (!is_plan)
			return {false, resulting_state};
		current_state = get_successor_and_update_search_space(resulting_state, rb_op->get_base_operator());
		current_partial_plan.clear();
		current_red_actions = red_actions_manager->get_red_actions_for_state(current_state);
		++current_marked_facts_it;
		std::tie(current_redblack_state, current_supporters) = static_cast<RBStateRegistry *>(&rb_search_engine->get_state_registry())->get_state_and_best_supporters(current_state.get_values());
	}
	auto goal_facts = std::vector<FactPair>();
	goal_facts.reserve(g_goal.size());
	std::transform(std::begin(g_goal), std::end(g_goal), std::back_inserter(goal_facts), [](const auto &goal) { return FactPair{goal.first, goal.second}; });
	// in the final layer, the marked facts should be a subset of the goal facts (some goals may have been achieved earlier)
	assert(std::is_sorted(std::begin(goal_facts), std::end(goal_facts)));
	assert(current_marked_facts_it == std::end(marked_facts) - 1);
	// NOTE: for consistency with the above we should include the deferred goal facts and marked facts into the set of goal facts
	// however, using only the actual goals is sufficient because the other marked and deferred facts are also either
	// goal facts themselves, or they are preconditions for actions that are used in the relaxed plan, and we can just
	// generate a new relaxed plan anyway (which might also have different preconditions than the original one)
	if (!std::all_of(std::begin(goal_facts), std::end(goal_facts), [&current_redblack_state](const auto &goal_fact) {
		return current_redblack_state.has_fact(goal_fact.var, goal_fact.value);
	}))
		// the goal is not reachable from this state
		return check_plan_and_update_search_space(current_state, current_partial_plan, goal_facts);
	if (always_recompute_red_plans || !is_valid_relaxed_plan(current_state, goal_facts, current_partial_plan))
		current_partial_plan = get_red_plan(current_supporters, current_state, goal_facts, true);
	assert(is_valid_relaxed_plan(current_state, goal_facts, current_partial_plan));
	return repair_plan_and_update_search_space(current_state, goal_facts, current_partial_plan, current_red_actions);
}

auto IncrementalRedBlackSearch::relaxed_repair_plan(const RBPlan &plan, const std::vector<std::set<FactPair>> &marked_facts) -> std::vector<OperatorID> {
#ifndef NDEBUG
	// NOTE: see note below, ignoring conditional effects
	verify_no_conditional_effects();
#endif
	// repair plan under red-black semantics (to make sure we only have conflicts on non-mercury-black variables)
	assert(plan_repair_heuristic);
	auto current_redblack_state = std::vector<boost::dynamic_bitset<>>();
	current_redblack_state.reserve(g_root_task()->get_num_variables());
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var) {
		current_redblack_state.emplace_back(boost::dynamic_bitset<>(g_root_task()->get_variable_domain_size(var)));
		current_redblack_state.back()[current_initial_state[var]] = true;
	}

	auto get_available_facts = [&current_redblack_state]() {
		auto available_facts = std::vector<FactPair>();
		for (auto var = 0u; var < current_redblack_state.size(); ++var)
			for (auto value = 0u; value < current_redblack_state[var].size(); ++value)
				if (current_redblack_state[var][value])
					available_facts.emplace_back(var, value);
		return available_facts;
	};

	auto current_partial_plan = std::vector<OperatorID>();
	auto repaired_plan = std::vector<OperatorID>();
	auto current_red_actions = red_actions_manager->get_red_actions_for_state(current_redblack_state);
	auto current_marked_facts_it = std::begin(marked_facts);

	auto retry_with_more_mercury_red = [this, &repaired_plan, &current_partial_plan](auto rb_plan_it) {
		auto to_be_painted_red = std::vector<int>();
		for (auto var = 0; var < g_root_task()->get_num_variables(); ++var) {
			if (!plan_repair_heuristic->is_black(var))
				continue;
			auto predecessors = causal_graph::get_causal_graph(g_root_task().get()).get_predecessors(var);
			assert(std::is_sorted(std::begin(predecessors), std::end(predecessors)));
			auto successors = causal_graph::get_causal_graph(g_root_task().get()).get_successors(var);
			assert(std::is_sorted(std::begin(successors), std::end(successors)));
			auto both = std::vector<int>();
			std::set_intersection(std::begin(predecessors), std::end(predecessors),
				std::begin(successors), std::end(successors), std::back_inserter(both));
			if (std::any_of(std::begin(both), std::end(both), [this](const auto var) { return rb_data->painting.is_black_var(var); })) {
				never_black_variables[var] = false;
				to_be_painted_red.push_back(var);
			}
		}
		std::sort(std::begin(to_be_painted_red), std::end(to_be_painted_red));
		to_be_painted_red.erase(std::unique(std::begin(to_be_painted_red), std::end(to_be_painted_red)), std::end(to_be_painted_red));
		if (plan_repair_heuristic->get_num_black() == to_be_painted_red.size()) {
			// would paint all remaining variables red ==> delete the plan repair heuristic
			plan_repair_heuristic.reset();
			repaired_plan.insert(std::end(repaired_plan), std::begin(current_partial_plan), std::end(current_partial_plan));
			std::transform(rb_plan_it, std::end(plan), std::back_inserter(repaired_plan), [](const auto rb_op) { return rb_op->get_id(); });
			return repaired_plan;
		}
		plan_repair_heuristic->make_red(to_be_painted_red);
		return relaxed_repair_plan(plan, marked_facts);
	};

	for (auto rb_plan_it = std::begin(plan); rb_plan_it != std::end(plan); ++rb_plan_it) {
		const auto rb_op = *rb_plan_it;
		assert(current_marked_facts_it != std::end(marked_facts));
		const auto op_index = get_op_index_hacked(rb_op);
		if (current_red_actions[op_index]) {
			current_partial_plan.emplace_back(op_index);
			continue;
		}
		// don't just pass the red preconditions of the next variable as facts that need to be achieved by the relaxed plan,
		// but also all the marked facts at that point, except the michael-black ones, except the ones directly needed as precondition
		auto current_goal_facts = std::vector<FactPair>();
		current_goal_facts.insert(std::end(current_goal_facts), std::begin(*current_marked_facts_it), std::end(*current_marked_facts_it));
		// remove all mercury-black facts from the current goal facts
		current_goal_facts.erase(std::remove_if(std::begin(current_goal_facts), std::end(current_goal_facts), [this](const auto &fact) {
			return std::find(std::begin(plan_repair_heuristic->get_black_indices()),
				                std::end(plan_repair_heuristic->get_black_indices()), fact.var)
				!= std::end(plan_repair_heuristic->get_black_indices());
		}), std::end(current_goal_facts));
		assert(std::is_sorted(std::begin(current_goal_facts), std::end(current_goal_facts)));
		const auto size = current_goal_facts.size();
		for (const auto &precondition : rb_op->get_red_preconditions()) {
			// NOTE: in theory, we only need to insert the mercury-black preconditions, because the red preconditions should already be achieved by the red plan,
			// (or in any previous red plan), and with red-black semantics they can't have been deleted again
			// however, we don't need to take them out of the set of goal facts (they will be ignored anyway by the plan repair)
			auto precondition_fact = FactPair(precondition->var, precondition->val);
			if (!std::binary_search(std::begin(current_goal_facts), std::begin(current_goal_facts) + size, precondition_fact))
				current_goal_facts.emplace_back(precondition_fact);
		}
		// NOTE: current_goal_facts no longer sorted
		auto [repaired, repaired_partial_plan] = plan_repair_heuristic->compute_semi_relaxed_plan(get_available_facts(), rb_data->painting.get_painting(), current_goal_facts, current_partial_plan, current_red_actions);
		if (!repaired)
			// relaxed plan repair failed. paint mercury-black variables red and try again
			return retry_with_more_mercury_red(rb_plan_it);
		repaired_partial_plan.emplace_back(rb_op->get_id());
		for (const auto &op_id : repaired_partial_plan) {
			assert(std::all_of(std::begin(g_operators[op_id.get_index()].get_preconditions()), std::end(g_operators[op_id.get_index()].get_preconditions()),
				[&current_redblack_state](const auto &precondition) { return current_redblack_state[precondition.var][precondition.val]; }));
			for (const auto &effect : g_operators[op_id.get_index()].get_effects()) {
				// NOTE: ignoring conditional effects here
				// if variable is black (either here, or in mercury), use normal semantics, otherwise accumulate values
				if (rb_data->painting.is_black_var(effect.var) || plan_repair_heuristic->is_black(effect.var))
					current_redblack_state[effect.var].reset();
				current_redblack_state[effect.var].set(effect.val);
			}
		}
		repaired_plan.insert(std::end(repaired_plan), std::begin(repaired_partial_plan), std::end(repaired_partial_plan));
		current_partial_plan.clear();
		current_red_actions = red_actions_manager->get_red_actions_for_state(current_redblack_state);
		++current_marked_facts_it;
	}
	assert(current_marked_facts_it + 1 == std::end(marked_facts));
	auto goal_facts = std::vector<FactPair>();
	goal_facts.reserve(g_goal.size());
	std::transform(std::begin(g_goal), std::end(g_goal), std::back_inserter(goal_facts), [](const auto &goal) { return FactPair{goal.first, goal.second}; });
	auto [repaired, repaired_partial_plan] = plan_repair_heuristic->compute_semi_relaxed_plan(get_available_facts(), rb_data->painting.get_painting(), goal_facts, current_partial_plan, current_red_actions);
	if (!repaired)
		return retry_with_more_mercury_red(std::end(plan));
	repaired_plan.insert(std::end(repaired_plan), std::begin(repaired_partial_plan), std::end(repaired_partial_plan));
	return repaired_plan;
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
	parser.add_option<bool>("always_recompute_red_plans", "when trying to repair red partial plans, always replace the old red plan by a new one based on the real state", "true");
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
	auto marked_facts = std::unique_ptr<std::vector<std::set<FactPair>>>(nullptr);
	if (plan_repair_heuristic) {
		marked_facts = static_cast<RBStateRegistry *>(&rb_search_engine->get_state_registry())->get_last_marked_facts();
		if (!is_plan)
			std::tie(is_plan, resulting_state) = repair_plan_and_update_search_space(rb_plan, *marked_facts);
	}
	if (is_plan) {
		auto plan = std::vector<const GlobalOperator *>();
		search_space->trace_path(resulting_state, plan);
		set_plan(plan);
		return SOLVED;
	}
	auto plan = std::vector<OperatorID>();
	if (plan_repair_heuristic) {
		assert(marked_facts);
		// at least repair conflicts on mercury-black variables
		plan = relaxed_repair_plan(rb_plan, *marked_facts);
	} else {
		plan.reserve(rb_plan.size());
		std::transform(std::begin(rb_plan), std::end(rb_plan), std::back_inserter(plan),
			[](const auto rb_operator) { return OperatorID(get_op_index_hacked(rb_operator)); });
	}
	rb_data = std::make_unique<RBData>(incremental_painting_strategy->generate_next_painting(rb_data->painting, plan, &never_black_variables));
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
	std::cout << "Number of broken red plans: " << incremental_redblack_search_statistics.num_broken_red_plans << std::endl;
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
