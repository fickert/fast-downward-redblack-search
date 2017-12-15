#include "hierarchical_redblack_search.h"

#include "search_space.h"
#include "util.h"
#include "../search_engine.h"
#include "../options/option_parser.h"
#include "../search_engines/lazy_search.h"
#include "../search_engines/search_common.h"
#include "incremental_painting_strategy.h"


namespace redblack {

#ifndef NDEBUG
void verify_black_variable_values(const RBState &rb_state, const GlobalState &global_state) {
	for (auto i = 0; i < g_root_task()->get_num_variables(); ++i)
		assert(rb_state.get_painting().is_red_var(i) || rb_state[i] == global_state[i]);
}
#else
void verify_black_variable_values(const RBState &, const GlobalState &) {}
#endif

HierarchicalRedBlackSearch::HierarchicalRedBlackSearch(const options::Options &opts,
                                                       std::shared_ptr<RBStateRegistry> state_registry,
                                                       std::shared_ptr<SearchSpace<RBState, RBOperator>> search_space,
                                                       GlobalState current_initial_state,
                                                       StateRegistryBase<GlobalState, GlobalOperator> &global_state_registry,
                                                       SearchSpace<GlobalState, GlobalOperator> &global_search_space,
                                                       std::map<InternalPaintingType, std::tuple<std::shared_ptr<RBData>, std::shared_ptr<RBStateRegistry>, std::shared_ptr<SearchSpace<RBState, RBOperator>>>> &rb_search_spaces,
                                                       int num_black,
                                                       bool initial_state_is_preferred,
                                                       int initial_state_h_value)
	: LazySearch<RBState, RBOperator>(opts, state_registry, search_space),
	  is_current_preferred(initial_state_is_preferred),
	  current_key(initial_state_h_value),
	  child_searches(),
	  current_child_search(nullptr),
	  current_child_search_index(-1),
	  current_best_supporters(static_cast<RBStateRegistry *>(this->state_registry.get())->get_initial_state_best_supporters()),
	  corresponding_global_state(),
	  pending_corresponding_global_states(),
	  global_goal_state(StateID::no_state),
	  search_options(opts),
	  incremental_painting_strategy(opts.get<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy")),
	  current_initial_state(current_initial_state),
	  global_state_registry(global_state_registry),
	  global_search_space(global_search_space),
	  rb_search_spaces(rb_search_spaces),
	  num_black(num_black),
	  hierarchical_redblack_search_statistics() {
	auto pref_operator_heuristics = search_options.get_list<Heuristic<RBState, RBOperator> *>("preferred");
	set_pref_operator_heuristics(pref_operator_heuristics);
	LazySearch<RBState, RBOperator>::initialize();
	corresponding_global_state.insert({current_state.get_id(), current_initial_state.get_id()});
	// auto num_black = std::count_if(
	// 	std::begin(static_cast<RBStateRegistry *>(this->state_registry.get())->get_painting().get_painting()),
	// 	std::end(static_cast<RBStateRegistry *>(this->state_registry.get())->get_painting().get_painting()),
	// 	[](auto b) { return !b; });
	// std::cout << "Starting red-black search with a painting with "
	// 	<< num_black << " black variables ("
	// 	<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)..." << std::endl;
	// std::cout << "Painting " << static_cast<RBStateRegistry *>(this->state_registry.get())->get_painting() << std::endl;
	// std::cout << "This search is: " << this << std::endl;
}


auto get_random_new_painting(const Painting &last_painting, int num_black) -> Painting {
	assert(!std::all_of(std::begin(last_painting.get_painting()), std::end(last_painting.get_painting()), [](const auto is_red) { return !is_red; }));
	auto red_variables = std::vector<std::size_t>();
	red_variables.reserve(g_root_task()->get_num_variables());
	for (auto i = 0; i < g_root_task()->get_num_variables(); ++i)
		if (last_painting.is_red_var(i))
			red_variables.push_back(i);
	assert(!red_variables.empty());
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(std::begin(red_variables), std::end(red_variables), g);
	auto next_painting = last_painting.get_painting();
	for (auto i = 0u; i < std::min<std::size_t>(red_variables.size(), num_black); ++i)
		next_painting[red_variables[i]] = false;
	return get_no_red_conditional_effect_conditions_painting(Painting(next_painting));
}

void HierarchicalRedBlackSearch::enqueue_new_search(const Painting &painting, const GlobalState &initial_state, int key, bool preferred, EvaluationContext<RBState, RBOperator> &new_eval_context) {
	auto rb_search_space_it = rb_search_spaces.find(painting.get_painting());
	auto painting_is_new = false;
	if (rb_search_space_it == std::end(rb_search_spaces)) {
		auto new_rb_data = std::make_shared<RBData>(painting);
		auto new_state_registry = std::shared_ptr<RBStateRegistry>(new_rb_data->construct_state_registry(initial_state.get_values()));
		auto new_search_space = std::make_shared<SearchSpace<RBState, RBOperator>>(*new_state_registry, static_cast<OperatorCost>(search_options.get_enum("cost_type")));
		rb_search_space_it = rb_search_spaces.insert({painting.get_painting(), {new_rb_data, new_state_registry, new_search_space}}).first;
		painting_is_new = true;
	}
	assert(!initial_state.get_values().empty());
	assert(rb_search_space_it != std::end(rb_search_spaces));
	child_searches[current_state.get_id()].emplace_back(std::make_unique<HierarchicalRedBlackSearch>(
		search_options, std::get<1>(rb_search_space_it->second), std::get<2>(rb_search_space_it->second),
		initial_state, global_state_registry, global_search_space, rb_search_spaces, num_black, preferred, key));
	if (!painting_is_new) {
		// state registry initial state doesn't match the actual initial state that should be used in the search
		auto &child_search = *child_searches.at(current_state.get_id()).back();
		std::tie(child_search.current_state, child_search.current_best_supporters) = std::get<1>(rb_search_space_it->second)->get_state_and_best_supporters(initial_state.get_values());
		child_search.current_eval_context = EvaluationContext<RBState, RBOperator>(child_search.current_state, 0, true, &child_search.statistics);
		child_search.corresponding_global_state.clear();
		child_search.corresponding_global_state.insert({child_search.current_state.get_id(), initial_state.get_id()});
	}
	assert(!child_searches.at(current_state.get_id()).empty());
	open_list->insert(new_eval_context, {current_state.get_id(), -static_cast<int>(child_searches.at(current_state.get_id()).size() - 1) - 1});
}

auto HierarchicalRedBlackSearch::get_hacked_cache_for_key(int key) const -> HeuristicCache<RBState, RBOperator> {
	auto hacked_eval_result = EvaluationResult();
	hacked_eval_result.set_h_value(key);
	// Note: the current_state in the cache is not always the intended state, but the open list doesn't care
	auto hacked_cache = HeuristicCache<RBState, RBOperator>(current_state);
	assert(heuristics.size() == 1);
	hacked_cache[heuristics.front()] = hacked_eval_result;
	return hacked_cache;
}

SearchStatus HierarchicalRedBlackSearch::step() {
	// Invariants:
	// - current_state is the next state for which we want to compute the heuristic.
	// - current_predecessor is a permanent pointer to the predecessor of that state.
	// - current_operator is the operator which leads to current_state from predecessor.
	// - current_g is the g value of the current state according to the cost_type
	// - current_real_g is the g value of the current state (using real costs)

	if (current_child_search) {
		assert(*current_child_search);
		auto result = (**current_child_search).step();
		switch (result) {
		case SOLVED:
			global_goal_state = (**current_child_search).get_goal_state();
			return SOLVED;
		case FAILED: {
			const auto &painting = static_cast<RBStateRegistry *>((**current_child_search).state_registry.get())->get_painting().get_painting();
			if (std::none_of(std::begin(painting), std::end(painting), [](const auto is_red) { return is_red; })) {
				// all black painting ==> search space exhausted below this child search's initial state
				current_child_search->reset();
				break;
			}
			// no plan to search conflicts in ...
			auto new_painting = get_random_new_painting(painting, num_black);
			// insert this child search into the open list, using the current state's key as the new key as well
			assert(*current_child_search);
			auto new_eval_context = EvaluationContext<RBState, RBOperator>(get_hacked_cache_for_key(current_key), current_g, is_current_preferred, nullptr);
			enqueue_new_search(new_painting, (**current_child_search).current_initial_state, current_key, is_current_preferred, new_eval_context);
			// Note: enqueue_new_search invalidates the current_child_search pointer
			child_searches[current_state.get_id()][current_child_search_index].reset();
			break;
		}
		case TIMEOUT:
			return TIMEOUT;
		case IN_PROGRESS: {
			// reinsert this "state" into the open list, using the key of the current search node from the child search
			auto hacked_eval_context = EvaluationContext<RBState, RBOperator>(get_hacked_cache_for_key((**current_child_search).get_current_key()), current_g, is_current_preferred, nullptr);
			assert(current_child_search_index >= 0);
			//std::cout << "continuing child search after having done a step in child search" << std::endl;
			open_list->insert(hacked_eval_context, {current_predecessor_id, -current_child_search_index - 1});
			break;
		}
		default:
			assert(false && "unreachable");
			utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
		}
	} else {
		//std::cout << "doing step in search " << this << ": expanding node with parent " << current_predecessor_id << " and operator " << current_operator << std::endl;
		auto node = search_space->get_node(current_state);
		bool reopen = reopen_closed_nodes && !node.is_new() && !node.is_dead_end() && (current_g < node.get_g());
		if (node.is_new() || reopen) {
			StateID dummy_id = current_predecessor_id;
			// HACK! HACK! we do this because SearchNode has no default/copy constructor
			if (dummy_id == StateID::no_state) {
				const auto &initial_state = state_registry->get_initial_state();
				dummy_id = initial_state.get_id();
			}
			auto parent_state = state_registry->lookup_state(dummy_id);
			auto parent_node = search_space->get_node(parent_state);

			if (current_operator) {
				for (auto heuristic : heuristics)
					heuristic->notify_state_transition(
						parent_state, *current_operator, current_state);
			}
			statistics.inc_evaluated_states();
			assert(current_state.get_id() == current_eval_context.get_state().get_id());
			if (!open_list->is_dead_end(current_eval_context)) {
				// TODO: Generalize code for using multiple heuristics.
				if (reopen) {
					node.reopen(parent_node, current_operator);
					statistics.inc_reopened();
				} else if (current_predecessor_id == StateID::no_state) {
					node.open_initial();
					if (search_progress.check_progress(current_eval_context))
						print_checkpoint_line(current_g);
				} else {
					node.open(parent_node, current_operator);
				}
				node.close();
				if (test_goal(current_state)) {
					assert(corresponding_global_state.find(current_state.get_id()) != std::end(corresponding_global_state));
					auto current_global_state = global_state_registry.lookup_state(corresponding_global_state.at(current_state.get_id()));
					verify_black_variable_values(current_state, current_global_state);
					assert(global_search_space.get_node(current_global_state).is_closed());
					auto goal_facts = std::vector<FactPair>();
					goal_facts.reserve(g_goal.size());
					std::transform(std::begin(g_goal), std::end(g_goal), std::back_inserter(goal_facts), [](const auto &goal) { return FactPair{goal.first, goal.second}; });
					auto red_plan = get_red_plan(current_best_supporters, current_global_state, goal_facts);
					auto [is_plan, resulting_state] = update_search_space_and_check_plan(current_global_state, red_plan, goal_facts);
					if (is_plan) {
						global_goal_state = resulting_state.get_id();
						auto num_black = std::count_if(
							std::begin(static_cast<RBStateRegistry *>(this->state_registry.get())->get_painting().get_painting()),
							std::end(static_cast<RBStateRegistry *>(this->state_registry.get())->get_painting().get_painting()),
							[](auto b) { return !b; });
						std::cout << "Final painting has " << num_black << " black variables ("
							<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)" << std::endl;
						return SOLVED;
					}
					// the goal is not reachable with a real plan, start a new search with a different painting from here and insert a corresponding node into the open list
					auto new_painting = incremental_painting_strategy->generate_next_painting(current_state.get_painting(), red_plan);
					auto new_eval_context = EvaluationContext<RBState, RBOperator>(get_hacked_cache_for_key(current_key), current_g, is_current_preferred, nullptr);
					enqueue_new_search(new_painting, current_global_state, current_key, is_current_preferred, new_eval_context);
				}
				if (search_progress.check_progress(current_eval_context)) {
					print_checkpoint_line(current_g);
					reward_progress();
				}
				generate_successors();
				statistics.inc_expanded();
			} else {
				node.mark_as_dead_end();
				statistics.inc_dead_ends();
			}
			if (current_predecessor_id == StateID::no_state) {
				print_initial_h_values(current_eval_context);
			}
		}
	}
	return fetch_next_state();
}

void HierarchicalRedBlackSearch::generate_successors() {
	ordered_set::OrderedSet<OperatorID> preferred_operators = collect_preferred_operators(current_eval_context, preferred_operator_heuristics);
	if (randomize_successors)
		preferred_operators.shuffle(*rng);

	std::vector<OperatorID> successor_operators = get_successor_operators(preferred_operators);
	for (OperatorID op_id : successor_operators) {
		const auto *op = get_operator(op_id.get_index());
		const auto &preconditions = op->get_base_operator().get_preconditions();
		auto precondition_facts = std::vector<FactPair>();
		precondition_facts.reserve(preconditions.size());
		std::transform(std::begin(preconditions), std::end(preconditions), std::back_inserter(precondition_facts), [](const auto &condition) { return FactPair{condition.var, condition.val}; });
		assert(corresponding_global_state.find(current_state.get_id()) != std::end(corresponding_global_state));
		auto current_global_state = global_state_registry.lookup_state(corresponding_global_state.at(current_state.get_id()));
		verify_black_variable_values(current_state, current_global_state);
		assert(global_search_space.get_node(current_global_state).is_closed());

		auto red_plan = get_red_plan(current_best_supporters, current_global_state, precondition_facts);
		auto [is_plan, resulting_state] = update_search_space_and_check_plan(current_global_state, red_plan, precondition_facts);

		auto is_preferred = preferred_operators.contains(op_id);
		if (is_plan) {
			// black operator preconditions are reachable with a real plan, insert search node as usual
			statistics.inc_generated();
			int new_g = current_g + get_adjusted_cost(*op);
			int new_real_g = current_real_g + op->get_cost();
			if (new_real_g < bound) {
				EvaluationContext<RBState, RBOperator> new_eval_context(
					current_eval_context.get_cache(), new_g, is_preferred, nullptr);
				open_list->insert(new_eval_context, std::make_pair(current_state.get_id(), get_op_index_hacked(op)));
				auto successor_state = global_state_registry.get_successor_state(resulting_state, op->get_base_operator());
				auto parent_node = global_search_space.get_node(resulting_state);
				assert(parent_node.is_closed());
				auto successor_node = global_search_space.get_node(successor_state);
				if (successor_node.is_new()) {
					successor_node.open(parent_node, &op->get_base_operator());
					successor_node.close();
				} else if (successor_node.is_closed() && parent_node.get_g() + get_adjusted_action_cost(*op, cost_type) < successor_node.get_g()) {
					successor_node.reopen(parent_node, &op->get_base_operator());
					successor_node.close();
				}
				pending_corresponding_global_states.insert({{current_state.get_id(), get_op_index_hacked(op)}, successor_state.get_id()});
			}
		} else {
			// black operator is not reachable with a real plan, start a new search with a different painting from here and insert a corresponding node into the open list
			auto new_painting = incremental_painting_strategy->generate_next_painting(current_state.get_painting(), red_plan);
			auto new_eval_context = EvaluationContext<RBState, RBOperator>(current_eval_context.get_cache(), current_g, is_preferred, nullptr);
			enqueue_new_search(new_painting, current_global_state, current_eval_context.get_heuristic_value(heuristics.front()), is_preferred, new_eval_context);
		}
	}
}

SearchStatus HierarchicalRedBlackSearch::fetch_next_state() {
	if (open_list->empty()) {
		std::cout << "Completely explored state space -- no solution!" << std::endl;
		return FAILED;
	}

	is_current_preferred = open_list->is_min_preferred();
	current_key = open_list->get_min_key();
	auto next = open_list->remove_min();

	current_predecessor_id = next.first;
	assert(current_predecessor_id != StateID::no_state);
	if (next.second < 0) {
		assert(child_searches.find(current_predecessor_id) != std::end(child_searches));
		current_child_search_index = -next.second - 1;
		assert(static_cast<int>(child_searches.find(current_predecessor_id)->second.size()) > current_child_search_index);
		current_child_search = &child_searches.find(current_predecessor_id)->second[current_child_search_index];
		if (!*current_child_search)
			// this open list entry was already handled (this can happen e.g. with the alternating queue, where entries are inserted in two different queues)
			return fetch_next_state();
		current_operator = nullptr;
		// the members set below are probably not relevant for the child search case
		current_state = state_registry->lookup_state(current_predecessor_id);
		auto current_node = search_space->get_node(current_state);
		current_g = current_node.get_g();
		current_real_g = current_node.get_real_g();
	} else {
		current_child_search = nullptr;
		current_operator = get_operator(next.second);
		auto current_predecessor = state_registry->lookup_state(current_predecessor_id);
		assert(current_operator->is_applicable(current_predecessor));
		auto corresponding_global_state_it = pending_corresponding_global_states.find({current_predecessor_id, next.second});
		if (corresponding_global_state_it == std::end(pending_corresponding_global_states))
			// this open list entry was already handled (this can happen e.g. with the alternating queue, where entries are inserted in two different queues)
			return fetch_next_state();
		std::tie(current_state, current_best_supporters) =
			static_cast<RBStateRegistry *>(state_registry.get())->get_state_and_best_supporters(
				global_state_registry.lookup_state(corresponding_global_state_it->second).get_values());
		corresponding_global_state.insert({current_state.get_id(), corresponding_global_state_it->second});
		verify_black_variable_values(current_state, global_state_registry.lookup_state(corresponding_global_state_it->second));
		pending_corresponding_global_states.erase(corresponding_global_state_it);
		auto pred_node = search_space->get_node(current_predecessor);
		current_g = pred_node.get_g() + get_adjusted_cost(*current_operator);
		current_real_g = pred_node.get_real_g() + current_operator->get_cost();
	}
	current_eval_context = EvaluationContext<RBState, RBOperator>(current_state, current_g, true, &statistics);
	return IN_PROGRESS;
}

SearchStatus HierarchicalRedBlackSearchWrapper::step() {
	auto status = root_search_engine->step();
	if (status != SOLVED)
		return status;
	assert(test_goal(state_registry->lookup_state(root_search_engine->get_goal_state())));
	check_goal_and_set_plan(state_registry->lookup_state(root_search_engine->get_goal_state()));
	return SOLVED;
}


void HierarchicalRedBlackSearchWrapper::print_statistics() const {
	// TODO: print statistics like:
	// how many different red-black searches did we launch?
	// average expansions per search
	// %black in lowest search after finding solution
	// ...
	statistics.print_detailed_statistics();
	search_space->print_statistics();
}

auto HierarchicalRedBlackSearch::update_search_space_and_check_plan(const GlobalState &state, const std::vector<OperatorID> &plan, const std::vector<FactPair> &goal_facts) -> std::pair<bool, GlobalState> {
	auto current_state = state;
	for (const auto op_id : plan) {
		const auto &op = g_operators[op_id.get_index()];
		if (!op.is_applicable(current_state))
			return {false, current_state};
		auto current_parent_node = global_search_space.get_node(current_state);
		assert(current_parent_node.is_closed());
		current_state = global_state_registry.get_successor_state(current_state, op);
		auto successor_node = global_search_space.get_node(current_state);
		if (successor_node.is_new()) {
			successor_node.open(current_parent_node, &op);
			successor_node.close();
		} else if (successor_node.is_closed() && current_parent_node.get_g() + get_adjusted_action_cost(op, cost_type) < successor_node.get_g()) {
			successor_node.reopen(current_parent_node, &op);
			successor_node.close();
		}
		assert(successor_node.is_closed());
	}
	return {std::all_of(std::begin(goal_facts), std::end(goal_facts), [&current_state](const auto &goal_fact) { return current_state[goal_fact.var] == goal_fact.value; }), current_state};
}

auto HierarchicalRedBlackSearch::get_current_key() const -> int {
	return current_key;
}

auto HierarchicalRedBlackSearch::get_goal_state() const -> StateID {
	assert(global_goal_state != StateID::no_state);
	return global_goal_state;
}

HierarchicalRedBlackSearchWrapper::HierarchicalRedBlackSearchWrapper(const options::Options &opts)
	: SearchEngine<GlobalState, GlobalOperator>(opts),
	  root_search_engine(),
	  rb_search_spaces(),
	  num_black(get_num_black(opts, true)) {
	auto rb_search_options = get_rb_search_options(opts);
	auto root_rb_data = std::make_shared<RBData>(*opts.get<std::shared_ptr<Painting>>("base_painting"));
	auto root_state_registry = std::shared_ptr<RBStateRegistry>(root_rb_data->construct_state_registry(g_initial_state_data));
	auto root_search_space = std::make_shared<SearchSpace<RBState, RBOperator>>(*root_state_registry, static_cast<OperatorCost>(rb_search_options.get_enum("cost_type")));
	rb_search_spaces.insert({root_rb_data->painting.get_painting(), {root_rb_data, root_state_registry, root_search_space}});
	root_search_engine = std::make_unique<HierarchicalRedBlackSearch>(rb_search_options, root_state_registry, root_search_space, state_registry->get_initial_state(), *state_registry, *search_space, rb_search_spaces, num_black);
	auto initial_node = search_space->get_node(state_registry->get_initial_state());
	initial_node.open_initial();
	initial_node.close();
}

auto HierarchicalRedBlackSearchWrapper::get_rb_search_options(const options::Options &opts) -> options::Options {
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

void HierarchicalRedBlackSearchWrapper::add_options_to_parser(options::OptionParser &parser) {
	parser.add_option<std::shared_ptr<Painting>>("base_painting", "painting to be used in the initial red-black search", "all_red()");
	parser.add_option<Heuristic<RBState, RBOperator> *>("heuristic", "red-black heuristic that will be passed to the underlying red-black search engine", "ff_rb(transform=adapt_costs(cost_type=1))");
	parser.add_option<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy", "strategy for painting more variables black after finding a red-black solution with conflicts", "least_conflicts()");
	add_num_black_options(parser);
	add_succ_order_options(parser);
}

static std::shared_ptr<SearchEngine<GlobalState, GlobalOperator>> _parse(options::OptionParser &parser) {
	SearchEngine<GlobalState, GlobalOperator>::add_options_to_parser(parser);
	HierarchicalRedBlackSearchWrapper::add_options_to_parser(parser);

	auto opts = parser.parse();
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<HierarchicalRedBlackSearchWrapper>(opts);
}

static options::PluginShared<SearchEngine<GlobalState, GlobalOperator>> _plugin("hierarchical_rb_search", _parse);
}
