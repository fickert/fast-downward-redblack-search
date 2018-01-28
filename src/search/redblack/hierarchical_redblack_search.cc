#include "hierarchical_redblack_search.h"

#include "search_space.h"
#include "util.h"
#include "../search_engine.h"
#include "../options/option_parser.h"
#include "../search_engines/lazy_search.h"
#include "../search_engines/search_common.h"
#include "incremental_painting_strategy.h"
#include <numeric>


namespace redblack {

// TODO: when going down into a child search, we might want to update the key and first check if the key is still up to date before expanding that node
// the best time to do this would probably be upon fetching the next real state (so, not when going down some level, but only when reaching the bottom)
// this will create "empty" steps, where all we do is update keys, but that should be fine

// TODO (optional): to improve overall solution quality, it would make sense to always update the global search space when doing realizability checks
// this way we can update the parents of nodes to which cheaper paths are found

HierarchicalRedBlackSearch::HierarchicalRedBlackSearch(const options::Options &opts,
                                                       std::shared_ptr<RBStateRegistry> state_registry,
                                                       std::shared_ptr<SearchSpace<RBState, RBOperator>> search_space,
                                                       std::map<InternalPaintingType, std::pair<std::unique_ptr<RBData>, std::unique_ptr<HierarchicalRedBlackSearch>>> &rb_searches,
                                                       std::shared_ptr<RedBlackDAGFactFollowingHeuristic> plan_repair_heuristic,
                                                       std::shared_ptr<RedActionsManager> red_actions_manager,
                                                       HierarchicalRedBlackSearchWrapper &wrapper)
	: LazySearch<RBState, RBOperator>(opts, state_registry, search_space),
	  painting(state_registry->get_painting()),
	  plan_repair_heuristic(plan_repair_heuristic),
	  red_actions_manager(red_actions_manager),
	  always_recompute_red_plans(opts.get<bool>("always_recompute_red_plans")),
	  split_on_immediate_conflict_variables(opts.get<bool>("split_on_immediate_conflict_variables")),
	  is_current_preferred(false),
	  child_searches(),
	  current_child_search(nullptr),
	  current_child_search_index(-1),
	  search_options(opts),
	  incremental_painting_strategy(opts.get<std::shared_ptr<IncrementalPaintingStrategy>>("incremental_painting_strategy")),
	  rb_searches(rb_searches),
	  wrapper(wrapper) {
	auto pref_operator_heuristics = search_options.get_list<Heuristic<RBState, RBOperator> *>("preferred");
	set_pref_operator_heuristics(pref_operator_heuristics);
	LazySearch<RBState, RBOperator>::initialize();
}

auto HierarchicalRedBlackSearch::get_hacked_cache_for_key(int key, const RBState &state) const -> HeuristicCache<RBState, RBOperator> {
	auto hacked_eval_result = EvaluationResult();
	hacked_eval_result.set_h_value(key);
	// Note: the current_state in the cache is not always the intended state, but the open list doesn't care
	auto hacked_cache = HeuristicCache<RBState, RBOperator>(state);
	assert(heuristics.size() == 1);
	hacked_cache[heuristics.front()] = hacked_eval_result;
	return hacked_cache;
}

SearchStatus HierarchicalRedBlackSearch::step() {
	assert(rb_searches.find(painting.get_painting()) != std::end(rb_searches));

	// Contrary to the standard LazySearch implementation, we fetch the next state at
	// the beginning of the step instead of at the end of it, because the open list may
	// be modified from the outside. Thus, in order to make sure we have the state with
	// the lowest key from the open list, we need to fetch it at the start of each step.
	if (fetch_next_state() == FAILED)
		return FAILED;

	// Invariants:
	// - current_state is the next state for which we want to compute the heuristic.
	// - current_predecessor is a permanent pointer to the predecessor of that state.
	// - current_operator is the operator which leads to current_state from predecessor.
	// - current_g is the g value of the current state according to the cost_type
	// - current_real_g is the g value of the current state (using real costs)

	if (current_child_search) {
		const auto result = current_child_search->step();
		switch (result) {
		case SOLVED:
			// the plan should have been set in the wrapper search engine
			assert(!wrapper.get_plan().empty());
			return SOLVED;
		case FAILED:
			// TODO: is this still complete if we don't reinsert child search w/ key int max or something?
			return IN_PROGRESS;
		case TIMEOUT:
			return TIMEOUT;
		case IN_PROGRESS: {
			// reinsert this "state" into the open list, using the key of the current search node from the child search
			auto hacked_eval_context = EvaluationContext<RBState, RBOperator>(get_hacked_cache_for_key(current_child_search->get_current_key(), current_state), current_g, is_current_preferred, nullptr);
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
			wrapper.statistics.inc_evaluated_states();
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
					// extract red-black plan and check if it is a real plan
					const auto path = get_path_to_current_state(true);
					std::cout << "##################### extracted plan: #####################" << std::endl;
					auto ops = get_op_sequence(path);
					for (const auto op : ops)
						std::cout << "#### Operator " << get_op_index_hacked(op) << " (" << op->get_name() << ")" << std::endl;
					std::cout << "###########################################################" << std::endl;

					if (check_path(path, true)) {
						wrapper.set_plan(get_op_sequence(path));
						const auto num_black = painting.count_num_black();
						std::cout << "Final painting has " << num_black << " black variables ("
							<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)" << std::endl;
						return SOLVED;
					}
					// check if the red-black plan can be repaired to a real plan
					const auto marked_facts = collect_marked_facts(path);
					auto [repaired, repaired_path] = repair_path<false>(path, marked_facts);
					if (repaired) {
						wrapper.set_plan(get_op_sequence(repaired_path));
						const auto num_black = painting.count_num_black();
						std::cout << "Final painting has " << num_black << " black variables ("
							<< (num_black / static_cast<double>(g_root_task()->get_num_variables())) * 100 << "%)" << std::endl;
						return SOLVED;
					}
					// try to repair the red-black plan to a better red-black plan before looking for conflicts
					std::tie(repaired, repaired_path) = repair_path<true>(path, marked_facts);
					const auto &best_plan = repaired ? repaired_path : path;
					perform_split_at_first_conflict(best_plan, get_goal_facts());
					return IN_PROGRESS;
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
			//if (current_predecessor_id == StateID::no_state) {
			if (current_state.get_id() == state_registry->get_initial_state().get_id()) {
				print_initial_h_values(current_eval_context);
			}
		}
	}
	return IN_PROGRESS;
}

void HierarchicalRedBlackSearch::recursive_split(const RBState &state, const std::vector<int> &split_vars, std::vector<int>::size_type current_pos, std::vector<boost::dynamic_bitset<>> &values, std::vector<RBState> &new_states) {
	const auto split_var = split_vars[current_pos];
	assert(values[split_var].none());
	for (auto value = 0; value < g_root_task()->get_variable_domain_size(split_var); ++value) {
		if (state.has_fact(split_var, value)) {
			values[split_var][value] = true;
			if (current_pos == split_vars.size() - 1) {
				new_states.emplace_back(static_cast<RBStateRegistry *>(state_registry.get())->get_state(values));
			} else {
				recursive_split(state, split_vars, current_pos + 1, values, new_states);
			}
			values[split_var][value] = false;
		}
	}
}

auto HierarchicalRedBlackSearch::split_state(const RBState &state, const std::vector<int> &split_vars) -> std::vector<RBState> {
#ifndef NDEBUG
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var)
		assert(painting.is_black_var(var) || state.get_painting().is_red_var(var));
	for (auto split_var : split_vars) {
		assert(painting.is_black_var(split_var));
		assert(state.get_painting().is_red_var(split_var));
	}
#endif
	auto values = state.get_redblack_values();
	auto new_states = std::vector<RBState>();
	new_states.reserve(std::accumulate(std::begin(split_vars), std::end(split_vars), 1, [&values](const auto product, const auto split_var) {
		return product * values[split_var].count();
	}));
	for (auto split_var : split_vars)
		values[split_var].reset();
	recursive_split(state, split_vars, 0, values, new_states);
	return new_states;
}

void HierarchicalRedBlackSearch::enqueue_initial() {
	auto initial_eval_context = EvaluationContext<RBState, RBOperator>(state_registry->get_initial_state(), 0, true, &statistics);
	open_list->insert(initial_eval_context, {state_registry->get_initial_state().get_id(), NO_PARENT});
}

void HierarchicalRedBlackSearch::enqueue_states_from_split(const RBState &state, const std::vector<int> &split_vars, HierarchicalRedBlackSearch &parent_search, int parent_h, bool preferred, int parent_g) {
	auto new_states = split_state(state, split_vars);
	for (const auto &new_state : new_states) {
		auto eval_context = EvaluationContext<RBState, RBOperator>(get_hacked_cache_for_key(parent_h, new_state), parent_g, preferred, &statistics);
		open_list->insert(eval_context, {new_state.get_id(), NO_PARENT});
		// NOTE: choice point: how to select the parent during plan reconstruction?
		// the simplest methods are
		// -- using always the first generating parent (i.e., never modify parent after it was set once)
		// -- [currently used here] using always the last generating parent (i.e., always update the parent)
		// -- store all parents and use some clever heuristic to select one of them during plan reconstruction
		parents.insert_or_assign(new_state.get_id(), std::make_tuple(&parent_search, state.get_id()));
	}
}

auto HierarchicalRedBlackSearch::get_path_to_current_state(bool require_goal) -> RBPath {
	auto path = RBPath();
	path.emplace_back(search_space->trace_rb_path(current_state, require_goal ? get_goal_facts() : std::vector<FactPair>()), this);
	for (;;) {
		const auto current_search = path.back().second;
		const auto current_starting_state_id = std::get<StateID>(path.back().first.second.front());
		const auto current_parent_it = current_search->parents.find(current_starting_state_id);
		if (current_parent_it == std::end(current_search->parents))
			break;
		const auto [parent_search, parent_state] = current_parent_it->second;
		const auto current_starting_state = current_search->state_registry->lookup_state(current_starting_state_id);
		// add the facts for the variables that were split when changing paintings as goal facts for the previous search space
		auto next_goal_facts = std::vector<FactPair>();
		for (auto var = 0; var < g_root_task()->get_num_variables(); ++var)
			if (current_search->painting.is_black_var(var) && parent_search->painting.is_red_var(var))
				next_goal_facts.emplace_back(var, current_starting_state[var]);
		for (const auto &remaining_marked : path.back().first.first)
			next_goal_facts.emplace_back(remaining_marked);
		path.emplace_back(parent_search->search_space->trace_rb_path(parent_search->state_registry->lookup_state(parent_state), next_goal_facts), parent_search);
	}
	std::reverse(std::begin(path), std::end(path));
	return path;
}

auto HierarchicalRedBlackSearch::get_rb_op_sequence(const RBPath &path) -> std::vector<const RBOperator *> {
	auto sequenced_path = std::vector<const RBOperator *>();
	for (const auto &segment : path) {
		const auto &current_operators = static_cast<RBStateRegistry *>(segment.second->state_registry.get())->get_operators();
		for (const auto &step : segment.first.second) {
			const auto &red_actions = std::get<std::vector<OperatorID>>(step);
			for (const auto &red_action : red_actions)
				sequenced_path.push_back(&current_operators[red_action.get_index()]);
			const auto &black_action = std::get<OperatorID>(step);
			if (black_action.get_index() != -1)
				sequenced_path.push_back(&current_operators[black_action.get_index()]);
		}
	}
	return sequenced_path;
}

auto HierarchicalRedBlackSearch::get_op_sequence(const RBPath &path) -> std::vector<const GlobalOperator *> {
	auto sequenced_path = std::vector<const GlobalOperator *>();
	for (const auto &segment : path) {
		for (const auto &step : segment.first.second) {
			const auto &red_actions = std::get<std::vector<OperatorID>>(step);
			for (const auto &red_action : red_actions)
				sequenced_path.push_back(&g_operators[red_action.get_index()]);
			const auto &black_action = std::get<OperatorID>(step);
			if (black_action.get_index() != -1)
				sequenced_path.push_back(&g_operators[black_action.get_index()]);
		}
	}
	return sequenced_path;
}

auto HierarchicalRedBlackSearch::get_op_id_sequence(const RBPath &path) -> std::vector<OperatorID> {
	auto sequenced_path = std::vector<OperatorID>();
	for (const auto &segment : path) {
		for (const auto &step : segment.first.second) {
			const auto &red_actions = std::get<std::vector<OperatorID>>(step);
			for (const auto &red_action : red_actions)
				sequenced_path.push_back(red_action);
			const auto &black_action = std::get<OperatorID>(step);
			if (black_action.get_index() != -1)
				sequenced_path.push_back(black_action);
		}
	}
	return sequenced_path;
}

auto HierarchicalRedBlackSearch::check_path(const RBPath &path, bool require_goal) -> bool {
	auto current_state_values = g_root_task()->get_initial_state_values();
	auto check_condition = [&current_state_values](const auto &condition) {
		return current_state_values[condition.var] == condition.val;
	};
	auto apply_effect = [&current_state_values, check_condition](const auto &effect) {
		if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), check_condition))
			current_state_values[effect.var] = effect.val;
	};
	auto try_apply = [check_condition, apply_effect](const auto &action) {
		if (!std::all_of(std::begin(action.get_preconditions()), std::end(action.get_preconditions()), check_condition))
			return false;
		for (const auto &effect : action.get_effects())
			apply_effect(effect);
		return true;
	};
	for (const auto &segment : path) {
		for (const auto &step : segment.first.second) {
			const auto &red_actions = std::get<std::vector<OperatorID>>(step);
			for (const auto &red_action : red_actions)
				if (!try_apply(g_operators[red_action.get_index()]))
					return false;
			const auto &black_action = std::get<OperatorID>(step);
			if (black_action.get_index() != -1 && !try_apply(g_operators[black_action.get_index()]))
				return false;
		}
	}
	return !require_goal || std::all_of(std::begin(g_goal), std::end(g_goal), [&current_state_values](const auto &goal_pair) {
		return current_state_values[goal_pair.first] == goal_pair.second;
	});
}

auto HierarchicalRedBlackSearch::collect_marked_facts(const RBPath &path) -> std::vector<MarkedFacts> {
	auto all_marked_facts = std::vector<MarkedFacts>();
	all_marked_facts.reserve(path.size());
	for (const auto &segment : path) {
		auto &current_state_registry = *static_cast<RBStateRegistry *>(segment.second->state_registry.get());
		all_marked_facts.emplace_back(std::move(*current_state_registry.get_last_marked_facts()));
	}
	return all_marked_facts;
}

template<>
auto HierarchicalRedBlackSearch::repair_path<true>(const std::vector<boost::dynamic_bitset<>> &state_values,
	const std::vector<OperatorID> &relaxed_plan, const std::vector<std::vector<OperatorID>> &current_supporters,
	const std::vector<FactPair> &goal_facts, const InternalPaintingType &painting,
	const boost::dynamic_bitset<> &red_actions) const -> std::pair<bool, std::vector<OperatorID>> {
	auto available_facts = std::vector<FactPair>();
	for (auto var = 0u; var < state_values.size(); ++var)
		for (auto value = 0u; value < state_values[var].size(); ++value)
			if (state_values[var][value])
				available_facts.emplace_back(var, value);
	return plan_repair_heuristic->compute_semi_relaxed_plan(available_facts, painting, goal_facts,
		always_recompute_red_plans || !is_valid_relaxed_plan(state_values, goal_facts, relaxed_plan) ?
			get_red_plan(current_supporters, state_values, goal_facts, true) : relaxed_plan, red_actions);
}

template<>
auto HierarchicalRedBlackSearch::repair_path<false>(const std::vector<int> &state_values,
	const std::vector<OperatorID> &relaxed_plan, const std::vector<std::vector<OperatorID>> &current_supporters,
	const std::vector<FactPair> &goal_facts, const InternalPaintingType &,
	const boost::dynamic_bitset<> &red_actions) const -> std::pair<bool, std::vector<OperatorID>> {
	return  plan_repair_heuristic->compute_semi_relaxed_plan(state_values, goal_facts,
		always_recompute_red_plans || !is_valid_relaxed_plan(state_values, goal_facts, relaxed_plan) ?
			get_red_plan(current_supporters, state_values, goal_facts, true) : relaxed_plan, red_actions);
}

template<bool relaxed>
auto HierarchicalRedBlackSearch::repair_path(const RBPath &path, const std::vector<MarkedFacts> &marked_facts) -> std::pair<bool, RBPath> {
	if (!plan_repair_heuristic)
		return {false, {}};

	// initialize state values
	using state_values_t = std::conditional_t<relaxed, std::vector<boost::dynamic_bitset<>>, std::vector<int>>;
	auto current_state_values = state_values_t();
	if constexpr (relaxed) {
		current_state_values.reserve(g_root_task()->get_num_variables());
		for (auto var = 0; var < g_root_task()->get_num_variables(); ++var) {
			current_state_values.emplace_back(boost::dynamic_bitset<>(g_root_task()->get_variable_domain_size(var)));
			current_state_values.back()[g_initial_state_data[var]] = true;
		}
	} else {
		current_state_values = g_root_task()->get_initial_state_values();
	}

	// helper function to check if a condition is satisfied for the current state values
	auto check_condition = [&current_state_values](const auto &condition) {
		if constexpr (relaxed) {
			return current_state_values[condition.var][condition.val];
		} else {
			return current_state_values[condition.var] == condition.val;
		}
	};

	// helper function to handle the case if the repair fails
	auto repair_failed = [&path, &marked_facts, this]() -> std::pair<bool, RBPath> {
		if constexpr (relaxed) {
			adjust_plan_repair_painting();
			return repair_path<true>(path, marked_facts);
		} else {
			utils::unused_variable(path);
			utils::unused_variable(marked_facts);
			utils::unused_variable(this);
			return {false, {}};
		}
	};

	auto repaired_path = RBPath();
	auto deferred_goal_facts = std::vector<FactPair>();

	assert(marked_facts.size() == path.size());
	auto marked_facts_it = std::begin(marked_facts);

	for (const auto &segment : path) {
		auto repaired_segment = RBPathSegment::second_type();
		repaired_segment.reserve(segment.first.second.size());
		const auto &current_search_engine = *segment.second;
		const auto &current_painting = current_search_engine.painting;

		// helper function to apply an effect of an action to the current state values
		auto apply_effect = [&current_state_values, check_condition, &current_painting](const auto &effect) {
			if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), check_condition)) {
				if constexpr (relaxed) {
					if (current_painting.is_black_var(effect.var))
						current_state_values[effect.var].reset();
					current_state_values[effect.var][effect.val] = true;
				} else {
					utils::unused_variable(current_painting);
					current_state_values[effect.var] = effect.val;
				}
			}
		};

		// helper function to try to apply an action to the current state values (prob. not needed TODO)
		auto try_apply = [check_condition, apply_effect](const auto &action) {
			if (!std::all_of(std::begin(action.get_preconditions()), std::end(action.get_preconditions()), check_condition))
				return false;
			for (const auto &effect : action.get_effects())
				apply_effect(effect);
			return true;
		};

		assert(marked_facts_it->size() == segment.first.second.size());
		auto current_marked_facts_it = std::begin(*marked_facts_it);
		auto &current_rb_state_registry = *static_cast<RBStateRegistry *>(current_search_engine.state_registry.get());

		for (const auto &step : segment.first.second) {
			auto [current_redblack_state, current_supporters] = current_rb_state_registry.get_state_and_best_supporters(current_state_values);

			const auto &current_black_action = std::get<OperatorID>(step);
			const auto &current_red_actions = std::get<std::vector<OperatorID>>(step);

			// construct the set of facts that need to be achieved by the current red plan
			auto current_goal_facts = std::vector<FactPair>();
			current_goal_facts.insert(std::end(current_goal_facts), std::begin(*current_marked_facts_it), std::end(*current_marked_facts_it));

			if (current_black_action.get_index() == -1) {
				for (const auto &goal_fact : current_goal_facts)
					if (!current_redblack_state.has_fact(goal_fact.var, goal_fact.value))
						return repair_failed();
				auto [repaired, repaired_partial_plan] = repair_path<relaxed>(current_state_values, current_red_actions, current_supporters, current_goal_facts,
					current_painting.get_painting(), current_search_engine.red_actions_manager->get_red_actions_for_state(current_state_values));
				if (!repaired)
					return repair_failed();
				repaired_segment.emplace_back(std::get<StateID>(step), repaired_partial_plan, current_black_action);
				continue;
			}

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
			const auto size = current_goal_facts.size();
			assert(current_black_action.get_index() != -1);
			const auto &rb_op = current_rb_state_registry.get_operators()[current_black_action.get_index()];
			for (const auto &precondition : rb_op.get_red_preconditions()) {
				if (!current_redblack_state.has_fact(precondition->var, precondition->val))
					return repair_failed();
				// NOTE: in theory, we only need to insert the mercury-black preconditions, because the red preconditions should already be achieved by the red plan,
				// (or in any previous red plan), and with red-black semantics they can't have been deleted again
				// however, we don't need to take them out of the set of goal facts (they will be ignored anyway by the plan repair)
				auto precondition_fact = FactPair(precondition->var, precondition->val);
				if (!std::binary_search(std::begin(current_goal_facts), std::begin(current_goal_facts) + size, precondition_fact))
					current_goal_facts.emplace_back(precondition_fact);
			}
			// NOTE: current_goal_facts no longer sorted

			// repair the red plan and insert it into the repaired segment
			auto [repaired, repaired_partial_plan] = repair_path<relaxed>(current_state_values, current_red_actions, current_supporters, current_goal_facts,
				current_painting.get_painting(), current_search_engine.red_actions_manager->get_red_actions_for_state(current_state_values));
			if (!repaired)
				return repair_failed();
			repaired_segment.emplace_back(std::get<StateID>(step), repaired_partial_plan, current_black_action);

			// update the current state values
			for (const auto &red_action_id : std::get<std::vector<OperatorID>>(repaired_segment.back())) {
				const auto &red_action = g_operators[red_action_id.get_index()];
				assert(std::all_of(std::begin(red_action.get_preconditions()), std::end(red_action.get_preconditions()), check_condition));
				for (const auto &effect : red_action.get_effects())
					apply_effect(effect);
			}
			const auto &black_action = g_operators[std::get<OperatorID>(repaired_segment.back()).get_index()];
			assert(std::all_of(std::begin(black_action.get_preconditions()), std::end(black_action.get_preconditions()), check_condition));
			for (const auto &effect : black_action.get_effects())
				apply_effect(effect);

			++current_marked_facts_it;
		} // end step

		++marked_facts_it;
	} // end segment
	return {true, repaired_path};
}

template auto HierarchicalRedBlackSearch::repair_path<true>(const RBPath &path, const std::vector<MarkedFacts> &marked_facts) -> std::pair<bool, RBPath>;

template auto HierarchicalRedBlackSearch::repair_path<false>(const RBPath &path, const std::vector<MarkedFacts> &marked_facts) -> std::pair<bool, RBPath>;

void HierarchicalRedBlackSearch::adjust_plan_repair_painting() {
	// NOTE: while we just change the painting globally, ideally we would have different
	// instances of the plan repair heuristic with different paintings that fit the
	// paintings that are used in the red-black search
	auto conflicting_variables = get_conflicting_variables(*plan_repair_heuristic, painting);
	assert(!conflicting_variables.empty());
	if (conflicting_variables.size() == static_cast<int>(plan_repair_heuristic->get_num_black())) {
		for (auto &search : rb_searches) {
			search.second.second->plan_repair_heuristic.reset();
			search.second.second->red_actions_manager.reset();
		}
	} else {
		plan_repair_heuristic->make_red(conflicting_variables);
	}
}

auto HierarchicalRedBlackSearch::get_split_vars_max_conflicts(const std::vector<FactPair> &goal_facts, const std::vector<OperatorID> &relaxed_plan, const Painting &painting) -> std::vector<int> {
	auto conflicts = get_conflicts(g_initial_state_data, goal_facts, relaxed_plan);
	auto max_var = -1;
	auto max_conflicts = -1;
	for (auto i = 0; i < static_cast<int>(conflicts.size()); ++i) {
		if (painting.is_red_var(i) && conflicts[i] > max_conflicts) {
			max_var = i;
			max_conflicts = conflicts[i];
		}
	}
	assert(max_var != -1);
	assert(max_conflicts > 0);
	return {max_var};
}

auto HierarchicalRedBlackSearch::get_split_vars_immediate_conflict(const std::vector<int> &state_values, const std::vector<FactPair> &expected_facts) -> std::vector<int> {
	auto split_vars = std::vector<int>();
	for (const auto &expected_fact : expected_facts)
		if (state_values[expected_fact.var] != expected_fact.value)
			split_vars.push_back(expected_fact.var);
	return split_vars;
}

auto HierarchicalRedBlackSearch::get_split_vars_immediate_conflict_expected_goal(const std::vector<int> &state_values) -> std::vector<int> {
	return get_split_vars_immediate_conflict(state_values, get_goal_facts());
}

auto HierarchicalRedBlackSearch::get_split_vars_immediate_conflict_expected_operator(const std::vector<int> &state_values, const GlobalOperator &failed) -> std::vector<int> {
	auto precondition_facts = std::vector<FactPair>();
	precondition_facts.reserve(failed.get_preconditions().size());
	std::transform(std::begin(failed.get_preconditions()), std::end(failed.get_preconditions()), std::back_inserter(precondition_facts), [](const auto &precondition) {
		return FactPair(precondition.var, precondition.val);
	});
	return get_split_vars_immediate_conflict(state_values, precondition_facts);
}

void HierarchicalRedBlackSearch::perform_split(const RBState &state, const std::vector<int> &split_vars) {
	auto resulting_painting = state.get_painting().get_painting();
	for (auto var : split_vars) {
		assert(resulting_painting[var]);
		resulting_painting[var] = false;
	}
	auto rb_search_it = rb_searches.find(resulting_painting);
	if (rb_search_it == std::end(rb_searches)) {
		// no search was started yet using the resulting painting
		auto new_painting = Painting(resulting_painting);
		++wrapper.hierarchical_red_black_search_statistics.num_distinct_paintings;
		wrapper.hierarchical_red_black_search_statistics.max_num_black = std::max(new_painting.count_num_black(), wrapper.hierarchical_red_black_search_statistics.max_num_black);
		auto new_rb_data = std::make_unique<RBData>(new_painting);
		auto new_state_registry = std::shared_ptr<RBStateRegistry>(new_rb_data->construct_state_registry(g_initial_state_data));
		auto new_red_actions_manager = plan_repair_heuristic ? std::make_shared<RedActionsManager>(new_state_registry->get_operators()) : nullptr;
		auto new_search_space = std::make_shared<SearchSpace<RBState, RBOperator>>(*new_state_registry, static_cast<OperatorCost>(search_options.get_enum("cost_type")));
		auto new_rb_search = std::make_unique<HierarchicalRedBlackSearch>(search_options, new_state_registry, new_search_space, rb_searches, plan_repair_heuristic, new_red_actions_manager, wrapper);
		rb_search_it = rb_searches.emplace(std::move(resulting_painting), std::make_pair(std::move(new_rb_data), std::move(new_rb_search))).first;
	}
	assert(rb_search_it != std::end(rb_searches));
	auto &parent_search_engine = *rb_searches.find(state.get_painting().get_painting())->second.second;
	auto &target_search_engine = *rb_search_it->second.second;
	auto parent_search_node = parent_search_engine.search_space->get_node(state);
	auto eval_context = EvaluationContext<RBState, RBOperator>(state);
	const auto result = eval_context.get_result(heuristics.front());
	assert(result.get_count_evaluation() == false && "result should have been cached");
	assert(!result.is_infinite() && "state should not have been a dead end");
	// TODO: using preferred status of the current state... probably doesn't make much sense?
	target_search_engine.enqueue_states_from_split(state, split_vars, parent_search_engine, result.get_h_value(), is_current_preferred, parent_search_node.get_g());
	parent_search_engine.child_searches[state.get_id()].emplace_back(&target_search_engine);
	auto parent_search_eval_context = EvaluationContext<RBState, RBOperator>(state, parent_search_node.get_g(), is_current_preferred, &parent_search_engine.statistics);
	parent_search_engine.open_list->insert(parent_search_eval_context, {state.get_id(), -static_cast<int>(parent_search_engine.child_searches.at(state.get_id()).size() - 1) - 1});
}

void HierarchicalRedBlackSearch::perform_split_at_first_conflict(const RBPath &path, const std::vector<FactPair> &goal_facts) {
	auto current_state_values = g_root_task()->get_initial_state_values();
	auto check_condition = [&current_state_values](const auto &condition) {
		return current_state_values[condition.var] == condition.val;
	};
	auto apply_effect = [&current_state_values, check_condition](const auto &effect) {
		if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), check_condition))
			current_state_values[effect.var] = effect.val;
	};
	auto try_apply = [check_condition, apply_effect](const auto &action) {
		if (!std::all_of(std::begin(action.get_preconditions()), std::end(action.get_preconditions()), check_condition))
			return false;
		for (const auto &effect : action.get_effects())
			apply_effect(effect);
		return true;
	};
	for (const auto &segment : path) {
		for (const auto &step : segment.first.second) {
			const auto &red_actions = std::get<std::vector<OperatorID>>(step);
			for (const auto &red_action : red_actions) {
				const auto &action = g_operators[red_action.get_index()];
				if (!std::all_of(std::begin(action.get_preconditions()), std::end(action.get_preconditions()), check_condition)) {
					const auto split_vars = split_on_immediate_conflict_variables ? get_split_vars_immediate_conflict_expected_operator(current_state_values, action) : get_split_vars_max_conflicts(goal_facts, get_op_id_sequence(path), segment.second->painting);
					perform_split(segment.second->state_registry->lookup_state(std::get<StateID>(step)), split_vars);
					return;
				}
				for (const auto &effect : action.get_effects())
					apply_effect(effect);
			}
			const auto &black_action_id = std::get<OperatorID>(step);
			if (black_action_id.get_index() == -1)
				continue;
			const auto &black_action = g_operators[black_action_id.get_index()];
			if (!std::all_of(std::begin(black_action.get_preconditions()), std::end(black_action.get_preconditions()), check_condition)) {
				const auto split_vars = split_on_immediate_conflict_variables ? get_split_vars_immediate_conflict_expected_operator(current_state_values, black_action) : get_split_vars_max_conflicts(goal_facts, get_op_id_sequence(path), segment.second->painting);
				perform_split(segment.second->state_registry->lookup_state(std::get<StateID>(step)), split_vars);
				return;
			}
			for (const auto &effect : black_action.get_effects())
				apply_effect(effect);
		}
	}
	if (!std::all_of(std::begin(goal_facts), std::end(goal_facts), [&current_state_values](const auto &goal_fact) {
		return current_state_values[goal_fact.var] == goal_fact.value;
	})) {
		const auto split_vars = split_on_immediate_conflict_variables ? get_split_vars_immediate_conflict(current_state_values, goal_facts) : get_split_vars_max_conflicts(goal_facts, get_op_id_sequence(path), path.back().second->painting);
		perform_split(path.back().second->state_registry->lookup_state(std::get<StateID>(path.back().first.second.back())), split_vars);
		return;
	}
	std::cerr << "perform_split_at_first_conflict() called with conflict-free path" << std::endl;
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}


SearchStatus HierarchicalRedBlackSearch::fetch_next_state() {
	if (open_list->empty()) {
		std::cout << "Completely explored state space -- no solution!" << std::endl;
		return FAILED;
	}

	const auto next = open_list->remove_min();

	current_predecessor_id = next.first;
	assert(current_predecessor_id != StateID::no_state);
	assert(NO_PARENT >= 0);
	if (next.second < 0) {
		assert(child_searches.find(current_predecessor_id) != std::end(child_searches));
		current_child_search_index = -next.second - 1;
		assert(static_cast<int>(child_searches.find(current_predecessor_id)->second.size()) > current_child_search_index);
		current_child_search = child_searches.find(current_predecessor_id)->second[current_child_search_index];
		if (!current_child_search)
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
		if (next.second == NO_PARENT) {
			current_operator = nullptr;
			current_state = state_registry->lookup_state(current_predecessor_id);
			current_predecessor_id = StateID::no_state;
			// TODO: these g values are obviously incorrect, but does it really matter? the correct values would be the g values from the parent state
			// in principle, these could be looked up in the parent search space (which we can access via the parents map)
			current_g = 0;
			current_real_g = 0;
		} else {
			current_operator = get_operator(next.second);
			const auto current_predecessor = state_registry->lookup_state(current_predecessor_id);
			assert(current_operator->is_applicable(current_predecessor));
			current_state = state_registry->get_successor_state(current_predecessor, *current_operator);
			auto pred_node = search_space->get_node(current_predecessor);
			current_g = pred_node.get_g() + get_adjusted_cost(*current_operator);
			current_real_g = pred_node.get_real_g() + current_operator->get_cost();
		}
	}
	current_eval_context = EvaluationContext<RBState, RBOperator>(current_state, current_g, true, &statistics);
	return IN_PROGRESS;
}

auto HierarchicalRedBlackSearch::get_current_key() const -> int {
	return open_list->empty() ? std::numeric_limits<int>::max() : open_list->get_min_key();
}

// Wrapper class

SearchStatus HierarchicalRedBlackSearchWrapper::step() {
	const auto status = root_search_engine->step();
	// periodically print red black search statistics
	if (statistics_interval != -1 && search_timer() > next_print_time) {
		print_rb_search_statistics();
		next_print_time = search_timer() + statistics_interval;
	}
	if (status != SOLVED)
		return status;
	const auto &rb_plan = root_search_engine->get_plan();
	auto plan = std::vector<const GlobalOperator *>();
	plan.reserve(rb_plan.size());
	std::transform(std::begin(rb_plan), std::end(rb_plan), std::back_inserter(plan), [](const auto rb_op) { return &rb_op->get_base_operator(); });
	set_plan(plan);
	return SOLVED;
}

void HierarchicalRedBlackSearchWrapper::print_rb_search_statistics() const {
	std::cout << "Number of openend searches: " << hierarchical_red_black_search_statistics.num_openend_searches << std::endl;
	std::cout << "Number of distinct paintings: " << hierarchical_red_black_search_statistics.num_distinct_paintings << std::endl;
	std::cout << "Maximum number of black variables: " << hierarchical_red_black_search_statistics.max_num_black
		<< " (" << hierarchical_red_black_search_statistics.max_num_black / static_cast<double>(g_root_task()->get_num_variables()) << "%)" << std::endl;
	std::cout << "Average evaluations per search: " << statistics.get_evaluated_states() / static_cast<double>(hierarchical_red_black_search_statistics.num_openend_searches) << std::endl;
}

void HierarchicalRedBlackSearchWrapper::print_statistics() const {
	print_rb_search_statistics();
	statistics.print_detailed_statistics();
	search_space->print_statistics();
}

HierarchicalRedBlackSearchWrapper::HierarchicalRedBlackSearchWrapper(const options::Options &opts)
	: SearchEngine<GlobalState, GlobalOperator>(opts),
	  root_search_engine(nullptr),
	  rb_searches(),
	  num_black(get_num_black(opts, true)),
	  hierarchical_red_black_search_statistics(),
	  search_timer(),
	  statistics_interval(opts.get<int>("statistics_interval")),
	  next_print_time(0) {
	auto rb_search_options = get_rb_search_options(opts);
	auto root_rb_data = std::make_unique<RBData>(*opts.get<std::shared_ptr<Painting>>("base_painting"));
	auto root_state_registry = std::shared_ptr<RBStateRegistry>(root_rb_data->construct_state_registry(g_initial_state_data));
	auto root_search_space = std::make_shared<SearchSpace<RBState, RBOperator>>(*root_state_registry, static_cast<OperatorCost>(rb_search_options.get_enum("cost_type")));
	auto plan_repair_heuristic = get_rb_plan_repair_heuristic(opts);
	auto root_red_actions_manager = plan_repair_heuristic ? std::make_shared<RedActionsManager>(root_state_registry->get_operators()) : nullptr;
	auto root_rb_search = std::make_unique<HierarchicalRedBlackSearch>(rb_search_options, root_state_registry, root_search_space, rb_searches, plan_repair_heuristic, root_red_actions_manager, *this);
	root_rb_search->enqueue_initial();
	auto painting = root_rb_data->painting.get_painting();
	rb_searches.emplace(std::move(painting), std::make_pair(std::move(root_rb_data), std::move(root_rb_search)));
	assert(rb_searches.size() == 1);
	root_search_engine = std::begin(rb_searches)->second.second.get();
	search_timer.reset();
	next_print_time = statistics_interval;
}

auto HierarchicalRedBlackSearchWrapper::get_rb_plan_repair_heuristic(const options::Options &opts) -> std::shared_ptr<RedBlackDAGFactFollowingHeuristic> {
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
	parser.add_option<bool>("repair_red_plans", "attempt to repair red plans using Mercury", "true");
	parser.add_option<bool>("always_recompute_red_plans", "when trying to repair red partial plans, always replace the old red plan by a new one based on the real state", "true");
	parser.add_option<bool>("split_on_immediate_conflict_variables", "split on immediate conflict variables as opposed to the variable with the most conflicts in the overall plan", "true");
	parser.add_option<int>("statistics_interval", "Print statistics every x seconds. If this is set to -1, statistics will not be printed during search.", "30");
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
