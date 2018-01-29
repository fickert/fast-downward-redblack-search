#include "util.h"

#include "operator.h"
#include "../operator_cost.h"
#include "../options/bounds.h"
#include "../options/option_parser.h"
#include "../globals.h"
#include "mercury/red_black_DAG_fact_following_heuristic.h"

auto get_adjusted_action_cost(const redblack::RBOperator &op, OperatorCost cost_type) -> int {
	return get_adjusted_action_cost(op.get_base_operator(), cost_type);
}

auto get_op_index_hacked(const redblack::RBOperator *op) -> int {
	return get_op_index_hacked(&op->get_base_operator());
}

auto test_goal(const redblack::RBState &state) -> bool {
	for (const auto &goal_fact : g_goal)
		if (!state.has_fact(goal_fact.first, goal_fact.second))
			return false;
	return true;
}

auto transform_goal_facts() -> std::vector<FactPair> {
	auto goal_facts = std::vector<FactPair>();
	goal_facts.reserve(g_goal.size());
	std::transform(std::begin(g_goal), std::end(g_goal), std::back_inserter(goal_facts),
		[](const auto &goal) { return FactPair{goal.first, goal.second}; });
	return goal_facts;
}

auto redblack::get_goal_facts() -> const std::vector<FactPair> & {
	static auto goal_facts = transform_goal_facts();
	return goal_facts;
}

namespace redblack {
void add_num_black_options(options::OptionParser &parser) {
	parser.add_option<int>("num_black", "number of variables to be painted black", "1", options::Bounds("-1", "infinity"));
	parser.add_option<double>("ratio_black", "ratio of variables to be painted black", "0", options::Bounds("0", "1"));
}

auto get_num_black(const options::Options &opts, bool min_one_if_ratio) -> int {
	assert(opts.contains("num_black") || opts.contains("ratio_black"));
	const auto num_variables = g_root_task()->get_num_variables();
	if (!opts.contains("num_black") || opts.get<int>("num_black") < 0) {
		assert(opts.contains("ratio_black"));
		return std::min(num_variables, std::max<int>(min_one_if_ratio ? 1 : 0, num_variables * opts.get<double>("ratio_black")));
	}
	return std::min(num_variables, opts.get<int>("num_black"));
}

auto any_conditional_effect_condition_is_red(const Painting &painting) -> int {
	for (const auto &op : g_operators)
		for (const auto &effect : op.get_effects())
			for (const auto &condition : effect.conditions)
				if (painting.is_red_var(condition.var))
					return true;
	return false;
}

auto get_no_red_conditional_effect_conditions_painting(const Painting &painting) -> Painting {
	auto new_painting = painting.get_painting();
	for (const auto &op : g_operators)
		for (const auto &effect : op.get_effects())
			for (const auto &condition : effect.conditions)
				if (new_painting[condition.var])
					new_painting[condition.var] = false;
	return Painting(new_painting);
}

auto is_valid_relaxed_plan(const std::vector<boost::dynamic_bitset<>> &achieved_facts,
                           const std::vector<FactPair> &goal_facts,
                           const std::vector<OperatorID> &relaxed_plan) -> bool {
	// NOTE: this assumes that the relaxed plan is applicable in the given order which may not be guaranteed
	auto current_achieved_facts = achieved_facts;
	for (const auto &op_id : relaxed_plan) {
		const auto &op = g_operators[op_id.get_index()];
		if (!std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), [&current_achieved_facts](const auto &precondition) {
			return current_achieved_facts[precondition.var][precondition.val];
		}))
			return false;
		for (const auto &effect : op.get_effects())
			if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), [&current_achieved_facts](const auto &condition) {
				return current_achieved_facts[condition.var][condition.val];
			}))
				current_achieved_facts[effect.var].set(effect.val);
	}
	return std::all_of(std::begin(goal_facts), std::end(goal_facts), [&current_achieved_facts](const auto &goal_fact) {
		return current_achieved_facts[goal_fact.var][goal_fact.value];
	});
}

auto is_valid_relaxed_plan(const std::vector<int> &state_values,
                           const std::vector<FactPair> &goal_facts,
                           const std::vector<OperatorID> &relaxed_plan) -> bool {
	auto achieved_facts = std::vector<boost::dynamic_bitset<>>();
	achieved_facts.reserve(g_root_task()->get_num_variables());
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var) {
		achieved_facts.emplace_back(boost::dynamic_bitset<>(g_root_task()->get_variable_domain_size(var)));
		achieved_facts.back()[state_values[var]] = true;
	}
	return is_valid_relaxed_plan(achieved_facts, goal_facts, relaxed_plan);
}

auto is_valid_relaxed_plan_short(const std::vector<int> &state_values, const std::vector<FactPair> &goal_facts, const std::vector<OperatorID> &relaxed_plan) -> bool {
	auto current_achieved_facts = std::unordered_set<FactPair>();
	current_achieved_facts.reserve(g_root_task()->get_num_variables());
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var)
		current_achieved_facts.emplace(var, state_values[var]);
	for (const auto &op_id : relaxed_plan) {
		const auto &op = g_operators[op_id.get_index()];
		if (!std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), [&current_achieved_facts](const auto &precondition) {
			return current_achieved_facts.find({precondition.var, precondition.val}) != std::end(current_achieved_facts);
		}))
			return false;
		for (const auto &effect : op.get_effects())
			if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), [&current_achieved_facts](const auto &condition) {
				return current_achieved_facts.find({condition.var, condition.val}) != std::end(current_achieved_facts);
			}))
				current_achieved_facts.emplace(effect.var, effect.val);
	}
	return std::all_of(std::begin(goal_facts), std::end(goal_facts), [&current_achieved_facts](const auto &goal_fact) {
		return current_achieved_facts.find({goal_fact.var, goal_fact.value}) != std::end(current_achieved_facts);
	});
}

auto get_conflicts(const std::vector<int> &initial_state_values, const std::vector<FactPair> &goal_facts, const std::vector<OperatorID> &plan) -> std::vector<int> {
	auto conflicts = std::vector<int>(g_root_task()->get_num_variables(), 0);
	auto current_state = initial_state_values;
	for (auto op_id : plan) {
		const auto &op = g_operators[op_id.get_index()];
		for (const auto &pre : op.get_preconditions())
			if (current_state[pre.var] != pre.val)
				++conflicts[pre.var];
		for (const auto &eff : op.get_effects())
			if (std::all_of(std::begin(eff.conditions), std::end(eff.conditions), [&current_state](const auto &condition) {
				return current_state[condition.var] == condition.val;
			}))
				current_state[eff.var] = eff.val;
	}
	for (const auto &goal_fact : goal_facts)
		if (current_state[goal_fact.var] != goal_fact.value)
			++conflicts[goal_fact.var];

#ifndef NDEBUG
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var)
		// causal graph leaves should never have any conflicts
		assert(!causal_graph::get_causal_graph(g_root_task().get()).get_successors(var).empty() || conflicts[var] == 0);
#endif

	return conflicts;
}

#ifndef NDEBUG
void debug_verify_relaxed_plan(const GlobalState &state, const std::vector<OperatorID> &relaxed_plan, const std::vector<FactPair> &goal_facts) {
	auto current_achieved_values = std::vector<std::unordered_set<int>>(g_root_task()->get_num_variables());
	for (auto i = 0; i < g_root_task()->get_num_variables(); ++i)
		current_achieved_values[i].insert(state[i]);
	auto open = relaxed_plan;
	auto next_open = std::vector<OperatorID>();
	while (!open.empty()) {
		for (const auto op_id : open) {
			const auto &op = g_operators[op_id.get_index()];
			if (std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), [&current_achieved_values](const auto &precondition) {
				return current_achieved_values[precondition.var].find(precondition.val) != std::end(current_achieved_values[precondition.var]);
			})) {
				for (const auto &effect : op.get_effects())
					current_achieved_values[effect.var].insert(effect.val);
			} else {
				next_open.push_back(op_id);
			}
		}
		assert(next_open.size() < open.size());
		open = next_open;
		next_open.clear();
	}
	assert(std::all_of(std::begin(goal_facts), std::end(goal_facts), [&current_achieved_values](const auto &goal_fact) {
		return current_achieved_values[goal_fact.var].find(goal_fact.value) != std::end(current_achieved_values[goal_fact.var]);
	}));
}
#else
void debug_verify_relaxed_plan(const GlobalState &, const std::vector<OperatorID> &, const std::vector<FactPair> &) {}
#endif


void order_relaxed_plan_lazy_short(const std::vector<int> &state_values, std::vector<OperatorID> &relaxed_plan) {
	auto current_achieved_facts = std::unordered_set<FactPair>();
	current_achieved_facts.reserve(g_root_task()->get_num_variables());
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var)
		current_achieved_facts.emplace(var, state_values[var]);
	// check if a condition is satisfied
	const auto check_condition = [&current_achieved_facts](const auto &precondition) {
		return current_achieved_facts.find({precondition.var, precondition.val}) != std::end(current_achieved_facts);
	};
	for (auto rp_it = std::begin(relaxed_plan); rp_it != std::end(relaxed_plan); ++rp_it) {
		// swap this operator with the first one that is applicable (does nothing if the current one is applicable)
		std::iter_swap(rp_it, std::find_if(rp_it, std::end(relaxed_plan), [check_condition](const auto &op_id) {
			const auto &op = g_operators[op_id.get_index()];
			return std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), check_condition);
		}));
		for (const auto &effect : g_operators[rp_it->get_index()].get_effects())
			if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), check_condition))
				current_achieved_facts.emplace(effect.var, effect.val);
	}
}

void order_relaxed_plan_lazy(const std::vector<boost::dynamic_bitset<>> &state, std::vector<OperatorID> &relaxed_plan) {
	auto current_achieved_values = state;
	// check if a condition is satisfied
	const auto check_condition = [&current_achieved_values](const auto &precondition) {
		return current_achieved_values[precondition.var][precondition.val];
	};
	for (auto rp_it = std::begin(relaxed_plan); rp_it != std::end(relaxed_plan); ++rp_it) {
		// swap this operator with the first one that is applicable (does nothing if the current one is applicable)
		std::iter_swap(rp_it, std::find_if(rp_it, std::end(relaxed_plan), [check_condition](const auto &op_id) {
			const auto &op = g_operators[op_id.get_index()];
			return std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), check_condition);
		}));
		for (const auto &effect : g_operators[rp_it->get_index()].get_effects())
			if (std::all_of(std::begin(effect.conditions), std::end(effect.conditions), check_condition))
				current_achieved_values[effect.var][effect.val] = true;
	}
}


auto get_ordered_relaxed_plan(const GlobalState &state, const std::vector<OperatorID> &relaxed_plan) -> std::vector<OperatorID> {
	return get_ordered_relaxed_plan(state.get_values(), relaxed_plan);
}

auto get_red_plan(const std::vector<std::vector<OperatorID>> &best_supporters, const GlobalState &state, const std::vector<FactPair> &goal_facts, bool ordered) -> std::vector<OperatorID> {
	return get_red_plan(best_supporters, state.get_values(), goal_facts, ordered);
}

auto get_ordered_relaxed_plan(const std::vector<int> &state_values, const std::vector<OperatorID> &relaxed_plan) -> std::vector<OperatorID> {
	auto ordered_relaxed_plan = std::vector<OperatorID>();
	ordered_relaxed_plan.reserve(relaxed_plan.size());
	auto current_achieved_values = std::vector<boost::dynamic_bitset<>>();
	current_achieved_values.reserve(g_root_task()->get_num_variables());
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var) {
		current_achieved_values.emplace_back(boost::dynamic_bitset<>(g_root_task()->get_variable_domain_size(var)));
		current_achieved_values.back()[state_values[var]] = true;
	}
	auto open = relaxed_plan;
	auto next_open = std::vector<OperatorID>();
	while (!open.empty()) {
		for (const auto op_id : open) {
			const auto &op = g_operators[op_id.get_index()];
			if (std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), [&current_achieved_values](const auto &precondition) {
				return current_achieved_values[precondition.var][precondition.val];
			})) {
				ordered_relaxed_plan.push_back(op_id);
				for (const auto &effect : op.get_effects())
					current_achieved_values[effect.var].set(effect.val);
			} else {
				next_open.push_back(op_id);
			}
		}
		assert(next_open.size() < open.size());
		open = next_open;
		next_open.clear();
	}
	return ordered_relaxed_plan;
}

auto get_red_plan(const std::vector<std::vector<OperatorID>> &best_supporters, const std::vector<int> &state_values, const std::vector<FactPair> &goal_facts, bool ordered) -> std::vector<OperatorID> {
	auto open = std::unordered_set<FactPair>();
	for (const auto &goal_fact : goal_facts)
		if (state_values[goal_fact.var] != goal_fact.value)
			open.insert(goal_fact);
	auto closed = std::unordered_set<FactPair>();
	auto relaxed_plan = std::vector<OperatorID>();
	while (!open.empty()) {
		auto next_open = std::unordered_set<FactPair>();
		auto this_phase_closed = std::unordered_set<FactPair>();
		for (const auto &open_fact : open) {
			if (closed.find(open_fact) != std::end(closed) || this_phase_closed.find(open_fact) != std::end(this_phase_closed))
				continue;
			auto supporter_id = best_supporters[open_fact.var][open_fact.value];
			assert(supporter_id.get_index() != -1);
			if (std::find(std::begin(relaxed_plan), std::end(relaxed_plan), supporter_id) == std::end(relaxed_plan)) {
				relaxed_plan.push_back(supporter_id);
				const auto &supporter = g_operators[supporter_id.get_index()];
				for (const auto &precondition : supporter.get_preconditions())
					if (state_values[precondition.var] != precondition.val)
						next_open.emplace(precondition.var, precondition.val);
			}
		}
		open = std::move(next_open);
	}
	std::reverse(std::begin(relaxed_plan), std::end(relaxed_plan));
	if (ordered && relaxed_plan.size() > 1)
		order_relaxed_plan_lazy_short(state_values, relaxed_plan);
	return relaxed_plan;
}

auto get_ordered_relaxed_plan(const std::vector<boost::dynamic_bitset<>> &state, const std::vector<OperatorID> &relaxed_plan) -> std::vector<OperatorID> {
	auto ordered_relaxed_plan = std::vector<OperatorID>();
	ordered_relaxed_plan.reserve(relaxed_plan.size());
	auto current_achieved_values = state;
	auto open = relaxed_plan;
	auto next_open = std::vector<OperatorID>();
	while (!open.empty()) {
		for (const auto op_id : open) {
			const auto &op = g_operators[op_id.get_index()];
			if (std::all_of(std::begin(op.get_preconditions()), std::end(op.get_preconditions()), [&current_achieved_values](const auto &precondition) {
				return current_achieved_values[precondition.var][precondition.val];
			})) {
				ordered_relaxed_plan.push_back(op_id);
				for (const auto &effect : op.get_effects())
					current_achieved_values[effect.var].set(effect.val);
			} else {
				next_open.push_back(op_id);
			}
		}
		assert(next_open.size() < open.size());
		open = next_open;
		next_open.clear();
	}
	return ordered_relaxed_plan;
}

auto get_red_plan(const std::vector<std::vector<OperatorID>> &best_supporters, const std::vector<boost::dynamic_bitset<>> &state, const std::vector<FactPair> &goal_facts, bool ordered) -> std::vector<OperatorID> {
	auto open = std::unordered_set<FactPair>();
	for (const auto &goal_fact : goal_facts)
		if (!state[goal_fact.var][goal_fact.value])
			open.insert(goal_fact);
	auto closed = std::unordered_set<FactPair>();
	auto relaxed_plan = std::vector<OperatorID>();
	while (!open.empty()) {
		auto next_open = std::unordered_set<FactPair>();
		auto this_phase_closed = std::unordered_set<FactPair>();
		for (const auto &open_fact : open) {
			if (closed.find(open_fact) != std::end(closed) || this_phase_closed.find(open_fact) != std::end(this_phase_closed))
				continue;
			auto supporter_id = best_supporters[open_fact.var][open_fact.value];
			assert(supporter_id.get_index() != -1);
			if (std::find(std::begin(relaxed_plan), std::end(relaxed_plan), supporter_id) == std::end(relaxed_plan)) {
				relaxed_plan.push_back(supporter_id);
				const auto &supporter = g_operators[supporter_id.get_index()];
				for (const auto &precondition : supporter.get_preconditions())
					if (!state[precondition.var][precondition.val])
						next_open.emplace(precondition.var, precondition.val);
			}
		}
		open = std::move(next_open);
	}
	std::reverse(std::begin(relaxed_plan), std::end(relaxed_plan));
	if (ordered && relaxed_plan.size() > 1)
		order_relaxed_plan_lazy(state, relaxed_plan);
	return relaxed_plan;
}

auto get_conflicting_variables(const RedBlackDAGFactFollowingHeuristic &plan_repair_heuristic, const Painting &painting) -> std::vector<int> {
	auto conflicting_variables = std::vector<int>();
	for (const auto black_var : plan_repair_heuristic.get_black_indices()) {
		const auto &predecessors = causal_graph::get_causal_graph(g_root_task().get()).get_predecessors(black_var);
		if (std::any_of(std::begin(predecessors), std::end(predecessors), [&painting](const auto predecessor_var) {
			return painting.is_black_var(predecessor_var);
		}))
			conflicting_variables.push_back(black_var);
	}
	std::sort(std::begin(conflicting_variables), std::end(conflicting_variables));
	return conflicting_variables;
}

}
