#include "util.h"

#include "operator.h"
#include "../operator_cost.h"
#include "../options/bounds.h"
#include "../options/option_parser.h"
#include "../globals.h"

auto get_adjusted_action_cost(const redblack::RBOperator &op, OperatorCost cost_type) -> int {
	return get_adjusted_action_cost(op.get_base_operator(), cost_type);
}

auto get_op_index_hacked(const redblack::RBOperator *op) -> int {
	return get_op_index_hacked(&op->get_base_operator());
}

auto test_goal(const redblack::RBState &state) -> bool {
	for (size_t i = 0; i < g_goal.size(); ++i)
		if (!state.has_fact(g_goal[i].first, g_goal[i].second))
			return false;
	return true;
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

auto get_ordered_relaxed_plan(const GlobalState &state, const std::vector<OperatorID> &relaxed_plan) -> std::vector<OperatorID> {
	auto ordered_relaxed_plan = std::vector<OperatorID>();
	ordered_relaxed_plan.reserve(relaxed_plan.size());
	auto current_achieved_values = std::vector<boost::dynamic_bitset<>>();
	current_achieved_values.reserve(g_root_task()->get_num_variables());
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var) {
		current_achieved_values.emplace_back(boost::dynamic_bitset<>(g_root_task()->get_variable_domain_size(var)));
		current_achieved_values.back()[state[var]] = true;
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

auto get_red_plan(const std::vector<std::vector<OperatorID>> &best_supporters, const GlobalState &state, const std::vector<FactPair> &goal_facts, bool ordered) -> std::vector<OperatorID> {
	auto open = std::unordered_set<FactPair>();
	for (const auto &goal_fact : goal_facts)
		if (state[goal_fact.var] != goal_fact.value)
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
					if (state[precondition.var] != precondition.val)
						next_open.emplace(precondition.var, precondition.val);
			}
		}
		open = std::move(next_open);
	}
	std::reverse(std::begin(relaxed_plan), std::end(relaxed_plan));
	return ordered && relaxed_plan.size() > 1 ? get_ordered_relaxed_plan(state, relaxed_plan) : relaxed_plan;
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
	return ordered ? get_ordered_relaxed_plan(state, relaxed_plan) : relaxed_plan;
}

}
