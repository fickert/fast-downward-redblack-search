#include "util.h"

#include "operator.h"
#include "../operator_cost.h"
#include "../options/bounds.h"
#include "../options/option_parser.h"
#include "../globals.h"

int get_adjusted_action_cost(const redblack::RBOperator &op, OperatorCost cost_type) {
	return get_adjusted_action_cost(op.get_base_operator(), cost_type);
}

int get_op_index_hacked(const redblack::RBOperator *op) {
	return get_op_index_hacked(&op->get_base_operator());
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
}
