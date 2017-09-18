#include "operator.h"

namespace redblack {

auto RBOperator::is_applicable(const RBState &state) const -> bool {
	return std::all_of(std::begin(base_operator.get_preconditions()), std::end(base_operator.get_preconditions()), [&state](const auto &precondition) {
		return state.has_fact(precondition.var, precondition.val);
	});
}

RBOperator::RBOperator(const GlobalOperator &base_operator)
	: base_operator(base_operator),
	  black_preconditions(),
	  red_preconditions(),
	  black_effects(),
	  red_effects() {}

void RBOperator::apply_painting(const Painting &painting) {
	black_preconditions.clear();
	red_preconditions.clear();
	black_effects.clear();
	red_effects.clear();
	for (const auto &precondition : base_operator.get_preconditions()) {
		if (painting.is_black_var(precondition.var))
			black_preconditions.push_back(&precondition);
		else
			red_preconditions.push_back(&precondition);
	}
	for (const auto &effect : base_operator.get_effects()) {
		if (painting.is_black_var(effect.var))
			black_effects.push_back(&effect);
		else
			red_effects.push_back(&effect);
	}
}

}
