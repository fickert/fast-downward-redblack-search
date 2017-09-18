#ifndef REDBLACK_OPERATOR_H
#define REDBLACK_OPERATOR_H

#include "state.h"
#include "../global_operator.h"
#include "../operator_id.h"

namespace redblack {

class RBOperator {
	const GlobalOperator &base_operator;

	std::vector<const GlobalCondition *> black_preconditions;
	std::vector<const GlobalCondition *> red_preconditions;
	std::vector<const GlobalEffect *> black_effects;
	std::vector<const GlobalEffect *> red_effects;

public:
	explicit RBOperator(const GlobalOperator &base_operator);

	void apply_painting(const Painting &painting);
	auto is_applicable(const RBState &state) const -> bool;

	auto is_black() const -> bool {
		return black_effects.empty();
	}

	auto get_base_operator() const -> const GlobalOperator & {
		return base_operator;
	}

	auto get_id() const -> OperatorID {
		return OperatorID(get_op_index_hacked(&base_operator));
	}

	auto get_black_preconditions() const -> std::vector<const GlobalCondition *> {
		return black_preconditions;
	}

	auto get_red_preconditions() const -> std::vector<const GlobalCondition *> {
		return red_preconditions;
	}

	auto get_black_effects() const -> std::vector<const GlobalEffect *> {
		return black_effects;
	}

	auto get_red_effects() const -> std::vector<const GlobalEffect *> {
		return red_effects;
	}

};

}

#endif
