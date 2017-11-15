#include "util.h"

#include "operator.h"
#include "../operator_cost.h"

int get_adjusted_action_cost(const redblack::RBOperator &op, OperatorCost cost_type) {
	return get_adjusted_action_cost(op.get_base_operator(), cost_type);
}

int get_op_index_hacked(const redblack::RBOperator *op) {
	return get_op_index_hacked(&op->get_base_operator());
}
