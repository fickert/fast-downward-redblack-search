#ifndef REDBLACK_UTIL_H
#define REDBLACK_UTIL_H

#include "../operator_cost.h"

namespace redblack {
class RBOperator;
}

int get_adjusted_action_cost(const redblack::RBOperator &op, OperatorCost cost_type);
int get_op_index_hacked(const redblack::RBOperator *op);

namespace options {
class Options;
}

namespace redblack {
void add_num_black_options(options::OptionParser &parser);
auto get_num_black(const options::Options &opts, bool min_one_if_ratio = false) -> int;
}



#endif
