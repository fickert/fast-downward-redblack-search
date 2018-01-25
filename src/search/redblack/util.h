#ifndef REDBLACK_UTIL_H
#define REDBLACK_UTIL_H

#include "../operator_cost.h"
#include <vector>
#include <boost/dynamic_bitset/dynamic_bitset.hpp>

struct FactPair;
class OperatorID;
class GlobalState;

namespace redblack {
class Painting;
class RBState;
class RBOperator;
}

auto get_adjusted_action_cost(const redblack::RBOperator &op, OperatorCost cost_type) -> int;
auto get_op_index_hacked(const redblack::RBOperator *op) -> int;
auto test_goal(const redblack::RBState &state) -> bool;

namespace options {
class Options;
}

namespace redblack {
void add_num_black_options(options::OptionParser &parser);
auto get_num_black(const options::Options &opts, bool min_one_if_ratio = false) -> int;

auto any_conditional_effect_condition_is_red(const Painting &painting) -> int;
auto get_no_red_conditional_effect_conditions_painting(const Painting &painting) -> Painting;

void debug_verify_relaxed_plan(const GlobalState &state, const std::vector<OperatorID> &relaxed_plan, const std::vector<FactPair> &goal_facts);
auto get_ordered_relaxed_plan(const GlobalState &state, const std::vector<OperatorID> &relaxed_plan) -> std::vector<OperatorID>;
auto get_red_plan(const std::vector<std::vector<OperatorID>> &best_supporters, const GlobalState &state, const std::vector<FactPair> &goal_facts, bool ordered) -> std::vector<OperatorID>;
auto get_ordered_relaxed_plan(const std::vector<boost::dynamic_bitset<>> &state, const std::vector<OperatorID> &relaxed_plan) -> std::vector<OperatorID>;
auto get_red_plan(const std::vector<std::vector<OperatorID>> &best_supporters, const std::vector<boost::dynamic_bitset<>> &state, const std::vector<FactPair> &goal_facts, bool ordered) -> std::vector<OperatorID>;
}



#endif
