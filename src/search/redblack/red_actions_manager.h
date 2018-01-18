#ifndef REDBLACK_RED_ACTIONS_MANAGER_H
#define REDBLACK_RED_ACTIONS_MANAGER_H

#include "state.h"
#include "../operator_id.h"
#include <vector>

#include <boost/dynamic_bitset.hpp>

namespace redblack {

class RedActionsManager {
	boost::dynamic_bitset<> red_operators;
	std::unordered_map<std::vector<FactPair>, boost::dynamic_bitset<>> conditionally_red_operators;

public:
	RedActionsManager(const std::vector<RBOperator> &operators);

	auto get_red_actions_for_state(const GlobalState &state) -> boost::dynamic_bitset<>;
};
}

#endif
