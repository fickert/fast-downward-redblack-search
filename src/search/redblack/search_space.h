#ifndef REDBLACK_SEARCH_SPACE_H
#define REDBLACK_SEARCH_SPACE_H

#include "../search_space.h"


namespace redblack {
class RBOperator;
class RBState;
}

template<>
void SearchSpace<redblack::RBState, redblack::RBOperator>::trace_path(const redblack::RBState &, std::vector<const redblack::RBOperator*> &) const;

template<>
auto SearchSpace<redblack::RBState, redblack::RBOperator>::trace_rb_path(const redblack::RBState &state, const std::vector<FactPair> &additional_goal_facts) const
	-> std::pair<std::set<FactPair>, std::vector<std::tuple<StateID, std::vector<OperatorID>, OperatorID>>>;

#endif
