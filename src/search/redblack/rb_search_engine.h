#ifndef REDBLACK_RB_SEARCH_ENGINE_H
#define REDBLACK_RB_SEARCH_ENGINE_H

#include "../search_engine.h"

namespace redblack {
class RBOperator;
class RBState;
}

template<>
bool SearchEngine<redblack::RBState, redblack::RBOperator>::check_goal_and_set_plan(const redblack::RBState &state);

#endif
