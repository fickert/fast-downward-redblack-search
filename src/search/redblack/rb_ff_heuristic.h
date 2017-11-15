#ifndef REDBLACK_RB_FF_HEURISTIC_H
#define REDBLACK_RB_FF_HEURISTIC_H

#include "../heuristics/ff_heuristic.h"

namespace redblack {
class RBState;
class RBOperator;
}

namespace additive_heuristic {

template<>
template<>
auto AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::convert_state(const redblack::RBState &state);

template<>
template<>
void AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::setup_exploration_queue_state(const redblack::RBState &state);

template<>
template<>
auto AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::is_operator_applicable(const redblack::RBState &state, int operator_no) -> bool;

template<>
int AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::compute_heuristic(const State &);

template<>
void AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::compute_heuristic_for_cegar(const State &);

}


#endif
