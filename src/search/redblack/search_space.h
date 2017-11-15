#ifndef REDBLACK_SEARCH_SPACE_H
#define REDBLACK_SEARCH_SPACE_H

#include "../search_space.h"


namespace redblack {
class RBOperator;
class RBState;
}

template<>
void SearchSpace<redblack::RBState, redblack::RBOperator>::trace_path(const redblack::RBState &, std::vector<const redblack::RBOperator*> &) const;

#endif
