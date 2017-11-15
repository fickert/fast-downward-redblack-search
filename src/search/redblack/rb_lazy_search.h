#ifndef REDBLACK_RB_LAZY_SEARCH_H
#define REDBLACK_RB_LAZY_SEARCH_H

#include "../search_engines/lazy_search.h"
#include "state.h"
#include "operator.h"
#include "util.h"


namespace lazy_search {

template<>
auto LazySearch<redblack::RBState, redblack::RBOperator>::get_operator(int op_id) const -> const redblack::RBOperator *;

}

#endif
