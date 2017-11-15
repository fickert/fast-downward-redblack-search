#include "lazy_search.h"

namespace lazy_search {

template<>
auto LazySearch<GlobalState, GlobalOperator>::get_operator(int op_id) const -> const GlobalOperator * {
	return &g_operators[op_id];
};

}