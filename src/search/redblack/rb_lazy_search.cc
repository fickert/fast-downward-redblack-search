#include "rb_lazy_search.h"

namespace lazy_search {

template<>
auto LazySearch<redblack::RBState, redblack::RBOperator>::get_operator(int op_id) const -> const redblack::RBOperator * {
	return &dynamic_cast<redblack::RBStateRegistry *>(state_registry.get())->get_operators()[op_id];
}

}
