#include "search_space.h"


template<>
void SearchSpace<GlobalState, GlobalOperator>::trace_path(const GlobalState &goal_state,
                                                          std::vector<const GlobalOperator *> &path) const {
    GlobalState current_state = goal_state;
    assert(path.empty());
    for (;;) {
        const SearchNodeInfo &info = search_node_infos[current_state];
        if (info.creating_operator == -1) {
            assert(info.parent_state_id == StateID::no_state);
            break;
        }
        assert(utils::in_bounds(info.creating_operator, g_operators));
        const GlobalOperator *op = &g_operators[info.creating_operator];
        path.push_back(op);
        current_state = state_registry.lookup_state(info.parent_state_id);
    }
    reverse(path.begin(), path.end());
}
