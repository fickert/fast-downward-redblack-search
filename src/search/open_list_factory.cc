#include "open_list_factory.h"

#include "plugin.h"

using namespace std;

/*
template<class StateType, class OperatorType>
template<>
unique_ptr<StateOpenList<StateType, OperatorType>> OpenListFactory<StateType, OperatorType>::create_open_list<StateOpenList<StateType, OperatorType>>() {
    return create_state_open_list();
}

template<class StateType, class OperatorType>
template<>
unique_ptr<EdgeOpenList<StateType, OperatorType>> OpenListFactory<StateType, OperatorType>::create_open_list<EdgeOpenList<StateType, OperatorType>>() {
    return create_edge_open_list();
}
*/


static PluginTypePlugin<OpenListFactory<GlobalState, GlobalOperator>> _type_plugin(
    "OpenList",
    // TODO: Replace empty string by synopsis for the wiki page.
    "");
