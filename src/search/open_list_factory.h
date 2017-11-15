#ifndef OPEN_LIST_FACTORY_H
#define OPEN_LIST_FACTORY_H

#include "open_list.h"

#include <memory>


template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class OpenListFactory {
public:
    OpenListFactory() = default;
    virtual ~OpenListFactory() = default;

    OpenListFactory(const OpenListFactory<StateType, OperatorType> &) = delete;

    virtual std::unique_ptr<StateOpenList<StateType, OperatorType>> create_state_open_list() = 0;
    virtual std::unique_ptr<EdgeOpenList<StateType, OperatorType>> create_edge_open_list() = 0;

    /*
      The following template receives manual specializations (in the
      cc file) for the open list types we want to support. It is
      intended for templatized callers, e.g. the constructor of
      AlternationOpenList.
    */
    template<typename T>
    std::unique_ptr<OpenList<T, StateType, OperatorType>> create_open_list();
};


template<class StateType, class OperatorType>
template<typename T>
std::unique_ptr<OpenList<T, StateType, OperatorType>> OpenListFactory<StateType, OperatorType>::create_open_list() {
	if constexpr (std::is_same_v<T, StateOpenListEntry>) {
		return create_state_open_list();
	} else if constexpr (std::is_same_v<T, EdgeOpenListEntry>) {
		return create_edge_open_list();
	} else {
		static_assert(false, "illegal open list entry type");
	}
}
// TODO: use partial specialization instead...?

#endif
