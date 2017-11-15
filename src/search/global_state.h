#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

#include "state_registry_base.h"

#include "algorithms/int_packer.h"


class GlobalOperator;
template<class StateType, class OperatorType>
class StateRegistryBase;

using PackedStateBin = int_packer::IntPacker::Bin;

// For documentation on classes relevant to storing and working with registered
// states see the file state_registry.h.
class GlobalState : public StateBase<StateRegistryBase<GlobalState, GlobalOperator>> {
protected:
    friend class StateRegistryBase<GlobalState, GlobalOperator>;

    // Only used by the state registry.
    GlobalState(
        const PackedStateBin *buffer, const StateRegistryBase<GlobalState, GlobalOperator> &registry, StateID id);

public:
    virtual ~GlobalState() = default;

    void dump_pddl() const override;
    void dump_fdr() const override;
};


#endif
