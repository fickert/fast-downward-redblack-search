#include "global_state.h"

#include "state_registry.h"
#include "task_proxy.h"

GlobalState::GlobalState(const PackedStateBin *buffer, const StateRegistryBase<GlobalState, GlobalOperator> &registry, StateID id)
	: StateBase<StateRegistryBase<GlobalState, GlobalOperator>>(buffer, registry, id) {}

void GlobalState::dump_pddl() const {
    State state(registry->get_task(), get_values());
    state.dump_pddl();
}

void GlobalState::dump_fdr() const {
    State state(registry->get_task(), get_values());
    state.dump_fdr();
}
