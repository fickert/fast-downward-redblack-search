#include "state_registry.h"
#include "global_operator.h"

template<>
GlobalState StateRegistryBase<GlobalState, GlobalOperator>::get_successor_state(const GlobalState &predecessor, const GlobalOperator &op) {
	assert(!op.is_axiom());
	state_data_pool.push_back(predecessor.get_packed_buffer());
	PackedStateBin *buffer = state_data_pool[state_data_pool.size() - 1];
	for (size_t i = 0; i < op.get_effects().size(); ++i) {
		const GlobalEffect &effect = op.get_effects()[i];
		if (effect.does_fire(predecessor))
			state_packer.set(buffer, effect.var, effect.val);
	}
	axiom_evaluator.evaluate(buffer, state_packer);
	StateID id = insert_id_or_pop_state();
	return lookup_state(id);
}

template<>
GlobalState StateRegistryBase<GlobalState, GlobalOperator>::lookup_state(StateID id) const {
	return GlobalState(state_data_pool[id.value], *this, id);
}

StateRegistry::StateRegistry(const AbstractTask &task, const int_packer::IntPacker &state_packer, AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data)
	: StateRegistryBase<GlobalState, GlobalOperator>(task, state_packer, axiom_evaluator, initial_state_data) {}
