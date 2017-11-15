#ifndef STATE_REGISTRY_H
#define STATE_REGISTRY_H

#include "state_registry_base.h"

class StateRegistry : public StateRegistryBase<GlobalState, GlobalOperator> {
public:
	StateRegistry(
		const AbstractTask &task, const int_packer::IntPacker &state_packer,
		AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data);
};

#endif
