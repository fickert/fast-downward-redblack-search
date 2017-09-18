#ifndef REDBLACK_STATE_REGISTRY_H
#define REDBLACK_STATE_REGISTRY_H

#include "../task_utils/successor_generator.h"
#include "../state_registry.h"

#include "int_packer.h"
#include "operator.h"
#include "state.h"

namespace redblack {
class RBStateRegistry : public StateRegistry {
	const successor_generator::SuccessorGenerator &successor_generator;
	const Painting &painting;

	std::vector<std::vector<OperatorID>> initial_state_best_supporters;
	mutable std::vector<std::vector<OperatorID>> cached_best_supporters;
	const std::vector<const RBOperator> &operators;

	auto convert_state(const GlobalState &state) const -> RBState;
	auto convert_state(const RBState &state) const -> GlobalState;

	auto rb_state_packer() const -> const RBIntPacker & {
		return static_cast<const RBIntPacker &>(state_packer);
	}

	void saturate_state(PackedStateBin *buffer, bool store_best_supporters = false) const;

public:
	RBStateRegistry(const AbstractTask &task, const RBIntPacker &state_packer,
	                AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data,
	                const successor_generator::SuccessorGenerator &successor_generator, const std::vector<const RBOperator> &operators);
	RBStateRegistry::~RBStateRegistry();

	auto get_stored_best_supporters() const -> const std::vector<std::vector<OperatorID>> & {
		return cached_best_supporters;
	}

	auto get_initial_state_best_supporters() const -> const std::vector<std::vector<OperatorID>> & {
		return initial_state_best_supporters;
	}

	auto rb_lookup_state(StateID id) const -> RBState;
	auto rb_get_initial_state() -> const RBState &;
	auto rb_get_successor_state(const RBState &predecessor, const RBOperator &op) -> RBState;

};
}

#endif
