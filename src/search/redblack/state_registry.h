#ifndef REDBLACK_STATE_REGISTRY_H
#define REDBLACK_STATE_REGISTRY_H

#include "../state_registry_base.h"
#include "../task_utils/successor_generator.h"

#include "int_packer.h"


namespace redblack {
class RBState;
class RBOperator;
class StateSaturation;

class RBStateRegistry : public StateRegistryBase<RBState, RBOperator> {
	const Painting *painting;
	const std::vector<RBOperator> operators;
	std::vector<std::vector<OperatorID>> initial_state_best_supporters;

	auto rb_state_packer() const -> const RBIntPacker & {
		return static_cast<const RBIntPacker &>(state_packer);
	}

	void build_unsaturated_successor(const RBState &predecessor, const RBOperator &op, PackedStateBin *buffer) const;

	std::unique_ptr<StateSaturation> state_saturation;

	static auto get_state_saturation(const AbstractTask &task, const RBIntPacker &state_packer, const std::vector<RBOperator> &operators) -> std::unique_ptr<StateSaturation>;
	static auto construct_redblack_operators(const Painting &painting) -> std::vector<RBOperator>;

	auto get_state(const std::vector<int> &values, bool get_best_supporters) -> std::pair<RBState, std::vector<std::vector<OperatorID>>>;
	auto get_successor_state(const RBState &predecessor, const RBOperator &op, bool get_best_supporters) -> std::pair<RBState, std::vector<std::vector<OperatorID>>>;

public:
	RBStateRegistry(const AbstractTask &task, const RBIntPacker &state_packer,
	                AxiomEvaluator &axiom_evaluator, std::vector<int> &&initial_state_data,
	                PackedStateBin *rb_initial_state_data = nullptr);
	RBStateRegistry(const AbstractTask &task, const RBIntPacker &state_packer,
	                AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data,
	                PackedStateBin *rb_initial_state_data = nullptr);
	~RBStateRegistry();

	auto get_initial_state_best_supporters() const -> const std::vector<std::vector<OperatorID>> & {
		return initial_state_best_supporters;
	}

	auto lookup_state(StateID id) const -> RBState override;
	auto get_initial_state() -> const RBState & override;
	auto get_successor_state(const RBState &predecessor, const RBOperator &op) -> RBState override;
	auto get_successor_state_and_best_supporters(const RBState &predecessor, const RBOperator &op) -> std::pair<RBState, std::vector<std::vector<OperatorID>>>;
	auto get_best_supporters_for_successor(const RBState &predecessor, const RBOperator &op) const -> std::vector<std::vector<OperatorID>>;
	auto get_state(const std::vector<int> &values) -> RBState;
	auto get_state_and_best_supporters(const std::vector<int> &values) -> std::pair<RBState, std::vector<std::vector<OperatorID>>>;

	auto get_painting() const -> const Painting & { return *painting; }

	auto get_operators() const -> const std::vector<RBOperator> & { return operators; }
};

}


template<>
auto StateRegistryBase<redblack::RBState, redblack::RBOperator>::lookup_state(StateID) const -> redblack::RBState;

template<>
auto StateRegistryBase<redblack::RBState, redblack::RBOperator>::get_successor_state(const redblack::RBState &, const redblack::RBOperator &) -> redblack::RBState;

#endif
