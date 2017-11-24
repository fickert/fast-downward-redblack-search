#ifndef REDBLACK_STATE_REGISTRY_H
#define REDBLACK_STATE_REGISTRY_H

#include "../state_registry_base.h"
#include "../task_utils/successor_generator.h"

#include "int_packer.h"


namespace redblack {
class RBState;
class RBOperator;

class RBStateRegistry : public StateRegistryBase<RBState, RBOperator> {
	const Painting *painting;

	std::vector<std::vector<OperatorID>> initial_state_best_supporters;
	mutable std::vector<std::vector<OperatorID>> cached_best_supporters;
	const std::vector<RBOperator> operators;

	auto rb_state_packer() const -> const RBIntPacker & {
		return static_cast<const RBIntPacker &>(state_packer);
	}

	void saturate_state(PackedStateBin *buffer, bool store_best_supporters = false) const;
	void build_unsaturated_successor(const RBState &predecessor, const RBOperator &op, PackedStateBin *buffer) const;


	// data structures and class members for the relaxed exploration
	struct Counter {
		Counter(int num_preconditions,
		        const std::vector<std::vector<FactPair>> &negative_preconditions,
		        const std::vector<std::pair<FactPair, std::vector<FactPair>>> &condeff_preconditions) :
			effects(),
			num_preconditions(num_preconditions),
			negative_preconditions(negative_preconditions),
			condeff_preconditions(condeff_preconditions),
			value(0) {}

		struct Effect {
			Effect(const FactPair &fact, const OperatorID &supporter) :
				fact(fact), supporter(supporter) {}

			const FactPair fact;
			const OperatorID supporter;
		};

		std::vector<Effect> effects;
		const int num_preconditions;
		std::vector<std::vector<FactPair>> negative_preconditions;
		std::vector<std::pair<FactPair, std::vector<FactPair>>> condeff_preconditions;
		int value;
	};

	mutable std::vector<Counter> counters;
	std::vector<std::vector<std::vector<std::size_t>>> precondition_of;

public:
	RBStateRegistry(const AbstractTask &task, const RBIntPacker &state_packer,
	                AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data,
	                const std::vector<RBOperator> &operators, PackedStateBin *rb_initial_state_data = nullptr);
	~RBStateRegistry();

	auto get_stored_best_supporters() const -> const std::vector<std::vector<OperatorID>> & {
		return cached_best_supporters;
	}

	auto get_initial_state_best_supporters() const -> const std::vector<std::vector<OperatorID>> & {
		return initial_state_best_supporters;
	}

	auto lookup_state(StateID id) const -> RBState override;
	auto get_initial_state() -> const RBState & override;
	auto get_successor_state(const RBState &predecessor, const RBOperator &op) -> RBState override;

	auto get_best_supporters_for_successor(const RBState &predecessor, const RBOperator &op) const -> const std::vector<std::vector<OperatorID>> &;

	auto get_painting() const -> const Painting & { return *painting; }

	auto get_operators() const -> const std::vector<RBOperator> & { return operators; }
};

}


template<>
auto StateRegistryBase<redblack::RBState, redblack::RBOperator>::lookup_state(StateID) const -> redblack::RBState;

template<>
auto StateRegistryBase<redblack::RBState, redblack::RBOperator>::get_successor_state(const redblack::RBState &, const redblack::RBOperator &) -> redblack::RBState;

#endif
