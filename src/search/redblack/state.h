#ifndef REDBLACK_STATE_H
#define REDBLACK_STATE_H

#include "../state_registry_base.h"

#include "int_packer.h"
#include "state_registry.h"

#include <vector>

namespace redblack {
class RBState;
class RBOperator;
using RBStateRegistryBase = StateRegistryBase<RBState, RBOperator>;

using PackedStateBin = RBIntPacker::Bin;

class RBState : public StateBase<RBStateRegistryBase> {
	friend RBStateRegistry;

	// Only used by the (red-black) state registry.
	RBState(const PackedStateBin *buffer, const RBStateRegistryBase &registry, StateID id, const Painting &painting, const RBIntPacker &int_packer);

	const Painting * painting;
	const RBIntPacker * int_packer;

public:
	~RBState() = default;

	RBState(const RBState &other) = default;
	RBState(RBState &&other) = default;

	auto operator=(const RBState &other) -> RBState & = default;
	auto operator=(RBState &&other) -> RBState & = default;

	int operator[](int var) const override;
	auto has_fact(int var, int value) const -> bool;

	auto get_rb_state_registry() const -> const RBStateRegistry & {
		assert(dynamic_cast<const RBStateRegistry *>(registry));
		return *dynamic_cast<const RBStateRegistry *>(registry);
	}

	auto get_painting() const -> const Painting & {
		return *painting;
	}

	std::vector<int> get_values() const override;

	void dump_pddl() const override;
	void dump_fdr() const override;

};
}

#endif
