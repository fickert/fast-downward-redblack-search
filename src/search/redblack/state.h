#ifndef REDBLACK_STATE_H
#define REDBLACK_STATE_H

#include "../state_id.h"
#include "../global_state.h"

#include "state_registry.h"
#include "int_packer.h"

#include <vector>

namespace redblack {
using PackedStateBin = RBIntPacker::Bin;

class RBState : public GlobalState {
	friend class RBStateRegistry;

	// Only used by the (red-black) state registry.
	RBState(const PackedStateBin *buffer, const RBStateRegistry &registry, StateID id, const Painting &painting, const RBIntPacker &int_packer);

	const Painting &painting;
	const RBIntPacker &int_packer;

public:
	~RBState() = default;

	int operator[](int var) const override;
	auto has_fact(int var, int value) const -> bool;

	auto get_painting() const -> const Painting & {
		return painting;
	}

	std::vector<int> get_values() const override;
};
}

#endif
