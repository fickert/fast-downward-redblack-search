#include "state.h"

#include "../utils/system.h"

namespace redblack {

RBState::RBState(const PackedStateBin *buffer, const RBStateRegistryBase &registry, StateID id, const Painting &painting, const RBIntPacker &int_packer)
	: StateBase<RBStateRegistryBase>(buffer, registry, id),
	  painting(&painting),
	  int_packer(&int_packer) {}

int RBState::operator[](int var) const {
	assert(painting->is_black_var(var));
	return StateBase<RBStateRegistryBase>::operator[](var);
}

auto RBState::has_fact(int var, int value) const -> bool {
	return painting->is_red_var(var) ? int_packer->get_bit(buffer, var, value) : StateBase<RBStateRegistryBase>::operator[](var) == value;
}

std::vector<int> RBState::get_values() const {
	assert(false && "don't call this on red-black states");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

void RBState::dump_pddl() const {
	// TODO: implement
}

void RBState::dump_fdr() const {
	// TODO: implement
}

}
