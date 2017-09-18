#include "state.h"

#include "../utils/system.h"

namespace redblack {

RBState::RBState(const PackedStateBin *buffer, const RBStateRegistry &registry, StateID id, const Painting &painting, const RBIntPacker &int_packer)
	: GlobalState(buffer, registry, id),
	  painting(painting),
	  int_packer(int_packer) {}

int RBState::operator[](int var) const {
	assert(painting.is_black_var(var));
	return GlobalState::operator[](var);
}

auto RBState::has_fact(int var, int value) const -> bool {
	return painting.is_red_var(var) ? int_packer.get_bit(buffer, var, value) : GlobalState::operator[](var) == value;
}

std::vector<int> RBState::get_values() const {
	assert(false && "don't call this on red-black states");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

}
