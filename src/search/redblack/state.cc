#include "state.h"

#include "../utils/system.h"
#include "../globals.h"

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

auto RBState::get_redblack_values() const -> std::vector<boost::dynamic_bitset<>> {
	auto values = std::vector<boost::dynamic_bitset<>>(g_root_task()->get_num_variables());
	for (auto var = 0u; var < values.size(); ++var) {
		values[var].resize(g_root_task()->get_variable_domain_size(var));
		if (painting->is_black_var(var)) {
			values[var][this->operator[](var)] = true;
		} else {
			for (auto value = 0; value < g_root_task()->get_variable_domain_size(var); ++value)
				if (int_packer->get_bit(buffer, var, value))
					values[var][value] = true;
			assert(values[var].any());
		}
	}
	return values;
}

void RBState::dump_pddl() const {
	// TODO: implement
}

void RBState::dump_fdr() const {
	// TODO: implement
}

}
