#include "int_packer.h"

#include "../globals.h"
#include "../abstract_task.h"

namespace redblack {

RBIntPacker::RBIntPacker(const Painting &painting)
	: IntPacker(),
	  painting(painting),
	  num_additional_bins(0),
	  var_to_bin(g_root_task()->get_num_variables(), -1) {}

RBIntPacker::~RBIntPacker() {}

auto RBIntPacker::get_bit(const Bin *buffer, int var, int value) const -> bool {
	if (value >= BITS_PER_BIN) {
		var = var_to_bin[var];
		value -= BITS_PER_BIN;
		while (value >= BITS_PER_BIN) {
			++var;
			value -= BITS_PER_BIN;
		}
	}
	return var_infos[var].get_bit(buffer, value);
}

void RBIntPacker::set_bit(Bin *buffer, int var, int value) const {
	if (value >= BITS_PER_BIN) {
		var = var_to_bin[var];
		value -= BITS_PER_BIN;
		while (value >= BITS_PER_BIN) {
			++var;
			value -= BITS_PER_BIN;
		}
	}
	var_infos[var].set_bit(buffer, value);
}

void RBIntPacker::init_zero(Bin *buffer, int var) const {
	var_infos[var].init_zero(buffer);
	int i = 0;
	int number_bits = BITS_PER_BIN;
	while (number_bits < g_variable_domain[var]) {
		var_infos[var_to_bin[var] + i].init_zero(buffer);
		number_bits += BITS_PER_BIN;
		++i;
	}
}

auto RBIntPacker::get_bits_for_var(const std::vector<int> &ranges, int var, std::vector<std::vector<int>> &bits_to_vars) -> int {
	if (!painting.is_red_var(var))
		return IntPacker::get_bits_for_var(ranges, var, bits_to_vars);
	auto bits = g_root_task()->get_variable_domain_size(var);
	if (bits >= static_cast<int>(bits_to_vars.size()))
		bits_to_vars.resize(bits + 1);
	return bits;
}

void RBIntPacker::update_var_info(int var, const std::vector<int> &ranges, int bin_index, int used_bits, int bits) {
	if (!painting.is_red_var(var)) {
		IntPacker::update_var_info(var, ranges, bin_index, used_bits, bits);
		return;
	}
	auto stored_bits = 0;
	while (stored_bits < bits) {
		unsigned int range;
		if (bits - stored_bits >= BITS_PER_BIN) {
			range = pow(2, BITS_PER_BIN) - 1;
		}
		else {
			range = pow(2, bits - stored_bits);
		}
		if (stored_bits == 0) {
			var_infos[var] = VariableInfo(range, bin_index, used_bits);
			if (bits > BITS_PER_BIN) {
				var_to_bin[var] = g_variable_domain.size() + num_additional_bins;
			}
			stored_bits += std::min(BITS_PER_BIN, bits - stored_bits);
		}
		else {
			++num_bins;
			++num_additional_bins;

			var_infos.resize(g_variable_domain.size() + num_additional_bins);
			var_infos[var_infos.size() - 1] = VariableInfo(range, num_bins - 1, used_bits);

			stored_bits += std::min(BITS_PER_BIN, bits - stored_bits);
		}
	}
}

}
