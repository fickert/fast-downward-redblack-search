#include "int_packer.h"

#include <cassert>

using namespace std;

namespace int_packer {

auto IntPacker::get_bit_mask(int from, int to) -> Bin {
	// Return mask with all bits in the range [from, to) set to 1.
	assert(from >= 0 && to >= from && to <= BITS_PER_BIN);
	int length = to - from;
	if (length == BITS_PER_BIN) {
		// 1U << BITS_PER_BIN has undefined behaviour in C++; e.g.
		// 1U << 32 == 1 (not 0) on 32-bit Intel platforms. Hence this
		// special case.
		assert(from == 0 && to == BITS_PER_BIN);
		return ~Bin(0);
	} else {
		return ((Bin(1) << length) - 1) << from;
	}
}

static int get_bit_size_for_range(int range) {
    int num_bits = 0;
    while ((1U << num_bits) < static_cast<unsigned int>(range))
        ++num_bits;
    return num_bits;
}

IntPacker::VariableInfo::VariableInfo(int range_, int bin_index_, int shift_)
	: range(range_),
	  bin_index(bin_index_),
	  shift(shift_) {
	if (static_cast<unsigned int>(range) == pow(2u, BITS_PER_BIN) - 1) {
		clear_mask = 0;
		read_mask = ~clear_mask;
	} else {
		int bit_size = get_bit_size_for_range(range);
		read_mask = get_bit_mask(shift, shift + bit_size);
		clear_mask = ~read_mask;
	}
}

IntPacker::VariableInfo::VariableInfo()
	: range(0), bin_index(-1), shift(0), read_mask(0), clear_mask(0) {
	// Default constructor needed for resize() in pack_bins.
}

IntPacker::VariableInfo::~VariableInfo() {}

auto IntPacker::VariableInfo::get(const Bin *buffer) const -> int {
	return (buffer[bin_index] & read_mask) >> shift;
}

void IntPacker::VariableInfo::set(Bin *buffer, int value) const {
	assert(value >= 0 && static_cast<unsigned int>(value) < static_cast<unsigned int>(range));
	Bin &bin = buffer[bin_index];
	bin = (bin & clear_mask) | (value << shift);
}

bool IntPacker::VariableInfo::get_bit(const Bin *buffer, int value) const {
	return (buffer[bin_index] & 1 << (shift + value)) >> (shift + value);
}

void IntPacker::VariableInfo::set_bit(Bin *buffer, int value) const {
	assert(value >= 0 && pow(2u, value) < static_cast<unsigned int>(range));
	Bin &bin = buffer[bin_index];
	bin = bin | (1 << (shift + value));
}

void IntPacker::VariableInfo::init_zero(Bin *buffer) const {
	Bin &bin = buffer[bin_index];
	bin = (bin & clear_mask);
}

IntPacker::IntPacker()
    : num_bins(0) {}

IntPacker::~IntPacker() {
}

void IntPacker::initialize(const std::vector<int> &ranges) {
	assert(var_infos.empty() && "initialize() was probably called twice");
	pack_bins(ranges);
}

int IntPacker::get(const Bin *buffer, int var) const {
    return var_infos[var].get(buffer);
}

void IntPacker::set(Bin *buffer, int var, int value) const {
    var_infos[var].set(buffer, value);
}

auto IntPacker::get_available_bits(int used_bits, std::vector<std::vector<int>> &) -> int {
	return BITS_PER_BIN - used_bits;
}

auto IntPacker::get_bits_for_var(const vector<int> &ranges, int var, std::vector<std::vector<int>> &) -> int {
	auto bits = get_bit_size_for_range(ranges[var]);
	assert(bits <= BITS_PER_BIN);
	return bits;
}

void IntPacker::pack_bins(const vector<int> &ranges) {
    assert(var_infos.empty());

    int num_vars = ranges.size();
    var_infos.resize(num_vars);

    // bits_to_vars[k] contains all variables that require exactly k
    // bits to encode. Once a variable is packed into a bin, it is
    // removed from this index.
    // Loop over the variables in reverse order to prefer variables with
    // low indices in case of ties. This might increase cache-locality.
    vector<vector<int>> bits_to_vars(BITS_PER_BIN + 1);
    for (int var = num_vars - 1; var >= 0; --var) {
        auto bits = get_bits_for_var(ranges, var, bits_to_vars);
        bits_to_vars[bits].push_back(var);
    }

    int packed_vars = 0;
    while (packed_vars != num_vars)
        packed_vars += pack_one_bin(ranges, bits_to_vars);
}

void IntPacker::update_var_info(int variable, const vector<int> &ranges, int bin_index, int used_bits, int) {
	var_infos[variable] = VariableInfo(ranges[variable], bin_index, used_bits);
}

int IntPacker::pack_one_bin(const vector<int> &ranges,
                            vector<vector<int>> &bits_to_vars) {
    // Returns the number of variables added to the bin. We pack each
    // bin with a greedy strategy, always adding the largest variable
    // that still fits.

    ++num_bins;
    int bin_index = num_bins - 1;
    int used_bits = 0;
    int num_vars_in_bin = 0;

    while (true) {
        // Determine size of largest variable that still fits into the bin.
        int bits = get_available_bits(used_bits, bits_to_vars);
        while (bits > 0 && bits_to_vars[bits].empty())
            --bits;

        if (bits == 0) {
            // No more variables fit into the bin.
            // (This also happens when all variables have been packed.)
            return num_vars_in_bin;
        }

        // We can pack another variable of size bits into the current bin.
        // Remove the variable from bits_to_vars and add it to the bin.
        vector<int> &best_fit_vars = bits_to_vars[bits];
        int var = best_fit_vars.back();
        best_fit_vars.pop_back();

        update_var_info(var, ranges, bin_index, used_bits, bits);
        used_bits += bits;
        ++num_vars_in_bin;
    }
}

}
