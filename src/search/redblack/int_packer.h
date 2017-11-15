#ifndef REDBLACK_INT_PACKER_H
#define REDBLACK_INT_PACKER_H

#include <vector>

#include "painting.h"
#include "../algorithms/int_packer.h"

/*
  Utility class to pack lots of unsigned integers (called "variables"
  in the code below) with a small domain {0, ..., range - 1}
  tightly into memory. This works like a bitfield except that the
  fields and sizes don't need to be known at compile time.

  For example, if we have 40 binary variables and 20 variables with
  range 4, storing them would theoretically require at least 80 bits,
  and this class would pack them into 12 bytes (three 4-byte "bins").

  Uses a greedy bin-packing strategy to pack the variables, which
  should be close to optimal in most cases. (See code comments for
  details.)
*/
namespace redblack {
class RBIntPacker : public int_packer::IntPacker {
	const Painting &painting;

	int num_additional_bins;
	std::vector<int> var_to_bin;
public:
	explicit RBIntPacker(const Painting &painting);
	~RBIntPacker();

	bool get_bit(const Bin *buffer, int var, int value) const;
	void set_bit(Bin *buffer, int var, int value) const;
	void init_zero(Bin *buffer, int var) const;

	auto get_available_bits(int used_bits, std::vector<std::vector<int>> &bits_to_vars) -> int override;
	auto get_bits_for_var(const std::vector<int> &ranges, int var, std::vector<std::vector<int>> &bits_to_vars) -> int override;
	void update_var_info(int var, const std::vector<int> &ranges, int bin_index, int used_bits, int bits) override;

	auto get_painting() const -> const Painting & { return painting; }
};
}

#endif
