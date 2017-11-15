#ifndef ALGORITHMS_INT_PACKER_H
#define ALGORITHMS_INT_PACKER_H

#include <vector>

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
namespace int_packer {
class IntPacker {
public:
	using Bin = unsigned int;

    IntPacker();
    virtual ~IntPacker();

    /*
      The initialize function takes the range for each variable. The domain of
      variable i is {0, ..., ranges[i] - 1}. Because we are using signed
      ints for the ranges (and genenerally for the values of variables),
      a variable can take up at most 31 bits if int is 32-bit.
    */
	void initialize(const std::vector<int> &ranges);

    int get(const Bin *buffer, int var) const;
    void set(Bin *buffer, int var, int value) const;

	int get_num_bins() const {return num_bins; }

protected:
	static constexpr auto BITS_PER_BIN = static_cast<int>(sizeof(Bin) * 8);
	static auto get_bit_mask(int from, int to) -> Bin;

	class VariableInfo {
		int range;
		int bin_index;
		int shift;
		Bin read_mask;
		Bin clear_mask;
    public:
		VariableInfo(int range_, int bin_index_, int shift_);
		VariableInfo();
		~VariableInfo();

		int get(const Bin *buffer) const;
		void set(Bin *buffer, int value) const;

		bool get_bit(const Bin *buffer, int value) const;
		void set_bit(Bin *buffer, int value) const;
		void init_zero(Bin *buffer) const;
    };

    std::vector<VariableInfo> var_infos;
    int num_bins;

	virtual auto get_available_bits(int used_bits, std::vector<std::vector<int>> &bits_to_vars) -> int;
	virtual auto get_bits_for_var(const std::vector<int> &ranges, int var, std::vector<std::vector<int>> &bits_to_vars) -> int;
	virtual void update_var_info(int variable, const std::vector<int> &ranges, int bin_index, int used_bits, int bits);

    int pack_one_bin(const std::vector<int> &ranges,
                     std::vector<std::vector<int>> &bits_to_vars);
    void pack_bins(const std::vector<int> &ranges);

};
}

#endif
