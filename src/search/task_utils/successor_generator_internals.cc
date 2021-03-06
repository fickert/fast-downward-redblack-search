#include "successor_generator_internals.h"

#include "../global_state.h"
#include "../task_proxy.h"
#include "../globals.h"

#include <cassert>
#include "../redblack/operator.h"

using namespace std;

/*
  Notes on possible optimizations:

  - Using specialized allocators (e.g. an arena allocator) could
    improve cache locality and reduce memory.

  - We could keep the different nodes in a single vector (for example
    of type unique_ptr<GeneratorBase>) and then use indices rather
    than pointers for representing child nodes. This would reduce the
    memory overhead for pointers in 64-bit builds. However, this
    overhead is not as bad as it used to be.

  - Going further down this route, on the more extreme end of the
    spectrum, we could use a "byte-code" style representation, where
    the successor generator is just a long vector of ints combining
    information about node type with node payload.

    For example, we could represent different node types as follows,
    where BINARY_FORK etc. are symbolic constants for tagging node
    types:

    - binary fork: [BINARY_FORK, child_1, child_2]
    - multi-fork:  [MULTI_FORK, n, child_1, ..., child_n]
    - vector switch: [VECTOR_SWITCH, var_id, child_1, ..., child_k]
    - single switch: [SINGLE_SWITCH, var_id, value, child_index]
    - hash switch: [HASH_SWITCH, var_id, map_no]
      where map_no is an index into a separate vector of hash maps
      (represented out of band)
    - single leaf: [SINGLE_LEAF, op_id]
    - vector leaf: [VECTOR_LEAF, n, op_id_1, ..., op_id_n]

    We could compact this further by permitting to use operator IDs
    directly wherever child nodes are used, by using e.g. negative
    numbers for operatorIDs and positive numbers for node IDs,
    obviating the need for SINGLE_LEAF. This would also make
    VECTOR_LEAF redundant, as MULTI_FORK could be used instead.

    Further, if the other symbolic constants are negative numbers,
    we could represent forks just as [n, child_1, ..., child_n] without
    symbolic constant at the start, unifying binary and multi-forks.

    To completely unify the representation, not needing hash values
    out of band, we might consider switches of the form [SWITCH, k,
    var_id, value_1, child_1, ..., value_k, child_k] that permit
    binary searching. This would only leave switch and fork nodes, and
    we could do away with the type tags by just using +k for one node
    type and -k for the other. (But it may be useful to leave the
    possibility of the current vector switches for very dense switch
    nodes, which could be used in the case where k equals the domain
    size of the variable in question.)

  - More modestly, we could stick with the current polymorphic code,
    but just use more types of nodes, such as switch nodes that stores
    a vector of (value, child) pairs to be scanned linearly or with
    binary search.

  - We can also try to optimize memory usage of the existing nodes
    further, e.g. by replacing vectors with something smaller, like a
    zero-terminated heap-allocated array.
*/

namespace successor_generator {

auto operator_affects_black_variable(const redblack::RBState &state, OperatorID op_id) -> bool {
	const auto &op = state.get_rb_state_registry().get_operators()[op_id.get_index()];
	return std::any_of(std::begin(op.get_black_effects()), std::end(op.get_black_effects()), [&state](const auto effect) {
		return std::all_of(std::begin(effect->conditions), std::end(effect->conditions), [&state](const auto &precondition) {
			return state.has_fact(precondition.var, precondition.val);
		}) && !state.has_fact(effect->var, effect->val);
	});
}

GeneratorForkBinary::GeneratorForkBinary(
    unique_ptr<GeneratorBase> generator1,
    unique_ptr<GeneratorBase> generator2)
    : generator1(move(generator1)),
      generator2(move(generator2)) {
    /* There is no reason to use a fork if only one of the generators exists.
       Use the existing generator directly if one of them exists or a nullptr
       otherwise. */
    assert(this->generator1);
    assert(this->generator2);
}

void GeneratorForkBinary::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
    generator1->generate_applicable_ops(state, applicable_ops);
    generator2->generate_applicable_ops(state, applicable_ops);
}

void GeneratorForkBinary::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
    generator1->generate_applicable_ops(state, applicable_ops);
    generator2->generate_applicable_ops(state, applicable_ops);
}

void GeneratorForkBinary::generate_applicable_ops(
	const redblack::RBState &state, std::vector<OperatorID> &applicable_ops, bool black_only) const {
	generator1->generate_applicable_ops(state, applicable_ops, black_only);
	generator2->generate_applicable_ops(state, applicable_ops, black_only);
}

GeneratorForkMulti::GeneratorForkMulti(vector<unique_ptr<GeneratorBase>> children)
    : children(move(children)) {
    /* Note that we permit 0-ary forks as a way to define empty
       successor generators (for tasks with no operators). It is
       the responsibility of the factory code to make sure they
       are not generated in other circumstances. */
    assert(this->children.empty() || this->children.size() >= 2);
}

void GeneratorForkMulti::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
    for (const auto &generator : children)
        generator->generate_applicable_ops(state, applicable_ops);
}

void GeneratorForkMulti::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
    for (const auto &generator : children)
        generator->generate_applicable_ops(state, applicable_ops);
}

void GeneratorForkMulti::generate_applicable_ops(
	const redblack::RBState &state, std::vector<OperatorID> &applicable_ops, bool black_only) const {
	for (const auto &generator : children)
		generator->generate_applicable_ops(state, applicable_ops, black_only);
}

GeneratorSwitchVector::GeneratorSwitchVector(
    int switch_var_id, vector<unique_ptr<GeneratorBase>> &&generator_for_value)
    : switch_var_id(switch_var_id),
      generator_for_value(move(generator_for_value)) {
}

void GeneratorSwitchVector::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
    int val = state[switch_var_id].get_value();
    const unique_ptr<GeneratorBase> &generator_for_val = generator_for_value[val];
    if (generator_for_val) {
        generator_for_val->generate_applicable_ops(state, applicable_ops);
    }
}

void GeneratorSwitchVector::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
    int val = state[switch_var_id];
    const unique_ptr<GeneratorBase> &generator_for_val = generator_for_value[val];
    if (generator_for_val) {
        generator_for_val->generate_applicable_ops(state, applicable_ops);
    }
}

void GeneratorSwitchVector::generate_applicable_ops(
	const redblack::RBState &state, std::vector<OperatorID> &applicable_ops, bool black_only) const {
	auto recursion = [&state, &applicable_ops, black_only, this](int val) {
		const auto &generator_for_val = generator_for_value[val];
		if (generator_for_val)
			generator_for_val->generate_applicable_ops(state, applicable_ops, black_only);
	};
	if (state.get_painting().is_black_var(switch_var_id)) {
		recursion(state[switch_var_id]);
	} else {
		for (auto i = 0; i < g_root_task()->get_variable_domain_size(switch_var_id); ++i) {
			if (!state.has_fact(switch_var_id, i))
				continue;
			recursion(i);
		}
	}
}

GeneratorSwitchHash::GeneratorSwitchHash(
    int switch_var_id,
    unordered_map<int, unique_ptr<GeneratorBase>> &&generator_for_value)
    : switch_var_id(switch_var_id),
      generator_for_value(move(generator_for_value)) {
}

void GeneratorSwitchHash::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
    int val = state[switch_var_id].get_value();
    const auto &child = generator_for_value.find(val);
    if (child != generator_for_value.end()) {
        const unique_ptr<GeneratorBase> &generator_for_val = child->second;
        generator_for_val->generate_applicable_ops(state, applicable_ops);
    }
}

void GeneratorSwitchHash::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
    int val = state[switch_var_id];
    const auto &child = generator_for_value.find(val);
    if (child != generator_for_value.end()) {
        const unique_ptr<GeneratorBase> &generator_for_val = child->second;
        generator_for_val->generate_applicable_ops(state, applicable_ops);
    }
}

void GeneratorSwitchHash::generate_applicable_ops(
	const redblack::RBState &state, std::vector<OperatorID> &applicable_ops, bool black_only) const {
	auto recursion = [&state, &applicable_ops, black_only, this](int val) {
		const auto &child = generator_for_value.find(val);
		if (child != generator_for_value.end()) {
			const auto &generator_for_val = child->second;
			generator_for_val->generate_applicable_ops(state, applicable_ops, black_only);
		}
	};
	if (state.get_painting().is_black_var(switch_var_id)) {
		recursion(state[switch_var_id]);
	} else {
		for (auto i = 0; i < g_root_task()->get_variable_domain_size(switch_var_id); ++i) {
			if (!state.has_fact(switch_var_id, i))
				continue;
			recursion(i);
		}
	}
}

GeneratorSwitchSingle::GeneratorSwitchSingle(
    int switch_var_id, int value, unique_ptr<GeneratorBase> generator_for_value)
    : switch_var_id(switch_var_id),
      value(value),
      generator_for_value(move(generator_for_value)) {
}

void GeneratorSwitchSingle::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
    if (value == state[switch_var_id].get_value()) {
        generator_for_value->generate_applicable_ops(state, applicable_ops);
    }
}

void GeneratorSwitchSingle::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
    if (value == state[switch_var_id]) {
        generator_for_value->generate_applicable_ops(state, applicable_ops);
    }
}

void GeneratorSwitchSingle::generate_applicable_ops(
	const redblack::RBState &state, std::vector<OperatorID> &applicable_ops, bool black_only) const {
	if (state.has_fact(switch_var_id, value))
		generator_for_value->generate_applicable_ops(state, applicable_ops, black_only);
}

GeneratorLeafVector::GeneratorLeafVector(vector<OperatorID> &&applicable_operators)
    : applicable_operators(move(applicable_operators)) {
}

void GeneratorLeafVector::generate_applicable_ops(
    const State &, vector<OperatorID> &applicable_ops) const {
    /*
      In our experiments (issue688), a loop over push_back was faster
      here than doing this with a single insert call because the
      containers are typically very small. However, we have changed
      the container type from list to vector since then, so this might
      no longer apply.
    */
    for (OperatorID id : applicable_operators) {
        applicable_ops.push_back(id);
    }
}

void GeneratorLeafVector::generate_applicable_ops(
    const GlobalState &, vector<OperatorID> &applicable_ops) const {
    // See above for the reason for using push_back instead of insert.
    for (OperatorID id : applicable_operators) {
        applicable_ops.push_back(id);
    }
}

void GeneratorLeafVector::generate_applicable_ops(
	const redblack::RBState &state, std::vector<OperatorID> &applicable_ops, bool black_only) const {
	for (OperatorID id : applicable_operators)
		if (!black_only || operator_affects_black_variable(state, id))
			applicable_ops.push_back(id);
}

GeneratorLeafSingle::GeneratorLeafSingle(OperatorID applicable_operator)
    : applicable_operator(applicable_operator) {
}

void GeneratorLeafSingle::generate_applicable_ops(
    const State &, vector<OperatorID> &applicable_ops) const {
    applicable_ops.push_back(applicable_operator);
}

void GeneratorLeafSingle::generate_applicable_ops(
    const GlobalState &, vector<OperatorID> &applicable_ops) const {
    applicable_ops.push_back(applicable_operator);
}

void GeneratorLeafSingle::generate_applicable_ops(
	const redblack::RBState &state, vector<OperatorID> &applicable_ops, bool black_only) const {
	if (!black_only || operator_affects_black_variable(state, applicable_operator))
		applicable_ops.push_back(applicable_operator);
}
}
