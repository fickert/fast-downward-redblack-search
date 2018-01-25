#include "state_registry.h"

#include "operator.h"
#include "state.h"
#include "state_saturation.h"

#include "../tasks/cost_adapted_task.h"
#include "../globals.h"

#include <map>

template<>
auto StateRegistryBase<redblack::RBState, redblack::RBOperator>::lookup_state(StateID) const -> redblack::RBState {
	assert(false && "implemented in child class");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

template<>
auto StateRegistryBase<redblack::RBState, redblack::RBOperator>::get_successor_state(const redblack::RBState &, const redblack::RBOperator &) -> redblack::RBState {
	assert(false && "implemented in child class");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}


namespace redblack {

auto RBStateRegistry::get_state_saturation(const AbstractTask &task, const RBIntPacker &state_packer, const std::vector<RBOperator> &operators) -> std::unique_ptr<StateSaturation> {
	if (has_conditional_effects())
		return std::make_unique<CounterBasedStateSaturation<true>>(task, state_packer, operators);
	return std::make_unique<CounterBasedStateSaturation<false>>(task, state_packer, operators);
}

auto RBStateRegistry::construct_redblack_operators(const Painting &painting) -> std::vector<RBOperator> {
	auto rb_operators = std::vector<RBOperator>();
	rb_operators.reserve(g_operators.size());
	for (const auto &op : g_operators) {
		rb_operators.emplace_back(op);
		rb_operators.back().apply_painting(painting);
	}
	return rb_operators;
}

RBStateRegistry::RBStateRegistry(const AbstractTask &task, const RBIntPacker &state_packer,
	                             AxiomEvaluator &axiom_evaluator, std::vector<int> &&initial_state_data,
	                             PackedStateBin *rb_initial_state_data)
	: StateRegistryBase<RBState, RBOperator>(task, state_packer, axiom_evaluator, std::move(initial_state_data)),
	  painting(&state_packer.get_painting()),
	  operators(construct_redblack_operators(*painting)),
	  initial_state_best_supporters(),
	  state_saturation(get_state_saturation(task, state_packer, this->operators)) {
	if (rb_initial_state_data) {
		// TODO: make sure the passed initial state data matches the painting
		state_data_pool.push_back(rb_initial_state_data);
		StateID id = insert_id_or_pop_state();
		cached_initial_state = new RBState(state_data_pool[id.value], *this, id, *painting, rb_state_packer());
	}
}

RBStateRegistry::RBStateRegistry(const AbstractTask &task, const RBIntPacker &state_packer,
	                             AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data,
	                             PackedStateBin *rb_initial_state_data)
	: StateRegistryBase<RBState, RBOperator>(task, state_packer, axiom_evaluator, initial_state_data),
	  painting(&state_packer.get_painting()),
	  operators(construct_redblack_operators(*painting)),
	  initial_state_best_supporters(),
	  state_saturation(get_state_saturation(task, state_packer, this->operators)) {
	if (rb_initial_state_data) {
		// TODO: make sure the passed initial state data matches the painting
		state_data_pool.push_back(rb_initial_state_data);
		StateID id = insert_id_or_pop_state();
		cached_initial_state = new RBState(state_data_pool[id.value], *this, id, *painting, rb_state_packer());
	}
}

RBStateRegistry::~RBStateRegistry() {}

auto state_buffer_sanity_check(const PackedStateBin *buffer, const RBIntPacker &int_packer) -> bool {
	for (auto var = 0; var < g_root_task()->get_num_variables(); ++var) {
		if (int_packer.get_painting().is_black_var(var)) {
			if (!(int_packer.get(buffer, var) >= 0))
				return false;
			if (!(int_packer.get(buffer, var) < g_root_task()->get_variable_domain_size(var)))
				return false;
		} else {
			auto at_least_one_red = false;
			for (auto val = 0; val < g_root_task()->get_variable_domain_size(var); ++val) {
				if (int_packer.get_bit(buffer, var, val)) {
					at_least_one_red = true;
					break;
				}
			}
			if (!at_least_one_red)
				return false;
		}
	}
	return true;
}

void RBStateRegistry::build_unsaturated_successor(const RBState &predecessor, const RBOperator &op, PackedStateBin *buffer) const {
	auto effect_does_fire = [](const auto &effect, const auto &state) {
		return std::all_of(std::begin(effect.conditions), std::end(effect.conditions), [&state](const auto &condition) {
			return state.has_fact(condition.var, condition.val);
		});
	};
	assert(predecessor.get_painting() == rb_state_packer().get_painting());
	assert(state_buffer_sanity_check(predecessor.get_packed_buffer(), rb_state_packer()));
	for (const auto &effect : op.get_base_operator().get_effects()) {
		if (effect_does_fire(effect, predecessor)) {
			assert(effect.val < g_root_task()->get_variable_domain_size(effect.var));
			if (painting->is_black_var(effect.var)) {
				rb_state_packer().set(buffer, effect.var, effect.val);
				assert(rb_state_packer().get(buffer, effect.var) == effect.val);
			} else {
				rb_state_packer().set_bit(buffer, effect.var, effect.val);
				assert(rb_state_packer().get_bit(buffer, effect.var, effect.val));
			}
		}
	}
	assert(state_buffer_sanity_check(buffer, rb_state_packer()));
}

auto RBStateRegistry::get_successor_state(const RBState &predecessor, const RBOperator &op, bool get_best_supporters) -> std::pair<RBState, std::vector<std::vector<OperatorID>>> {
	assert(op.is_applicable(predecessor));
	assert(!op.get_base_operator().is_axiom());
	assert(op.is_black());
	state_data_pool.push_back(predecessor.get_packed_buffer());
	PackedStateBin *buffer = state_data_pool[state_data_pool.size() - 1];
	build_unsaturated_successor(predecessor, op, buffer);
	assert(state_buffer_sanity_check(buffer, rb_state_packer()));
	auto supporters = state_saturation->saturate_state(buffer, get_best_supporters);
	assert(state_buffer_sanity_check(buffer, rb_state_packer()));
	axiom_evaluator.evaluate(buffer, state_packer);
	auto id = insert_id_or_pop_state();
	assert(static_cast<int>(lookup_state(id).get_painting().get_painting().size()) == g_root_task()->get_num_variables());
	return {lookup_state(id), supporters};
}

auto RBStateRegistry::get_successor_state(const RBState &predecessor, const RBOperator &op) -> RBState {
	return get_successor_state(predecessor, op, false).first;
}

auto RBStateRegistry::get_successor_state_and_best_supporters(const RBState &predecessor, const RBOperator &op) -> std::pair<RBState, std::vector<std::vector<OperatorID>>> {
	return get_successor_state(predecessor, op, true);
}

void RBStateRegistry::populate_buffer(PackedStateBin *buffer, const std::vector<int> &values) const {
	std::fill_n(buffer, get_bins_per_state(), 0);
	for (size_t i = 0; i < values.size(); ++i) {
		if (painting->is_red_var(i)) {
			rb_state_packer().init_zero(buffer, i);
			rb_state_packer().set_bit(buffer, i, values[i]);
		} else {
			rb_state_packer().set(buffer, i, values[i]);
		}
	}
}

void RBStateRegistry::populate_buffer(PackedStateBin *buffer, const std::vector<boost::dynamic_bitset<>> &values) const {
	std::fill_n(buffer, get_bins_per_state(), 0);
	for (size_t i = 0; i < values.size(); ++i) {
		assert(values[i].any());
		if (painting->is_red_var(i)) {
			rb_state_packer().init_zero(buffer, i);
			for (auto pos = values[i].find_first(); pos != values[i].npos; pos = values[i].find_next(pos))
				rb_state_packer().set_bit(buffer, i, pos);
		} else {
			assert(values[i].count() == 1);
			rb_state_packer().set(buffer, i, values[i].find_first());
		}
	}
}

auto RBStateRegistry::get_state(const std::vector<int> &values) -> RBState {
	return get_state(values, false).first;
}

auto RBStateRegistry::get_state(const std::vector<boost::dynamic_bitset<>> &values) -> RBState {
	return get_state(values, false).first;
}

auto RBStateRegistry::get_state_and_best_supporters(const std::vector<int> &values) -> std::pair<RBState, std::vector<std::vector<OperatorID>>> {
	return get_state(values, true);
}

auto RBStateRegistry::get_state_and_best_supporters(const std::vector<boost::dynamic_bitset<>> &values) -> std::pair<RBState, std::vector<std::vector<OperatorID>>> {
	return get_state(values, true);
}

auto RBStateRegistry::lookup_state(StateID id) const -> RBState {
	return RBState(state_data_pool[id.value], *this, id, *painting, rb_state_packer());
}

auto RBStateRegistry::get_initial_state() -> const RBState & {
	if (!cached_initial_state) {
		auto [initial_state, initial_state_best_supporters_tmp] = get_state_and_best_supporters(initial_state_data);
		cached_initial_state = new RBState(std::move(initial_state));
		initial_state_best_supporters = std::move(initial_state_best_supporters_tmp);
	}
	return *cached_initial_state;
}

auto RBStateRegistry::get_best_supporters_for_successor(const RBState &predecessor, const RBOperator &op) const -> std::vector<std::vector<OperatorID>> {
	auto tmp = new PackedStateBin[get_bins_per_state()];
	std::copy(predecessor.get_packed_buffer(), predecessor.get_packed_buffer() + get_bins_per_state(), tmp);
	build_unsaturated_successor(predecessor, op, tmp);
	assert(state_buffer_sanity_check(tmp, rb_state_packer()));
	auto best_supporters = state_saturation->saturate_state(tmp, true);
	assert(state_buffer_sanity_check(tmp, rb_state_packer()));
	delete[] tmp;
	return best_supporters;
}

}
