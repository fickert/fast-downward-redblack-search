#include "state_registry.h"
#include "state.h"
#include "operator.h"
#include "util.h"

#include "../tasks/cost_adapted_task.h"
#include "../globals.h"

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

auto contains_mutex(const std::vector<FactPair> &facts) -> bool {
	for (std::size_t i = 0; i < facts.size(); ++i)
		for (std::size_t j = i + 1; j < facts.size(); ++j)
			if (are_mutex(facts[i], facts[j]))
				return true;
	return false;
}

RBStateRegistry::RBStateRegistry(const AbstractTask &task, const RBIntPacker &state_packer,
	                             AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data,
	                             const std::vector<RBOperator> &operators, PackedStateBin *rb_initial_state_data)
	: StateRegistryBase<RBState, RBOperator>(task, state_packer, axiom_evaluator, initial_state_data),
	  painting(&state_packer.get_painting()),
	  initial_state_best_supporters(),
	  cached_best_supporters(task.get_num_variables()),
	  operators(operators),
	  counters(),
	  precondition_of(task.get_num_variables()) {
	if (rb_initial_state_data) {
		// TODO: make sure the passed initial state data matches the painting
		state_data_pool.push_back(rb_initial_state_data);
		StateID id = insert_id_or_pop_state();
		cached_initial_state = new RBState(state_data_pool[id.value], *this, id, *painting, rb_state_packer());
	}
	// initialize counters
	assert(!any_conditional_effect_condition_is_red(*painting));
	auto counter_for_preconditions = std::unordered_map<std::vector<FactPair>, std::size_t>();
	auto get_counter_pos_for_preconditions = [&counter_for_preconditions, this](const auto &preconditions) {
		auto [pos, inserted] = counter_for_preconditions.insert({preconditions, counters.size()});
		if (inserted) {
			for (const auto &precondition : preconditions)
				precondition_of[precondition.var][precondition.value].push_back(counters.size());
			counters.emplace_back(preconditions.size());
		}
		return pos->second;
	};
	auto add_effect = [](auto &counter, auto var, auto val, const auto &op) {
		counter.effects.emplace_back(FactPair{var, val}, op.get_id());
	};
	for (auto var = 0; var < task.get_num_variables(); ++var)
		precondition_of[var].resize(task.get_variable_domain_size(var));
	for (const auto &op : operators) {
		if (op.get_red_effects().empty())
			continue;
		// 3 cases:
		// a) doesn't modify black variables ==> fine
		// b) conditionally modifies black variables ==> need negative preconditions that prevent these conditional effects from triggering
		// c) always modifies black variables ==> ignore this operator
		auto changes_black_variable = false;
		auto preconditions = std::vector<FactPair>();
		for (const auto &precondition : op.get_base_operator().get_preconditions())
			preconditions.emplace_back(precondition.var, precondition.val);
		std::sort(std::begin(preconditions), std::end(preconditions));
		auto black_conditional_effects_preconditions = std::vector<std::pair<FactPair, std::vector<FactPair>>>();
		for (const auto effect : op.get_black_effects()) {
			assert(std::none_of(std::begin(op.get_black_preconditions()), std::end(op.get_black_preconditions()), [effect](const auto precondition) {
				return precondition->var == effect->var && precondition->val == effect->val;
			}));
			assert(effect->conditions.empty());

			if (std::any_of(std::begin(op.get_black_preconditions()), std::end(op.get_black_preconditions()), [effect](const auto precondition) {
				return precondition->var == effect->var;
			})) {
				// there is a precondition on the same variable, so the black variable would always change when applying this action
				changes_black_variable = true;
				break;
			} else {
				// the variable may not change if the effect is already contained in the state
				preconditions.emplace_back(effect->var, effect->val);
			}
		}
		if (changes_black_variable)
			continue;
		std::sort(std::begin(preconditions) + op.get_base_operator().get_preconditions().size(), std::end(preconditions));
		std::inplace_merge(std::begin(preconditions), std::begin(preconditions) + op.get_base_operator().get_preconditions().size(), std::end(preconditions));
		assert(std::unique(std::begin(preconditions), std::end(preconditions)) == std::end(preconditions));
		
		// cache for the counter without additional conditional preconditions
		const auto counter_pos = get_counter_pos_for_preconditions(preconditions);
		for (const auto effect : op.get_red_effects()) {
			assert(effect->conditions.empty());
			add_effect(counters[counter_pos], effect->var, effect->val, op);
		}
	}
	counters.shrink_to_fit();
	for (auto var = 0; var < task.get_num_variables(); ++var)
		for (auto val = 0; val < task.get_variable_domain_size(var); ++val)
			precondition_of[var][val].shrink_to_fit();
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

void RBStateRegistry::saturate_state(PackedStateBin *buffer, bool store_best_supporters) const {
	assert(state_buffer_sanity_check(buffer, rb_state_packer()));

	if (store_best_supporters)
		for (auto var = 0; var < task.get_num_variables(); ++var)
			cached_best_supporters[var].assign(task.get_variable_domain_size(var), OperatorID(-1));

	// no need to iterate over all facts if there are no counters anyway (e.g. if all variables are black)
	if (counters.empty())
		return;

	// reset counter values
	for (auto &counter : counters)
		counter.value = counter.num_preconditions;

	auto triggered = std::vector<Counter *>();
	auto update_counters = [&triggered, this](auto var, auto val) {
		for (auto counter_pos : precondition_of[var][val])
			if (--counters[counter_pos].value == 0)
				triggered.push_back(&counters[counter_pos]);
	};

	for (auto var = 0; var < task.get_num_variables(); ++var) {
		if (painting->is_black_var(var)) {
			update_counters(var, rb_state_packer().get(buffer, var));
		} else {
			for (auto val = 0; val < task.get_variable_domain_size(var); ++val) {
				if (rb_state_packer().get_bit(buffer, var, val))
					update_counters(var, val);
			}
		}
	}

	while (!triggered.empty()) {
		auto next_triggered = std::vector<Counter *>();
		for (const auto counter : triggered) {
			assert(counter->value == 0);
			for (const auto &effect : counter->effects) {
				assert(painting->is_red_var(effect.fact.var));
				if (!rb_state_packer().get_bit(buffer, effect.fact.var, effect.fact.value)) {
					rb_state_packer().set_bit(buffer, effect.fact.var, effect.fact.value);
					if (store_best_supporters)
						cached_best_supporters[effect.fact.var][effect.fact.value] = effect.supporter;
					for (const auto counter_pos : precondition_of[effect.fact.var][effect.fact.value])
						if (--counters[counter_pos].value == 0)
							next_triggered.push_back(&counters[counter_pos]);
				}
			}
		}
		triggered = next_triggered;
	}
	assert(state_buffer_sanity_check(buffer, rb_state_packer()));
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

auto RBStateRegistry::get_successor_state(const RBState &predecessor, const RBOperator &op) -> RBState {
	assert(op.is_applicable(predecessor));
	assert(!op.get_base_operator().is_axiom());
	assert(op.is_black());
	state_data_pool.push_back(predecessor.get_packed_buffer());
	PackedStateBin *buffer = state_data_pool[state_data_pool.size() - 1];
	build_unsaturated_successor(predecessor, op, buffer);
	saturate_state(buffer);
	axiom_evaluator.evaluate(buffer, state_packer);
	auto id = insert_id_or_pop_state();
	return lookup_state(id);
}

auto RBStateRegistry::lookup_state(StateID id) const -> RBState {
	return RBState(state_data_pool[id.value], *this, id, *painting, rb_state_packer());
}

auto RBStateRegistry::get_initial_state() -> const RBState & {
	if (!cached_initial_state) {
		auto buffer = new PackedStateBin[get_bins_per_state()];
		// Avoid garbage values in half-full bins.
		std::fill_n(buffer, get_bins_per_state(), 0);
		for (size_t i = 0; i < initial_state_data.size(); ++i) {
			if (painting->is_red_var(i)) {
				rb_state_packer().init_zero(buffer, i);
				rb_state_packer().set_bit(buffer, i, initial_state_data[i]);
			} else {
				rb_state_packer().set(buffer, i, initial_state_data[i]);
			}
		}
		saturate_state(buffer, true);
		initial_state_best_supporters = get_stored_best_supporters();
		axiom_evaluator.evaluate(buffer, state_packer);
		state_data_pool.push_back(buffer);
		// buffer is copied by push_back
		delete[] buffer;
		StateID id = insert_id_or_pop_state();
		cached_initial_state = new RBState(lookup_state(id));
	}
	return *cached_initial_state;
}

auto RBStateRegistry::get_best_supporters_for_successor(const RBState &predecessor, const RBOperator &op) const -> const std::vector<std::vector<OperatorID>> & {
	auto tmp = new PackedStateBin[get_bins_per_state()];
	std::copy(predecessor.get_packed_buffer(), predecessor.get_packed_buffer() + get_bins_per_state(), tmp);
	build_unsaturated_successor(predecessor, op, tmp);
	saturate_state(tmp, true);
	delete[] tmp;
	return cached_best_supporters;
}

}
