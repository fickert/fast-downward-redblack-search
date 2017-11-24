#include "state_registry.h"
#include "state.h"
#include "operator.h"
#include "util.h"

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

auto contains_mutex(const std::vector<FactPair> &facts) -> bool {
	for (std::size_t i = 0; i < facts.size(); ++i)
		for (std::size_t j = i + 1; j < facts.size(); ++j)
			if (are_mutex(facts[i], facts[j]))
				return true;
	return false;
}

auto simplify_condeff_preconditions(const std::vector<FactPair> &preconditions,
                                    std::vector<std::vector<FactPair>> negative_preconditions,
                                    std::vector<std::pair<FactPair, std::vector<FactPair>>> condeff_preconditions)
-> std::tuple<bool, std::vector<std::vector<FactPair>>, std::vector<std::pair<FactPair, std::vector<FactPair>>>> {
	auto change = true;
	while (change) {
		change = false;
		for (auto &additional_condition : condeff_preconditions) {
			assert(!additional_condition.second.empty());
			assert(!std::binary_search(std::begin(preconditions), std::end(preconditions), additional_condition.first));
			assert(std::none_of(std::begin(additional_condition.second), std::end(additional_condition.second), [&preconditions](const auto &precondition) {
				return std::binary_search(std::begin(preconditions), std::end(preconditions), precondition);
			}));
			if (std::any_of(std::begin(preconditions), std::end(preconditions), [&additional_condition](const auto &precondition) {
				return are_mutex(additional_condition.first, precondition);
			})) {
				// the effect can not occur in a state where the other preconditions are satisfied
				// ==> the effect will always change the variable value so we need to prevent the conditional effect from triggering (via the negative preconditions)
				negative_preconditions.emplace_back(std::move(additional_condition.second));
				additional_condition.second.clear();
				change = true;
			}
		}
		condeff_preconditions.erase(
			std::remove_if(std::begin(condeff_preconditions),
				std::end(condeff_preconditions),
				[](const auto &condition) { return condition.second.empty(); }),
			std::end(condeff_preconditions));
	}
	for (auto &negative_disjunctive_precondition : negative_preconditions) {
		negative_disjunctive_precondition.erase(
			std::remove_if(std::begin(negative_disjunctive_precondition), std::end(negative_disjunctive_precondition),
				[&preconditions](const auto &negative_precondition) { return std::binary_search(std::begin(preconditions), std::end(preconditions), negative_precondition); }),
			std::end(negative_disjunctive_precondition));
		if (negative_disjunctive_precondition.empty()) {
			// deleted all disjunctive alternatives
			return {false, {}, {}};
		}
	}
	return {true, negative_preconditions, condeff_preconditions};
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
	auto counter_for_preconditions = std::map<std::tuple<std::vector<FactPair>, std::vector<std::vector<FactPair>>, std::vector<std::pair<FactPair, std::vector<FactPair>>>>, std::size_t>();
	auto get_counter_pos_for_preconditions = [&counter_for_preconditions, this](const auto &preconditions, const auto &negative_preconditions, const auto &condeff_preconditions) {
		auto [pos, inserted] = counter_for_preconditions.insert({{preconditions, negative_preconditions, condeff_preconditions }, counters.size()});
		if (inserted) {
			for (const auto &precondition : preconditions)
				precondition_of[precondition.var][precondition.value].push_back(counters.size());
			counters.emplace_back(preconditions.size(), negative_preconditions, condeff_preconditions);
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
		// b) conditionally modifies black variables ==> need negative preconditions that prevent these conditional effects from triggering, or the effect does nothing in the current state (add precondition)
		// c) always modifies black variables ==> ignore this operator
		auto changes_black_variable = false;
		auto preconditions = std::vector<FactPair>();
		for (const auto &precondition : op.get_base_operator().get_preconditions())
			preconditions.emplace_back(precondition.var, precondition.val);
		auto negative_preconditions = std::vector<std::vector<FactPair>>();
		auto condeff_preconditions = std::vector<std::pair<FactPair, std::vector<FactPair>>>();
		for (const auto effect : op.get_black_effects()) {
			assert(std::none_of(std::begin(op.get_black_preconditions()), std::end(op.get_black_preconditions()), [effect](const auto precondition) {
				return precondition->var == effect->var && precondition->val == effect->val;
			}));
			assert(std::none_of(std::begin(effect->conditions), std::end(effect->conditions), [effect](const auto &condition) {
				return condition.var == effect->var && condition.val == effect->val;
			}));
			if (effect->conditions.empty()) {
				// no conditional effect
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
			} else {
				auto conditions = std::vector<FactPair>();
				auto condition_on_effect_variable = false;
				for (const auto &condition : effect->conditions) {
					conditions.emplace_back(condition.var, condition.val);
					if (condition.var == effect->var) {
						assert(condition.val != effect->val);
						condition_on_effect_variable = true;
					}
				}
				assert(!contains_mutex(conditions));
				if (condition_on_effect_variable)
					negative_preconditions.emplace_back(std::move(conditions));
				else
					condeff_preconditions.emplace_back(FactPair{effect->var, effect->val}, std::move(conditions));
			}
		}
		if (changes_black_variable)
			continue;
		std::sort(std::begin(preconditions), std::end(preconditions));
		assert(std::unique(std::begin(preconditions), std::end(preconditions)) == std::end(preconditions));

		// simplify black conditional effect preconditions
		auto negative_preconditions_reachable = false;
		std::tie(negative_preconditions_reachable, negative_preconditions, condeff_preconditions) =
			simplify_condeff_preconditions(preconditions, negative_preconditions, condeff_preconditions);
		if (!negative_preconditions_reachable)
			continue;

		// initialize counters
		// cache for the counter without additional conditional preconditions
		auto base_counter_pos = std::numeric_limits<std::size_t>::max();
		for (const auto effect : op.get_red_effects()) {
			if (effect->conditions.empty()) {
				if (base_counter_pos == std::numeric_limits<std::size_t>::max())
					base_counter_pos = get_counter_pos_for_preconditions(preconditions, negative_preconditions, condeff_preconditions);
				add_effect(counters[base_counter_pos], effect->var, effect->val, op);
			} else {
				auto this_effect_preconditions = std::vector<FactPair>();
				this_effect_preconditions.reserve(preconditions.size() + effect->conditions.size());
				this_effect_preconditions.insert(std::begin(this_effect_preconditions), std::begin(preconditions), std::end(preconditions));
				for (const auto &condition : effect->conditions)
					this_effect_preconditions.emplace_back(condition.var, condition.val);

				auto [this_effect_negative_preconditions_reachable, this_effect_negative_preconditions, this_effect_condeff_preconditions] =
					simplify_condeff_preconditions(this_effect_preconditions, negative_preconditions, condeff_preconditions);
				if (!this_effect_negative_preconditions_reachable)
					continue;

				const auto counter_pos = get_counter_pos_for_preconditions(this_effect_preconditions, this_effect_negative_preconditions, this_effect_condeff_preconditions);
				add_effect(counters[counter_pos], effect->var, effect->val, op);
			}
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

	auto triggered = std::vector<Counter *>();

	// reset counter values
	for (auto &counter : counters) {
		counter.value = counter.num_preconditions;
		if (!std::all_of(std::begin(counter.negative_preconditions), std::end(counter.negative_preconditions), [this, buffer](const auto &negative_disjunctive_precondition) {
			return std::any_of(std::begin(negative_disjunctive_precondition), std::end(negative_disjunctive_precondition), [this, buffer](const auto &precondition) {
				assert(painting->is_black_var(precondition.var));
				return rb_state_packer().get(buffer, precondition.var) != precondition.value;
			});
		}) || !std::all_of(std::begin(counter.condeff_preconditions), std::end(counter.condeff_preconditions), [this, buffer](const auto &condeff_precondition) {
			assert(painting->is_black_var(condeff_precondition.first.var));
			return rb_state_packer().get(buffer, condeff_precondition.first.var) == condeff_precondition.first.value
				|| std::any_of(std::begin(condeff_precondition.second), std::end(condeff_precondition.second), [this, buffer](const auto &precondition) {
				assert(painting->is_black_var(precondition.var));
				return rb_state_packer().get(buffer, precondition.var) != precondition.value;
			});
		}))
			// black conditional effects will change black variables, make counter unreachable
			++counter.value;
		if (counter.value == 0)
			triggered.push_back(&counter);
	}

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
			// for each effect, the supporting operator may not trigger any black effects (unless the black effect doesn't do anything on the current state)
			assert(std::all_of(std::begin(counter->effects), std::end(counter->effects), [this, buffer](const auto &effect) {
				const auto &black_effects = operators[effect.supporter.get_index()].get_black_effects();
				return std::all_of(std::begin(black_effects), std::end(black_effects), [this, buffer](const auto black_effect) {
					assert(painting->is_black_var(black_effect->var));
					return rb_state_packer().get(buffer, black_effect->var) == black_effect->val
						|| !std::all_of(std::begin(black_effect->conditions), std::end(black_effect->conditions), [this, buffer](const auto &condition) {
						assert(painting->is_black_var(condition.var));
						return rb_state_packer().get(buffer, condition.var) == condition.val;
					});
				});
			}));
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
