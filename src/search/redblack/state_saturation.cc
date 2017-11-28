#include "state_saturation.h"

#include "operator.h"
#include "../globals.h"
#ifndef NDEBUG
#include "util.h"
#endif

#include <map>

namespace redblack {

StateSaturation::StateSaturation(const AbstractTask &task, const RBIntPacker &state_packer, const std::vector<RBOperator> &operators)
	: task(task), state_packer(state_packer), operators(operators) {}

auto contains_mutex(const std::vector<FactPair> &facts) -> bool {
	for (std::size_t i = 0; i < facts.size(); ++i)
		for (std::size_t j = i + 1; j < facts.size(); ++j)
			if (are_mutex(facts[i], facts[j]))
				return true;
	return false;
}

auto simplify_condeff_preconditions(std::vector<FactPair> preconditions,
                                    std::vector<std::vector<FactPair>> negative_preconditions,
                                    std::vector<std::pair<FactPair, std::vector<FactPair>>> condeff_preconditions)
-> std::tuple<bool, std::vector<FactPair>, std::vector<std::vector<FactPair>>, std::vector<std::pair<FactPair, std::vector<FactPair>>>> {
	auto change = true;
	while (change) {
		change = false;
		for (auto &condeff_precondition : condeff_preconditions) {
			assert(!condeff_precondition.second.empty());
			assert(!std::binary_search(std::begin(preconditions), std::end(preconditions), condeff_precondition.first));
			assert(std::none_of(std::begin(condeff_precondition.second), std::end(condeff_precondition.second), [&preconditions](const auto &precondition) {
				return std::binary_search(std::begin(preconditions), std::end(preconditions), precondition);
			}));
			if (std::any_of(std::begin(preconditions), std::end(preconditions), [&condeff_precondition](const auto &precondition) {
				return are_mutex(condeff_precondition.first, precondition);
			})) {
				// the effect can not occur in a state where the other preconditions are satisfied
				// ==> the effect will always change the variable value so we need to prevent the conditional effect from triggering (via the negative preconditions)
				negative_preconditions.emplace_back(std::move(condeff_precondition.second));
				condeff_precondition.second.clear();
				change = true;
			}
			if (!condeff_precondition.second.empty()) {
				condeff_precondition.second.erase(std::remove_if(std::begin(condeff_precondition.second), std::end(condeff_precondition.second), [&preconditions](const auto &negative_precondition) {
					return std::binary_search(std::begin(preconditions), std::end(preconditions), negative_precondition);
				}), std::end(condeff_precondition.second));
				if (condeff_precondition.second.empty()) {
					preconditions.emplace_back(std::move(condeff_precondition.first));
					std::inplace_merge(std::begin(preconditions), std::end(preconditions) - 1, std::end(preconditions));
					assert(std::unique(std::begin(preconditions), std::end(preconditions)) == std::end(preconditions));
					change = true;
				}
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
			return {false, {}, {}, {}};
		}
	}
	return {true, preconditions, negative_preconditions, condeff_preconditions};
}

template<>
CounterBasedStateSaturation<false>::CounterBasedStateSaturation(const AbstractTask &task, const RBIntPacker &state_packer, const std::vector<RBOperator> &operators)
	: StateSaturation(task, state_packer, operators), counters(), precondition_of(task.get_num_variables()) {
	// initialize counters
	assert(!any_conditional_effect_condition_is_red(state_packer.get_painting()));
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
		// b) conditionally modifies black variables ==> fine if the effect does nothing in the current state (add precondition)
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

		auto &counter = counters[get_counter_pos_for_preconditions(preconditions)];
		for (const auto effect : op.get_red_effects()) {
			assert(effect->conditions.empty());
			add_effect(counter, effect->var, effect->val, op);
		}
	}
	counters.shrink_to_fit();
	for (auto var = 0; var < task.get_num_variables(); ++var)
		for (auto val = 0; val < task.get_variable_domain_size(var); ++val)
			precondition_of[var][val].shrink_to_fit();
}

template<>
CounterBasedStateSaturation<true>::CounterBasedStateSaturation(const AbstractTask &task, const RBIntPacker &state_packer, const std::vector<RBOperator> &operators)
	: StateSaturation(task, state_packer, operators), counters(), precondition_of(task.get_num_variables()) {
	// initialize counters
	assert(!any_conditional_effect_condition_is_red(state_packer.get_painting()));
	auto counter_for_preconditions = std::map<std::tuple<std::vector<FactPair>, std::vector<std::vector<FactPair>>, std::vector<std::pair<FactPair, std::vector<FactPair>>>>, std::size_t>();
	auto get_counter_pos_for_preconditions = [&counter_for_preconditions, this](const auto &preconditions, const auto &negative_preconditions, const auto &condeff_preconditions) {
		auto[pos, inserted] = counter_for_preconditions.insert({{preconditions, negative_preconditions, condeff_preconditions}, counters.size()});
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
					condeff_preconditions.emplace_back(FactPair{ effect->var, effect->val }, std::move(conditions));
			}
		}
		if (changes_black_variable)
			continue;
		std::sort(std::begin(preconditions), std::end(preconditions));
		assert(std::unique(std::begin(preconditions), std::end(preconditions)) == std::end(preconditions));

		if (!negative_preconditions.empty() || !condeff_preconditions.empty()) {
			// simplify black conditional effect preconditions
			auto negative_preconditions_reachable = false;
			std::tie(negative_preconditions_reachable, preconditions, negative_preconditions, condeff_preconditions) =
				simplify_condeff_preconditions(preconditions, negative_preconditions, condeff_preconditions);
			if (!negative_preconditions_reachable)
				continue;
		}

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

				auto[this_effect_negative_preconditions_reachable, this_effect_simplified_preconditions, this_effect_negative_preconditions, this_effect_condeff_preconditions] =
					simplify_condeff_preconditions(this_effect_preconditions, negative_preconditions, condeff_preconditions);
				if (!this_effect_negative_preconditions_reachable)
					continue;

				const auto counter_pos = get_counter_pos_for_preconditions(this_effect_simplified_preconditions, this_effect_negative_preconditions, this_effect_condeff_preconditions);
				add_effect(counters[counter_pos], effect->var, effect->val, op);
			}
		}
	}
	counters.shrink_to_fit();
	for (auto var = 0; var < task.get_num_variables(); ++var)
		for (auto val = 0; val < task.get_variable_domain_size(var); ++val)
			precondition_of[var][val].shrink_to_fit();
}

template<bool support_conditional_effects>
auto CounterBasedStateSaturation<support_conditional_effects>::saturate_state(PackedStateBin *buffer, bool store_best_supporters) -> std::vector<std::vector<OperatorID>> {
	auto best_supporters = std::vector<std::vector<OperatorID>>();
	if (store_best_supporters) {
		best_supporters.resize(task.get_num_variables());
		for (auto var = 0; var < task.get_num_variables(); ++var)
			best_supporters[var].assign(task.get_variable_domain_size(var), OperatorID(-1));
	}

	// no need to iterate over all facts if there are no counters anyway (e.g. if all variables are black)
	if (counters.empty())
		return best_supporters;

	auto triggered = std::vector<CounterType *>();

	// reset counter values
	for (auto &counter : counters) {
		counter.value = counter.num_preconditions;
		if constexpr(support_conditional_effects) {
			if (!std::all_of(std::begin(counter.negative_preconditions), std::end(counter.negative_preconditions), [this, buffer](const auto &negative_disjunctive_precondition) {
				return std::any_of(std::begin(negative_disjunctive_precondition), std::end(negative_disjunctive_precondition), [this, buffer](const auto &precondition) {
					assert(state_packer.get_painting().is_black_var(precondition.var));
					return state_packer.get(buffer, precondition.var) != precondition.value;
				});
			}) || !std::all_of(std::begin(counter.condeff_preconditions), std::end(counter.condeff_preconditions), [this, buffer](const auto &condeff_precondition) {
				assert(state_packer.get_painting().is_black_var(condeff_precondition.first.var));
				return state_packer.get(buffer, condeff_precondition.first.var) == condeff_precondition.first.value
					|| std::any_of(std::begin(condeff_precondition.second), std::end(condeff_precondition.second), [this, buffer](const auto &precondition) {
					assert(state_packer.get_painting().is_black_var(precondition.var));
					return state_packer.get(buffer, precondition.var) != precondition.value;
				});
			}))
				// black conditional effects will change black variables, make counter unreachable
				++counter.value;
		}
		if (counter.value == 0)
			triggered.push_back(&counter);
	}

	auto update_counters = [&triggered, this](auto var, auto val) {
		for (auto counter_pos : precondition_of[var][val])
			if (--counters[counter_pos].value == 0)
				triggered.push_back(&counters[counter_pos]);
	};

	for (auto var = 0; var < task.get_num_variables(); ++var) {
		if (state_packer.get_painting().is_black_var(var)) {
			update_counters(var, state_packer.get(buffer, var));
		} else {
			for (auto val = 0; val < task.get_variable_domain_size(var); ++val) {
				if (state_packer.get_bit(buffer, var, val))
					update_counters(var, val);
			}
		}
	}

	while (!triggered.empty()) {
		auto next_triggered = std::vector<CounterType *>();
		for (const auto counter : triggered) {
			assert(counter->value == 0);
			if constexpr(support_conditional_effects) {
				// for each effect, the supporting operator may not trigger any black effects (unless the black effect doesn't do anything on the current state)
				assert(std::all_of(std::begin(counter->effects), std::end(counter->effects), [this, buffer](const auto &effect) {
					const auto &black_effects = operators[effect.supporter.get_index()].get_black_effects();
					return std::all_of(std::begin(black_effects), std::end(black_effects), [this, buffer](const auto black_effect) {
						assert(state_packer.get_painting().is_black_var(black_effect->var));
						return state_packer.get(buffer, black_effect->var) == black_effect->val
							|| !std::all_of(std::begin(black_effect->conditions), std::end(black_effect->conditions), [this, buffer](const auto &condition) {
							assert(state_packer.get_painting().is_black_var(condition.var));
							return state_packer.get(buffer, condition.var) == condition.val;
						});
					});
				}));
			}
			for (const auto &effect : counter->effects) {
				assert(state_packer.get_painting().is_red_var(effect.fact.var));
				if (!state_packer.get_bit(buffer, effect.fact.var, effect.fact.value)) {
					state_packer.set_bit(buffer, effect.fact.var, effect.fact.value);
					if (store_best_supporters)
						best_supporters[effect.fact.var][effect.fact.value] = effect.supporter;
					for (const auto counter_pos : precondition_of[effect.fact.var][effect.fact.value])
						if (--counters[counter_pos].value == 0)
							next_triggered.push_back(&counters[counter_pos]);
				}
			}
		}
		triggered = next_triggered;
	}

	return best_supporters;
}

}
