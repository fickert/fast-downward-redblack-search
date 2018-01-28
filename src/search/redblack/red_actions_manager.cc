#include "red_actions_manager.h"

#include "operator.h"

namespace redblack {

RedActionsManager::RedActionsManager(const std::vector<RBOperator> &operators)
	: red_operators(operators.size()),
	  conditionally_red_operators() {
	for (auto i = 0u; i < operators.size(); ++i) {
		const auto &op = operators[i];
		if (op.get_red_effects().empty())
			continue;
		// 3 cases:
		// a) doesn't modify black variables ==> fine
		// b) conditionally modifies black variables ==> fine if the effect does nothing in the current state (add precondition)
		// c) always modifies black variables ==> ignore this operator
		auto changes_black_variable = false;
		auto preconditions = std::vector<FactPair>();
		for (const auto &precondition : op.get_black_preconditions())
			preconditions.emplace_back(precondition->var, precondition->val);
		std::sort(std::begin(preconditions), std::end(preconditions));
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
		std::sort(std::begin(preconditions) + op.get_black_preconditions().size(), std::end(preconditions));
		std::inplace_merge(std::begin(preconditions), std::begin(preconditions) + op.get_black_preconditions().size(), std::end(preconditions));
		assert(std::unique(std::begin(preconditions), std::end(preconditions)) == std::end(preconditions));

		if (preconditions.empty()) {
			red_operators[i] = true;
		} else {
			auto insertion_result = conditionally_red_operators.try_emplace(std::move(preconditions), operators.size());
			insertion_result.first->second[i] = true;
		}
	}
}

auto RedActionsManager::get_red_actions_for_state(const GlobalState &state) -> boost::dynamic_bitset<> {
	return get_red_actions_for_state(state.get_values());
}

auto RedActionsManager::get_red_actions_for_state(const std::vector<int> &state_values) -> boost::dynamic_bitset<> {
	auto result = red_operators;
	for (const auto &conditionally_red_operator : conditionally_red_operators) {
		assert(conditionally_red_operator.second.any());
		if (std::all_of(std::begin(conditionally_red_operator.first), std::end(conditionally_red_operator.first), [&state_values](const auto &precondition) { return state_values[precondition.var] == precondition.value; }))
			result |= conditionally_red_operator.second;
	}
	return result;
}

auto RedActionsManager::get_red_actions_for_state(const std::vector<boost::dynamic_bitset<>> &state) -> boost::dynamic_bitset<> {
	auto result = red_operators;
	for (const auto &conditionally_red_operator : conditionally_red_operators) {
		assert(conditionally_red_operator.second.any());
		if (std::all_of(std::begin(conditionally_red_operator.first), std::end(conditionally_red_operator.first), [&state](const auto &precondition) { return state[precondition.var][precondition.value]; }))
			result |= conditionally_red_operator.second;
	}
	return result;
}

}
