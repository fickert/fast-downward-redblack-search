#ifndef REDBLACK_STATE_SATURATION_H
#define REDBLACK_STATE_SATURATION_H

#include "state.h"
#include "../operator_id.h"
#include <vector>

namespace redblack {
namespace detail {
struct Counter {
	Counter(int num_preconditions) :
		effects(),
		num_preconditions(num_preconditions),
		value(0) {}

	struct Effect {
		Effect(const FactPair &fact, const OperatorID &supporter) :
			fact(fact), supporter(supporter) {}

		const FactPair fact;
		const OperatorID supporter;
	};

	std::vector<Effect> effects;
	const int num_preconditions;
	int value;
};

struct CondEffCounter : Counter {
	CondEffCounter(int num_preconditions,
	               const std::vector<std::vector<FactPair>> &negative_preconditions,
	               const std::vector<std::pair<FactPair, std::vector<FactPair>>> &condeff_preconditions) :
		Counter(num_preconditions),
		negative_preconditions(negative_preconditions),
		condeff_preconditions(condeff_preconditions) {}

	std::vector<std::vector<FactPair>> negative_preconditions;
	std::vector<std::pair<FactPair, std::vector<FactPair>>> condeff_preconditions;
};
}

class StateSaturation {
public:
	StateSaturation(const AbstractTask &task, const RBIntPacker &state_packer, const std::vector<RBOperator> &operators);
	virtual ~StateSaturation() = default;

	virtual auto saturate_state(PackedStateBin *buffer, bool store_best_supporters = false) -> std::vector<std::vector<OperatorID>> = 0;

protected:
	const AbstractTask &task;
	const RBIntPacker &state_packer;
	const std::vector<RBOperator> &operators;
};

template<bool support_conditional_effects>
class CounterBasedStateSaturation : public StateSaturation {
public:
	CounterBasedStateSaturation(const AbstractTask &task, const RBIntPacker &state_packer, const std::vector<RBOperator> &operators);
	~CounterBasedStateSaturation() = default;

	auto saturate_state(PackedStateBin *buffer, bool store_best_supporters) -> std::vector<std::vector<OperatorID>> override;

protected:
	using CounterType = std::conditional_t<support_conditional_effects, detail::CondEffCounter, detail::Counter>;

	std::vector<CounterType> counters;
	std::vector<std::vector<std::vector<std::size_t>>> precondition_of;
};
}

#endif
