#ifndef REDBLACK_INCREMENTAL_PAINTING_STRATEGY_H
#define REDBLACK_INCREMENTAL_PAINTING_STRATEGY_H

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

#include "painting.h"
#include "../operator_id.h"

namespace redblack {

class IncrementalPaintingStrategy {
protected:
	explicit IncrementalPaintingStrategy(const options::Options &opts);
	virtual ~IncrementalPaintingStrategy();

public:
	virtual auto generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan) -> Painting = 0;
};


class LeastConflictsPaintingStrategy : public IncrementalPaintingStrategy {
	const bool prefer_lvl;
	const int num_black;

	static auto get_variable_levels() -> std::vector<int>;

public:
	LeastConflictsPaintingStrategy(const options::Options &opts);
	~LeastConflictsPaintingStrategy() = default;

	auto generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan) -> Painting override;
};


class RandomPaintingStrategy : public IncrementalPaintingStrategy {
	const int num_black;
	const std::shared_ptr<utils::RandomNumberGenerator> rng;

public:
	RandomPaintingStrategy(const options::Options &opts);
	~RandomPaintingStrategy() = default;

	auto generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan) -> Painting override;
};

}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
