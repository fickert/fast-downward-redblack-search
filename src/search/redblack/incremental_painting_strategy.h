#ifndef REDBLACK_INCREMENTAL_PAINTING_STRATEGY_H
#define REDBLACK_INCREMENTAL_PAINTING_STRATEGY_H

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

#include "painting.h"
#include "../operator_id.h"
#include "../abstract_task.h"
#include "../globals.h"


namespace redblack {

class IncrementalPaintingStrategy {
protected:
	explicit IncrementalPaintingStrategy(const options::Options &opts);
	virtual ~IncrementalPaintingStrategy();

	const int num_black;

public:
	auto generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan, const std::vector<bool> *never_black_variables = nullptr) -> Painting;
	virtual auto generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan, const std::vector<FactPair> &goal_facts, const std::vector<bool> *never_black_variables = nullptr) -> Painting = 0;

	static void add_options_to_parser(options::OptionParser &parser);
};


class LeastConflictsPaintingStrategy : public IncrementalPaintingStrategy {
	const bool prefer_lvl;

	static auto get_variable_levels() -> std::vector<int>;

public:
	LeastConflictsPaintingStrategy(const options::Options &opts);
	~LeastConflictsPaintingStrategy() = default;

	auto generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan, const std::vector<FactPair> &goal_facts, const std::vector<bool> *never_black_variables) -> Painting override;
};


class RandomPaintingStrategy : public IncrementalPaintingStrategy {
	const std::shared_ptr<utils::RandomNumberGenerator> rng;

public:
	RandomPaintingStrategy(const options::Options &opts);
	~RandomPaintingStrategy() = default;

	auto generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan, const std::vector<FactPair> &goal_facts, const std::vector<bool> *never_black_variables) -> Painting override;
};

}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
