#include "successor_generator.h"

#include "successor_generator_factory.h"
#include "successor_generator_internals.h"

#include "../global_state.h"

using namespace std;

namespace successor_generator {
SuccessorGenerator::SuccessorGenerator(const TaskProxy &task_proxy)
    : root(SuccessorGeneratorFactory(task_proxy).create()) {
}

SuccessorGenerator::~SuccessorGenerator() = default;

void SuccessorGenerator::generate_applicable_ops(
	const State &state, vector<OperatorID> &applicable_ops) const {
	root->generate_applicable_ops(state, applicable_ops);
}

void SuccessorGenerator::generate_applicable_ops(
	const GlobalState &state, vector<OperatorID> &applicable_ops) const {
	root->generate_applicable_ops(state, applicable_ops);
}

void SuccessorGenerator::generate_applicable_ops(
	const redblack::RBState &state, vector<OperatorID> &applicable_ops, bool black_only) const {
	root->generate_applicable_ops(state, applicable_ops, black_only);
}

}
