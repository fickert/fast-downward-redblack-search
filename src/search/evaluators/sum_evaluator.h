#ifndef EVALUATORS_SUM_EVALUATOR_H
#define EVALUATORS_SUM_EVALUATOR_H

#include "combining_evaluator.h"

#include <vector>
#include <cassert>

namespace options {
class Options;
}

namespace sum_evaluator {
template<class StateType, class OperatorType>
class SumEvaluator : public combining_evaluator::CombiningEvaluator<StateType, OperatorType> {
protected:
    virtual int combine_values(const std::vector<int> &values) override;
public:
    explicit SumEvaluator(const options::Options &opts);
    explicit SumEvaluator(const std::vector<Evaluator<StateType, OperatorType> *> &evals);
    virtual ~SumEvaluator() override;
};

template<class StateType, class OperatorType>
SumEvaluator<StateType, OperatorType>::SumEvaluator(const options::Options &opts)
    : combining_evaluator::CombiningEvaluator<StateType, OperatorType>(opts.get_list<Evaluator<StateType, OperatorType> *>("evals")) {
}

template<class StateType, class OperatorType>
SumEvaluator<StateType, OperatorType>::SumEvaluator(const std::vector<Evaluator<StateType, OperatorType> *> &evals)
    : combining_evaluator::CombiningEvaluator<StateType, OperatorType>(evals) {
}

template<class StateType, class OperatorType>
SumEvaluator<StateType, OperatorType>::~SumEvaluator() {
}

template<class StateType, class OperatorType>
int SumEvaluator<StateType, OperatorType>::combine_values(const std::vector<int> &values) {
    int result = 0;
    for (int value : values) {
        assert(value >= 0);
        result += value;
        assert(result >= 0); // Check against overflow.
    }
    return result;
}

}

#endif
