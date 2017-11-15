#ifndef EVALUATORS_PREF_EVALUATOR_H
#define EVALUATORS_PREF_EVALUATOR_H

#include "../evaluator.h"

namespace pref_evaluator {
template<class StateType, class OperatorType>
class PrefEvaluator : public Evaluator<StateType, OperatorType> {
public:
    PrefEvaluator();
    virtual ~PrefEvaluator() override;

    virtual EvaluationResult compute_result(
        EvaluationContext<StateType, OperatorType> &eval_context) override;
    virtual void get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &) override {}
};

template<class StateType, class OperatorType>
PrefEvaluator<StateType, OperatorType>::PrefEvaluator() {
}

template<class StateType, class OperatorType>
PrefEvaluator<StateType, OperatorType>::~PrefEvaluator() {
}

template<class StateType, class OperatorType>
EvaluationResult PrefEvaluator<StateType, OperatorType>::compute_result(
    EvaluationContext<StateType, OperatorType> &eval_context) {
    EvaluationResult result;
    if (eval_context.is_preferred())
        result.set_h_value(0);
    else
        result.set_h_value(1);
    return result;
}
}

#endif
