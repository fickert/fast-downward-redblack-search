#ifndef EVALUATORS_G_EVALUATOR_H
#define EVALUATORS_G_EVALUATOR_H

#include "../evaluator.h"

class GlobalState;
class GlobalOperator;
template<class StateType, class OperatorType>
class Heuristic;

namespace g_evaluator {
template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class GEvaluator : public Evaluator<StateType, OperatorType> {
public:
    GEvaluator() = default;
    virtual ~GEvaluator() override = default;

    virtual EvaluationResult compute_result(
        EvaluationContext<StateType, OperatorType> &eval_context) override;

    virtual void get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &) override {}
};


template<class StateType, class OperatorType>
EvaluationResult GEvaluator<StateType, OperatorType>::compute_result(EvaluationContext<StateType, OperatorType> &eval_context) {
    EvaluationResult result;
    result.set_h_value(eval_context.get_g_value());
    return result;
}

}

#endif
