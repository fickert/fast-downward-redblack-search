#ifndef EVALUATORS_WEIGHTED_EVALUATOR_H
#define EVALUATORS_WEIGHTED_EVALUATOR_H

#include "../evaluator.h"
#include "../options/options.h"

namespace options {
class Options;
}

namespace weighted_evaluator {
template<class StateType, class OperatorType>
class WeightedEvaluator : public Evaluator<StateType, OperatorType> {
    Evaluator<StateType, OperatorType> *evaluator;
    int w;

public:
    explicit WeightedEvaluator(const options::Options &opts);
    WeightedEvaluator(Evaluator<StateType, OperatorType> *eval, int weight);
    virtual ~WeightedEvaluator() override;

    virtual bool dead_ends_are_reliable() const override;
    virtual EvaluationResult compute_result(
        EvaluationContext<StateType, OperatorType> &eval_context) override;
    virtual void get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &hset) override;
};

template<class StateType, class OperatorType>
WeightedEvaluator<StateType, OperatorType>::WeightedEvaluator(const options::Options &opts)
    : evaluator(opts.get<Evaluator<StateType, OperatorType> *>("eval")),
      w(opts.get<int>("weight")) {
}

template<class StateType, class OperatorType>
WeightedEvaluator<StateType, OperatorType>::WeightedEvaluator(Evaluator<StateType, OperatorType> *eval, int weight)
    : evaluator(eval), w(weight) {
}

template<class StateType, class OperatorType>
WeightedEvaluator<StateType, OperatorType>::~WeightedEvaluator() {
}

template<class StateType, class OperatorType>
bool WeightedEvaluator<StateType, OperatorType>::dead_ends_are_reliable() const {
    return evaluator->dead_ends_are_reliable();
}

template<class StateType, class OperatorType>
EvaluationResult WeightedEvaluator<StateType, OperatorType>::compute_result(
    EvaluationContext<StateType, OperatorType> &eval_context) {
    // Note that this produces no preferred operators.
    EvaluationResult result;
    int h_val = eval_context.get_heuristic_value_or_infinity(evaluator);
    if (h_val != EvaluationResult::INFTY) {
        // TODO: Check for overflow?
        h_val *= w;
    }
    result.set_h_value(h_val);
    return result;
}

template<class StateType, class OperatorType>
void WeightedEvaluator<StateType, OperatorType>::get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &hset) {
    evaluator->get_involved_heuristics(hset);
}

}

#endif
