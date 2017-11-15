#ifndef EVALUATORS_COMBINING_EVALUATOR_H
#define EVALUATORS_COMBINING_EVALUATOR_H

#include "../evaluator.h"
#include "../evaluation_context.h"

#include <set>
#include <vector>

namespace combining_evaluator {
/*
  CombiningEvaluator is the base class for SumEvaluator and
  MaxEvaluator, which captures the common aspects of their behaviour.
*/
template<class StateType, class OperatorType>
class CombiningEvaluator : public Evaluator<StateType, OperatorType> {
    std::vector<Evaluator<StateType, OperatorType> *> subevaluators;
    bool all_dead_ends_are_reliable;
protected:
    virtual int combine_values(const std::vector<int> &values) = 0;
public:
    explicit CombiningEvaluator(
        const std::vector<Evaluator<StateType, OperatorType> *> &subevaluators_);
    virtual ~CombiningEvaluator() override;

    /*
      Note: dead_ends_are_reliable() is a state-independent method, so
      it only returns true if all subevaluators report dead ends reliably.

      Note that we could get more fine-grained information when
      considering of reliability for a given evaluated state. For
      example, if we use h1 (unreliable) and h2 (reliable) and have a
      state where h1 is finite and h2 is infinite, then we can
      *reliably* mark the state as a dead end. There is currently no
      way to exploit such state-based information, and hence we do not
      compute it.
    */

    virtual bool dead_ends_are_reliable() const override;
    virtual EvaluationResult compute_result(
        EvaluationContext<StateType, OperatorType> &eval_context) override;
    virtual void get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &hset) override;
};


template<class StateType, class OperatorType>
CombiningEvaluator<StateType, OperatorType>::CombiningEvaluator(
    const std::vector<Evaluator<StateType, OperatorType> *> &subevaluators_)
    : subevaluators(subevaluators_) {
    all_dead_ends_are_reliable = true;
    for (const Evaluator<StateType, OperatorType> *subevaluator : subevaluators)
        if (!subevaluator->dead_ends_are_reliable())
            all_dead_ends_are_reliable = false;
}

template<class StateType, class OperatorType>
CombiningEvaluator<StateType, OperatorType>::~CombiningEvaluator() {
}

template<class StateType, class OperatorType>
bool CombiningEvaluator<StateType, OperatorType>::dead_ends_are_reliable() const {
    return all_dead_ends_are_reliable;
}

template<class StateType, class OperatorType>
EvaluationResult CombiningEvaluator<StateType, OperatorType>::compute_result(
    EvaluationContext<StateType, OperatorType> &eval_context) {
    // This marks no preferred operators.
    EvaluationResult result;
    std::vector<int> values;
    values.reserve(subevaluators.size());

    // Collect component values. Return infinity if any is infinite.
    for (Evaluator<StateType, OperatorType> *subevaluator : subevaluators) {
        int h_val = eval_context.get_heuristic_value_or_infinity(subevaluator);
        if (h_val == EvaluationResult::INFTY) {
            result.set_h_value(h_val);
            return result;
        } else {
            values.push_back(h_val);
        }
    }

    // If we arrived here, all subevaluator values are finite.
    result.set_h_value(combine_values(values));
    return result;
}

template<class StateType, class OperatorType>
void CombiningEvaluator<StateType, OperatorType>::get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &hset) {
    for (size_t i = 0; i < subevaluators.size(); ++i)
        subevaluators[i]->get_involved_heuristics(hset);
}

}

#endif
