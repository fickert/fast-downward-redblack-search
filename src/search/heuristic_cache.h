#ifndef HEURISTIC_CACHE_H
#define HEURISTIC_CACHE_H

#include "evaluation_result.h"
#include "global_state.h"
#include "heuristic.h"

#include <unordered_map>

template<class StateType, class OperatorType>
class Evaluator;

/*
  Store a state and evaluation results for this state.
*/
template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class HeuristicCache {
	using EvaluationResults = std::unordered_map<Evaluator<StateType, OperatorType> *, EvaluationResult>;

    EvaluationResults eval_results;
    StateType state;

public:
    explicit HeuristicCache(const StateType &state);
    ~HeuristicCache() = default;

    EvaluationResult &operator[](Evaluator<StateType, OperatorType> *heur);

    const StateType &get_state() const;

    template<class Callback>
    void for_each_heuristic_value(const Callback &callback) const {
        for (const auto &element : eval_results) {
            const Evaluator<StateType, OperatorType> *eval = element.first;
            const EvaluationResult &result = element.second;
            const Heuristic<StateType, OperatorType> *heuristic = dynamic_cast<const Heuristic<StateType, OperatorType> *>(eval);
            if (heuristic) {
                /* We want to consider only Heuristic instances, not other
                   Evaluator instances. */
                callback(heuristic, result);
            }
        }
    }
};


template<class StateType, class OperatorType>
HeuristicCache<StateType, OperatorType>::HeuristicCache(const StateType &state)
    : state(state) {
}

template<class StateType, class OperatorType>
EvaluationResult &HeuristicCache<StateType, OperatorType>::operator[](Evaluator<StateType, OperatorType> *heur) {
    return eval_results[heur];
}

template<class StateType, class OperatorType>
const StateType &HeuristicCache<StateType, OperatorType>::get_state() const {
    return state;
}


#endif
