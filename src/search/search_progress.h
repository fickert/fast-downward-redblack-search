#ifndef SEARCH_PROGRESS_H
#define SEARCH_PROGRESS_H

#include "evaluation_context.h"
#include "heuristic.h"

#include <unordered_map>
#include <iostream>

/*
  This class helps track search progress.

  It maintains a record of best heuristic values and can determine if
  an evaluated state has a better heuristic value in at least one of
  the heuristics than all previously seen ones.
*/

template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class SearchProgress {
    std::unordered_map<const Heuristic<StateType, OperatorType> *, int> best_heuristic_values;

    bool process_heuristic_value(const Heuristic<StateType, OperatorType> *heuristic, int h);

public:
    SearchProgress() = default;
    ~SearchProgress() = default;

    /*
      Call the following function after each state evaluation.

      It keeps track of the best heuristic value for each heuristic
      evaluated, returning true if at least one heuristic value is the
      best value seen for this heuristic so far. (This includes the
      case where the evaluation context includes a heuristic that has
      not been evaluated previously, e.g. after evaluating the initial
      state.)

      Prints one line of output for each new best heuristic value.
    */
    bool check_progress(const EvaluationContext<StateType, OperatorType> &eval_context);
};


template<class StateType, class OperatorType>
bool SearchProgress<StateType, OperatorType>::process_heuristic_value(const Heuristic<StateType, OperatorType> *heuristic, int h) {
    /*
      Handle one heuristic value:
      1. insert into or update best_heuristic_values if necessary
      2. return true if this is a new best heuristic value
         (includes case where we haven't seen this heuristic before)
    */
    auto insert_result = best_heuristic_values.insert(std::make_pair(heuristic, h));
    auto iter = insert_result.first;
    bool was_inserted = insert_result.second;
    if (was_inserted) {
        // We haven't seen this heuristic before.
        return true;
    } else {
        int &best_h = iter->second;
        if (h < best_h) {
            best_h = h;
            return true;
        }
    }
    return false;
}

template<class StateType, class OperatorType>
bool SearchProgress<StateType, OperatorType>::check_progress(const EvaluationContext<StateType, OperatorType> &eval_context) {
    bool progress = false;
    eval_context.get_cache().for_each_heuristic_value(
        [this, &progress](const Heuristic<StateType, OperatorType> *heur, const EvaluationResult &result) {
        int h = result.get_h_value();
        if (process_heuristic_value(heur, h)) {
            std::cout << "New best heuristic value for "
                 << heur->get_description() << ": " << h << std::endl;
            progress = true;
        }
    }
        );
    return progress;
}



#endif
