#ifndef SEARCH_ENGINES_SEARCH_COMMON_H
#define SEARCH_ENGINES_SEARCH_COMMON_H

/*
  This module contains functions for creating open list factories used
  by the search engines.

  TODO: Think about where this code should ideally live. One possible
  ordering principle: avoid unnecessary plug-in dependencies.

  This code currently depends on multiple different open list
  implementations, so it would be good if it would not be a dependency
  of search engines that don't need all these open lists. Under this
  maxim, EHC should not depend on this file. A logical solution might
  be to move every creation function that only uses one particular
  open list type into the header for this open list type, and leave
  this file for cases that require more complex set-up and are common
  to eager and lazy search.
*/

#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../evaluators/weighted_evaluator.h"

#include "../open_lists/standard_scalar_open_list.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../open_lists/alternation_open_list.h"

#include <memory>

template<class StateType, class OperatorType>
class Evaluator;
template<class StateType, class OperatorType>
class OpenListFactory;

namespace options {
class Options;
}

namespace search_common {

namespace detail {
template<class StateType, class OperatorType>
using GEval = g_evaluator::GEvaluator<StateType, OperatorType>;

template<class StateType, class OperatorType>
using SumEval = sum_evaluator::SumEvaluator<StateType, OperatorType>;

template<class StateType, class OperatorType>
using WeightedEval = weighted_evaluator::WeightedEvaluator<StateType, OperatorType>;


template<class StateType, class OperatorType>
std::shared_ptr<OpenListFactory<StateType, OperatorType>> create_alternation_open_list_factory(
    const std::vector<std::shared_ptr<OpenListFactory<StateType, OperatorType>>> &subfactories, int boost) {
    Options options;
    options.set("sublists", subfactories);
    options.set("boost", boost);
    return std::make_shared<alternation_open_list::AlternationOpenListFactory<StateType, OperatorType>>(options);
}

/*
  Helper function for common code of create_greedy_open_list_factory
  and create_wastar_open_list_factory.
*/
template<class StateType, class OperatorType>
std::shared_ptr<OpenListFactory<StateType, OperatorType>> create_alternation_open_list_factory_aux(
    const std::vector<Evaluator<StateType, OperatorType> *> &evals,
    const std::vector<Heuristic<StateType, OperatorType> *> &preferred_heuristics,
    int boost) {
    if (evals.size() == 1 && preferred_heuristics.empty()) {
        return create_standard_scalar_open_list_factory(evals[0], false);
    } else {
        std::vector<std::shared_ptr<OpenListFactory<StateType, OperatorType>>> subfactories;
        for (Evaluator<StateType, OperatorType> *evaluator : evals) {
            subfactories.push_back(
                create_standard_scalar_open_list_factory(
                    evaluator, false));
            if (!preferred_heuristics.empty()) {
                subfactories.push_back(
                    create_standard_scalar_open_list_factory(
                        evaluator, true));
            }
        }
        return create_alternation_open_list_factory(subfactories, boost);
    }
}

/*
  Helper function for creating a single g + w * h evaluator
  for weighted A*-style search.

  If w = 1, we do not introduce an unnecessary weighted evaluator:
  we use g + h instead of g + 1 * h.

  If w = 0, we omit the heuristic altogether:
  we use g instead of g + 0 * h.
*/
template<class StateType, class OperatorType>
Evaluator<StateType, OperatorType> *create_wastar_eval(GEval<StateType, OperatorType> *g_eval, int w, Evaluator<StateType, OperatorType> *h_eval) {
    if (w == 0)
        return g_eval;
    Evaluator<StateType, OperatorType> *w_h_eval = nullptr;
    if (w == 1)
        w_h_eval = h_eval;
    else
        w_h_eval = new WeightedEval<StateType, OperatorType>(h_eval, w);
    return new SumEval<StateType, OperatorType>(vector<Evaluator<StateType, OperatorType> *>({g_eval, w_h_eval}));
}
}

/*
  Create a standard scalar open list factory with the given "eval" and
  "pref_only" options.
*/
template<class StateType, class OperatorType>
extern std::shared_ptr<OpenListFactory<StateType, OperatorType>> create_standard_scalar_open_list_factory(
    Evaluator<StateType, OperatorType> *eval, bool pref_only) {
    options::Options options;
    options.set("eval", eval);
    options.set("pref_only", pref_only);
    return std::make_shared<standard_scalar_open_list::StandardScalarOpenListFactory<StateType, OperatorType>>(options);
}

/*
  Create open list factory for the eager_greedy or lazy_greedy plugins.

  Uses "evals", "preferred" and "boost" from the passed-in Options
  object to construct an open list factory of the appropriate type.

  This is usually an alternation open list with:
  - one sublist for each heuristic, considering all successors
  - one sublist for each heuristic, considering only preferred successors

  However, the preferred-only open lists are omitted if no preferred
  operator heuristics are used, and if there would only be one sublist
  for the alternation open list, then that sublist is returned
  directly.
*/
template<class StateType, class OperatorType>
extern std::shared_ptr<OpenListFactory<StateType, OperatorType>> create_greedy_open_list_factory(
    const options::Options &opts) {
    return detail::create_alternation_open_list_factory_aux(
        opts.get_list<Evaluator<StateType, OperatorType> *>("evals"),
        opts.get_list<Heuristic<StateType, OperatorType> *>("preferred"),
        opts.get<int>("boost"));
}

/*
  Create open list factory for the lazy_wastar plugin.

  Uses "evals", "preferred", "boost" and "w" from the passed-in
  Options object to construct an open list factory of the appropriate
  type.

  This works essentially the same way as parse_greedy (see
  documentation there), except that the open lists use evalators based
  on g + w * h rather than using h directly.
*/
template<class StateType, class OperatorType>
std::shared_ptr<OpenListFactory<StateType, OperatorType>> create_wastar_open_list_factory(
    const options::Options &opts) {
    std::vector<Evaluator<StateType, OperatorType> *> base_evals =
        opts.get_list<Evaluator<StateType, OperatorType> *>("evals");
    int w = opts.get<int>("w");

    detail::GEval<StateType, OperatorType> *g_eval = new detail::GEval<StateType, OperatorType>();
    std::vector<Evaluator<StateType, OperatorType> *> f_evals;
    f_evals.reserve(base_evals.size());
    for (Evaluator<StateType, OperatorType> *eval : base_evals)
        f_evals.push_back(detail::create_wastar_eval<StateType, OperatorType>(g_eval, w, eval));

    return detail::create_alternation_open_list_factory_aux<StateType, OperatorType>(
        f_evals,
        opts.get_list<Heuristic<StateType, OperatorType> *>("preferred"),
        opts.get<int>("boost"));
}

/*
  Create open list factory and f_evaluator (used for displaying progress
  statistics) for A* search.

  The resulting open list factory produces a tie-breaking open list
  ordered primarily on g + h and secondarily on h. Uses "eval" from
  the passed-in Options object as the h evaluator.
*/
template<class StateType, class OperatorType>
extern std::pair<std::shared_ptr<OpenListFactory<StateType, OperatorType>>, Evaluator<StateType, OperatorType> *>
create_astar_open_list_factory_and_f_eval(const options::Options &opts) {
    detail::GEval<StateType, OperatorType> *g = new detail::GEval<StateType, OperatorType>();
    Evaluator<StateType, OperatorType> *h = opts.get<Evaluator<StateType, OperatorType> *>("eval");
    Evaluator<StateType, OperatorType> *f = new detail::SumEval<StateType, OperatorType>(std::vector<Evaluator<StateType, OperatorType> *>({g, h}));
    std::vector<Evaluator<StateType, OperatorType> *> evals = {f, h};

    Options options;
    options.set("evals", evals);
    options.set("pref_only", false);
    options.set("unsafe_pruning", false);
    std::shared_ptr<OpenListFactory<StateType, OperatorType>> open =
        std::make_shared<tiebreaking_open_list::TieBreakingOpenListFactory<StateType, OperatorType>>(options);
    return make_pair(open, f);
}
}

#endif
