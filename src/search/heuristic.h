#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "evaluation_context.h"
#include "evaluator.h"
#include "operator_id.h"
#include "per_state_information.h"
#include "task_proxy.h"

#include "options/options.h"
#include "task_utils/task_properties.h"

#include "algorithms/ordered_set.h"

#include <memory>
#include "options/option_parser.h"

#include "globals.h"

class GlobalOperator;
class GlobalState;
class TaskProxy;

template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class Heuristic : public Evaluator<StateType, OperatorType> {
    struct HEntry {
        /* dirty is conceptually a bool, but Visual C++ does not support
           packing ints and bools together in a bitfield. */
        int h : 31;
        unsigned int dirty : 1;

        HEntry(int h, bool dirty)
            : h(h), dirty(dirty) {
        }
    };
    static_assert(sizeof(HEntry) == 4, "HEntry has unexpected size.");

    std::string description;

    /*
      TODO: We might want to get rid of the preferred_operators
      attribute. It is currently only used by compute_result() and the
      methods it calls (compute_heuristic() directly, further methods
      indirectly), and we could e.g. change this by having
      compute_heuristic return an EvaluationResult object.

      If we do this, we should be mindful of the cost incurred by not
      being able to reuse the data structure from one iteration to the
      next, but this seems to be the only potential downside.
    */
    ordered_set::OrderedSet<OperatorID> preferred_operators;

protected:
    /*
      Cache for saving h values
      Before accessing this cache always make sure that the cache_h_values
      flag is set to true - as soon as the cache is accessed it will create
      entries for all existing states
    */
    PerStateInformation<HEntry, StateType, OperatorType> heuristic_cache;
    bool cache_h_values;

    // Hold a reference to the task implementation and pass it to objects that need it.
    const std::shared_ptr<AbstractTask> task;
    // Use task_proxy to access task information.
    TaskProxy task_proxy;

    enum {DEAD_END = -1, NO_VALUE = -2};

    // TODO: Call with State directly once all heuristics support it.
    virtual int compute_heuristic(const StateType &state) = 0;

    /*
      Usage note: Marking the same operator as preferred multiple times
      is OK -- it will only appear once in the list of preferred
      operators for this heuristic.
    */
    void set_preferred(const OperatorProxy &op);

    /* TODO: Make private and use State instead of GlobalState once all
       heuristics use the TaskProxy class. */
    State convert_global_state(const StateType &global_state) const;

public:
    explicit Heuristic(const options::Options &options);
    virtual ~Heuristic() override;

    virtual void notify_initial_state(const StateType & /*initial_state*/) {
    }

    virtual bool notify_state_transition(
        const StateType &parent_state, const OperatorType &op,
        const StateType &state);

    virtual void get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &hset) override {
        hset.insert(this);
    }

    static void add_options_to_parser(options::OptionParser &parser);
    static options::Options default_options();

    virtual EvaluationResult compute_result(
        EvaluationContext<StateType, OperatorType> &eval_context) override;

    std::string get_description() const;
};


template<class StateType, class OperatorType>
Heuristic<StateType, OperatorType>::Heuristic(const options::Options &opts)
    : description(opts.get_unparsed_config()),
      heuristic_cache(HEntry(NO_VALUE, true)), //TODO: is true really a good idea here?
      cache_h_values(opts.get<bool>("cache_estimates")),
      task(opts.get<std::shared_ptr<AbstractTask>>("transform")),
      task_proxy(*task) {}

template<class StateType, class OperatorType>
Heuristic<StateType, OperatorType>::~Heuristic() {}

template<class StateType, class OperatorType>
void Heuristic<StateType, OperatorType>::set_preferred(const OperatorProxy &op) {
    preferred_operators.insert(op.get_global_operator_id());
}

template<class StateType, class OperatorType>
bool Heuristic<StateType, OperatorType>::notify_state_transition(
    const StateType & /*parent_state*/,
    const OperatorType & /*op*/,
    const StateType & /*state*/) {
    return false;
}

template<class StateType, class OperatorType>
State Heuristic<StateType, OperatorType>::convert_global_state(const StateType &global_state) const {
    State state(*g_root_task(), global_state.get_values());
    return task_proxy.convert_ancestor_state(state);
}

template<class StateType, class OperatorType>
void Heuristic<StateType, OperatorType>::add_options_to_parser(options::OptionParser &parser) {
    parser.add_option<std::shared_ptr<AbstractTask>>(
        "transform",
        "Optional task transformation for the heuristic."
        " Currently, adapt_costs() and no_transform() are available.",
        "no_transform()");
    parser.add_option<bool>("cache_estimates", "cache heuristic estimates", "true");
}

// This solution to get default values seems nonoptimal.
// This is currently only used by the LAMA/FF synergy.
template<class StateType, class OperatorType>
options::Options Heuristic<StateType, OperatorType>::default_options() {
	options::Options opts = options::Options();
    opts.set<std::shared_ptr<AbstractTask>>("transform", g_root_task());
    opts.set<bool>("cache_estimates", false);
    return opts;
}

template<class StateType, class OperatorType>
EvaluationResult Heuristic<StateType, OperatorType>::compute_result(EvaluationContext<StateType, OperatorType> &eval_context) {
    EvaluationResult result;

    assert(preferred_operators.empty());

    const StateType &state = eval_context.get_state();
    bool calculate_preferred = eval_context.get_calculate_preferred();

    int heuristic = NO_VALUE;

    if (!calculate_preferred && cache_h_values &&
        heuristic_cache[state].h != NO_VALUE && !heuristic_cache[state].dirty) {
        heuristic = heuristic_cache[state].h;
        result.set_count_evaluation(false);
    } else {
        heuristic = compute_heuristic(state);
        if (cache_h_values) {
            heuristic_cache[state] = HEntry(heuristic, false);
        }
        result.set_count_evaluation(true);
    }

    assert(heuristic == DEAD_END || heuristic >= 0);

    if (heuristic == DEAD_END) {
        /*
          It is permissible to mark preferred operators for dead-end
          states (thus allowing a heuristic to mark them on-the-fly
          before knowing the final result), but if it turns out we
          have a dead end, we don't want to actually report any
          preferred operators.
        */
        preferred_operators.clear();
        heuristic = EvaluationResult::INFTY;
    }

#ifndef NDEBUG
    if constexpr (std::is_same_v<StateType, GlobalState>) {
        TaskProxy global_task_proxy = TaskProxy(*g_root_task());
        State global_state(*g_root_task(), state.get_values());
        OperatorsProxy global_operators = global_task_proxy.get_operators();
        if (heuristic != EvaluationResult::INFTY) {
            for (OperatorID op_id : preferred_operators)
                assert(task_properties::is_applicable(global_operators[op_id], global_state));
        }
    }
#endif

    result.set_h_value(heuristic);
    result.set_preferred_operators(preferred_operators.pop_as_vector());
    assert(preferred_operators.empty());

    return result;
}

template<class StateType, class OperatorType>
std::string Heuristic<StateType, OperatorType>::get_description() const {
    return description;
}


#endif
