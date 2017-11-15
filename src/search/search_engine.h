#ifndef SEARCH_ENGINE_H
#define SEARCH_ENGINE_H

#include "global_operator.h"
#include "operator_cost.h"
#include "search_progress.h"
#include "search_space.h"
#include "search_statistics.h"
#include "state_registry.h"

#include <vector>
#include "utils/countdown_timer.h"
#include "options/option_parser.h"
#include "utils/rng_options.h"
#include "options/plugin.h"

namespace redblack {
class IncrementalRedBlackSearch;
}

class PruningMethod;
template<class StateType, class OperatorType>
class Heuristic;

namespace options {
class OptionParser;
class Options;
}

namespace ordered_set {
template<typename T>
class OrderedSet;
}

enum SearchStatus {IN_PROGRESS, TIMEOUT, FAILED, SOLVED};

template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class SearchEngine {
public:
    typedef std::vector<const OperatorType *> Plan;
	friend class redblack::IncrementalRedBlackSearch;
private:
    SearchStatus status;
    bool solution_found;
    Plan plan;
protected:
    SearchEngine(const options::Options &opts, std::unique_ptr<StateRegistryBase<StateType, OperatorType>> state_registry);

    std::unique_ptr<StateRegistryBase<StateType, OperatorType>> state_registry;
    SearchSpace<StateType, OperatorType> search_space;
    SearchProgress<StateType, OperatorType> search_progress;
    SearchStatistics statistics;
    int bound;
    OperatorCost cost_type;
    double max_time;

    virtual void initialize() {}
    virtual SearchStatus step() = 0;

    void set_plan(const Plan &plan);
    bool check_goal_and_set_plan(const StateType &state);
    int get_adjusted_cost(const OperatorType &op) const;
public:
    SearchEngine(const options::Options &opts);
    virtual ~SearchEngine();
    virtual void print_statistics() const;
    virtual void save_plan_if_necessary() const;
    bool found_solution() const;
    SearchStatus get_status() const;
    const Plan &get_plan() const;
    void search();
    const SearchStatistics &get_statistics() const {return statistics; }
    void set_bound(int b) {bound = b; }
    int get_bound() {return bound; }

    /* The following three methods should become functions as they
       do not require access to private/protected class members. */
    static void add_pruning_option(options::OptionParser &parser);
    static void add_options_to_parser(options::OptionParser &parser);
    static void add_succ_order_options(options::OptionParser &parser);
};

/*
template<class StateType, class OperatorType>
SearchEngine<StateType, OperatorType>::SearchEngine(const options::Options &opts)
    : status(IN_PROGRESS),
      solution_found(false),
      state_registry(
          *g_root_task(), *g_state_packer, *g_axiom_evaluator, g_initial_state_data),
      search_space(state_registry,
                   static_cast<OperatorCost>(opts.get_enum("cost_type"))),
      cost_type(static_cast<OperatorCost>(opts.get_enum("cost_type"))),
      max_time(opts.get<double>("max_time")) {
    if (opts.get<int>("bound") < 0) {
        cerr << "error: negative cost bound " << opts.get<int>("bound") << endl;
        utils::exit_with(ExitCode::INPUT_ERROR);
    }
    bound = opts.get<int>("bound");
}*/

template <class StateType, class OperatorType>
SearchEngine<StateType, OperatorType>::SearchEngine(const options::Options &opts,
	std::unique_ptr<StateRegistryBase<StateType, OperatorType>> state_registry)
	: status(IN_PROGRESS),
	  solution_found(false),
	  state_registry(std::move(state_registry)),
	  search_space(*this->state_registry, static_cast<OperatorCost>(opts.get_enum("cost_type"))),
	  cost_type(static_cast<OperatorCost>(opts.get_enum("cost_type"))),
	  max_time(opts.get<double>("max_time")) {
	if (opts.get<int>("bound") < 0) {
		std::cerr << "error: negative cost bound " << opts.get<int>("bound") << std::endl;
		utils::exit_with(utils::ExitCode::INPUT_ERROR);
	}
	bound = opts.get<int>("bound");
}

template<class StateType, class OperatorType>
SearchEngine<StateType, OperatorType>::~SearchEngine() {}

template<class StateType, class OperatorType>
void SearchEngine<StateType, OperatorType>::print_statistics() const {
    std::cout << "Bytes per state: "
         << state_registry->get_state_size_in_bytes() << std::endl;
}

template<class StateType, class OperatorType>
bool SearchEngine<StateType, OperatorType>::found_solution() const {
    return solution_found;
}

template<class StateType, class OperatorType>
SearchStatus SearchEngine<StateType, OperatorType>::get_status() const {
    return status;
}

template<class StateType, class OperatorType>
const typename SearchEngine<StateType, OperatorType>::Plan &SearchEngine<StateType, OperatorType>::get_plan() const {
    assert(solution_found);
    return plan;
}

template<class StateType, class OperatorType>
void SearchEngine<StateType, OperatorType>::set_plan(const Plan &p) {
    solution_found = true;
    plan = p;
}

template<class StateType, class OperatorType>
void SearchEngine<StateType, OperatorType>::search() {
    initialize();
    utils::CountdownTimer timer(max_time);
    while (status == IN_PROGRESS) {
        status = step();
        if (timer.is_expired()) {
            std::cout << "Time limit reached. Abort search." << std::endl;
            status = TIMEOUT;
            break;
        }
    }
    // TODO: Revise when and which search times are logged.
    std::cout << "Actual search time: " << timer
         << " [t=" << utils::g_timer << "]" << std::endl;
}

template<class StateType, class OperatorType>
void SearchEngine<StateType, OperatorType>::save_plan_if_necessary() const {
	assert(false && "must be specialized or overridden");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

template<class StateType, class OperatorType>
int SearchEngine<StateType, OperatorType>::get_adjusted_cost(const OperatorType &op) const {
    return get_adjusted_action_cost(op, cost_type);
}

/* TODO: merge this into add_options_to_parser when all search
         engines support pruning.

   Method doesn't belong here because it's only useful for certain derived classes.
   TODO: Figure out where it belongs and move it there. */
template<class StateType, class OperatorType>
void SearchEngine<StateType, OperatorType>::add_pruning_option(options::OptionParser &parser) {
    parser.add_option<std::shared_ptr<PruningMethod>>(
        "pruning",
        "Pruning methods can prune or reorder the set of applicable operators in "
        "each state and thereby influence the number and order of successor states "
        "that are considered.",
        "null()");
}

template<class StateType, class OperatorType>
void SearchEngine<StateType, OperatorType>::add_options_to_parser(options::OptionParser &parser) {
    ::add_cost_type_option_to_parser(parser);
    parser.add_option<int>(
        "bound",
        "exclusive depth bound on g-values. Cutoffs are always performed according to "
        "the real cost, regardless of the cost_type parameter", "infinity");
    parser.add_option<double>(
        "max_time",
        "maximum time in seconds the search is allowed to run for. The "
        "timeout is only checked after each complete search step "
        "(usually a node expansion), so the actual runtime can be arbitrarily "
        "longer. Therefore, this parameter should not be used for time-limiting "
        "experiments. Timed-out searches are treated as failed searches, "
        "just like incomplete search algorithms that exhaust their search space.",
        "infinity");
}

/* Method doesn't belong here because it's only useful for certain derived classes.
   TODO: Figure out where it belongs and move it there. */
template<class StateType, class OperatorType>
void SearchEngine<StateType, OperatorType>::add_succ_order_options(options::OptionParser &parser) {
	std::vector<std::string> options;
    parser.add_option<bool>(
        "randomize_successors",
        "randomize the order in which successors are generated",
        "false");
    parser.add_option<bool>(
        "preferred_successors_first",
        "consider preferred operators first",
        "false");
    parser.document_note(
        "Successor ordering",
        "When using randomize_successors=true and "
        "preferred_successors_first=true, randomization happens before "
        "preferred operators are moved to the front.");
    utils::add_rng_options(parser);
}


template<>
SearchEngine<GlobalState, GlobalOperator>::SearchEngine(const options::Options &opts);

template<>
bool SearchEngine<GlobalState, GlobalOperator>::check_goal_and_set_plan(const GlobalState &state);

template<>
void SearchEngine<GlobalState, GlobalOperator>::save_plan_if_necessary() const;


/*
  Print heuristic values of all heuristics evaluated in the evaluation context.
*/
template<class StateType = GlobalState, class OperatorType = GlobalOperator>
void print_initial_h_values(const EvaluationContext<StateType, OperatorType> &eval_context) {
    eval_context.get_cache().for_each_heuristic_value(
        [] (const Heuristic<StateType, OperatorType> *heur, const EvaluationResult &result) {
        std::cout << "Initial heuristic value for "
             << heur->get_description() << ": ";
        if (result.is_infinite())
            std::cout << "infinity";
        else
            std::cout << result.get_h_value();
        std::cout << std::endl;
    }
        );
}


template<class StateType = GlobalState, class OperatorType = GlobalOperator>
ordered_set::OrderedSet<OperatorID> collect_preferred_operators(
    EvaluationContext<StateType, OperatorType> &eval_context,
    const std::vector<Heuristic<StateType, OperatorType> *> &preferred_operator_heuristics) {
    ordered_set::OrderedSet<OperatorID> preferred_operators;
    for (Heuristic<StateType, OperatorType> *heuristic : preferred_operator_heuristics) {
        /*
          Unreliable heuristics might consider solvable states as dead
          ends. We only want preferred operators from finite-value
          heuristics.
        */
        if (!eval_context.is_heuristic_infinite(heuristic)) {
            for (OperatorID op_id : eval_context.get_preferred_operators(heuristic)) {
                preferred_operators.insert(op_id);
            }
        }
    }
    return preferred_operators;
}

#endif
