#include "additive_heuristic.h"

#include "../global_state.h"
#include "../option_parser.h"
#include "../plugin.h"

#include <cassert>

namespace additive_heuristic {

template<>
template<>
auto AdditiveHeuristic<GlobalState, GlobalOperator>::convert_state(const GlobalState &state) -> State {
	return convert_global_state(state);
}

template<>
int AdditiveHeuristic<GlobalState, GlobalOperator>::compute_heuristic(const GlobalState &global_state) {
	return compute_heuristic_internal(convert_state<State>(global_state));
}

template<>
template<>
void AdditiveHeuristic<GlobalState, GlobalOperator>::setup_exploration_queue_state(const State &state) {
	for (FactProxy fact : state) {
		Proposition *init_prop = get_proposition(fact);
		enqueue_if_necessary(init_prop, 0, 0);
	}
}

template<>
template<>
auto AdditiveHeuristic<GlobalState, GlobalOperator>::is_operator_applicable(const State &state, int operator_no) -> bool {
	return task_properties::is_applicable(task_proxy.get_operators()[operator_no], state);
}


static Heuristic<> *_parse(OptionParser &parser) {
    parser.document_synopsis("Additive heuristic", "");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "supported");
    parser.document_language_support(
        "axioms",
        "supported (in the sense that the planner won't complain -- "
        "handling of axioms might be very stupid "
        "and even render the heuristic unsafe)");
    parser.document_property("admissible", "no");
    parser.document_property("consistent", "no");
    parser.document_property("safe", "yes for tasks without axioms");
    parser.document_property("preferred operators", "yes");

    Heuristic<GlobalState, GlobalOperator>::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new AdditiveHeuristic<>(opts);
}

static Plugin<Heuristic<>> _plugin("add", _parse);
}
