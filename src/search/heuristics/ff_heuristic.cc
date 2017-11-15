#include "ff_heuristic.h"

#include "../global_state.h"
#include "../option_parser.h"
#include "../plugin.h"

#include <cassert>

namespace ff_heuristic {

template<>
int FFHeuristic<GlobalState, GlobalOperator>::compute_heuristic(const GlobalState &global_state) {
	return compute_heuristic_internal(convert_state<State>(global_state));
}


static Heuristic<> *_parse(OptionParser &parser) {
    parser.document_synopsis("FF heuristic", "See also Synergy.");
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

    Heuristic<>::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new FFHeuristic<>(opts);
}

static Plugin<Heuristic<>> _plugin("ff", _parse);
}
