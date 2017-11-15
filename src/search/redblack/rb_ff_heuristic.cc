#include "rb_ff_heuristic.h"

#include "../heuristics/additive_heuristic.h"
#include "state.h"
#include "operator.h"
#include "../options/plugin.h"

namespace additive_heuristic {
template<>
template<>
auto AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::convert_state(const redblack::RBState &state) -> redblack::RBState {
	return state;
}

template<>
template<>
void AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::setup_exploration_queue_state(const redblack::RBState &state) {
	for (auto i = 0; i < task->get_num_variables(); ++i)
		for (auto j = 0; j < task->get_variable_domain_size(i); ++j)
			if (state.has_fact(i, j))
				enqueue_if_necessary(get_proposition(FactProxy(*task, i, j)), 0, nullptr);
}

template<>
template<>
auto AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::is_operator_applicable(const redblack::RBState &state, int operator_no) -> bool {
	return state.get_rb_state_registry().get_operators()[operator_no].is_applicable(state);
}

template<>
int AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::compute_heuristic(const State &) {
	assert(false && "not implemented");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

template<>
void AdditiveHeuristic<redblack::RBState, redblack::RBOperator>::compute_heuristic_for_cegar(const State &) {
	assert(false && "not implemented");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}
}

namespace redblack {
static Heuristic<RBState, RBOperator> *_parse(options::OptionParser &parser) {
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

    Heuristic<RBState, RBOperator>::add_options_to_parser(parser);
	options::Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new ff_heuristic::FFHeuristic<RBState, RBOperator>(opts);
}

static options::Plugin<Heuristic<RBState, RBOperator>> _plugin("ff_rb", _parse);
}
