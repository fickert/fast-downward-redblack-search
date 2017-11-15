#include "sum_evaluator.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace sum_evaluator {

static Evaluator<GlobalState, GlobalOperator> *_parse(OptionParser &parser) {
    parser.document_synopsis("Sum evaluator",
                             "Calculates the sum of the sub-evaluators.");

    parser.add_list_option<Evaluator<GlobalState, GlobalOperator> *>("evals", "at least one evaluator");
    Options opts = parser.parse();

    opts.verify_list_non_empty<Evaluator<GlobalState, GlobalOperator> *>("evals");

    if (parser.dry_run())
        return 0;
    else
        return new SumEvaluator<GlobalState, GlobalOperator>(opts);
}

static Plugin<Evaluator<GlobalState, GlobalOperator>> _plugin("sum", _parse);
}
