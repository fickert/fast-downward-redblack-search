#include "pref_evaluator.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace pref_evaluator {
static Evaluator<GlobalState, GlobalOperator> *_parse(OptionParser &parser) {
    parser.document_synopsis("Preference evaluator",
                             "Returns 0 if preferred is true and 1 otherwise.");
    parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new PrefEvaluator<GlobalState, GlobalOperator>;
}

static Plugin<Evaluator<GlobalState, GlobalOperator>> _plugin("pref", _parse);
}
