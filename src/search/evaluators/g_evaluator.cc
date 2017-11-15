#include "g_evaluator.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace g_evaluator {
static Evaluator<> *_parse(OptionParser &parser) {
    parser.document_synopsis(
        "g-value evaluator",
        "Returns the g-value (path cost) of the search node.");
    parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new GEvaluator<>;
}

static Plugin<Evaluator<>> _plugin("g", _parse);
}
