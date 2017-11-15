#include "weighted_evaluator.h"

#include "../evaluation_context.h"
#include "../option_parser.h"
#include "../plugin.h"

#include <cstdlib>
#include <sstream>

using namespace std;

namespace weighted_evaluator {
static Evaluator<GlobalState, GlobalOperator> *_parse(OptionParser &parser) {
    parser.document_synopsis(
        "Weighted evaluator",
        "Multiplies the value of the evaluator with the given weight.");
    parser.add_option<Evaluator<GlobalState, GlobalOperator> *>("eval", "evaluator");
    parser.add_option<int>("weight", "weight");
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new WeightedEvaluator<GlobalState, GlobalOperator>(opts);
}

static Plugin<Evaluator<GlobalState, GlobalOperator>> _plugin("weight", _parse);
}
