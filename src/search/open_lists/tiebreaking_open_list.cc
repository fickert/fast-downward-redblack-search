#include "tiebreaking_open_list.h"

#include "../open_list.h"
#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/memory.h"

#include <cassert>
#include <deque>
#include <map>
#include <utility>
#include <vector>

using namespace std;

namespace tiebreaking_open_list {
static shared_ptr<OpenListFactory<GlobalState, GlobalOperator>> _parse(OptionParser &parser) {
    parser.document_synopsis("Tie-breaking open list", "");
    parser.add_list_option<Evaluator<GlobalState, GlobalOperator> *>("evals", "evaluators");
    parser.add_option<bool>(
        "pref_only",
        "insert only nodes generated by preferred operators", "false");
    parser.add_option<bool>(
        "unsafe_pruning",
        "allow unsafe pruning when the main evaluator regards a state a dead end",
        "true");
    Options opts = parser.parse();
    opts.verify_list_non_empty<Evaluator<GlobalState, GlobalOperator> *>("evals");
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<TieBreakingOpenListFactory<GlobalState, GlobalOperator>>(opts);
}

static PluginShared<OpenListFactory<GlobalState, GlobalOperator>> _plugin("tiebreaking", _parse);
}
