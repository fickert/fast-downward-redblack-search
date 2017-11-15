#include "standard_scalar_open_list.h"

#include "../open_list.h"
#include "../option_parser.h"
#include "../plugin.h"

#include <cassert>

using namespace std;

namespace standard_scalar_open_list {
static shared_ptr<OpenListFactory<GlobalState, GlobalOperator>> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Standard open list",
        "Standard open list that uses a single evaluator");
    parser.add_option<Evaluator<GlobalState, GlobalOperator> *>("eval", "evaluator");
    parser.add_option<bool>(
        "pref_only",
        "insert only nodes generated by preferred operators", "false");

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<StandardScalarOpenListFactory<GlobalState, GlobalOperator>>(opts);
}

static PluginShared<OpenListFactory<GlobalState, GlobalOperator>> _plugin("single", _parse);
}
