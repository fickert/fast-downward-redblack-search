#include "alternation_open_list.h"

#include "../open_list.h"
#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/system.h"

#include <cassert>
#include <memory>

using namespace std;
using utils::ExitCode;

namespace alternation_open_list {
static shared_ptr<OpenListFactory<GlobalState, GlobalOperator>> _parse(OptionParser &parser) {
    parser.document_synopsis("Alternation open list",
                             "alternates between several open lists.");
    parser.add_list_option<shared_ptr<OpenListFactory<GlobalState, GlobalOperator>>>(
        "sublists",
        "open lists between which this one alternates");
    parser.add_option<int>(
        "boost",
        "boost value for contained open lists that are restricted "
        "to preferred successors",
        "0");

    Options opts = parser.parse();
    opts.verify_list_non_empty<shared_ptr<OpenListFactory<GlobalState, GlobalOperator>>>("sublists");
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<AlternationOpenListFactory<GlobalState, GlobalOperator>>(opts);
}

static PluginShared<OpenListFactory<GlobalState, GlobalOperator>> _plugin("alt", _parse);
}
