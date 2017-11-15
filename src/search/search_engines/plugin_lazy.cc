#include "lazy_search.h"
#include "search_common.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace plugin_lazy {
static shared_ptr<SearchEngine<GlobalState, GlobalOperator>> _parse(OptionParser &parser) {
    parser.document_synopsis("Lazy best-first search", "");
    parser.add_option<shared_ptr<OpenListFactory<GlobalState, GlobalOperator>>>("open", "open list");
    parser.add_option<bool>("reopen_closed", "reopen closed nodes", "false");
    parser.add_list_option<Heuristic<GlobalState, GlobalOperator> *>(
        "preferred",
        "use preferred operators of these heuristics", "[]");
    SearchEngine<GlobalState, GlobalOperator>::add_succ_order_options(parser);
    SearchEngine<GlobalState, GlobalOperator>::add_options_to_parser(parser);
    Options opts = parser.parse();

    shared_ptr<lazy_search::LazySearch<GlobalState, GlobalOperator>> engine;
    if (!parser.dry_run()) {
        engine = make_shared<lazy_search::LazySearch<GlobalState, GlobalOperator>>(opts);
        /*
          TODO: The following two lines look fishy. If they serve a
          purpose, shouldn't the constructor take care of this?
        */
        vector<Heuristic<GlobalState, GlobalOperator> *> preferred_list = opts.get_list<Heuristic<GlobalState, GlobalOperator> *>("preferred");
        engine->set_pref_operator_heuristics(preferred_list);
    }

    return engine;
}
static PluginShared<SearchEngine<GlobalState, GlobalOperator>> _plugin("lazy", _parse);
}
