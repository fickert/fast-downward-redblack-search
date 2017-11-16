#include "open_list_factory.h"

#include "plugin.h"

static PluginTypePlugin<OpenListFactory<GlobalState, GlobalOperator>> _type_plugin(
    "OpenList",
    // TODO: Replace empty string by synopsis for the wiki page.
    "");
