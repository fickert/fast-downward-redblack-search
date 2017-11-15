#include "evaluator.h"

#include "plugin.h"

static PluginTypePlugin<Evaluator<>> _type_plugin(
	"Evaluator",
	// TODO: Replace empty string by synopsis for the wiki page.
	"");
