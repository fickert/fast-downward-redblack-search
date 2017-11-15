#include "../heuristic.h"

#include "../options/plugin.h"
#include "state.h"
#include "../open_list_factory.h"

namespace redblack {

static options::PluginTypePlugin<Evaluator<RBState, RBOperator>> _rb_evaluator_type_plugin(
	"Red-Black Evaluator",
	// TODO: Replace empty string by synopsis for the wiki page.
	"");

static options::PluginTypePlugin<Heuristic<RBState, RBOperator>> rb_heuristic_type_plugin(
	"Red-Black Heuristic",
	"A heuristic specification is either a newly created heuristic "
	"instance or a heuristic that has been defined previously. "
	"This page describes how one can specify a new heuristic instance. "
	"For re-using heuristics, see OptionSyntax#Heuristic_Predefinitions.\n\n"
	"Definitions of //properties// in the descriptions below:\n\n"
	" * **admissible:** h(s) <= h*(s) for all states s\n"
	" * **consistent:** h(s) <= c(s, s') + h(s') for all states s "
	"connected to states s' by an action with cost c(s, s')\n"
	" * **safe:** h(s) = infinity is only true for states "
	"with h*(s) = infinity\n"
	" * **preferred operators:** this heuristic identifies "
	"preferred operators ");


static options::PluginTypePlugin<OpenListFactory<RBState, RBOperator>> _rb_open_list_type_plugin(
	"Red-Black OpenList",
	// TODO: Replace empty string by synopsis for the wiki page.
	"");

}
