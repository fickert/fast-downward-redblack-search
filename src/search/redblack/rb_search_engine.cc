#include "rb_search_engine.h"

#include "state.h"
#include "../options/plugin.h"


template<>
bool SearchEngine<redblack::RBState, redblack::RBOperator>::check_goal_and_set_plan(const redblack::RBState &state) {
	if (std::all_of(std::begin(g_goal), std::end(g_goal), [&state](const auto &fact) {
		return state.has_fact(fact.first, fact.second);
	})) {
		std::cout << "Red-Black solution found!" << std::endl;
		Plan plan;
		search_space.trace_path(state, plan);
		set_plan(plan);
		return true;
	}
	return false;
}


static options::PluginTypePlugin<SearchEngine<redblack::RBState, redblack::RBOperator>> _type_plugin(
    "Red-Black SearchEngine",
    // TODO: Replace empty string by synopsis for the wiki page.
    "");
