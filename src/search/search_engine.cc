#include "search_engine.h"

#include "globals.h"
#include "option_parser.h"
#include "plugin.h"

#include "utils/countdown_timer.h"
#include "utils/system.h"

#include <cassert>
#include <iostream>

using namespace std;
using utils::ExitCode;

class PruningMethod;

template<>
SearchEngine<GlobalState, GlobalOperator>::SearchEngine(const Options &opts)
    : status(IN_PROGRESS),
      solution_found(false),
      state_registry(std::make_shared<StateRegistryBase<GlobalState, GlobalOperator>>(
          *g_root_task(), *g_state_packer, *g_axiom_evaluator, g_initial_state_data)),
      search_space(std::make_shared<SearchSpace<GlobalState, GlobalOperator>>(*state_registry,
                   static_cast<OperatorCost>(opts.get_enum("cost_type")))),
      cost_type(static_cast<OperatorCost>(opts.get_enum("cost_type"))),
      max_time(opts.get<double>("max_time")) {
    if (opts.get<int>("bound") < 0) {
        cerr << "error: negative cost bound " << opts.get<int>("bound") << endl;
        utils::exit_with(ExitCode::INPUT_ERROR);
    }
    bound = opts.get<int>("bound");
}

template<>
bool SearchEngine<GlobalState, GlobalOperator>::check_goal_and_set_plan(const GlobalState &state) {
    if (test_goal(state)) {
        cout << "Solution found!" << endl;
        Plan plan;
        search_space->trace_path(state, plan);
        set_plan(plan);
        return true;
    }
    return false;
}

template<>
void SearchEngine<GlobalState, GlobalOperator>::save_plan_if_necessary() const {
    if (found_solution())
        save_plan(get_plan());
}


static options::PluginTypePlugin<SearchEngine<GlobalState, GlobalOperator>> _type_plugin(
    "SearchEngine",
    // TODO: Replace empty string by synopsis for the wiki page.
    "");
