#include "incremental_painting_strategy.h"

#include "painting_utils.h"
#include "../abstract_task.h"
#include "../global_operator.h"
#include "../globals.h"
#include "../options/options.h"
#include "../options/option_parser.h"
#include "../options/plugin.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace redblack {

// incremental painting strategy base class

IncrementalPaintingStrategy::IncrementalPaintingStrategy(const options::Options &) {}

IncrementalPaintingStrategy::~IncrementalPaintingStrategy() {}


auto LeastConflictsPaintingStrategy::get_variable_levels() -> std::vector<int> {
	auto scc_levels = rbutils::get_scc_levels(rbutils::get_sccs({}));
	auto variable_levels = std::vector<int>(g_root_task()->get_num_variables(), -1);
	for (auto lvl = static_cast<std::size_t>(0); lvl < scc_levels.size(); ++lvl) {
		for (const auto &scc : scc_levels[lvl]) {
			for (auto var : scc) {
				assert(variable_levels[var] == -1);
				variable_levels[var] = lvl;
			}
		}
	}
	return variable_levels;
}

LeastConflictsPaintingStrategy::LeastConflictsPaintingStrategy(const options::Options &opts)
	: IncrementalPaintingStrategy(opts),
	  prefer_lvl(opts.get<bool>("prefer_lvl")) {}

auto LeastConflictsPaintingStrategy::generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan) -> Painting {
	auto conflicts = std::vector<int>(g_root_task()->get_num_variables(), 0);
	auto current_state = g_root_task()->get_initial_state_values();
	for (auto op_id : last_plan) {
		const auto &op = g_operators[op_id.get_index()];
		for (const auto &pre : op.get_preconditions())
			if (current_state[pre.var] != pre.val)
				++conflicts[pre.var];
		for (const auto &eff : op.get_effects())
			current_state[eff.var] = eff.val;
	}
	for (const auto &goal : g_goal)
		if (current_state[goal.first] != goal.second)
			++conflicts[goal.first];

	std::vector<int> level;
	auto max_level = -1;
	if (prefer_lvl) {
		level = get_variable_levels();
		for (auto var = 0; var < g_root_task()->get_num_variables(); ++var)
			max_level = std::max(max_level, level[var]);
	}

	auto painting = last_painting.get_painting();
	auto num_black = std::count_if(std::begin(painting), std::end(painting), [](auto b) { return !b; });
	int curr_lvl = 0;
	while (num_black < num_black_vars) {
		// ignoring the force_cg_leaves_red flag here, since CG leaves should
		// never have a conflict.
		int max = -1;
		int i = -1;
		for (size_t var = 0; var < g_variable_domain.size(); ++var) {
			if (conflicts[var] > max && painting[var] && (!prefer_lvl || (level[var] == curr_lvl && conflicts[var] > 0))) {
				max = conflicts[var];
				i = var;
			}
		}
		if (prefer_lvl && i == -1) {
			++curr_lvl;
			if (curr_lvl >= max_level) {
				// TODO: this was copy/pasted from prior code, where this was only run once. introduce a local variable for prefer_lvl
				prefer_lvl = false;
			}
			continue;
		}
		assert(painting[i]);
		painting[i] = false;
		++num_black;
	}

	return painting;
}

static auto _parse_least_conflicts(options::OptionParser &parser) -> std::shared_ptr<IncrementalPaintingStrategy> {
	//IncrementalPaintingStrategy::add_options_to_parser(parser);

	//parser.add_option<int>("number_black_vars", "TODO", "0");
	parser.add_option<bool>("prefer_lvl", "TODO", "false");

	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<LeastConflictsPaintingStrategy>(parser.parse());
}

// TODO: strategies


// strategy plugins

static options::PluginShared<IncrementalPaintingStrategy> _plugin_dynamic_balancing_time("least_conflicts", _parse_least_conflicts);

static options::PluginTypePlugin<IncrementalPaintingStrategy> _type_plugin("Incremental Painting Strategy",
	"Strategies to incrementally update the red-black painting.");

}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
