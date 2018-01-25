#include "incremental_painting_strategy.h"

#include "painting_utils.h"
#include "util.h"
#include "../global_operator.h"
#include "../globals.h"
#include "../options/options.h"
#include "../options/option_parser.h"
#include "../options/plugin.h"
#include "../utils/rng_options.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace redblack {

// TODO: make sure that invertible variables are never painted black
// TODO: in plan reconstruction: fix conflicts on invertible variables, and in the least conflicts strategy add an assertion that invertible variables don't have conflicts


// incremental painting strategy base class

IncrementalPaintingStrategy::IncrementalPaintingStrategy(const options::Options &opts)
	: num_black(get_num_black(opts, true)) {
	if (num_black < 1 || num_black > g_root_task()->get_num_variables()) {
		std::cerr << "Bad value for num_black in incremental painting strategy: " << num_black << std::endl;
		utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
	}
}

IncrementalPaintingStrategy::~IncrementalPaintingStrategy() {}

auto IncrementalPaintingStrategy::generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan, const std::vector<bool> *never_black_variables) -> Painting {
	auto goal_facts = std::vector<FactPair>();
	goal_facts.reserve(g_goal.size());
	std::transform(std::begin(g_goal), std::end(g_goal), std::back_inserter(goal_facts), [](const auto &goal) { return FactPair{goal.first, goal.second}; });
	return generate_next_painting(last_painting, last_plan, goal_facts, never_black_variables);
}


void IncrementalPaintingStrategy::add_options_to_parser(options::OptionParser &parser) {
	add_num_black_options(parser);
}

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

auto LeastConflictsPaintingStrategy::generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &last_plan, const std::vector<FactPair> &goal_facts, const std::vector<bool> *never_black_variables) -> Painting {
	assert(!std::all_of(std::begin(last_painting.get_painting()), std::end(last_painting.get_painting()), [](const auto is_red) { return !is_red; }));
	auto conflicts = std::vector<int>(g_root_task()->get_num_variables(), 0);
	auto current_state = g_root_task()->get_initial_state_values();
	for (auto op_id : last_plan) {
		const auto &op = g_operators[op_id.get_index()];
		for (const auto &pre : op.get_preconditions())
			if (current_state[pre.var] != pre.val)
				++conflicts[pre.var];
		for (const auto &eff : op.get_effects())
			if (std::all_of(std::begin(eff.conditions), std::end(eff.conditions), [&current_state](const auto &condition) {
				return current_state[condition.var] == condition.val;
			}))
				current_state[eff.var] = eff.val;
	}
	for (const auto &goal_fact : goal_facts)
		if (current_state[goal_fact.var] != goal_fact.value)
			++conflicts[goal_fact.var];

	std::vector<int> level;
	auto max_level = -1;
	if (prefer_lvl) {
		level = get_variable_levels();
		for (auto var = 0; var < g_root_task()->get_num_variables(); ++var)
			max_level = std::max(max_level, level[var]);
	}

	auto painting = last_painting.get_painting();
	auto current_num_black = static_cast<std::size_t>(std::count_if(std::begin(painting), std::end(painting), [](auto b) { return !b; }));
	int curr_lvl = 0;
	auto do_prefer_lvl = prefer_lvl;
	const auto target_num_black = std::min<std::size_t>(current_num_black + num_black, g_root_task()->get_num_variables());
	while (current_num_black < target_num_black) {
		// ignoring the force_cg_leaves_red flag here, since CG leaves should
		// never have a conflict.
		int max = -1;
		int i = -1;
		for (size_t var = 0; var < g_variable_domain.size(); ++var) {
			if (conflicts[var] > max && painting[var] && !(never_black_variables && never_black_variables->at(var)) && (!do_prefer_lvl || (level[var] == curr_lvl && conflicts[var] > 0))) {
				max = conflicts[var];
				i = var;
			}
		}
		if (do_prefer_lvl && i == -1) {
			++curr_lvl;
			if (curr_lvl >= max_level) {
				do_prefer_lvl = false;
			}
			continue;
		}
		assert(i != -1);
		assert(painting[i]);
		painting[i] = false;
		assert(!(never_black_variables && never_black_variables->at(i)));
		++current_num_black;
	}

	return painting;
}

static auto _parse_least_conflicts(options::OptionParser &parser) -> std::shared_ptr<IncrementalPaintingStrategy> {
	IncrementalPaintingStrategy::add_options_to_parser(parser);

	parser.add_option<bool>("prefer_lvl", "TODO", "false");

	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<LeastConflictsPaintingStrategy>(parser.parse());
}


RandomPaintingStrategy::RandomPaintingStrategy(const options::Options &opts)
	: IncrementalPaintingStrategy(opts),
	  rng(utils::parse_rng_from_options(opts)) {}

auto RandomPaintingStrategy::generate_next_painting(const Painting &last_painting, const std::vector<OperatorID> &, const std::vector<FactPair> &, const std::vector<bool> *never_black_variables) -> Painting {
	assert(!std::all_of(std::begin(last_painting.get_painting()), std::end(last_painting.get_painting()), [](const auto is_red) { return !is_red; }));
	auto red_variables = std::vector<std::size_t>();
	red_variables.reserve(g_root_task()->get_num_variables());
	for (auto i = 0; i < g_root_task()->get_num_variables(); ++i)
		if (last_painting.is_red_var(i) && !(never_black_variables && never_black_variables->at(i)))
			red_variables.push_back(i);
	assert(!red_variables.empty());
	rng->shuffle(red_variables);
	auto next_painting = last_painting.get_painting();
	for (auto i = 0u; i < std::min<std::size_t>(red_variables.size(), num_black); ++i)
		next_painting[red_variables[i]] = false;
	return Painting(next_painting);
}

static auto _parse_random(options::OptionParser &parser) -> std::shared_ptr<IncrementalPaintingStrategy> {
	IncrementalPaintingStrategy::add_options_to_parser(parser);
	utils::add_rng_options(parser);

	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<RandomPaintingStrategy>(parser.parse());
}


// strategy plugins

static options::PluginShared<IncrementalPaintingStrategy> _plugin_least_conflicts("least_conflicts", _parse_least_conflicts);
static options::PluginShared<IncrementalPaintingStrategy> _plugin_random("random", _parse_random);

static options::PluginTypePlugin<IncrementalPaintingStrategy> _type_plugin("Incremental Painting Strategy",
	"Strategies to incrementally update the red-black painting.");

}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
