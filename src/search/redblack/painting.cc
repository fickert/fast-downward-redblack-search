#include "painting.h" 

#include "painting_utils.h"
#include "../globals.h"
#include "../options/options.h"
#include "../options/plugin.h"
#include "../options/option_parser.h"
#include "../utils/timer.h"
#include "../utils/rng_options.h"

#include <iostream>
#include <map>

using namespace std;

namespace redblack {

Painting::Painting(const std::vector<bool> &painting) : painting(painting) {}
Painting::Painting(std::vector<bool> &&painting) : painting(std::move(painting)) {}

PaintingFactory::PaintingFactory(const options::Options &opts)
	: force_cg_leaves_red(opts.get<bool>("force_cg_leaves_red")) {}

auto PaintingFactory::get_all_red_painting() -> InternalPaintingType {
	return InternalPaintingType(g_root_task()->get_num_variables(), true);
}

auto PaintingFactory::get_all_black_painting() -> InternalPaintingType {
	return InternalPaintingType(g_root_task()->get_num_variables(), false);
}

auto PaintingFactory::get_cg_leaves_painting() -> InternalPaintingType {
	auto painting = get_all_black_painting();
	const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());
	for (auto i = 0; i < g_root_task()->get_num_variables(); ++i)
		if (causal_graph.get_successors(i).empty())
			painting[i] = true;
	return painting;
}


auto get_num_black_vars(const options::Options &opts) -> int {
	assert(opts.contains("num_black_vars") || opts.contains("ratio_black"));
	auto num_black_vars = opts.contains("num_black_vars") ? opts.get<int>("num_black_vars") : -1;
	if (num_black_vars == -1) {
		assert(opts.contains("ratio_black"));
		num_black_vars = g_root_task()->get_num_variables() * opts.get<int>("ratio_black") / 100.;
	}
	num_black_vars = std::min(num_black_vars, g_root_task()->get_num_variables());
	return num_black_vars;
}


CGTopFirstPaintingFactory::CGTopFirstPaintingFactory(const options::Options &opts) 
	: PaintingFactory(opts),
	  num_black_vars(get_num_black_vars(opts)) {}

auto CGTopFirstPaintingFactory::construct_painting() -> InternalPaintingType {
	auto painting = get_all_red_painting();
	if (force_cg_leaves_red) {
		const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());
		auto num_black = 0;
		for (auto i = 0; i < g_root_task()->get_num_variables() && num_black < num_black_vars; ++i) {
			if (!causal_graph.get_successors(i).empty()) {
				painting[i] = false;
				++num_black;
			}
		}
	} else {
		for (auto i = 0; i < num_black_vars; ++i)
			painting[i] = false;
	}
	return painting;
}

CGBranchFirstPaintingFactory::CGBranchFirstPaintingFactory(const options::Options &opts)
	: PaintingFactory(opts),
	  num_black_vars(get_num_black_vars(opts)),
	  scc_painted(),
	  scc_offset_to_level() {}

auto CGBranchFirstPaintingFactory::get_connected_components(std::vector<int> variables) -> std::vector<std::set<int>> {
	std::vector<std::set<int>> components;

	std::vector<bool> vars(g_variable_domain.size(), false);
	for (size_t i = 0; i < variables.size(); i++) {
		vars[variables[i]] = true;
	}

	std::vector<bool> seen_vars(g_variable_domain.size(), false);
	std::vector<int> var_comp_map(g_variable_domain.size(), -1);

	std::size_t number_seen_vars = 0;

	std::set<int> open_vars;

	const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());

	//cout << "before while" << endl;
	//cout << "variabled.size: " << variables.size() << endl;
	while (number_seen_vars < variables.size()) {
		//cout << "open_vars.size = " << open_vars.size() << endl;
		if (open_vars.empty()) {
			for (std::size_t i = 0; i < variables.size(); i++) {
				int var = variables[i];
				if (!seen_vars[var]) {
					open_vars.insert(var);
					var_comp_map[var] = components.size();
					components.push_back(std::set<int>());
					components[var_comp_map[var]].insert(var);
					break;
				}
			}
			continue;
		}

		//cout << "number_seen_vars: " << number_seen_vars << endl;

		int curr = *open_vars.begin();
		open_vars.erase(open_vars.begin());

		if (seen_vars[curr]) {
			continue;
		}

		seen_vars[curr] = true;
		number_seen_vars++;

		for (std::size_t pre = 0; pre < causal_graph.get_predecessors(curr).size(); pre++) {
			int var = causal_graph.get_predecessors(curr)[pre];
			if (vars[var]) {
				open_vars.insert(var);
				var_comp_map[var] = var_comp_map[curr];
				components[var_comp_map[var]].insert(var);
			}
		}
		for (std::size_t succ = 0; succ < causal_graph.get_successors(curr).size(); succ++) {
			int var = causal_graph.get_successors(curr)[succ];
			if (vars[var]) {
				open_vars.insert(var);
				var_comp_map[var] = var_comp_map[curr];
				components[var_comp_map[var]].insert(var);
			}
		}
	}
	return components;
}

auto CGBranchFirstPaintingFactory::paint_dfs_sccs(int cur_scc_offset, int starting_var_of_scc, std::vector<std::set<int>> sccs, InternalPaintingType &painting, int &already_black) -> bool {
	std::set<int> cur_scc = sccs[cur_scc_offset];
	//paint the current scc
	if (paint_succ_rec(starting_var_of_scc, painting, already_black, cur_scc)) {
		cout << "painting scc limit reached" << endl;
		return true;
	} else {
		scc_painted[cur_scc_offset] = true;
		cout << "painting scc completed" << endl;
	}

	const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());
	std::vector<int> succ_outside_scc;
	//compute successors of cur_scc
	for (std::set<int>::iterator it = cur_scc.begin(); it != cur_scc.end(); ++it) {
		int var = *it;
		for (std::size_t i = 0; i < causal_graph.get_successors(var).size(); i++) {
			int successor = causal_graph.get_successors(var)[i];
			if (cur_scc.find(successor) == cur_scc.end()) {
				//the successor is inside another scc
				succ_outside_scc.push_back(successor);
			}
		}
	}
	std::set<int> offsets_for_scc_succs_of_cur_scc;
	std::vector<int> map_scc_to_starting_var(sccs.size(), -1);
	for (std::size_t i = 0; i < succ_outside_scc.size(); i++) {
		for (std::size_t j = 0; j < sccs.size(); j++) {
			if (sccs[j].find(succ_outside_scc[i]) != sccs[j].end()) {
				//the scc containing [i] is [j]
				offsets_for_scc_succs_of_cur_scc.insert(j);

				if (scc_offset_to_level[j] == -1)
					scc_offset_to_level[j] = scc_offset_to_level[cur_scc_offset] + 1;

				if (map_scc_to_starting_var[j] == -1)
					map_scc_to_starting_var[j] = succ_outside_scc[i];

			}
		}
	}
	//cout << "successors of current scc: "<< offsets_for_scc_succs_of_cur_scc.size() << endl;

	auto compare_bound = [this](int scc_offset1, int scc_offset2) {
		return scc_offset_to_level[scc_offset1] > scc_offset_to_level[scc_offset2];
	};

	std::vector<int> offsets_for_scc_succs_as_array(offsets_for_scc_succs_of_cur_scc.begin(), offsets_for_scc_succs_of_cur_scc.end());
	std::sort(offsets_for_scc_succs_as_array.begin(), offsets_for_scc_succs_as_array.end(), compare_bound);

	//paint the successor sccs
	int limit_reached = false;
	for (std::size_t i = 0; i < offsets_for_scc_succs_as_array.size(); i++) {
		int scc_offset = offsets_for_scc_succs_as_array[i];
		if (!scc_painted[scc_offset]) {
			if (paint_dfs_sccs(scc_offset, map_scc_to_starting_var[scc_offset], sccs, painting, already_black)) {
				limit_reached = true;
				break;
			}
		}
	}

	return limit_reached;
}

auto CGBranchFirstPaintingFactory::paint_succ_rec(int cur_node, InternalPaintingType &painting, int &already_black, std::set<int> &scc) -> bool {
	//cout << "remaining_black: " << already_black << endl;
	if (already_black >= num_black_vars)
		return true;
	const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());
	if (!(causal_graph.get_successors(cur_node).empty() && force_cg_leaves_red)) {
		painting[cur_node] = false;
		already_black++;

	} else {
		//cout << "leave_red var" << endl;
		++num_black_vars;
	}
	int limit_reached = false;
	for (size_t i = 0; i < causal_graph.get_successors(cur_node).size(); i++) {
		int successor = causal_graph.get_successors(cur_node)[i];
		if (painting[successor] && (scc.find(successor) != scc.end())) {
			//cout << "recuring to child var inside cur scc" << endl;
			if (paint_succ_rec(successor, painting, already_black, scc)) {
				limit_reached = true;
				break;
			}
		}
	}
	return limit_reached;
}

auto CGBranchFirstPaintingFactory::construct_painting() -> InternalPaintingType {
	auto painting = get_all_red_painting();
	std::vector<int> all_vars(g_variable_domain.size());
	for (std::size_t i = 0; i < g_variable_domain.size(); ++i) {
		all_vars.at(i) = i;
	}

	std::vector<set<int>> ccs = get_connected_components(all_vars);
	std::cout << "connected compontents size: " << ccs.size() << std::endl;

	const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());
	bool limit_reached = false;
	for (std::size_t i = 0; i < ccs.size(); i++) {
		//paint one connected component at a time

		//find the strongly connected components
		std::vector<int> component(ccs[i].begin(), ccs[i].end());
		std::vector<std::set<int>> sccs = rbutils::get_sccs(component);

		//init arrays
		scc_painted = std::vector<bool>(sccs.size(), false);
		scc_offset_to_level = std::vector<int>(sccs.size(), -1);

		std::cout << "strongly cc size: " << sccs.size() << std::endl;

		// find the sccs that are a sources of the sccs graph
		// for every scc go through all nodes in the scc and find the scc which's nodes have no incoming edges from other sccs
		std::vector<int> source_sccs_offsets;
		for (std::size_t j = 0; j < sccs.size(); j++) {
			bool is_source = true;
			for (std::set<int>::iterator it = sccs[j].begin(); it != sccs[j].end(); ++it) {
				int var = *it;
				//check whether the predecessors are inside the current scc
				for (std::size_t k = 0; k < causal_graph.get_predecessors(var).size(); k++) {
					int pred = causal_graph.get_predecessors(var)[k];
					if (sccs[j].find(pred) == sccs[j].end()) {
						is_source = false;
						break;
					}
				}
				if (!is_source) {
					break;
				}

			}
			if (is_source) {
				source_sccs_offsets.push_back(j);
				scc_offset_to_level[j] = 0;
			}

		}

		std::cout << "sources size: " << source_sccs_offsets.size() << std::endl;

		int already_black = 0;
		//paint the strongly connected components black starting from sources
		for (std::size_t j = 0; j < source_sccs_offsets.size(); j++) {
			if (paint_dfs_sccs(source_sccs_offsets[j], *sccs[source_sccs_offsets[j]].begin(), sccs, painting, already_black)) {
				limit_reached = true;
				break;
			}
			std::cout << "one source completely painted" << std::endl;
		}

		if (limit_reached) {
			break;
		}
	}

	return painting;
}


IncSCCLvlPaintingFactory::IncSCCLvlPaintingFactory(const options::Options &opts)
	: PaintingFactory(opts),
	  num_black_vars(get_num_black_vars(opts)),
	  random_within_scc(opts.get<bool>("scc_random")),
	  rng(utils::parse_rng_from_options(opts)) {}

void IncSCCLvlPaintingFactory::randomly_paint_scc(InternalPaintingType &painting, const std::vector<int> &scc, std::size_t number_black) {
	if (number_black == scc.size()) {
		for (const auto &var : scc) {
			//             cout << "painted " << g_fact_names[var][0] << " black" << endl;
			painting[var] = false;
		}
	} else {
		//rng.seed(static_cast<int>(time(0)));

		std::size_t painted_black = 0;
		bool prefer = true;
		while (painted_black < number_black) {
			auto var = scc[(*rng)(prefer ? scc.size() / 2 : scc.size())];
			prefer = !prefer;
			if (painting[var]) {
				//                 cout << "painted " << g_fact_names[var][0] << " black" << endl;
				painting[var] = false;
				++painted_black;
			}
		}
	}
}

auto IncSCCLvlPaintingFactory::construct_painting() -> InternalPaintingType {
    // special case if all vars shall be black
    if (num_black_vars == g_root_task()->get_num_variables())
		return force_cg_leaves_red && rbutils::get_sccs({}).size() > 1 ?
			get_cg_leaves_painting() :
			get_all_black_painting();

	// special case if no variables shall be black
	if (num_black_vars == 0)
		return get_all_red_painting();

	// special case if CG is strongly connected
	auto sccs = rbutils::get_sccs({});
	if (sccs.size() == 1) {
		std::cout << "CG is strongly connected" << std::endl;
		auto painting = get_all_red_painting();

		if (random_within_scc) {
			randomly_paint_scc(painting, std::vector<int>(sccs[0].begin(), sccs[0].end()), num_black_vars);
		} else {
			for (int i = 0; i < num_black_vars; ++i)
				painting[i] = false;
		}

		return painting;
	}

	//         if (sccs.size() > 3 && number_black_vars == 0){      // used for unsolvability IPC
	//             // if there are "many" SCCs, just paint one root SCC completely black in the first iteration of the search
	//             
	//             is_red_var = vector<bool>(g_variable_domain.size(), true);
	//             
	//             randomly_paint_scc(is_red_var, vector<int>(sccs_per_level[0][0].begin(), sccs_per_level[0][0].end()), sccs_per_level[0][0].size());
	//             
	//             return is_red_var;
	//         }    

	auto sccs_per_level = rbutils::get_scc_levels(sccs);

	if (num_black_vars == 1) {
		for (std::size_t lvl = 0; lvl < sccs_per_level.size(); ++lvl) {
			std::cout << "level = " << lvl << std::endl;
			for (const auto &scc : sccs_per_level[lvl]) {
				std::cout << "   SCC:" << std::endl;
				for (const int var : scc) {
					std::cout << "               " << g_root_task()->get_fact_name({var, 0}) << std::endl;
				}
			}
		}
	}

	// do actual painting

	auto painting = get_all_red_painting();
	const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());

	std::size_t left_to_paint_black = num_black_vars;
	for (std::size_t curr_lvl = 0; curr_lvl < sccs_per_level.size(); ++curr_lvl) {
		// can be, that we cannot paint number_black_vars variables black, because
		// of the force_cg_leaves_red constraint

		std::vector<std::vector<int>> sccs_curr_lvl;
		for (const auto &scc : sccs_per_level[curr_lvl])
			sccs_curr_lvl.push_back(std::vector<int>(scc.begin(), scc.end()));

		for (std::size_t index = 0; index < static_cast<std::size_t>(g_root_task()->get_num_variables()); ++index) {
			bool red_left = false;

			for (const auto &scc : sccs_curr_lvl) {
				if (index < scc.size() && (!force_cg_leaves_red || scc.size() > 1 || !causal_graph.get_successors(*scc.begin()).empty())) {
					painting[scc[index]] = false;
					red_left = true;
					--left_to_paint_black;
					if (left_to_paint_black == 0)
						break;
				}

			}
			if (!red_left || left_to_paint_black == 0)
				break;
		}


		// TODO think of reintroducing this again
		// probably only increase some counts in the loop above and perform the painting here

		//         size_t per_scc = ceil((float) left_to_paint_black / (float) sccs_per_level[curr_lvl].size());
		//         
		//         size_t left = 0;
		//         
		//         for (const auto &scc : sccs_per_level[curr_lvl]){
		//             size_t max_fit = scc.size();
		//             
		//             randomly_paint_scc(is_red_var, vector<int>(scc.begin(), scc.end()), min(left + per_scc, max_fit));
		//             left_to_paint_black -= min(left_to_paint_black, min(left + per_scc, max_fit));
		//             
		//             if (max_fit > per_scc){
		//                 if (left > 0){
		//                     left = max(0, (int) left - (int) (max_fit - per_scc));
		//                 }
		//             } else {
		//                 left += per_scc - max_fit;
		//             }
		//         }
		//     
		//         if (left > 0 && left_to_paint_black > 0){
		//             // if something left, check if there exists an SCC at curr_lvl that is not completely black
		//             for (const auto &scc : sccs_per_level[curr_lvl]){
		//                 for (const int var : scc){
		//                     // not very efficient, but who cares..
		//                     if (is_red_var[var]){
		//                         is_red_var[var] = false;
		//                         --left_to_paint_black;
		//                         --left;
		//                         if (left == 0 || left_to_paint_black == 0){
		//                             break;
		//                         }
		//                     }
		//                 }
		//                 if (left == 0 || left_to_paint_black == 0){
		//                     break;
		//                 }
		//             }
		//         }

		if (left_to_paint_black == 0)
			break;
	}
	return painting;
}


void PaintingFactory::add_options_to_parser(options::OptionParser &parser) {
    // TODO add docu
    parser.add_option<bool>("force_cg_leaves_red", "", "false");
    parser.add_option<bool>("incremental_search", "", "false");
	utils::add_rng_options(parser);
}

static auto _parse_cg_top_first(options::OptionParser &parser) -> std::shared_ptr<Painting> {
    // TODO docu
	PaintingFactory::add_options_to_parser(parser);
	parser.add_option<int>("num_black_vars", "The number of variables to paint black");
    parser.add_option<int>("ratio_black", "give the ratio in percent, i.e. ratio in [0, 100]", "0", options::Bounds("0", "100"));
    
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<Painting>(CGTopFirstPaintingFactory(parser.parse()).construct_painting());
}

static auto _parse_cg_branches_first(options::OptionParser &parser) -> std::shared_ptr<Painting> {
	PaintingFactory::add_options_to_parser(parser);
	parser.add_option<int>("num_black_vars", "The number of variables to paint black");
	parser.add_option<int>("ratio_black", "give the ratio in percent, i.e. ratio in [0, 100]", "0", options::Bounds("0", "100"));

	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<Painting>(CGBranchFirstPaintingFactory(parser.parse()).construct_painting());
}

static auto _parse_inc_scc_lvl(options::OptionParser &parser) -> std::shared_ptr<Painting> {
	PaintingFactory::add_options_to_parser(parser);
	parser.add_option<int>("num_black_vars", "The number of variables to paint black");
	parser.add_option<int>("ratio_black", "give the ratio in percent, i.e. ratio in [0, 100]", "0", options::Bounds("0", "100"));
    parser.add_option<bool>("scc_random", "TODO", "false");

	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<Painting>(IncSCCLvlPaintingFactory(parser.parse()).construct_painting());
}

static options::PluginShared<Painting> _plugin_cg_top_first("cg_top_first", _parse_cg_top_first);
static options::PluginShared<Painting> _plugin_cg_branches_first("cg_branches_first", _parse_cg_branches_first);
static options::PluginShared<Painting> _plugin_inc_scc_lvl("inc_scc_lvl", _parse_inc_scc_lvl);

static options::PluginTypePlugin<Painting> _type_plugin("Red-Black Painting",
	"Strategies to generate a painting for red-black partial delete relaxation.");

}
