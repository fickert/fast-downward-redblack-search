#pragma once

#include "../abstract_task.h"
#include "../task_utils/causal_graph.h"
#include "../algorithms/sccs.h"
#include "../globals.h"

#include <set>

namespace redblack {
namespace rbutils {

auto get_sccs(std::vector<int> variables) -> std::vector<std::set<int>> {
	std::vector<std::vector<int>> vars(g_root_task()->get_num_variables());
	std::size_t bound = variables.empty() ? g_root_task()->get_num_variables() : variables.size();
	const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());
	for (std::size_t i = 0; i < bound; i++) {
		if (variables.empty() || variables.size() == g_variable_domain.size()) {
			vars[i] = causal_graph.get_successors(i);
		} else {
			std::vector<int> successors = causal_graph.get_successors(variables[i]);
			for (std::size_t succ = 0; succ < successors.size(); succ++) {
				if (std::find(variables.begin(), variables.end(), successors[succ]) != variables.end()) {
					vars[variables[i]].push_back(successors[succ]);
				}
			}
		}
	}

	//SCC scc(vars);
	//auto found_sccs = scc.get_result();
	auto found_sccs = sccs::compute_maximal_sccs(vars);
	std::vector<std::set<int>> real_sccs;

	for (std::size_t i = 0; i < found_sccs.size(); i++) {
		if (found_sccs[i].size() != 1 || variables.empty() || variables.size() == g_variable_name.size()) {
			real_sccs.push_back(std::set<int>(found_sccs[i].begin(), found_sccs[i].end()));
		} else if (std::find(variables.begin(), variables.end(), found_sccs[i][0]) != variables.end()) {
			// this can happen if not all variables are in *variables*
			// the SCC class needs the input vector to be aligned very specifically
			real_sccs.push_back(std::set<int>(found_sccs[i].begin(), found_sccs[i].end()));
		}
	}

#ifdef DEBUG_PAINTING
	cout << "found " << real_sccs.size() << " SCCs" << endl;
#endif

	return real_sccs;
}

auto get_scc_levels(std::vector<std::set<int>> sccs) -> std::vector<std::vector<std::set<int>>> {
	// determine topology
	std::vector<std::vector<std::set<int>>> sccs_per_level(1);

	std::set<std::set<int>> root_sccs;
	std::set<std::set<int>> not_connected_sccs;

	const auto &causal_graph = causal_graph::get_causal_graph(g_root_task().get());

	for (const auto &scc : sccs) {
		bool all_contained = true;
		bool has_successors = false;
		for (const int var : scc) {
			const std::vector<int> &predecessors = causal_graph.get_predecessors(var);
			for (const int pred : predecessors) {
				if (std::find(scc.begin(), scc.end(), pred) == scc.end()) {
					all_contained = false;
					break;
				}
			}
			if (!all_contained) {
				break;
			}
			const std::vector<int> &successors = causal_graph.get_successors(var);
			for (const int succ : successors) {
				if (std::find(scc.begin(), scc.end(), succ) == scc.end()) {
					has_successors = true;
				}
			}
			if (!all_contained) {
				break;
			}
		}
		if (all_contained) {
			if (has_successors) {
				sccs_per_level[0].push_back(scc);
				root_sccs.insert(scc);
			} else {
				// SCCs in the CG that are not connected to the rest
				not_connected_sccs.insert(scc);
			}
		}
	}

	// TODO this is still not working 100% correct in all cases!
	// if some SCC with real lvl x is checked before his predecessor SCC x-1
	// it might get lvl x-1 if it has another predecessor with lvl x-2
	// this needs to be fixed
	for (std::size_t i = 0; i < sccs.size(); ++i) {
		if (root_sccs.find(sccs[i]) != root_sccs.end() || not_connected_sccs.find(sccs[i]) != not_connected_sccs.end()) {
			continue;
		}
		bool got_lvl = false;
		for (const int var : sccs[i]) {
			for (int lvl = sccs_per_level.size() - 1; lvl >= 0; --lvl) {
				for (const auto &scc : sccs_per_level[lvl]) {
					for (const int ref : scc) {
						const std::vector<int> &successors = causal_graph.get_successors(ref);
						if (std::find(successors.begin(), successors.end(), var) != successors.end()) {
							std::size_t new_lvl = lvl + 1;
							if (new_lvl >= sccs_per_level.size()) {
								sccs_per_level.resize(new_lvl + 1);
							}
							sccs_per_level[new_lvl].push_back(sccs[i]);
							got_lvl = true;
							break;
						}
					}
					if (got_lvl) {
						break;
					}
				}
				if (got_lvl) {
					break;
				}
			}
			if (got_lvl) {
				break;
			}
		}
	}

	for (const auto &scc : not_connected_sccs) {
		sccs_per_level.back().push_back(scc);
	}

	return sccs_per_level;
}

}
}
