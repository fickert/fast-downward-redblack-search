#include "search_space.h"

#include "operator.h"
#include "state.h"
#include "../globals.h"
#include "state_registry.h"

#include <set>


template<>
void SearchSpace<redblack::RBState, redblack::RBOperator>::trace_path(const redblack::RBState &goal_state, std::vector<const redblack::RBOperator*> &path) const {
	assert(dynamic_cast<redblack::RBStateRegistry *>(&state_registry));
	assert(dynamic_cast<redblack::RBStateRegistry *>(&state_registry) == &goal_state.get_rb_state_registry());
	const auto &rb_state_registry = goal_state.get_rb_state_registry();
	const auto &operators = rb_state_registry.get_operators();
	auto current_state = goal_state;
	std::vector<std::vector<std::vector<OperatorID>>> best_supporters;
    assert(path.empty());
    
    for (;;) {          // backtrace solution path
        const SearchNodeInfo &info = search_node_infos[current_state];

        if (info.creating_operator == -1) {  // reached initial state => done
            best_supporters.push_back(rb_state_registry.get_initial_state_best_supporters());
            assert(info.parent_state_id == StateID::no_state);
            break;
        }
        const redblack::RBOperator *op = &operators[info.creating_operator];
        assert(op->is_black());

        path.push_back(op);
        current_state = state_registry.lookup_state(info.parent_state_id);

        state_registry.get_successor_state(current_state, *op);
        best_supporters.push_back(rb_state_registry.get_stored_best_supporters());
        // TODO to not waste so much memory, could do the whole relaxed plan reconstruction here
    }
    std::reverse(path.begin(), path.end());
    std::reverse(best_supporters.begin(), best_supporters.end());

    std::set<std::pair<int, int> > marked_facts;
    for (auto const &goal : g_goal){
        if (goal_state.get_painting().is_red_var(goal.first)){
            marked_facts.insert(goal);
#ifdef DEBUG_PLAN_EXTRACTION
            cout << "marked goal fact " << g_fact_names[goal.first][goal.second] << endl;
#endif
        }
    }
    std::vector<const redblack::RBOperator *> plan;

    for (int step = best_supporters.size() - 1; step >= 0; --step){
#ifdef DEBUG_PLAN_EXTRACTION
        cout << "step " << step << endl;
#endif
        std::vector<OperatorID> ops_current_step;
        std::set<std::pair<int, int> > remaining_marked;
		std::set<std::pair<int, int> > new_marked;
            
        bool change = true;
        while (change){
            change = false;
                
            new_marked.clear();
            for (auto const &fact : marked_facts){
#ifdef DEBUG_PLAN_EXTRACTION
                cout << "handling fact " << g_fact_names[fact.first][fact.second];
#endif
                auto op_id = best_supporters[step][fact.first][fact.second];
                if (op_id.get_index() == -1){
#ifdef DEBUG_PLAN_EXTRACTION                        
                    cout << " => cannot be achieved in this step" << endl;
#endif
                    remaining_marked.insert(fact);
                    continue;
                }
				const auto &op = operators[op_id.get_index()];
#ifdef DEBUG_PLAN_EXTRACTION
                cout << endl << "best supporter: " << op->get_name() << endl;;
#endif
                    
                if (std::find(ops_current_step.begin(), ops_current_step.end(), op_id) == ops_current_step.end()){
                    ops_current_step.push_back(op_id);
                    for (auto const &pre : op.get_red_preconditions()) {
                        change |= new_marked.insert(std::make_pair(pre->var, pre->val)).second;
#ifdef DEBUG_PLAN_EXTRACTION
                        cout << "marked precondition fact " << g_fact_names[pre->var][pre->val] << endl;
#endif
                    }
                }
            }
            marked_facts.swap(new_marked);
        }
        marked_facts.insert(remaining_marked.begin(), remaining_marked.end());
            
            
        // sequence the red operators so that they form a valid relaxed plan
	    std::vector<std::vector<bool> > current_red_state(g_variable_domain.size());
        for (size_t var = 0; var < g_variable_domain.size(); ++var){
            if (goal_state.get_painting().is_red_var(var)){
                current_red_state[var].resize(g_variable_domain[var], false);
            }
        }

	    std::vector<const redblack::RBOperator *> sorted_ops;
	    std::vector<bool> handled_ops(ops_current_step.size(), false);
        while (sorted_ops.size() < ops_current_step.size()){
            for (size_t i = 0; i < ops_current_step.size(); ++i){
                if (handled_ops[i]){
                    continue;
                }
                const redblack::RBOperator *op = &operators[ops_current_step[i].get_index()];
#ifdef DEBUG_PLAN_EXTRACTION                        
                cout << "handling op " << op->get_name();
#endif                
                bool is_applicable = true;
                for (auto const &pre : op->get_red_preconditions()){
                    if (!current_red_state[pre->var][pre->val] && best_supporters[step][pre->var][pre->val].get_index() != -1){
                        is_applicable = false;
#ifdef DEBUG_PLAN_EXTRACTION                        
                        cout << " not applicable => keep for later";
#endif
                        break;
                    }
                }
                if (is_applicable){
                    handled_ops[i] = true;
                    sorted_ops.push_back(op);
                    for (auto const &eff : op->get_red_effects()){
                        current_red_state[eff->var][eff->val] = true;
                    }
                }
            }
        }
            
        plan.insert(plan.end(), sorted_ops.rbegin(), sorted_ops.rend());
        if (step != 0){
            const redblack::RBOperator *black_op = path[step - 1];
#ifdef DEBUG_PLAN_EXTRACTION
            cout << "appended black action " << black_op->get_name() << endl;
#endif
            plan.push_back(black_op);
            for (auto const &pre : black_op->get_red_preconditions()){
                marked_facts.insert(std::make_pair(pre->var, pre->val));
            }
            for (auto const &eff : black_op->get_red_effects()){
                marked_facts.erase(std::make_pair(eff->var, eff->val));
            }
        }
    }
        
    path = plan;
        
    reverse(path.begin(), path.end());
        
#ifdef DEBUG_PLAN_EXTRACTION
    cout << endl;
    for (auto const &op : path){
        cout << op->get_name() << endl;
    }
    cout << endl;
#endif
        
#ifndef NDEBUG
    size_t number_red = 0;
    for (size_t var = 0; var < g_variable_domain.size(); ++var){
        if (goal_state.get_painting().is_red_var(var)){
            ++number_red;
        }
    }
    // only init state fact must be marked in the end
    // this is kind of a weak condition, it still allows multiple marked
    // facts for a single red variable in case there are others that are
    // not used at all
    assert(marked_facts.size() <= number_red);
#endif        
    //assert(is_redblack_plan(path));
        
    //if (g_painting->is_incremental_search() && is_real_plan(path)){
    //    g_rb_plan_is_real_plan = true;
    //}
}
