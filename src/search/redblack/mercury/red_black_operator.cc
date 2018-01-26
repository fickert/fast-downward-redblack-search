#include "red_black_operator.h"

#include "../../globals.h"

using namespace std;


RedBlackOperator::RedBlackOperator(int _op_no) : op_no(_op_no) {
	reset();
}


RedBlackOperator::~RedBlackOperator() {
	red_precondition.clear();
	red_effect.clear();
	black_precondition.clear();
	black_effect.clear();
}

void RedBlackOperator::reset() {
	// Filling all pre and effects as red
	const GlobalOperator *op = &g_operators[op_no];
	for (const auto &precondition : op->get_preconditions())
		red_precondition.insert({ precondition.var, precondition.val });
	black_precondition.clear();
	for (const auto &effect : op->get_effects())
		red_effect.insert({ effect.var, effect.val });
	black_effect.clear();
}

void RedBlackOperator::set_black_pre_eff(const vector<bool>& black_vars) {
	// Separating black effects from red
	partial_assignment temp_prec;
	temp_prec.swap(red_precondition);
	for (partial_assignment::iterator it=temp_prec.begin(); it != temp_prec.end(); ++it) {
		int var = (*it).first;
		if (black_vars[var])
			black_precondition.insert(*it);
		else
			red_precondition.insert(*it);
	}

	partial_assignment temp_eff;
	temp_eff.swap(red_effect);
	for (partial_assignment::iterator it=temp_eff.begin(); it != temp_eff.end(); ++it) {
		int var = (*it).first;
		if (black_vars[var])
			black_effect.insert(*it);
		else
			red_effect.insert(*it);
	}

}

bool RedBlackOperator::is_red_applicable(const std::vector<std::vector<int>> &values) const {
	for (partial_assignment::iterator it=red_precondition.begin(); it != red_precondition.end(); ++it) {
		if (!std::binary_search(std::begin(values[it->first]), std::end(values[it->first]), it->second))
			return false;
	}
	return true;
}

bool RedBlackOperator::is_applicable(const std::vector<std::vector<int>> &values) const {
	for (partial_assignment::iterator it = red_precondition.begin(); it != red_precondition.end(); ++it) {
		if (!std::binary_search(std::begin(values[it->first]), std::end(values[it->first]), it->second))
			return false;
	}
	for (partial_assignment::iterator it = black_precondition.begin(); it != black_precondition.end(); ++it) {
		if (!std::binary_search(std::begin(values[it->first]), std::end(values[it->first]), it->second))
			return false;
	}
	return true;
}

bool RedBlackOperator::is_applicable(const GlobalState& state) const {
	for (partial_assignment::iterator it=red_precondition.begin(); it != red_precondition.end(); ++it) {
		if (state[(*it).first] != (*it).second)
			return false;
	}
	for (partial_assignment::iterator it=black_precondition.begin(); it != black_precondition.end(); ++it) {
		if (state[(*it).first] != (*it).second)
			return false;
	}
	return true;
}

void RedBlackOperator::apply(std::vector<std::vector<int>> &values, const std::vector<bool> *outside_red_variables) const {
	for (partial_assignment::iterator it=red_effect.begin(); it != red_effect.end(); ++it) {
		if (!outside_red_variables || !outside_red_variables->at(it->first)) {
			values[it->first] = {it->second};
		} else {
			if (!std::binary_search(std::begin(values[it->first]), std::end(values[it->first]), it->second)) {
				values[it->first].push_back(it->second);
				std::inplace_merge(std::begin(values[it->first]), std::end(values[it->first]) - 1, std::end(values[it->first]));
			}
		}
	}
	for (partial_assignment::iterator it=black_effect.begin(); it != black_effect.end(); ++it) {
		values[it->first] = {it->second};
	}
}

void RedBlackOperator::dump() const {
	cout << "< red: ";
	for (partial_assignment::iterator it=red_precondition.begin(); it != red_precondition.end(); ++it) {
		cout << "[" << g_variable_name[(*it).first] << " : " << (*it).second << "] ";
	}
	cout << ", black: ";
	for (partial_assignment::iterator it=black_precondition.begin(); it != black_precondition.end(); ++it) {
		cout << "[" << g_variable_name[(*it).first] << " : " << (*it).second << "] ";
	}
	cout << " | red: ";
	for (partial_assignment::iterator it=red_effect.begin(); it != red_effect.end(); ++it) {
		cout << "[" << g_variable_name[(*it).first] << " : " << (*it).second << "] ";
	}
	cout << ", black: ";
	for (partial_assignment::iterator it=black_effect.begin(); it != black_effect.end(); ++it) {
		cout << "[" << g_variable_name[(*it).first] << " : " << (*it).second << "] ";
	}
    cout << " >" << endl;
}
