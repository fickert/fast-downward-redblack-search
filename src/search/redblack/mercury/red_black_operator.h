#ifndef RED_BLACK_RED_BLACK_OPERATOR_H
#define RED_BLACK_RED_BLACK_OPERATOR_H


#include <vector>
#include <set>

#include "../operator.h"

#include <cassert>

typedef std::pair<int, int> assignment;
typedef std::set<assignment> partial_assignment;
typedef std::pair<partial_assignment, partial_assignment> sas_action;


class RedBlackOperator {
	partial_assignment red_precondition, black_precondition, red_effect, black_effect;
	int op_no;
public:
	RedBlackOperator(int _op_no);
	virtual ~RedBlackOperator();

	void set_black_pre_eff(const std::vector<bool>& black_vars);
	const partial_assignment& get_red_precondition() const { return red_precondition;}
	const partial_assignment& get_black_precondition() const { return black_precondition;}
	const partial_assignment& get_red_effect() const { return red_effect;}
	const partial_assignment& get_black_effect() const { return black_effect;}


	bool is_red_applicable(const std::vector<int> &values) const;
	bool is_applicable(const std::vector<int> &values) const;
	bool is_applicable(const GlobalState& state) const;
	void apply(std::vector<int> &values) const;
	void dump() const;
	int get_op_no() const { return op_no; }
};

typedef const RedBlackOperator* sas_operator;

#endif
