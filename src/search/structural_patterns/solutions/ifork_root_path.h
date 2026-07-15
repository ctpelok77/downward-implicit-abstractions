#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_IFORK_ROOT_PATH_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_IFORK_ROOT_PATH_H

#include <vector>
#include "../d_operator.h"
#include <stdio.h>
#include "../SP_LPConstraint.h"

class IforkRootPath {
	std::vector<DOperator*> path;
	std::vector<int> first_needed;
	std::vector<std::pair<int,int> > first_needed_pairs;
	double cost;
	int num_vars;
	// For LP
	SPLPConstraint* lpc;
	bool is_borrowed_lpc;
public:
	IforkRootPath();
	IforkRootPath(int sz);
	IforkRootPath(IforkRootPath* cp);

	virtual ~IforkRootPath();

	const std::vector<DOperator*>& get_path() const {return path;}
	void set_path(std::vector<DOperator*>& p);

	int get_first_needed(int v) const;
	void set_first_needed(int v, int val);
	void set_num_vars(int vars);
	int get_num_vars();
	int get_path_size() const;
	void get_path_support(int v, std::vector<int>& support) const;

	void get_applicable_vals(int root_bound, std::vector<int>& vals) const;
	bool is_dominated(IforkRootPath* path_b) const;

	double get_path_cost() const;
	double get_needed_cost() const;
	void set_needed_cost(double c);

	void set_first_needed_pairs();
//	void get_first_needed_pairs(std::vector<std::pair<int,int> >& pairs);
	const std::vector<std::pair<int,int> > &get_first_needed_pairs() const {return first_needed_pairs;}

	SPLPConstraint* get_LP_constraint();
	void set_LP_constraint(SPLPConstraint* c);
	void clear_path_actions() {path.clear();}

	void dump() const;
};

#endif /* IFORK_ROOT_PATH_H */
