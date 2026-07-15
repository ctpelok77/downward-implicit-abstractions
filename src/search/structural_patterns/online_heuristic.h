#ifndef STRUCTURAL_PATTERNS_ONLINEHEURISTIC_H
#define STRUCTURAL_PATTERNS_ONLINEHEURISTIC_H

#include <vector>

#include "abstractions/forks_abstraction.h"
#include "SP_heuristic.h"
#include "problem.h"
#include "abstractions/general_abstraction.h"
#include "abstractions/var_projection.h"
#include "mappings/var_proj_mapping.h"
#include "mappings/mapping.h"
#include "abstractions/domain_abstraction.h"

#include <iostream>

class Problem;

/*
 *      This class is intended for solving online without LP solver.
 *      Specifying cost partitioning allows to solve each ensemble member individually.
 */

class OnlineHeuristic: public SPHeuristic {

protected:
	int num_LP_vars(const SPState &state);

	virtual SolutionMethod* add_pattern(std::vector<int>& pattern);

	virtual SolutionMethod* add_binary_fork(GeneralAbstraction* abs);
	virtual SolutionMethod* add_bounded_inverted_fork(GeneralAbstraction* abs);
	virtual SolutionMethod* add_pattern(GeneralAbstraction* abs);

	virtual SolutionMethod* add_binary_fork(ForksAbstraction* fork, Domain* abs_domain);
	virtual SolutionMethod* add_bounded_inverted_fork(IforksAbstraction* ifork, Domain* abs_domain);

public:
	OnlineHeuristic(const Options &options);
	OnlineHeuristic(const Options &options, const Problem* prob);
	virtual ~OnlineHeuristic();
	virtual void print_statistics() const {};

};

#endif /* ONLINEHEURISTIC_H */
