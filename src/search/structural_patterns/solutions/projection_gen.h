#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_PROJECTION_GEN_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_PROJECTION_GEN_H

#include "../abstractions/var_projection.h"
#include <cfloat>
#include "solution_method.h"

class Projection: public SolutionMethod {
	int num_vars;
	std::vector<int> multiplier;

protected:
	void set_distances(double **sol, std::vector<int> free_vars, std::vector<int> state, const std::vector<int>& doms, DOperator* op);
	void set_goal_distances(double **sol, std::vector<int> free_vars, std::vector<int> state, const std::vector<int>& doms, int last);

	int number_of_d_variables, number_of_h_variables, number_of_w_var_variables;
public:
	Projection();
	Projection(GeneralAbstraction* abs);
	Projection(std::vector<int>& pattern);
	virtual ~Projection();

	virtual void initiate();
	virtual void solve();
	virtual double get_solution_value(const SPState &state);

	int d_var(const SPState &state) const;
	int d_var(std::vector<int>& state) const;

	virtual int get_num_vars() const;

};

#endif /* PROJECTION_GEN_H */
