#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_BINARY_FORKS_GEN_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_BINARY_FORKS_GEN_H

#include "../abstractions/binary_forks.h"
#include <cfloat>
#include "../solution.h"
#include "solution_method.h"


/* This is a general class for a binary fork abstraction, used by online, offline and LP solutions
 */

class BinaryFork: public SolutionMethod {
	std::vector<int> lower_bound, upper_bound;
	int sigma_step;
	int* sigma_bounds;

protected:
	int sigma_size;
	int num_leafs;

	int number_of_d_variables, number_of_p_variables, number_of_h_variables;
	int number_of_w_r_variables, number_of_w_v_variables, number_of_w_var_variables;

public:
	BinaryFork();
	BinaryFork(GeneralAbstraction* abs);
	BinaryFork(ForksAbstraction* f, Domain* abs_domain);
	virtual ~BinaryFork();

	virtual void initiate();
	virtual void solve();
	virtual void solve(const SPState &state);

	virtual double get_h_val(int sigma, const SPState &eval_state) const;
	virtual double get_solution_value(const SPState &state);
	int d_var(int var, int val, int i, int theta = 0) const;
	int p_var(int var, int val1, int val2, int root_val) const;
	virtual int h_var() const;
	int w_r(int root_zero) const;

	virtual void set_default_number_of_variables();

	virtual int get_num_vars() const;
	int bound_ind(int var, int val, int theta) const;

	int get_sigma_lower_bound(int var, int val, int theta) const;
	int get_sigma_upper_bound(int var, int val, int theta) const;

	void set_sigma_lower_bounds_default() {lower_bound.assign(2*num_leafs*sigma_size,1);}

	void set_sigma_lower_bound(int var, int val, int theta, int bound) {lower_bound[bound_ind(var, val, theta)] = bound;}
	void set_sigma_upper_bound(int var, int val, int theta, int bound) {upper_bound[bound_ind(var, val, theta)] = bound;}


};



#endif /* BINARY_FORKS_GEN_H */
