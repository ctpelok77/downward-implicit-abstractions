#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_BINARY_ONE_DEP_HOURGLASSES_GEN_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_BINARY_ONE_DEP_HOURGLASSES_GEN_H

#include "../abstractions/one_dep_binary_hourglasses.h"
#include "binary_semiforks_gen.h"
#include <cfloat>
#include "../solution.h"
#include "solution_method.h"


/* This is a general class for a binary hourglass abstraction, used by online, offline and LP solutions
 */

class Hat {
	std::vector<int> vars;
	std::vector<int> multiplier;
	int num_hat_states;
	std::vector<Solution*> hat_solutions;

public:
    Hat(const std::vector<int>& par, const std::vector<int>& mult, int num)
    	: vars(par), multiplier(mult), num_hat_states(num) {
    }
	virtual ~Hat() {
		clear_hat_solutions();
	}
    void set_solutions(const std::vector<Solution*>& sols) { hat_solutions = sols; }
    double get_solution_value(int changes, int var) const {
    	std::cout << "Getting value for " << changes << " changes out of " << hat_solutions.size() << " possible values" << std::endl;
    	return hat_solutions[changes]->get_value(var); }
    const std::vector<int>& get_vars() const { return vars; }
    int get_num_hat_states() const { return num_hat_states; }

    void clear_hat_solutions() {
		for (size_t i=0; i < hat_solutions.size(); ++i) {
			if (hat_solutions[i])
				delete hat_solutions[i];
		}
		hat_solutions.clear();
	}

};

class BinaryOneDepHourglass: public SolutionMethod {
	std::vector<Solution*> parent_solutions;
	std::vector<int> lower_bound, upper_bound;
	int* sigma_bounds;

	void solve_hats();
	void solve_fork();
//	void set_distances(double **sol, std::vector<int> free_vars, std::vector<int> state, const std::vector<int>& doms, DOperator* op, int changes, int theta) const;
//	void set_goal_distances(double **sol, std::vector<int> free_vars, std::vector<int> state, const std::vector<int>& doms, int changes, int theta) const;
//	void set_base_goal_distances(double **sol, std::vector<int> free_vars, std::vector<int> state, const std::vector<int>& doms, int theta) const;
//	Solution* solve_step(const std::vector<std::vector<int> >& states, int changes) const;
	Solution* solve_parent(int var) const;
	void solve_parents();
	void solve_using_parents(Hat* parents);
	void clear_hat_solutions();
	void clear_parent_solutions();

	double get_solution_value_for_hat(const Hat* hat, const SPState &state);
	double get_parents_value(const SPState &state) const;
	double get_parents_value(const Hat* hat, const SPState &state) const;

//	int short_m_var(int x, int y, int changes, int theta) const;
	int short_m_var(int x, int y, int theta) const;

	int short_par_var(int dom, int x, int y) const;

protected:
	int num_leafs;
	std::vector<int> parent_vars;
	std::vector<int> leaf_vars;
	std::vector<bool> is_parent;

	int sigma_size;
//	std::vector<int> parent_domains;
	std::vector<Hat*> parent_hats;
	int parent_max_domain;
//	std::vector<std::vector<int> > parent_pairs; // These three vectors are of the same size.
//	std::vector<std::vector<int> > multiplier;  // Inner vector consisting of two values: 1 and the first domain
//	std::vector<int> num_hat_states;      // Multiplication of the two domains for each pair
	int sigma_step;

	int number_of_d_variables, number_of_p_variables, number_of_h_variables;
	int number_of_w_r_variables, number_of_w_v_variables, number_of_w_var_variables;
	int number_of_m_variables;

	int get_abs_root() const {return abstraction->get_abstract_root();}

public:
	BinaryOneDepHourglass();
	BinaryOneDepHourglass(GeneralAbstraction* abs);
	BinaryOneDepHourglass(OneDependentHourglassesAbstraction* f, Domain* abs_domain);
	virtual ~BinaryOneDepHourglass();

	virtual void initiate();
	virtual void solve();
	virtual void solve(const SPState &) {};

	virtual double get_solution_value(const SPState &state);
	int d_var(int var, int val, int i, int theta = 0) const;
	int p_var(int var, int val1, int val2, int root_val) const;
	virtual int h_var() const;
	int w_r(int root_zero) const;

	virtual int get_num_vars() const;

	virtual double get_h_val(const Hat* hat, int sigma, const SPState &eval_state) const;

	virtual void set_default_number_of_variables();
	virtual void dump_number_of_variables() const;

	int bound_ind(int var, int val, int theta) const;

	virtual int get_sigma_lower_bound(int var, int val, int theta) const;
	virtual int get_sigma_upper_bound(int var, int val, int theta) const;

	void set_sigma_lower_bounds_default() {lower_bound.assign(2*num_leafs*sigma_size,1);}

	void set_sigma_lower_bound(int var, int val, int theta, int bound) {lower_bound[bound_ind(var, val, theta)] = bound;}
	void set_sigma_upper_bound(int var, int val, int theta, int bound) {upper_bound[bound_ind(var, val, theta)] = bound;}

//	int short_m_var(const state_var_t* state, int theta) const;
//	int short_m_var(const std::vector<int>& state, int theta) const;

};



#endif /* BINARY_ONE_DEP_HOURGLASSES_GEN_H */
