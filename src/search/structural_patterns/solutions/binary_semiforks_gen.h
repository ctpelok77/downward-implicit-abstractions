#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_BINARY_SEMIFORKS_GEN_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_BINARY_SEMIFORKS_GEN_H

#include "../abstractions/binary_semiforks.h"
#include "../abstractions/one_dep_binary_hourglasses.h"

#include <cfloat>
#include <iostream>
#include "../solution.h"
#include "solution_method.h"


/* This is a general class for a binary semifork abstraction, used by online, offline and LP solutions
 */

class BinarySemiFork: public SolutionMethod {
	std::vector<int> lower_bound, upper_bound;
	int* sigma_bounds;
//	Solution* hat_solution;
	std::vector<Solution*> hat_solutions;

	void solve_hat();
	void solve_fork();
	void set_distances(double **sol, std::vector<int> free_vars, std::vector<int> state, const std::vector<int>& doms, DOperator* op, int changes, int theta) const;
//	void set_goal_distances(double **sol, std::vector<int> free_vars, std::vector<int> state, const std::vector<int>& doms, int changes, int theta) const;
	void set_base_goal_distances(double **sol, std::vector<int> free_vars, std::vector<int> state, const std::vector<int>& doms, int theta) const;
	Solution* solve_step(const std::vector<std::vector<int> >& states, int changes) const;

	void clear_hat_solutions();

protected:
	int sigma_size;
	int num_leafs;
	std::vector<int> multiplier;
	int num_hat_states;
	std::vector<int> hat_vars;
	std::vector<int> leaf_vars;
	int sigma_step;
	std::vector<bool> is_in_hat;

	int number_of_d_variables, number_of_p_variables, number_of_h_variables;
	int number_of_w_r_variables, number_of_w_v_variables, number_of_w_var_variables;
	int number_of_m_variables;

	int get_abs_root() const {return abstraction->get_abstract_root();}

public:
	BinarySemiFork();
	BinarySemiFork(GeneralAbstraction* abs);
//	BinarySemiFork(SemiForksAbstraction* f, Domain* abs_domain, std::vector<int>& hat_vars);
	BinarySemiFork(SemiForksAbstraction* f, Domain* abs_domain);
	BinarySemiFork(OneDependentHourglassesAbstraction* hg, Domain* abs_domain);

	virtual ~BinarySemiFork();

	virtual void initiate();
	virtual void solve();
	virtual void solve(const SPState &) {};

	virtual double get_h_val(int sigma, const SPState &eval_state) const;
	virtual double get_solution_value(const SPState &state);
	int d_var(int var, int val, int i, int theta = 0) const;
	int p_var(int var, int val1, int val2, int root_val) const;
	virtual int h_var() const;
	int w_r(int root_zero) const;

	virtual void set_default_number_of_variables();
	virtual void dump_number_of_variables() const;

	virtual int get_num_vars() const;
	int bound_ind(int var, int val, int theta) const;

	virtual int get_sigma_lower_bound(int var, int val, int theta) const;
	virtual int get_sigma_upper_bound(int var, int val, int theta) const;

	void set_sigma_lower_bounds_default() {lower_bound.assign(2*num_leafs*sigma_size,1);}

	void set_sigma_lower_bound(int var, int val, int theta, int bound) {lower_bound[bound_ind(var, val, theta)] = bound;}
	void set_sigma_upper_bound(int var, int val, int theta, int bound) {upper_bound[bound_ind(var, val, theta)] = bound;}

	int short_m_var(const SPState &state, int theta) const;

	virtual void dump_solution() const {
		std::cout << "------------------------------------------------------------" << std::endl;
		SolutionMethod::dump_solution();

		int m_add1 = number_of_d_variables + number_of_p_variables + number_of_h_variables +
				number_of_w_r_variables + number_of_w_v_variables;
		int m_add = m_add1 + get_mapping()->get_abstract()->get_actions_number();
		std::cout << "Action variables" << std::endl;
		const std::vector<DOperator*> &A = get_mapping()->get_abstract()->get_operators();
		for (size_t a=0; a < A.size(); ++a) {
			std::cout << "x_" << m_add1 + get_abstraction_index() + A[a]->get_index() << " = " << A[a]->get_double_cost() << std::endl;
		}

		std::cout << "Hat variables" << std::endl;
		for (int num=0;num<2*num_hat_states*sigma_size;num++) {
			int theta = num % 2;
			int changes = ((num - theta) / 2) % sigma_size;
			int st = (num - 2*changes - theta) / (2*sigma_size);
//			std::cout << "Test: 2*" << sigma_size << "*" << st << " + 2*" << changes << " + " << theta << " = " << num << std::endl;

			double val = hat_solutions[changes]->get_value(2*st + theta);
			if (-1.0 == val)
				continue;
			std::cout << "x_" << m_add + get_abstraction_index() + 2*sigma_size*st + 2*changes + theta << " = " << val << std::endl;
		}

/*
		for (size_t changes=0; changes < hat_solutions.size(); ++changes) {

			for (int st=0;st<num_hat_states;st++) {
				for (int theta=0;theta<2;theta++) {
					double val = hat_solutions[changes]->get_value(2*st + theta);
					if (-1.0 == val)
						continue;
					std::cout << "x_" << m_add + 2*sigma_size*st + 2*changes + theta << " = " << val << std::endl;
				}

			}

		}
*/
	}

};



#endif /* BINARY_SEMIFORKS_GEN_H */
