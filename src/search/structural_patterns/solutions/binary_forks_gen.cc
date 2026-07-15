// globals removed — use TaskProxy
// timer removed

#include "../SP_globals.h"
#include "binary_forks_gen.h"
#include "../mappings/mapping.h"
#include <math.h>
#include <limits>

using namespace std;

BinaryFork::BinaryFork() :SolutionMethod() {
	set_abstraction(new BinaryForksAbstraction());
}

BinaryFork::BinaryFork(GeneralAbstraction* abs) :SolutionMethod() {
	set_abstraction(abs);
}


BinaryFork::BinaryFork(ForksAbstraction* f, Domain* abs_domain) :SolutionMethod() {
	set_abstraction(new BinaryForksAbstraction(f, abs_domain));
}


BinaryFork::~BinaryFork() {

}

void BinaryFork::initiate() {
	cout << "Initializing Fork with";
	if (!use_cache)
		cout << "out";
	cout << " caching for abstract states" << endl;

	const Problem* abs = get_mapping()->get_abstract();
	const vector<int> &doms = abs->get_variable_domains();

	int max_domain_size = 0;


	num_leafs = doms.size() - 1;
	if (0 == num_leafs){
		sigma_size = 2;
	} else {
		for (int i = 1; i<=num_leafs; i++)
			if (max_domain_size < doms[i])
				max_domain_size = doms[i];

		sigma_size = max_domain_size+1;
	}
	// The bounds on sigma
	lower_bound.assign(2*num_leafs*sigma_size,-1);
	upper_bound.assign(2*num_leafs*sigma_size,-1);
	// Setting the solution
	set_solution(new Solution());
	// Initializing variables holding the number of variables for each type
	set_default_number_of_variables();

	// Setting the step of the sigma
	if (-1 != abs->get_goal_val(0)){ // There is a goal on the root -- go over all even/odd possibilities for sigma
		sigma_step = 2;
	} else {
		sigma_step = 1;
	}
	sigma_bounds = new int[num_leafs+1];
}

void BinaryFork::set_default_number_of_variables( ) {
	number_of_d_variables = d_vars_multiplier*num_leafs*sigma_size*sigma_size;
	number_of_p_variables = 2*num_leafs*sigma_size*sigma_size;
	number_of_h_variables = 1;
	number_of_w_r_variables = 2;
	number_of_w_v_variables = 0;
	number_of_w_var_variables = 0;
}

void BinaryFork::solve( ) {

//	cout << "Start Solving " << g_timer << endl;
	const Problem* abs = get_mapping()->get_abstract();
	const vector<int> &doms = abs->get_variable_domains();
	int root_dom = doms[0];
	assert(root_dom==2);
	// Freed in the end of this function
	double* sol = new double[get_num_vars()];
	solution->clear_solution();
//	cout << "Start calculating p and d values " << g_timer << endl;
	//For each leaf v (the first variable is always the root)
	int var_num = doms.size();
	for (int v = 1; v < var_num; v++) {
//		cout << "Start calculating p and d values for variable " << v << " " << g_timer << endl;
		int dom_size = doms[v];

		for (int val0=0;val0<dom_size;val0++) {  // Initialize
			for (int val1=0;val1<dom_size;val1++) {
				for (int theta=0;theta<root_dom;theta++) {
					int p_ind = p_var(v,val0,val1,theta);
					if (val0 == val1) {
						sol[p_ind] = 0.0;
					} else {
						sol[p_ind] = infinity;
					}
				}
			}
		}
//		cout << "End first setting initial p(v,val0,val1, theta) for variable " << v << " " << g_timer << endl;

		const vector<DOperator*> &A_v = abs->get_var_actions(v);
		int A_v_size = A_v.size();
		// Initial cost matrix
		for (int a = 0; a < A_v_size; a++) {
			//int prv = A_v[a]->get_prevail_val(0);
			//int pre = A_v[a]->get_pre_val(v);
			//int post = A_v[a]->get_post_val(v);
			double c = A_v[a]->get_double_cost();

			// For conditional effects
			vector<PrePost> pre_v;
			A_v[a]->get_explicit_pre_post_for_var(v,pre_v);

			assert(pre_v.size() > 0);
			for (size_t p=0; p < pre_v.size(); ++p) {
				int start_val = (-1 == pre_v[p].pre) ? 0 : pre_v[p].pre;
				int end_val = (-1 == pre_v[p].pre) ? dom_size-1 : pre_v[p].pre;

				// fork - at most one parent.
				assert(pre_v[p].cond.size() == 0 ||
						(pre_v[p].cond.size() == 1 && pre_v[p].cond[0].var == 0));
				int start_theta = (0 == pre_v[p].cond.size()) ? 0 : pre_v[p].cond[0].prev;
				int end_theta = (0 == pre_v[p].cond.size()) ? doms[0]-1 : pre_v[p].cond[0].prev;

				for (int val=start_val;val<=end_val;val++) {
					if (val != pre_v[p].post) {
						for (int theta=start_theta;theta<=end_theta;theta++) {
							int p_ind = p_var(v,val,pre_v[p].post,theta);
							sol[p_ind] = min(sol[p_ind],c);
						}
					}
				}
			}
		}
//		cout << "End setting initial p(v,val0,val1, theta) for variable " << v << " with domain " << dom_size << " " << g_timer << endl;

		//Calculating shortest paths
		// It seems like this way is faster...
		for (int k=0;k<dom_size;k++) {
			for (int i=0;i<dom_size;i++) {
				int p_indik0 = p_var(v,i,k,0);
				int p_indik1 = p_var(v,i,k,1);
				for (int j=0;j<dom_size;j++) {
					int p_indij0 = p_var(v,i,j,0);
					int p_indkj0 = p_var(v,k,j,0);
					int p_indij1 = p_var(v,i,j,1);
					int p_indkj1 = p_var(v,k,j,1);

					sol[p_indij0] = min(sol[p_indij0], sum_infinity(sol[p_indik0],sol[p_indkj0]));
					sol[p_indij1] = min(sol[p_indij1], sum_infinity(sol[p_indik1],sol[p_indkj1]));
				}
			}
		}

//		cout << "End calculating p(v,val0,val1, theta) for variable " << v << " " << g_timer << endl;

		int g_v = abs->get_goal_val(v);
		// Setting the initial values for dynamic calculation of d() variables
		for (int val_0=0; val_0 < dom_size; val_0++){
			for (int theta=0; theta < root_dom; theta++){
				// d(v, val_0, 1, theta) = p(v, val_0, G'[v], theta)
				int d_ind = d_var(v,val_0,1,theta);
				int p_ind = p_var(v,val_0,g_v,theta);
				sol[d_ind] = sol[p_ind];
			}
		}
//		cout << "Start first step calculation for variable " << v << " " << g_timer << endl;

		for (int sz=2; sz <= dom_size+1; sz++){
			for (int val_0=0; val_0 < dom_size; val_0++){
				for (int theta=0; theta < root_dom; theta++){
					int d_ind = d_var(v,val_0,sz,theta);
					sol[d_ind] = infinity;
					// d(v, val_0, sz, theta) <= d(v, val_1, sz-1,1-theta) + p(v, val_0, val_1, theta)
					for (int val_1=0; val_1 < dom_size; val_1++){
						int d_ind1 = d_var(v,val_1,sz-1,1-theta);
						int p_ind1 = p_var(v,val_0,val_1,theta);
						sol[d_ind] = min(sol[d_ind], sum_infinity(sol[d_ind1],sol[p_ind1]));
					}
				}
			}
		}

//		cout << "Start extra step calculation for variable " << v << " " << g_timer << endl;

		// Making the extra step
		for (int sz=dom_size+2; sz <= sigma_size; sz++){
			for (int val_0=0; val_0 < dom_size; val_0++){
				for (int theta=0; theta < root_dom; theta++){
					// d(v, val_0, sz,theta) = d(v, val_0, sz-1,theta)
					int d_ind20 = d_var(v,val_0,sz,theta);
					int d_ind21 = d_var(v,val_0,sz-1,theta);
					sol[d_ind20] = sol[d_ind21];
				}
			}
		}
//		cout << "End calculating d values for variable " << v << " " << g_timer << endl;

	}
//	cout << "End calculating p and d values " << g_timer << endl;

/*    This part comes to deal with economy in database size, and, maybe calculation time
 *        In fact, it can increase the calculation time of small instances                    */
	// Finding for each variable and each value the lower and upper bounds on the parent support (for each parent value).
	for (int v = 1; v < var_num; v++) {
		for (int theta=0; theta < root_dom; theta++){
			int dom_size = doms[v];
			int g_v = abs->get_goal_val(v);
			for (int val0=0; val0<dom_size; val0++) {
				int min_sigma = 1;
				int max_sigma = sigma_size;
				if (val0 == g_v) {
					max_sigma = 1;
				} else {
					// Find lower bound
				    while((min_sigma <= sigma_size) && (sol[d_var(v,val0,min_sigma,theta)] == infinity))
				    	min_sigma++;

				    // Find upper bound
				    double max_val = sol[d_var(v,val0,sigma_size,theta)];
				    while((max_sigma > 0) && (sol[d_var(v,val0,max_sigma,theta)] == max_val))
				    	max_sigma--;

				    max_sigma++;
				}
				// Save the bounds for v, val0, theta.
				int ind = bound_ind(v,val0,theta);
				lower_bound[ind] = min_sigma;
				upper_bound[ind] = max_sigma;
				// Keep the needed entries only
				for (int sz = min_sigma; sz <= max_sigma; sz++) {
					int d_ind = d_var(v,val0,sz,theta);
					solution->set_value(d_ind, sol[d_ind]);
				}
			}
		}
	}
//	cout << "End calculating min and max sigma and setting solution for d and p " << g_timer << endl;

	const vector<DOperator*> &A_r = abs->get_var_actions(0);
	// Dividing root changing actions into two sets, by the post value
	double min0 = infinity;
	double min1 = infinity;

	for (size_t a=0; a < A_r.size(); ++a) {
		double c = A_r[a]->get_double_cost();

		// For conditional effects
		vector<PrePost> pre_r;
		A_r[a]->get_explicit_pre_post_for_var(0, pre_r);

		assert(pre_r.size() > 0);
		for (size_t p=0; p < pre_r.size(); ++p) {
			assert(pre_r[p].cond.size() == 0);
			int pre = pre_r[p].pre;
			int post = pre_r[p].post;

			// If the action does nothing...
			if (pre == post)
				continue;

			if (0 == post) {
				min0 = (min0 < c) ? min0 : c;
			} else {
				min1 = (min1 < c) ? min1 : c;
			}
		}
	}

	int w_ind0 = w_r(0);
	int w_ind1 = w_r(1);

	solution->set_value(w_ind0, min0);
	solution->set_value(w_ind1, min1);
//	solution->dump();
//	cout << "End setting solution for w " << g_timer << endl;
	// Freeing the allocated temporal solution array.
	delete [] sol;
//	cout << "End deallocating " << g_timer << endl;
}


void BinaryFork::solve(const SPState &state) {
	Mapping* map = get_mapping();
	const Problem* abs = map->get_abstract();
	const vector<int> &doms = abs->get_variable_domains();

	solution->clear_solution();
	// Freed in the end of this function
	double* sol = new double[get_num_vars()];
	const SPState &abstract_state = get_abstract_state(state);

	int root_zero = abstract_state[0];

	assert(doms[0]==2);

	//For each leaf v (the first variable is always the root)
	int var_num = doms.size();
	for (int v = 1; v < var_num; v++) {

		int dom_size = doms[v];
		int s_v = abstract_state[v];

		const vector<DOperator*> &A_v = abs->get_var_actions(v);
		int A_v_size = A_v.size();

		// Calculating the distances in DTGs of the leaves.
		for (int val0=0;val0<dom_size;val0++) {  // Initialize
			for (int val1=0;val1<dom_size;val1++) {
				for (int theta=0;theta<doms[0];theta++) {
					int p_ind = p_var(v,val0,val1,theta);
					if (val0 == val1) {
						sol[p_ind] = 0.0;
					} else {
						sol[p_ind] = infinity;
					}
				}
			}
		}


		// Initial cost matrix
		for (int a = 0; a < A_v_size; a++) {
//			int root_prv = A_v[a]->get_prevail_val(0);
//			int prv_v = A_v[a]->get_prevail_val(v);
			double c = A_v[a]->get_double_cost();

			// For conditional effects
			vector<PrePost> pre_v;
			A_v[a]->get_explicit_pre_post_for_var(v,pre_v);
			assert(pre_v.size() > 0);
			for (size_t p=0; p < pre_v.size(); ++p) {
				int start_val = (-1 == pre_v[p].pre) ? 0 : pre_v[p].pre;
				int end_val = (-1 == pre_v[p].pre) ? dom_size-1 : pre_v[p].pre;

				// fork - at most one parent.
				assert(pre_v[p].cond.size() == 0 ||
						(pre_v[p].cond.size() == 1 && pre_v[p].cond[0].var == 0));
				int start_theta = (0 == pre_v[p].cond.size()) ? 0 : pre_v[p].cond[0].prev;
				int end_theta = (0 == pre_v[p].cond.size()) ? doms[0]-1 : pre_v[p].cond[0].prev;

				for (int val=start_val;val<=end_val;val++) {
					if (val != pre_v[p].post) {
						for (int theta=start_theta;theta<=end_theta;theta++) {
							int p_ind = p_var(v,val,pre_v[p].post,theta);
							sol[p_ind] = min(sol[p_ind],c);
						}
					}
				}
			}
		}

		//Calculating shortest paths
		for (int k=0;k<dom_size;k++) {
			for (int i=0;i<dom_size;i++) {
				for (int j=0;j<dom_size;j++) {
					for (int theta=0; theta<doms[0]; theta++) {
						int p_indij = p_var(v,i,j,theta);
						int p_indik = p_var(v,i,k,theta);
						int p_indkj = p_var(v,k,j,theta);

						sol[p_indij] = min(sol[p_indij], sum_infinity(sol[p_indik], sol[p_indkj]));
					}
				}
			}
		}
		// End of Calculating the distances in DTGs of the leaves.

		// Setting the initial values for dynamic calculation of d() variables
		// From the state to goal
		for (int val_0=0; val_0 < dom_size; val_0++){
			// d(v, val_0, 1) = p(v, s[v], val_0, s[r])
			int d_ind = d_var(v,val_0,1,root_zero);
			int p_ind = p_var(v,s_v,val_0,root_zero);
			sol[d_ind] = sol[p_ind];
		}

		// Making the step calculation
		for (int sz=2; sz <= dom_size+1; sz++){
			int theta = (1 + sz + root_zero)%2;
			for (int val_0=0; val_0 < dom_size; val_0++){
				int d_ind = d_var(v,val_0,sz,root_zero);
				double min_val = infinity;
				// d(v, val_0, sz) <= d(v, val_1, sz-1) + p(v, val_1, val_0, 0/1)
				for (int val_1=0; val_1 < dom_size; val_1++){
					int d_ind1 = d_var(v,val_1,sz-1,root_zero);
					int p_ind1 = p_var(v,val_1,val_0,theta);
					double res = sum_infinity(sol[d_ind1],sol[p_ind1]);
					if (min_val > res)
						min_val = res;
				}
				sol[d_ind] = min_val;
			}
		}

		// Making the extra step
		for (int sz=dom_size+2; sz <= sigma_size; sz++){
			for (int val_0=0; val_0 < dom_size; val_0++){
				// d(v, val_0, sz) = d(v, val_0, sz-1)
				int d_ind20 = d_var(v,val_0,sz,root_zero);
				int d_ind21 = d_var(v,val_0,sz-1,root_zero);
				sol[d_ind20] = sol[d_ind21];
			}
		}
	}

	// Finding for each variable and each value the lower and upper bounds on the parent support (for each parent value).
	for (int v = 1; v < var_num; v++) {
		int dom_size = doms[v];
		for (int val0=0;val0<dom_size;val0++) {
			int min_sigma = 1;
			int max_sigma = sigma_size;

			// Save the bounds for v, val0, theta.
			int ind = bound_ind(v,val0,root_zero);
			lower_bound[ind] = min_sigma;
			upper_bound[ind] = max_sigma;
			// Keep the needed entries only
			for (int sz = min_sigma; sz <= max_sigma; sz++) {
				int d_ind = d_var(v,val0,sz,root_zero);
				solution->set_value(d_ind, sol[d_ind]);
			}
		}
	}

//////////////////////////////////////////////////////////////////////////////////////////

	const vector<DOperator*> &A_r = abs->get_var_actions(0);
	// Dividing root changing actions into two sets, by the post value
	double min0 = numeric_limits<double>::max();
	double min1 = numeric_limits<double>::max();

	for (size_t a=0; a < A_r.size(); ++a) {
		double c = A_r[a]->get_double_cost();

		// For conditional effects
		vector<PrePost> pre_r;
		A_r[a]->get_explicit_pre_post_for_var(0, pre_r);

		assert(pre_r.size() > 0);
		for (size_t p=0; p < pre_r.size(); ++p) {
			assert(pre_r[p].cond.size() == 0);
			int pre = pre_r[p].pre;
			int post = pre_r[p].post;

			// If the action does nothing...
			if (pre == post)
				continue;

			if (0 == post) {
				min0 = (min0 < c) ? min0 : c;
			} else {
				min1 = (min1 < c) ? min1 : c;
			}
		}
	}

	int w_ind0 = w_r(0);
	int w_ind1 = w_r(1);

	solution->set_value(w_ind0, min0);
	solution->set_value(w_ind1, min1);

	// Freeing the allocated temporal solution array.
	delete [] sol;

}



double BinaryFork::get_solution_value(const SPState &state) {

	Mapping* map = get_mapping();
	const Problem* abs = map->get_abstract();
//	const SPState &abs_state = map->get_abs_state(state);
	const SPState &abs_state = get_abstract_state(state);

	// Taking from cache
	if (use_cache) {
		double cached_val = get_cached_state_value(abs_state);
//		cout << "Cached value is " << cached_val << endl;
		if (cached_val != -1.0)
			return cached_val;
	}

//	const state_var_t * eval_state = abs_state->get_buffer();

	int root_zero = abs_state[0];
	int root_goal = abs->get_goal_val(0);

	// Counting the lower and upper bounds on sigma
	int lower_b = 0;
	int upper_b = 0;
	for (int v = 1; v <= num_leafs; v++) {
		int val = abs_state[v];
		int b_ind = bound_ind(v,val,root_zero);
//		cout << " " << upper_bound[b_ind];
		lower_b = (lower_b < lower_bound[b_ind]) ? lower_bound[b_ind] : lower_b;
		upper_b = (upper_b < upper_bound[b_ind]) ? upper_bound[b_ind] : upper_b;
		// Filling the bounds in advance
		sigma_bounds[v] = upper_bound[b_ind];
	}
//	cout << endl;

	// Fixing the lower and upper bounds (+1 when needed)
	if (-1 != root_goal){ // There is a goal on the root -- go over all even/odd possibilities for sigma
		if (root_zero == root_goal) {   //  if the goal is defined and equal to the initial,
			if ((lower_b % 2) == 0)
				lower_b++;              //   then sigma's length is odd (1, 3, 5, etc).
			if ((upper_b % 2) == 0)
				upper_b++;
		} else {
			if ((lower_b % 2) == 1)
				lower_b++;              // otherwise, sigma's length is even (2, 4, 6, etc).
			if ((upper_b % 2) == 1)
				upper_b++;
		}
	}
//	if ((lower_b > step) || (upper_b < sigma_size))
//		cout << "Lower bound: " << lower_b <<", Upper bound: " << upper_b << " ("<< sigma_size<<"), step: " << step << endl;

	// Going over all the valid sigmas
	double min_sol = numeric_limits<double>::max();
	// Getting all the bounds in advance, fixing them as needed inside the function

	for (int sigma = lower_b; sigma<=upper_b; sigma=sigma+sigma_step ) {
//		cout << "Calculating h value for sigma = " << sigma << " and state " << endl;
//		abs_state->dump();
		double sol = get_h_val(sigma,abs_state);
//		cout << "h value is " << sol << endl;
		if (sol == 0.0)
			return sol; //minimal possible - no need to continue.

		min_sol = (min_sol < sol) ? min_sol : sol;
	}

	// Adding to cache
	if (use_cache)
		set_cached_state_value(abs_state, min_sol);

	return min_sol;
}

double BinaryFork::get_h_val(int sigma, const SPState &eval_state) const {

	int root_zero = eval_state[0];
	//For each leaf v (the first variable is always the root)
	double sol = 0.0;
	double sig =  ((double) sigma - 1)/2;
	double tmp_sol;
	if (sigma > 1) {
		tmp_sol = solution->get_value(w_r(1 - root_zero));
		if (tmp_sol  == infinity)
			return tmp_sol;
		sol += (ceil(sig) * tmp_sol);
	}
	if (sigma > 2) {
		tmp_sol = solution->get_value(w_r(root_zero));
		if (tmp_sol  == infinity)
			return tmp_sol;
		sol += (floor(sig) * tmp_sol);
	}
	for (int v = 1; v <= num_leafs; v++) {
		int val = eval_state[v];
		// Get the relevant sigma - if the sigma is bigger than an upper bound, use the bound.

		int v_sigma = (sigma < sigma_bounds[v]) ? sigma : sigma_bounds[v];  // min(sigma, bounds[v]);
		tmp_sol =solution->get_value(d_var(v,val,v_sigma,root_zero));
//		cout << "d(" << v << ","<<val<<","<<v_sigma<<","<<root_zero<<")="<<tmp_sol<<endl;
		if (tmp_sol  == infinity)
			return tmp_sol;
		sol += tmp_sol;
	}
	return sol;
}


int BinaryFork::d_var(int var, int val, int i, int theta) const {
	return theta*num_leafs*sigma_size*sigma_size +
			  (var-1)*sigma_size*sigma_size + (i-1)*sigma_size + val;
//	cout << "x_" << ret << " == d(v_" << var << "," << val << "," << i << "," << theta << ")" << endl;
//	return ret;
}

int BinaryFork::p_var(int var, int val1, int val2, int root_val) const {
	return number_of_d_variables + num_leafs*sigma_size*sigma_size*root_val +
								(var-1)*sigma_size*sigma_size + val1*sigma_size + val2;
//	cout << "x_" << ret << " == p(v_" << var << "," << val1 << "," << val2 << "," << root_val << ")" << endl;
//	return ret;
}

int BinaryFork::h_var() const {
	int ret = number_of_d_variables + number_of_p_variables;
//	cout << "x_" << ret << " = h" << endl;
	return ret;
}

int BinaryFork::w_r(int root_zero) const {
//	int ret = 2*num_leafs*sigma_size*sigma_size + root_zero;
	int ret = number_of_d_variables + number_of_p_variables + number_of_h_variables + root_zero;
//	cout << "x_" << ret << " == w_" << root_zero << endl;
	return ret;
}

int BinaryFork::get_num_vars() const {
	return number_of_d_variables + number_of_p_variables + number_of_h_variables +
	number_of_w_r_variables + number_of_w_v_variables + number_of_w_var_variables;
}

int BinaryFork::bound_ind(int var, int val, int theta) const {
	return theta*num_leafs*sigma_size + (var-1)*sigma_size + val;
}

int BinaryFork::get_sigma_lower_bound(int var, int val, int theta) const {
	return lower_bound[bound_ind(var, val, theta)];
}

int BinaryFork::get_sigma_upper_bound(int var, int val, int theta) const {
	return upper_bound[bound_ind(var, val, theta)];
}
