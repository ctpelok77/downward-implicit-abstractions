// globals removed — use TaskProxy
// timer removed

#include "../SP_globals.h"
#include "binary_semiforks_gen.h"
#include "../mappings/mapping.h"
#include <math.h>
#include <limits>

using namespace std;

BinarySemiFork::BinarySemiFork() :SolutionMethod() {
	set_abstraction(new BinarySemiForksAbstraction());
}

BinarySemiFork::BinarySemiFork(GeneralAbstraction* abs) :SolutionMethod() {
	set_abstraction(abs);
	// Getting the info on hat and leaf vars directly from the problem
	const Problem* p = get_mapping()->get_abstract();
	int abs_root = abs->get_abstract_root();

	for (int v=0; v < num_abs_vars; v++) {
		if (v == abs_root) // Skipping the root
			continue;
		const vector<int> &successors = p->get_causal_graph()->get_successors(v);
		const vector<int> &predecessors = p->get_causal_graph()->get_predecessors(v);

		if (successors.size() == 0 && predecessors.size() == 1 && predecessors[0] == abs_root) {
			// Definitely a leaf
			leaf_vars.push_back(v);
			continue;
		}
		// Otherwise, it's in the hat
		hat_vars.push_back(v);
	}

}


BinarySemiFork::BinarySemiFork(SemiForksAbstraction* f, Domain* abs_domain) :SolutionMethod() {
	BinarySemiForksAbstraction* bsf = new BinarySemiForksAbstraction(f, abs_domain);
	set_abstraction(bsf);
	hat_vars = bsf->get_abs_hat_vars();
	leaf_vars = bsf->get_abs_leaf_vars();
}

BinarySemiFork::BinarySemiFork(OneDependentHourglassesAbstraction* hg, Domain* abs_domain) :SolutionMethod() {
	OneDependentBinaryHourglassesAbstraction* bhg = new OneDependentBinaryHourglassesAbstraction(hg, abs_domain);
	set_abstraction(bhg);
	hat_vars = bhg->get_abs_parent_vars();
	leaf_vars = bhg->get_abs_leaf_vars();

}


BinarySemiFork::~BinarySemiFork() {
	clear_hat_solutions();

	if (sigma_bounds)
		delete sigma_bounds;

}

void BinarySemiFork::clear_hat_solutions() {
	for (size_t i=0; i < hat_solutions.size(); ++i) {
		if (hat_solutions[i])
			delete hat_solutions[i];
	}
	hat_solutions.clear();
}

void BinarySemiFork::initiate() {
	// Warning: Assuming that the first variable is the center, then come hat variables,
	cout << "Initializing SemiFork with";
	if (!use_cache)
		cout << "out";
	cout << " caching for abstract states" << endl;

	const Problem* abs = get_mapping()->get_abstract();
	const vector<int> &doms = abs->get_variable_domains();
//	abs->dump();
	int max_domain_size = 0;

	num_leafs = leaf_vars.size();
	if (0 == num_leafs){
		sigma_size = 2;
	} else {
		for (size_t i=0; i < leaf_vars.size(); ++i)
			if (max_domain_size < doms[leaf_vars[i]])
				max_domain_size = doms[leaf_vars[i]];

		sigma_size = max_domain_size+1;
	}
	cout << "Num leafs: " << num_leafs << ", sigma size: " << sigma_size << endl;
	// The bounds on sigma
	lower_bound.assign(2*num_leafs*sigma_size,-1);
	upper_bound.assign(2*num_leafs*sigma_size,-1);
	// Setting the solution
	set_solution(new Solution());

//	int abs_root = abstraction->get_abstract_root();

	num_hat_states = 1;
	for (size_t i=0; i < hat_vars.size(); ++i) {
		multiplier.push_back(num_hat_states);
		num_hat_states *= doms[hat_vars[i]];
	}
	cout << "Num hat vars: " << hat_vars.size() << ", num hat states: " << num_hat_states << endl;

	// Initializing variables holding the number of variables for each type
	set_default_number_of_variables();
	// Setting the step of the sigma
	int abs_root = get_abs_root();
	cout << "Abstract root: " << abs_root << endl;
	if (-1 != abs->get_goal_val(abs_root)){ // There is a goal on the root -- go over all even/odd possibilities for sigma
		sigma_step = 2;
	} else {
		sigma_step = 1;
	}
	cout << "Sigma step: " << sigma_step << endl;
	sigma_bounds = new int[num_leafs];
	// Setting the boolean vector for hat variables
	is_in_hat.assign(doms.size(),false);
	for (size_t i=0; i < hat_vars.size(); ++i)
		is_in_hat[hat_vars[i]] = true;

//	cout << "Done initializing semifork" << endl;
}

void BinarySemiFork::set_default_number_of_variables( ) {
	number_of_d_variables = d_vars_multiplier*num_leafs*sigma_size*sigma_size;
	number_of_p_variables = 2*num_leafs*sigma_size*sigma_size;
	number_of_h_variables = 1;
//	number_of_w_r_variables = 2;
	number_of_w_r_variables = 0;
	number_of_w_v_variables = 0;
	number_of_w_var_variables = 0;
//	number_of_m_variables = 2*num_hat_states*sigma_size;
	number_of_m_variables = 0;
}

void BinarySemiFork::dump_number_of_variables( ) const {
	cout << "number_of_d_variables: " << number_of_d_variables << endl;
	cout << "number_of_p_variables: " << number_of_p_variables << endl;
	cout << "number_of_h_variables: " << number_of_h_variables << endl;
	cout << "number_of_w_r_variables: " << number_of_w_r_variables << endl;
	cout << "number_of_w_v_variables: " << number_of_w_v_variables << endl;
	cout << "number_of_w_var_variables: " << number_of_w_var_variables << endl;
	cout << "number_of_m_variables: " << number_of_m_variables << endl;
}

void BinarySemiFork::solve( ) {
//	cout << "Solving the fork part" << endl;
	solve_fork();
//	cout << "Done solving the fork part, solving the hat part" << endl;
	solve_hat();
//	cout << "Done solving the hat part" << endl;
}

void BinarySemiFork::solve_fork( ) {

//	cout << "Start Solving " << g_timer << endl;
	const Problem* abs = get_mapping()->get_abstract();
	const vector<int> &doms = abs->get_variable_domains();
	int root_dom = doms[get_abs_root()];
	assert(root_dom==2);
	// Freed in the end of this function
	double* sol = new double[get_num_vars()];
	solution->clear_solution();
//	cout << "Start calculating p and d values " << g_timer << endl;
	//For each leaf v (the first variable is always the root)
//	int var_num = doms.size();
//	for (int v = 1; v < var_num; v++) {
	for (size_t leaf_ind=0; leaf_ind < leaf_vars.size(); ++leaf_ind) {
		int v = leaf_vars[leaf_ind];
//		cout << "Start calculating p and d values for variable " << v << " " << g_timer << endl;
		int dom_size = doms[v];

		for (int val0=0;val0<dom_size;val0++) {  // Initialize
			for (int val1=0;val1<dom_size;val1++) {
				for (int theta=0;theta<root_dom;theta++) {
					int p_ind = p_var(leaf_ind,val0,val1,theta);
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

				// semifork - at most one parent for the leaf variables.
				assert(pre_v[p].cond.size() == 0 ||
						(pre_v[p].cond.size() == 1 && pre_v[p].cond[0].var == 0));
				int start_theta = (0 == pre_v[p].cond.size()) ? 0 : pre_v[p].cond[0].prev;
				int end_theta = (0 == pre_v[p].cond.size()) ? doms[0]-1 : pre_v[p].cond[0].prev;

				for (int val=start_val;val<=end_val;val++) {
					if (val != pre_v[p].post) {
						for (int theta=start_theta;theta<=end_theta;theta++) {
							int p_ind = p_var(leaf_ind,val,pre_v[p].post,theta);
							sol[p_ind] = min(sol[p_ind],c);
						}
					}
				}

			}

		}
//		cout << "End setting initial p(v,val0,val1, theta) for variable " << v << " with domain " << dom_size << " " << g_timer << endl;

		//Calculating shortest paths
		for (int k=0;k<dom_size;k++) {
			for (int i=0;i<dom_size;i++) {
				int p_indik0 = p_var(leaf_ind,i,k,0);
				int p_indik1 = p_var(leaf_ind,i,k,1);
				for (int j=0;j<dom_size;j++) {
					int p_indij0 = p_var(leaf_ind,i,j,0);
					int p_indkj0 = p_var(leaf_ind,k,j,0);
					int p_indij1 = p_var(leaf_ind,i,j,1);
					int p_indkj1 = p_var(leaf_ind,k,j,1);

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
				int d_ind = d_var(leaf_ind,val_0,1,theta);
				int p_ind = p_var(leaf_ind,val_0,g_v,theta);
				sol[d_ind] = sol[p_ind];
			}
		}
//		cout << "Start first step calculation for variable " << v << " " << g_timer << endl;

		for (int sz=2; sz <= dom_size+1; sz++){
			for (int val_0=0; val_0 < dom_size; val_0++){
				for (int theta=0; theta < root_dom; theta++){
					int d_ind = d_var(leaf_ind,val_0,sz,theta);
					sol[d_ind] = infinity;
					// d(v, val_0, sz, theta) <= d(v, val_1, sz-1,1-theta) + p(v, val_0, val_1, theta)
					for (int val_1=0; val_1 < dom_size; val_1++){
						int d_ind1 = d_var(leaf_ind,val_1,sz-1,1-theta);
						int p_ind1 = p_var(leaf_ind,val_0,val_1,theta);
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
					int d_ind20 = d_var(leaf_ind,val_0,sz,theta);
					int d_ind21 = d_var(leaf_ind,val_0,sz-1,theta);
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

//	cout << "Going over " << leaf_vars.size() << " leafs" << endl;
	for (size_t leaf_ind=0; leaf_ind < leaf_vars.size(); ++leaf_ind) {
		int v = leaf_vars[leaf_ind];
		int dom_size = doms[v];
		int g_v = abs->get_goal_val(v);
//		cout << "Leaf " << v << " with domain size " << dom_size << " and goal value " << g_v << endl;
		for (int theta=0; theta < root_dom; theta++){
			for (int val0=0; val0<dom_size; val0++) {
//				cout << "Value = " << val0 << ", theta = " << theta << endl;
				int min_sigma = 1;
				int max_sigma = sigma_size;
				if (val0 == g_v) {
					max_sigma = 1;
				} else {
					// Find lower bound
				    while((min_sigma <= sigma_size) && (sol[d_var(leaf_ind,val0,min_sigma,theta)]  == infinity)) {
//				    	cout << "ASigma: " << min_sigma << ", value " << sol[d_var(leaf_ind,val0,min_sigma,theta)] << endl;
				    	min_sigma++;
				    }
//			    	cout << "Min sigma: " << min_sigma << ", value " << sol[d_var(leaf_ind,val0,min_sigma,theta)] << endl;

				    // Find upper bound
				    double max_val = sol[d_var(leaf_ind,val0,sigma_size,theta)];
				    while((max_sigma > 0) && (sol[d_var(leaf_ind,val0,max_sigma,theta)] == max_val)) {
//				    	cout << "BSigma: " << max_sigma << ", value " << sol[d_var(leaf_ind,val0,max_sigma,theta)] << endl;
				    	max_sigma--;
				    }

				    max_sigma++;
//			    	cout << "Max sigma: " << max_sigma << ", value " << sol[d_var(leaf_ind,val0,max_sigma,theta)] << endl;

				}
				// Save the bounds for v, val0, theta.
				int ind = bound_ind(leaf_ind,val0,theta);
				lower_bound[ind] = min_sigma;
				upper_bound[ind] = max_sigma;
				// Keep the needed entries only
				for (int sz = min_sigma; sz <= max_sigma; sz++) {
					int d_ind = d_var(leaf_ind,val0,sz,theta);
					solution->set_value(d_ind, sol[d_ind]);
				}
			}
		}
	}
//	cout << "End calculating min and max sigma and setting solution for d and p " << g_timer << endl;
	/*
	 * No need for this part
	const vector<DOperator*> &A_r = abs->get_var_actions(get_abs_root());
	// Dividing root changing actions into two sets, by the post value
	double min0 = numeric_limits<double>::max();
	double min1 = numeric_limits<double>::max();

	for (size_t a=0; a < A_r.size(); ++a) {
		double c = A_r[a]->get_double_cost();

		// For conditional effects
		vector<PrePost> pre_r;
		A_r[a]->get_explicit_pre_post_for_var(get_abs_root(), pre_r);

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
	*/
//	solution->dump();
//	cout << "End setting solution for w " << g_timer << endl;
	// Freeing the allocated temporal solution array.
	delete [] sol;
//	cout << "End deallocating " << g_timer << endl;
}




double BinarySemiFork::get_solution_value(const SPState &state) {
//	cout << "Getting solution value from the semifork" << endl;
	Mapping* map = get_mapping();
	const Problem* abs = map->get_abstract();
	const SPState &abs_state = get_abstract_state(state);
	// Taking from cache

	if (use_cache) {
		double cached_val = get_cached_state_value(abs_state);
//		cout << "Cached value is " << cached_val << endl;
		if (cached_val != -1.0)
			return cached_val;
	}

//	const state_var_t * eval_state = abs_state->get_buffer();

	int root_zero = abs_state[get_abs_root()];
	int root_goal = abs->get_goal_val(get_abs_root());

	// Counting the lower and upper bounds on sigma
//	cout <<  "Counting the lower and upper bounds on sigma" << endl;
	int lower_b = 0;
	int upper_b = 0;
	for (size_t leaf_ind=0; leaf_ind < leaf_vars.size(); ++leaf_ind) {
		int v = leaf_vars[leaf_ind];
		int val = abs_state[v];
		int b_ind = bound_ind(leaf_ind,val,root_zero);
//		cout << " " << upper_bound[b_ind];
		lower_b = (lower_b < lower_bound[b_ind]) ? lower_bound[b_ind] : lower_b;
		upper_b = (upper_b < upper_bound[b_ind]) ? upper_bound[b_ind] : upper_b;
		// Filling the bounds in advance
		sigma_bounds[leaf_ind] = upper_bound[b_ind];
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
	double min_sol = numeric_limits<double>::max();;
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

//	cout << "Binary semifork solution: " << min_sol << endl;
	return min_sol;
}

double BinarySemiFork::get_h_val(int sigma, const SPState &eval_state) const {

	int root_val = eval_state[get_abs_root()];
	int hat_var = short_m_var(eval_state, root_val);

	// The hat solutions are kept for the number of changes, which is sigma - 1.
	double sol = hat_solutions[sigma-1]->get_value(hat_var);
//	cout << "m(" << hat_var << ")="<<sol<<endl;

	double tmp_sol;
	if (sol == infinity)
		return sol;
	//For each leaf var add the cost of achieving its value assuming that the hat can provide sigma changes
	for (size_t leaf_ind=0; leaf_ind < leaf_vars.size(); ++leaf_ind) {
		int var = leaf_vars[leaf_ind];
		int val = (int) eval_state[var];
		// Get the relevant sigma - if the sigma is bigger than an upper bound, use the bound.

		int v_sigma = (sigma < sigma_bounds[leaf_ind]) ? sigma : sigma_bounds[leaf_ind];  // min(sigma, bounds[var]);
		tmp_sol = solution->get_value(d_var(leaf_ind,val,v_sigma,root_val));
//		cout << "d(" << var << ","<<val<<","<<v_sigma<<","<<root_val<<")="<<tmp_sol<<endl;
		if (tmp_sol == infinity)
			return tmp_sol;
		sol += tmp_sol;
	}
	return sol;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BinarySemiFork::solve_hat() {
//	cout << "Start solving the hat part " << g_timer << endl;
	clear_hat_solutions();

	Mapping* map = get_mapping();
	const Problem* abs = map->get_abstract();

	// Each entry is a vector representing the set of states, having -1 for non defined values.
	vector<vector<int> > states;
	abs->generate_state_transition_graph(states);

	for (int change = 0; change < sigma_size; change++)
		hat_solutions.push_back(solve_step(states,change));

//	cout << "End solving the hat part " << g_timer << endl;
}



Solution* BinarySemiFork::solve_step(const vector<vector<int> >& states, int changes) const {

//	cout << "Start solving step "  << changes << " " << g_timer << endl;
	Mapping* map = get_mapping();
	const Problem* abs = map->get_abstract();
	const vector<DOperator*> &ops = abs->get_operators();
	const vector<int> &doms = abs->get_variable_domains();

	// Initializing matrix
	int numLPvars = 2*num_hat_states + 1;
	double** sol = new double*[numLPvars];

//	cout << "Initializing matrix " << g_timer << endl;
	for (int i=0;i<numLPvars;i++) {
		sol[i] = new double[numLPvars];
		for (int j=0;j<numLPvars;j++) {
			if (i==j)
				sol[i][j] = 0.0;
			else
				sol[i][j] = infinity;
		}
	}

//	cout << "Setting distances " << g_timer << endl;
	// (I)   m(s',0,theta) <= m(s",0,theta') + w(a)        for all <s',a,s">
	//       m(s',i,theta) <= m(s",i,theta) + w(a)         if a does not change the center
	//                     <= m(s",i-1,theta') + w(a)       otherwise
	for (size_t it=0; it < ops.size(); ++it) {
		if (ops[it]->is_redundant())
			continue;

		// If the action does not affect hat and the center, skip.
		bool to_skip = true;
		const vector<PrePost> &pre = ops[it]->get_pre_post();
		for (size_t i=0; i < pre.size(); ++i) {
			if (pre[i].var == get_abs_root() || is_in_hat[pre[i].var]) {
				to_skip = false;
				break;
			}
		}
		if (to_skip)
			continue;
		// Per action we generate all the states this action is applicable in.
		vector<int> generated = states[it];
		vector<int> free_vars;

		// Setting the vector of free variables to consist only of hat vars
		for (size_t i=0; i < hat_vars.size(); ++i) {
			if (generated[hat_vars[i]] < 0) {
				free_vars.push_back(hat_vars[i]);
			}
		}
		for (int theta = 0; theta < doms[get_abs_root()]; theta++)
			set_distances(sol,free_vars,generated,doms,ops[it],changes,theta);
	}


	if (changes == 0) {
//		cout << "Setting base goal distances " << g_timer << endl;
		// (II)      m(s,0,theta) = 0           for all goal states and theta values
		vector<int> goals;
		abs->get_goal_vals(goals);
		vector<int> free_vars;

		for (size_t i=0; i < goals.size(); ++i) {
			if (goals[i] < 0 && is_in_hat[i]) {
				free_vars.push_back(i);
			}
		}
		// If the center has its goal defined,
		int start_theta = 0;
		int end_theta = doms[get_abs_root()] - 1;
		if (goals[get_abs_root()] != -1) {
			start_theta = goals[get_abs_root()];
			end_theta = goals[get_abs_root()];
		}
		for (int theta = start_theta; theta <= end_theta; theta++)
			set_base_goal_distances(sol,free_vars,goals,doms,theta);

	}

//	cout << "Floyd-Warshall step " << g_timer << endl;

	// Step
	for (int k=0;k<numLPvars;k++) {
		for (int i=0;i<numLPvars;i++) {
			for (int j=0;j<numLPvars;j++) {
				sol[i][j] = min(sol[i][j],sum_infinity(sol[i][k],sol[k][j]));
			}
		}
	}
//	cout << "Creating the solution " << g_timer << endl;

	Solution* ret = new Solution();
	ret->clear_solution();

	for (int k=0;k<numLPvars-1;k++) {
		ret->set_value(k,sol[k][numLPvars-1]);
	}
//	cout << "Deleting allocated matrix " << g_timer << endl;

	for (int i=0;i<numLPvars;i++) {
		delete [] sol[i];
	}
	delete [] sol;

	return ret;
}


void BinarySemiFork::set_distances(double **sol, vector<int> free_vars, vector<int> state, const vector<int>& doms, DOperator* op, int changes, int theta) const {
	// (I)   m(s',0,theta) <= m(s",0,theta') + w(a)        for all <s',a,s">
	// (II)  m(s',i,theta) <= m(s",i,theta) + w(a)         if a does not change the center
	// (III)               <= m(s",i-1,theta') + w(a)       otherwise
//	cout << "Setting distances for " << changes << " changes, starting with " << theta << " " << g_timer << endl;
//	cout << "Free vars:";
//	for (size_t i=0; i < free_vars.size(); ++i)
//		cout << " " << free_vars[i];
//	cout << endl << "State:";
//	for (size_t i=0; i < state.size(); ++i)
//		cout << " " << state[i];
//	cout << endl << "Action:" << endl;
//	op->dump_no_name();

	if (free_vars.size() == 0) {
		// Stopping criteria
//		cout << "Terminating step reached " << g_timer << endl;

		const vector<PrePost> &pre = op->get_pre_post();
		int new_theta = theta;
		// Creating the resulting state.
		vector<int> next_state = state;
		for (size_t i=0; i < pre.size(); ++i) {
			// check if the variable is in hat
			if (is_in_hat[pre[i].var])
				next_state[pre[i].var] = pre[i].post;

			if (get_abs_root() == pre[i].var) {
				new_theta = pre[i].post;
				assert(new_theta == 1-theta);
			}
		}
//		cout << "From state (" << theta << ") " ;
//		for (size_t i=0; i < state.size(); ++i) {
//			cout << " " << state[i] ;
//		}
//		cout << endl << "To state (" << new_theta << ") " ;
//		for (size_t i=0; i < next_state.size(); ++i) {
//			cout << " " << next_state[i] ;
//		}
//		cout << endl;

//		int d_ind0 = m_var(state, changes, theta);
		int d_ind0 = short_m_var(state, theta);
		double c = op->get_double_cost();
		// If the action changes the root
		if (theta != new_theta && changes > 0){
//			cout << "Terminating condition III " << g_timer << endl;

			// (III) If not reached 0 changes yet
//			int d_prev = m_var(next_state,changes-1,new_theta);
			int d_prev = short_m_var(next_state,new_theta);
			c = sum_infinity(c, hat_solutions[changes-1]->get_value(d_prev));
			sol[d_ind0][2*num_hat_states] = min(sol[d_ind0][2*num_hat_states],c);
			return;
		}
		// (I) and (II) -
//		int d_ind1 = m_var(next_state, changes, new_theta);
		int d_ind1 = short_m_var(next_state, new_theta);
		if (d_ind0==d_ind1)
			return;

//		cout << "Terminating conditions I and II " << g_timer << endl;
//		cout << "Indexes: " << d_ind0 << " -> " << d_ind1 << endl;
		sol[d_ind0][d_ind1] = min(sol[d_ind0][d_ind1],c);
//		cout << "Updated" << endl;
		return;
	}
//	cout << "Recursive step" << endl;
	// Recursive step (for each value of one of the free variables
	int free_var = free_vars[free_vars.size()-1];
	free_vars.pop_back();
	int dom_size = doms[free_var];
	for(int i = 0; i < dom_size; i++) {
		state[free_var] = i;
		set_distances(sol,free_vars,state,doms,op, changes, theta);
	}

}


void BinarySemiFork::set_base_goal_distances(double **sol, vector<int> free_vars, vector<int> state, const vector<int>& doms, int theta) const {

	if (free_vars.size() == 0) {
		// Stopping criteria
//		int d_ind = m_var(state, 0, theta);
		int d_ind = short_m_var(state, theta);
		sol[d_ind][2*num_hat_states] = 0;
		return;
	}
	// Recursive step (for each value of one of the free variables
	int free_var = free_vars[free_vars.size()-1];
	free_vars.pop_back();
	int dom_size = doms[free_var];
	for(int i = 0; i < dom_size; i++) {
		state[free_var] = i;
		set_base_goal_distances(sol,free_vars,state,doms, theta);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




int BinarySemiFork::d_var(int leaf_ind, int val, int i, int theta) const {
	return theta*num_leafs*sigma_size*sigma_size +
			leaf_ind*sigma_size*sigma_size + (i-1)*sigma_size + val;
//	cout << "x_" << ret << " == d(v_" << leaf_ind << "," << val << "," << i << "," << theta << ")" << endl;
//	return ret;
}

int BinarySemiFork::p_var(int leaf_ind, int val1, int val2, int root_val) const {
	return number_of_d_variables + num_leafs*sigma_size*sigma_size*root_val +
			leaf_ind*sigma_size*sigma_size + val1*sigma_size + val2;
//	cout << "x_" << ret << " == p(v_" << leaf_ind << "," << val1 << "," << val2 << "," << root_val << ")" << endl;
//	return ret;
}

int BinarySemiFork::h_var() const {
	int ret = number_of_d_variables + number_of_p_variables;
//	cout << "x_" << ret << " = h" << endl;
	return ret;
}

int BinarySemiFork::w_r(int root_zero) const {
//	int ret = 2*num_leafs*sigma_size*sigma_size + root_zero;
	int ret = number_of_d_variables + number_of_p_variables + number_of_h_variables + root_zero;
//	cout << "x_" << ret << " == w_" << root_zero << endl;
	return ret;
}

int BinarySemiFork::get_num_vars() const {
	return number_of_d_variables + number_of_p_variables + number_of_h_variables +
	number_of_w_r_variables + number_of_w_v_variables + number_of_w_var_variables + number_of_m_variables;
}

int BinarySemiFork::bound_ind(int leaf_ind, int val, int theta) const {
	return theta*num_leafs*sigma_size + leaf_ind*sigma_size + val;
}

int BinarySemiFork::get_sigma_lower_bound(int leaf_ind, int val, int theta) const {
	return lower_bound[bound_ind(leaf_ind, val, theta)];
}

int BinarySemiFork::get_sigma_upper_bound(int leaf_ind, int val, int theta) const {
	return upper_bound[bound_ind(leaf_ind, val, theta)];
}
/////////////////////////////////

int BinarySemiFork::short_m_var(const SPState &state, int theta) const {
	int ret = 0;
	for (size_t i=0; i < hat_vars.size(); ++i) {
		int s = state[hat_vars[i]];
		int m = multiplier[i];
		ret += s*m;
//		cout << " " << s;
	}
	return 2*ret + theta;
}


