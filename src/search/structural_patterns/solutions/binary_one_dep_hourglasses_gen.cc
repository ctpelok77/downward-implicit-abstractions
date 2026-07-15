// globals removed — use TaskProxy
// timer removed

#include "../SP_globals.h"
#include "binary_one_dep_hourglasses_gen.h"
#include "../abstractions/one_dep_binary_hourglasses.h"
#include "../mappings/mapping.h"
#include <math.h>
#include <limits>

using namespace std;

BinaryOneDepHourglass::BinaryOneDepHourglass() :SolutionMethod() {
	set_abstraction(new OneDependentBinaryHourglassesAbstraction());
}

BinaryOneDepHourglass::BinaryOneDepHourglass(GeneralAbstraction* abs) :SolutionMethod() {
	set_abstraction(abs);
	// Getting the info on parents and leaf vars directly from the problem.
	// Setting the parents to be all the predecessors that are not goal successors
	const Problem* p = get_mapping()->get_abstract();
	int abs_root = abs->get_abstract_root();
	const vector<int> &predecessors = p->get_causal_graph()->get_predecessors(abs_root);
	set<int> pred_set;
	pred_set.insert(predecessors.begin(), predecessors.end());
	const vector<int> &successors = p->get_causal_graph()->get_successors(abs_root);

	for (size_t i=0; i < successors.size(); ++i) {
		int ch = successors[i];
		if (!p->is_goal_var(ch)) // Skipping non goal successors
			continue;
		pred_set.erase(ch);
		leaf_vars.push_back(ch);
	}
	parent_vars.insert(parent_vars.begin(),pred_set.begin(),pred_set.end());
}


BinaryOneDepHourglass::BinaryOneDepHourglass(OneDependentHourglassesAbstraction* f, Domain* abs_domain) :SolutionMethod() {
	OneDependentBinaryHourglassesAbstraction* bhg = new OneDependentBinaryHourglassesAbstraction(f, abs_domain);
	set_abstraction(bhg);
	parent_vars = bhg->get_abs_parent_vars();
	leaf_vars = bhg->get_abs_leaf_vars();
}


BinaryOneDepHourglass::~BinaryOneDepHourglass() {
	clear_hat_solutions();

	if (sigma_bounds)
		delete sigma_bounds;
}

void BinaryOneDepHourglass::clear_hat_solutions() {
	clear_parent_solutions();

	for (size_t i=0; i < parent_hats.size(); ++i) {
		if (parent_hats[i])
			delete parent_hats[i];
	}
}

void BinaryOneDepHourglass::clear_parent_solutions() {
	for (size_t i=0; i < parent_solutions.size(); ++i) {
		if (parent_solutions[i])
			delete parent_solutions[i];
	}
	parent_solutions.clear();
}

void BinaryOneDepHourglass::initiate() {
	cout << "Initializing Hourglass with";
	if (!use_cache)
		cout << "out";
	cout << " caching for abstract states" << endl;

	const Problem* abs = get_mapping()->get_abstract();
	const vector<int> &doms = abs->get_variable_domains();

	int max_domain_size = 0;
	num_leafs = leaf_vars.size();
	if (0 == num_leafs){
		sigma_size = 2;
	} else {
		for (size_t i=0; i < leaf_vars.size(); ++i) {
			if (max_domain_size < doms[leaf_vars[i]])
				max_domain_size = doms[leaf_vars[i]];

		}
		sigma_size = max_domain_size+1;
	}
	cout << "Num leafs: " << num_leafs << ", sigma size: " << sigma_size << endl;
	// The bounds on sigma
	lower_bound.assign(2*num_leafs*sigma_size,-1);
	upper_bound.assign(2*num_leafs*sigma_size,-1);
	// Setting the solution
	set_solution(new Solution());

	// Preparing pairs for future use
	clear_hat_solutions();
	for(size_t i=0;i<parent_vars.size()-1;++i) {
		for (size_t j=i+1; j < parent_vars.size(); ++j) {
			// Each element of both vectors correspond to one parents pair.
			vector<int> mult, par;
			mult.push_back(1);
			mult.push_back(doms[parent_vars[i]]);
			par.push_back(parent_vars[i]);
			par.push_back(parent_vars[j]);
			int num_st = doms[parent_vars[i]] * doms[parent_vars[j]];
			parent_hats.push_back(new Hat(par, mult, num_st));
		}
	}
	if (parent_vars.size() == 1) {
		cout << "Hourglass with one parent should use semifork solution!!!" << endl;
		exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}

	// Initializing variables holding the number of variables for each type
	set_default_number_of_variables();
	// Setting the step of the sigma
	int abs_root = get_abs_root();
//	cout << "Abstract root: " << abs_root << endl;
	if (-1 != abs->get_goal_val(abs_root)){ // There is a goal on the root -- go over all even/odd possibilities for sigma
		sigma_step = 2;
	} else {
		sigma_step = 1;
	}
//	cout << "Sigma step: " << sigma_step << endl;
	sigma_bounds = new int[num_leafs];

	// Setting the boolean vector for parent variables
	is_parent.assign(doms.size(),false);
	parent_max_domain = 0;
	for (size_t i=0; i < parent_vars.size(); ++i) {
		int v = parent_vars[i];
		if (doms[v] > parent_max_domain)
			parent_max_domain = doms[v];

		is_parent[v] = true;
	}
//	cout << "Done initializing Hourglass" << endl;
}

void BinaryOneDepHourglass::set_default_number_of_variables( ) {
	number_of_d_variables = d_vars_multiplier*num_leafs*sigma_size*sigma_size;
	number_of_p_variables = 2*num_leafs*sigma_size*sigma_size;
	number_of_h_variables = 1;
	number_of_w_r_variables = 0;
	number_of_w_v_variables = 0;
	number_of_w_var_variables = 0;
	number_of_m_variables = 0;
}

void BinaryOneDepHourglass::dump_number_of_variables( ) const {
	cout << "number_of_d_variables: " << number_of_d_variables << endl;
	cout << "number_of_p_variables: " << number_of_p_variables << endl;
	cout << "number_of_h_variables: " << number_of_h_variables << endl;
	cout << "number_of_w_r_variables: " << number_of_w_r_variables << endl;
	cout << "number_of_w_v_variables: " << number_of_w_v_variables << endl;
	cout << "number_of_w_var_variables: " << number_of_w_var_variables << endl;
	cout << "number_of_m_variables: " << number_of_m_variables << endl;
}


void BinaryOneDepHourglass::solve( ) {
	// Creating semifork hat for each pair of parents, one fork part for all.
	// For each hat we are using only actions dependent on these variables or none.

//	cout << "Solving the fork part" << endl;
	solve_fork();
//	cout << "Done solving the fork part, solving the hat parts" << endl;
	solve_hats();
//	cout << "Done solving the hat parts" << endl;
}

void BinaryOneDepHourglass::solve_fork( ) {

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

					sol[p_indij0] = min(sol[p_indij0],sum_infinity(sol[p_indik0],sol[p_indkj0]));
					sol[p_indij1] = min(sol[p_indij1],sum_infinity(sol[p_indik1],sol[p_indkj1]));
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

double BinaryOneDepHourglass::get_parents_value(const SPState &state) const {
	// Gets abstract state, return the cost of achieving the goals of all parents
	// without doing anything else.

	const Problem* abs = get_mapping()->get_abstract();
	const vector<int> &doms = abs->get_variable_domains();
	double res = 0.0;
	for (size_t i=0; i < parent_vars.size(); ++i) {
		int v = parent_vars[i];
		int s_v = state[v];
		int g_v = abs->get_goal_val(v);
		if (-1 == g_v)
			continue;
		int par_ind = short_par_var(doms[v], s_v, g_v);
		double r_v = parent_solutions[v]->get_value(par_ind);
//		cout << "Parent " << v << " from " << s_v << " to " << g_v << " cost: " << r_v << endl;
		if (r_v  == infinity)
			return r_v;

		if (-1.0 != r_v)
			res += r_v;
	}
	return res;
}
double BinaryOneDepHourglass::get_parents_value(const Hat* hat, const SPState &state) const {
	// Gets abstract state, return the cost of achieving the goals of parents in hat
	// without doing anything else.
	const Problem* abs = get_mapping()->get_abstract();
	const vector<int>& doms = abs->get_variable_domains();
	const vector<int>& par_vars = hat->get_vars();
	double res = 0.0;
	for (size_t i=0; i < par_vars.size(); ++i) {
		int v = par_vars[i];
		int s_v = state[v];
		int g_v = abs->get_goal_val(v);
		if (-1 == g_v || s_v == g_v)
			continue;
		int par_ind = short_par_var(doms[v], s_v, g_v);
		double r_v = parent_solutions[v]->get_value(par_ind);
		if (r_v  == infinity)
			return r_v;
		res += r_v;
	}
	return res;

}


double BinaryOneDepHourglass::get_solution_value(const SPState &state) {
//	cout << "Getting solution value from the hourglass" << endl;
	const SPState &abs_state = get_abstract_state(state);
	// Taking from cache

	if (use_cache) {
		double cached_val = get_cached_state_value(abs_state);
//		cout << "Cached value is " << cached_val << endl;
		if (cached_val != -1.0)
			return cached_val;
	}
	double parents_sol = get_parents_value(abs_state);
	cout << "All parents cost of achieving the goals is " << parents_sol << endl;
	if (parents_sol  == infinity)
		return parents_sol;

	double min_sol = infinity;

	for (size_t i=0; i < parent_hats.size(); ++i) {

		double sol = parents_sol - get_parents_value(parent_hats[i], abs_state);
		cout << "Non covered parents cost of achieving the goals is " << sol << endl;
		sol += get_solution_value_for_hat(parent_hats[i], abs_state);
		if (sol == 0.0)
			return sol; //minimal possible - no need to continue.

		min_sol = (min_sol < sol) ? min_sol : sol;

	}

	// Adding to cache
	if (use_cache)
		set_cached_state_value(abs_state, min_sol);

	return min_sol;
}


double BinaryOneDepHourglass::get_solution_value_for_hat(const Hat* hat, const SPState &abs_state) {
//	cout << "Getting solution value for hat" << endl;
	Mapping* map = get_mapping();
	const Problem* abs = map->get_abstract();

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
	double min_sol = infinity;
	// Getting all the bounds in advance, fixing them as needed inside the function

	for (int sigma = lower_b; sigma<=upper_b; sigma=sigma+sigma_step ) {
		std::cout << "Calculating h value for sigma = " << sigma << " and state" << std::endl;
		double sol = get_h_val(hat,sigma,abs_state);
		cout << "h value is " << sol << endl;
		if (sol == 0.0)
			return sol; //minimal possible - no need to continue.

		min_sol = (min_sol < sol) ? min_sol : sol;
	}

	return min_sol;
}

double BinaryOneDepHourglass::get_h_val(const Hat* hat, int sigma, const SPState &eval_state) const {
	// return the value of solving the leafs, and root with the given hat (including hat)
	int root_val = (int) eval_state[get_abs_root()];
	// Getting the cost of the hat and root
	const vector<int> &par_vars = hat->get_vars();

	int x = par_vars[0];
	int y = par_vars[1];
	cout << "Parents: x="<<x<<" and y="<<y<<endl;

	int s_x = (int) eval_state[x];
	int s_y = (int) eval_state[y];

	int hat_ind = short_m_var(s_x, s_y, root_val);
	cout << "Index for m var:" << hat_ind << endl;
	cout << "m_{" <<x<<","<<y<<"}(" << s_x << "," << s_y << "," << root_val << " | "<< sigma-1 << " )" << endl;
	// The hat solutions are kept for the number of changes, which is sigma - 1.
	double sol = hat->get_solution_value(sigma-1, hat_ind);
	cout << "m_{" <<x<<","<<y<<"}(" << s_x << "," << s_y << "," << root_val << " | "<< sigma-1 << " ) = " << sol << endl;
	double tmp_sol;
	if (sol  == infinity)
		return sol;
	//For each leaf var add the cost of achieving its value assuming that the hat can provide sigma changes
	for (size_t leaf_ind=0; leaf_ind < leaf_vars.size(); ++leaf_ind) {
		int var = leaf_vars[leaf_ind];
		int val = (int) eval_state[var];
		// Get the relevant sigma - if the sigma is bigger than an upper bound, use the bound.

		int v_sigma = (sigma < sigma_bounds[leaf_ind]) ? sigma : sigma_bounds[leaf_ind];  // min(sigma, bounds[var]);
		tmp_sol = solution->get_value(d_var(leaf_ind,val,v_sigma,root_val));
		cout << "d(" << var << ","<<val<<","<<v_sigma<<","<<root_val<<")="<<tmp_sol<<endl;
		if (tmp_sol  == infinity)
			return tmp_sol;
		sol += tmp_sol;
	}
	return sol;
}


///////////////////////////////////////////////////
void BinaryOneDepHourglass::solve_hats() {
//	cout << "Start solving the hat part " << g_timer << endl;
//	clear_hat_solutions();
//	parent_solutions.assign(num_abs_vars,0);
	parent_solutions.resize(num_abs_vars, nullptr);
	for (size_t i=0; i < parent_vars.size(); ++i) {
		int v = parent_vars[i];
//		cout << "Solving parent " << v << endl;
		parent_solutions[v] = solve_parent(v);
	}
//	cout << "Done solving the individual parents" << endl;
	for (size_t i=0; i < parent_hats.size(); ++i) {
		// Solving for each pair of parents
//		cout << "Solving hat " <<i << endl;
		solve_using_parents(parent_hats[i]);
	}
//	clear_parent_solutions();
//	cout << "End solving the hat part " << g_timer << endl;
}
void BinaryOneDepHourglass::solve_using_parents(Hat* parents) {
//TODO: Modify

//	cout << "Start solving hat " << g_timer << endl;
	const vector<int> &pars = parents->get_vars();
	if (pars.size() != 2) {
		cout << "Parents size must be 2!!!" << endl;
		exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
	vector<Solution*> res;
	res.resize(sigma_size, nullptr);
	Mapping* map = get_mapping();
	const Problem* abs = map->get_abstract();
	const vector<DOperator*> &ops = abs->get_var_actions(get_abs_root());
	const vector<int> &doms = abs->get_variable_domains();

	int numLPvars = parent_max_domain*parent_max_domain*2;
	double* sol = new double[numLPvars];
	// Setting the initial value for the m variables
	Solution* init_sol = new Solution();
	init_sol->clear_solution();
	int x = pars[0];
	int y = pars[1];
//	cout << "Parents are " << x << " and " << y << ", total number of parents: " << parent_solutions.size() << endl;

	int g_x = abs->get_goal_val(x);
	int g_y = abs->get_goal_val(y);

	int doms_x = doms[x];
	int doms_y = doms[y];

//	cout << "Initializing matrix " << g_timer << endl;
	for (int i=0;i<numLPvars;i++) {
		sol[i] = infinity;
	}

//	cout <<  "Setting the initial value for the m variables" << endl;
	// Setting the initial value for the m variables
	// m(val_x, val_y, 0, G[r]) = p(val_x,G[x]) + p(val_y,G[y])
	// If G[r] is not defined, setting for both values.
	int root_goal = abs->get_goal_val(get_abs_root());
	int start_theta = (root_goal == -1) ? 0 : root_goal;
	int end_theta = (root_goal == -1) ? doms[get_abs_root()] - 1 : root_goal;
	for(int val_x = 0; val_x < doms_x; val_x++) {
		double c_x = 0.0;
		if (-1 != g_x) {
			int x_par_ind = short_par_var(doms_x, val_x, g_x);
			c_x = parent_solutions[x]->get_value(x_par_ind);
		}
		for(int val_y = 0; val_y < doms_y; val_y++) {
			// computing p(val_x,G[x]) and p(val_y,G[y])
			double c_y = 0.0;
			if (-1 != g_y) {
				int y_par_ind = short_par_var(doms_y, val_y, g_y);
				c_y = parent_solutions[y]->get_value(y_par_ind);
			}
			double c = c_x + c_y;
//			for(int theta = 0; theta < doms[get_abs_root()]; theta++) {
			for(int theta = start_theta; theta <= end_theta; theta++) {
				int m_ind = short_m_var(val_x,val_y,theta);
				init_sol->set_value(m_ind,c);
//				cout << "m_{" <<x<<","<<y<<"}(" << val_x << "," << val_y << "," << theta << " | 0 ) = " << c << endl;
			}
		}
	}
	res[0] = init_sol;
//	cout << "Done setting initial solution " << g_timer << endl;

	// For each number of changes, set new solution based on the previous one
	for (int changes = 1; changes < sigma_size; changes++) {
		for (int i=0;i<numLPvars;i++) {
			sol[i] = infinity;
		}


		// Going over the applicable actions, minimizing.
		assert(res.size() == changes);
		for (size_t it=0; it < ops.size(); ++it) {
			if (ops[it]->is_redundant())
				continue;

			// Check if the action is not prevailed by variables outside the hat
			set<int> all_vars;
			all_vars.insert(pars.begin(),pars.end());
			const vector<Prevail> &prv = ops[it]->get_prevail();
			for (size_t p=0; p < prv.size(); ++p) {
				all_vars.insert(prv[p].var);
			}
			if (all_vars.size() > pars.size())
				continue;

			int to_theta = ops[it]->get_post_val(get_abs_root());
			int from_theta = 1 - to_theta;   // Assuming theta values are 0,1

			// If G[r] is defined, we can skip actions that achieve goal value when changes are even
			// and non goal value when changes are odd.
			if (-1 == root_goal &&
					(((changes % 2 == 0) && (to_theta == root_goal)) || ((changes % 2 == 1) && (from_theta == root_goal))) )
				continue;

 			double c = ops[it]->get_double_cost();
			int prv_x = ops[it]->get_prevail_val(x);
			int prv_y = ops[it]->get_prevail_val(y);
//			ops[it]->dump_no_name();
//			cout << "Changes root from " << from_theta << " to " << to_theta << " prevailed on x=" << prv_x << " and y=" << prv_y << endl;


			// Setting the values for m variables
			// m(val_x, val_y, i, from_theta) <= p(val_x,prv(a)[x]) + p(val_y,prv(a)[y])
			//                                + cost(a) + m(prv(a)[x], prv(a)[y], i-1, to_theta)
			for(int val_x = 0; val_x < doms_x; val_x++) {
				int new_prv_x = prv_x;
				if (-1 == new_prv_x)
					new_prv_x = val_x;
				int x_par_ind = short_par_var(doms_x, val_x, new_prv_x);
				double c_x = c + parent_solutions[x]->get_value(x_par_ind);
				for(int val_y = 0; val_y < doms_y; val_y++) {
					int new_prv_y = prv_y;
					if (-1 == new_prv_y)
						new_prv_y = val_y;

					int y_par_ind = short_par_var(doms_y, val_y, new_prv_y);
					double c_rest = c_x + parent_solutions[y]->get_value(y_par_ind);
					int m_ind0 = short_m_var(val_x,val_y,from_theta);
					int m_ind1 = short_m_var(new_prv_x,new_prv_y,to_theta);
					double c_all = sum_infinity(c_rest,res[changes-1]->get_value(m_ind1));
					sol[m_ind0] = min(sol[m_ind0], c_all);
//					cout << "m_{" <<x<<","<<y<<"}(" << val_x << "," << val_y << "," << from_theta << " | " << changes << " ) <= " <<
//							"p_{" <<x<<"}(" << val_x << "," << new_prv_x << ") + p_{" << y<<"}("<< val_y << "," << new_prv_y << ") + cost(a_" <<
//							ops[it]->get_index() <<") + m_{" <<
//							x<<","<<y<<"}(" << new_prv_x << "," << new_prv_y << "," << to_theta << " | " << changes-1 << " ) = " <<
//							c_x - c << " + " << c_rest - c_x << " + " << c << " + " << c_all - c_rest << " = " << c_all << " [" << sol[m_ind0] << "]" << endl;
				}
			}

		}
//		cout << "End calculating m(val_x,val_y,i,theta) " << g_timer << endl;
		Solution* next_sol = new Solution();
		next_sol->clear_solution();

//		for (int k=0;k<numLPvars;k++) {
//			next_sol->set_value(k,sol[k]);
//		}

		for(int val_x = 0; val_x < doms_x; val_x++) {
			for(int val_y = 0; val_y < doms_y; val_y++) {
				for(int theta = 0; theta < 2; theta++) {
					int m_ind0 = short_m_var(val_x,val_y,theta);
					next_sol->set_value(m_ind0,sol[m_ind0]);
				}
			}
		}


		res[changes] = next_sol;
	}
//	cout << "End calculating all m variables " << g_timer << endl;
	parents->set_solutions(res);
	delete [] sol;
}

Solution* BinaryOneDepHourglass::solve_parent(int v) const {
//	cout << "Start solving parent "  << v << " " << g_timer << endl;
	Mapping* map = get_mapping();
	const Problem* abs = map->get_abstract();
	const vector<DOperator*> &ops = abs->get_var_actions(v);
	const vector<int> &doms = abs->get_variable_domains();
	int dom_size = doms[v];
	// Initializing matrix
	int numLPvars = dom_size*dom_size;
	double* sol = new double[numLPvars];

//	cout << "Initializing matrix " << g_timer << endl;
	for (int i=0;i<dom_size;i++) {
		for (int j=0;j<dom_size;j++) {
			int p_ind = short_par_var(dom_size,i,j);
			if (i==j)
				sol[p_ind] = 0.0;
			else
				sol[p_ind] = infinity;
		}
	}

	for (size_t it=0; it < ops.size(); ++it) {
		if (ops[it]->is_redundant())
			continue;

		double c = ops[it]->get_double_cost();

		// For conditional effects
		vector<PrePost> pre_v;
		ops[it]->get_explicit_pre_post_for_var(v,pre_v);

		assert(pre_v.size() > 0);
		for (size_t p=0; p < pre_v.size(); ++p) {
			int start_val = (-1 == pre_v[p].pre) ? 0 : pre_v[p].pre;
			int end_val = (-1 == pre_v[p].pre) ? dom_size-1 : pre_v[p].pre;

			for (int val=start_val;val<=end_val;val++) {
				if (val != pre_v[p].post) {
					int p_ind = short_par_var(dom_size,val,pre_v[p].post);
						sol[p_ind] = min(sol[p_ind],c);
				}
			}
		}
	}
//	cout << "End setting initial par(val0,val1) for variable " << v << " with domain " << dom_size << " " << g_timer << endl;

	//Calculating shortest paths
	for (int k=0;k<dom_size;k++) {
		for (int i=0;i<dom_size;i++) {
			int p_indik = short_par_var(dom_size,i,k);
			for (int j=0;j<dom_size;j++) {
				int p_indij = short_par_var(dom_size,i,j);
				int p_indkj = short_par_var(dom_size,k,j);

				sol[p_indij] = min(sol[p_indij],sum_infinity(sol[p_indik],sol[p_indkj]));
			}
		}
	}
//	cout << "End calculating par(val0,val1) for variable " << v << " " << g_timer << endl;

//	cout << "Creating the solution " << g_timer << endl;

	Solution* ret = new Solution();
	ret->clear_solution();

	for (int k=0;k<numLPvars;k++) {
		ret->set_value(k,sol[k]);
	}
//	cout << "Deleting allocated matrix " << g_timer << endl;

	delete [] sol;

	return ret;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




int BinaryOneDepHourglass::d_var(int leaf_ind, int val, int i, int theta) const {
	return theta*num_leafs*sigma_size*sigma_size +
			leaf_ind*sigma_size*sigma_size + (i-1)*sigma_size + val;
//	cout << "x_" << ret << " == d(v_" << leaf_ind << "," << val << "," << i << "," << theta << ")" << endl;
//	return ret;
}

int BinaryOneDepHourglass::p_var(int leaf_ind, int val1, int val2, int root_val) const {
	return number_of_d_variables + num_leafs*sigma_size*sigma_size*root_val +
			leaf_ind*sigma_size*sigma_size + val1*sigma_size + val2;
//	cout << "x_" << ret << " == p(v_" << leaf_ind << "," << val1 << "," << val2 << "," << root_val << ")" << endl;
//	return ret;
}

int BinaryOneDepHourglass::h_var() const {
	int ret = number_of_d_variables + number_of_p_variables;
//	cout << "x_" << ret << " = h" << endl;
	return ret;
}

int BinaryOneDepHourglass::w_r(int root_zero) const {
//	int ret = 2*num_leafs*sigma_size*sigma_size + root_zero;
	int ret = number_of_d_variables + number_of_p_variables + number_of_h_variables + root_zero;
//	cout << "x_" << ret << " == w_" << root_zero << endl;
	return ret;
}

int BinaryOneDepHourglass::get_num_vars() const {
	return number_of_d_variables + number_of_p_variables + number_of_h_variables +
	number_of_w_r_variables + number_of_w_v_variables + number_of_w_var_variables + number_of_m_variables;
}

int BinaryOneDepHourglass::bound_ind(int leaf_ind, int val, int theta) const {
	return theta*num_leafs*sigma_size + leaf_ind*sigma_size + val;
}

int BinaryOneDepHourglass::get_sigma_lower_bound(int leaf_ind, int val, int theta) const {
	return lower_bound[bound_ind(leaf_ind, val, theta)];
}

int BinaryOneDepHourglass::get_sigma_upper_bound(int leaf_ind, int val, int theta) const {
	return upper_bound[bound_ind(leaf_ind, val, theta)];
}
/////////////////////////////////

int BinaryOneDepHourglass::short_m_var(int x, int y, int theta) const {
	return x*parent_max_domain*2 + y*2 + theta;
}

int BinaryOneDepHourglass::short_par_var(int dom,int x, int y) const {
	return x*dom + y;
}
