#include "problem.h"
#include "d_operator.h"
#include "../utils/system.h"
#include "../abstract_task.h"
#include <vector>
#include <map>
#include <memory>
#include <math.h>
#include <fstream>

using namespace std;

// ---------------------------------------------------------------------------
// Helper: build a DOperator from new-FD AbstractTask data for one operator.
// op_idx: index in the task's operator list.  is_axiom: whether axiom.
// ---------------------------------------------------------------------------
static DOperator* build_doperator(const AbstractTask &task,
                                  int op_idx, bool is_axiom) {
    // Collect preconditions: var → required_value
    int num_pre = task.get_num_operator_preconditions(op_idx, is_axiom);
    map<int,int> pre_map;
    for (int i = 0; i < num_pre; ++i) {
        FactPair fp = task.get_operator_precondition(op_idx, i, is_axiom);
        pre_map[fp.var] = fp.value;
    }

    // Build PrePost list; track which vars are covered by an effect.
    int num_eff = task.get_num_operator_effects(op_idx, is_axiom);
    vector<PrePost> pre_post;
    set<int> effect_vars;
    for (int e = 0; e < num_eff; ++e) {
        FactPair eff = task.get_operator_effect(op_idx, e, is_axiom);
        int post_val = eff.value;
        int pre_val  = -1;
        auto it = pre_map.find(eff.var);
        if (it != pre_map.end()) {
            pre_val = it->second;
        }
        // Effect conditions → Prevail list in cond
        int num_cond = task.get_num_operator_effect_conditions(op_idx, e, is_axiom);
        vector<Prevail> cond;
        for (int c = 0; c < num_cond; ++c) {
            FactPair cf = task.get_operator_effect_condition(op_idx, e, c, is_axiom);
            cond.push_back(Prevail(cf.var, cf.value));
        }
        pre_post.push_back(PrePost(eff.var, pre_val, post_val, cond));
        effect_vars.insert(eff.var);
    }

    // Remaining preconditions with no effect → Prevail
    vector<Prevail> prevail;
    for (auto &kv : pre_map) {
        if (effect_vars.find(kv.first) == effect_vars.end()) {
            prevail.push_back(Prevail(kv.first, kv.second));
        }
    }

    double cost = (double)task.get_operator_cost(op_idx, is_axiom);
    string name = task.get_operator_name(op_idx, is_axiom);
    return new DOperator(is_axiom, prevail, pre_post, name, cost);
}

Problem::Problem(const shared_ptr<AbstractTask> &task) {
    borrowed_ops = false;
    cg = nullptr;
    CG_distances_matrix_computed = false;

    int num_vars = task->get_num_variables();

    // Variables
    vector<string> var_name;
    vector<int>    var_domain;
    for (int v = 0; v < num_vars; ++v) {
        var_name.push_back(task->get_variable_name(v));
        var_domain.push_back(task->get_variable_domain_size(v));
    }

    // Initial state
    initial_state_values = task->get_initial_state_values();

    // Goals
    vector<pair<int,int>> g;
    for (int i = 0; i < task->get_num_goals(); ++i) {
        FactPair fp = task->get_goal_fact(i);
        g.push_back({fp.var, fp.value});
    }

    // Operators
    vector<DOperator*> ops;
    for (int i = 0; i < task->get_num_operators(); ++i) {
        ops.push_back(build_doperator(*task, i, false));
    }

    // Axioms
    vector<DOperator> axi;
    for (int i = 0; i < task->get_num_axioms(); ++i) {
        DOperator* a = build_doperator(*task, i, true);
        axi.push_back(*a);
        delete a;
    }

    create_problem(var_name, var_domain, g, ops, axi);
    set_causal_graph(new SPCausalGraph(num_vars, operators, axioms));
}

Problem::Problem() {
    // Stub default constructor — only used internally (e.g. copy constructor path).
    // Real construction always goes through Problem(shared_ptr<AbstractTask>) or
    // the explicit-vector constructor.
    borrowed_ops = false;
    cg = nullptr;
    CG_distances_matrix_computed = false;
}

Problem::Problem(const vector<string>& var_name,
		const vector<int>& var_domain,
		const std::vector<int>& init_state_buffer,
		const vector<pair<int, int> >& g,
		const vector<DOperator*>& ops,
		const vector<DOperator>& axi) {

	borrowed_ops = true;
	initial_state_values = init_state_buffer;

	create_problem(var_name, var_domain, g, ops, axi);
	set_causal_graph(new SPCausalGraph(
	    static_cast<int>(variable_name.size()), operators, axioms));

	CG_distances_matrix_computed = false;
}

/* No Problem Copy Constructor currently present - need to implement Causal Graph Copy Constructor first.
Problem::Problem(const Problem& p) {
	is_cond = p.is_nonconditional();
	variable_name = p.get_variable_names();
	variable_domain = p.get_variable_domains();
	initial_state = new State(*(p.get_initial_state()));
	p.get_goal(goal);
	vector<Operator*> ops = p.get_operators();
	for (size_t i=0; i < ops.size(); ++i) {
		operators.push_back(new Operator(*(ops[i])));
	}
	cg = new CausalGraph(*(p.get_causal_graph()));
	p.get_DTGs(dtgs); 								// No Copying DTGs allowed.
	get_goal_vals(goal_vars);
	int num_vars = variable_name.size();
	v_operators.resize(num_vars);
	for (size_t it=0; it < operators.size(); ++it){
		const vector<PrePost> &pre = operators[it]->get_pre_post();
		for (size_t it2=0; it2 < pre.size(); ++it2) {
			int ind = pre[it2].var;
			assert(ind >= 0 && ind < num_vars);
			vector<Operator*> op;
			if (0 < v_operators[ind].size()) {
				op = v_operators[ind];
			}
			op.push_back(operators[it]);
			v_operators[ind] = op;
		}
	}
	axioms = p.get_axioms();
}
*/

Problem::~Problem() {
	delete_causal_graph();
	delete_operators();
	/*
	if (!borrowed_ops)
		for (size_t i=0; i < operators.size(); ++i)
			delete operators[i];
	*/
	if (CG_distances_matrix_computed)
		delete CG_distances_matrix;

}

void Problem::create_problem(const vector<string>& var_name,
							 const vector<int>& var_domain,
							 const vector<pair<int, int> >& g,
							 const vector<DOperator*>& ops,
							 const vector<DOperator>& axi) {

	variable_name = var_name;
	variable_domain = var_domain;
	goal = g;
//	int num_ops = ops.size();
	int num_vars = var_name.size();
	v_operators.resize(num_vars);
	axioms = axi;
//	cout << "Creating planning task over " << num_vars << " variables." << endl;
	// Adding index to the operator - by its position
	for (size_t i=0; i < ops.size(); ++i) {
		ops[i]->set_index(i);
		operators.push_back(ops[i]);
//		ops[i]->dump_no_name();
	}

	goal_vars.assign(num_vars,-1);
	for (size_t i=0; i < goal.size(); ++i)
		goal_vars[goal[i].first] = goal[i].second;

	// Partitioning operators by modified variables for quicker access.
	for (size_t it=0; it < operators.size(); ++it){
		const vector<PrePost> &pre = operators[it]->get_pre_post();
		vector<int> used;
		used.assign(num_vars,-1);
		for (size_t it2=0; it2 < pre.size(); ++it2) {
			int ind = pre[it2].var;
			assert(ind >= 0 && ind < num_vars);
			if (pre[it2].pre == pre[it2].post) {
				cout << "====> Check operator prepost condition " << endl;
				operators[it]->dump();
			}
			used[ind] = 0;
		}
		for(int v = 0; v < num_vars; v++) {
			if (used[v] < 0)
				continue;

			vector<DOperator*> op;
			if (0 < v_operators[v].size()) {
				op = v_operators[v];
			}
			op.push_back(operators[it]);
			v_operators[v] = op;
		}
	}
	nonconditional_task = check_nonconditional();
}

bool Problem::is_goal(const SPState &state) const {
	for (size_t i=0; i < goal.size(); ++i){
		if(state[goal[i].first] != goal[i].second)
			return false;
	}
	return true;
}


void Problem::generate_state_transition_graph(vector<vector<int> >& states) const {
	// The vector that is returned is of length equal to number of actions.
	// Each entry i consist of vector of integers, representing a set of states
	// in which action i is applicable.

	const vector<DOperator*> &ops = get_operators();
	for (size_t it=0; it < ops.size(); ++it) {
		// Per action we generate all the states this action is applicable in.
		vector<int> generated;
		get_applicable_states(ops[it],generated);
		states.push_back(generated);
	}
}

void Problem::get_applicable_states(DOperator* op, vector<int>& vals) const {

	const vector<Prevail> &prv = op->get_prevail();
	const vector<PrePost> &pre = op->get_pre_post();

	int num_vars = variable_domain.size();

	vals.assign(num_vars,-1);

	for (size_t i=0; i < prv.size(); ++i) {
		vals[prv[i].var] = prv[i].prev;
	}

	for (size_t i=0; i < pre.size(); ++i) {
		vals[pre[i].var] = pre[i].pre;
	}

}


void Problem::fill_DTG(int ) {
// TODO: Filling the DTG for variable var manually

}

void Problem::get_dtg_successors(int v, int val, vector<int>& vals) const {
	assert(vals.size() == 0);
	set<int> vals_set;
	// Getting successors from the actions
    const std::vector<DOperator*>& ops = get_var_actions(v);
    for (size_t a=0; a < ops.size(); ++a) {
    	// Works only with non conditional effects (check implemented within the called method)

    	int eff = ops[a]->get_eff_for_var_val(v, val);
    	if (eff != -1)
    		vals_set.insert(eff);

    }
    vals.insert(vals.begin(),vals_set.begin(),vals_set.end());
}


void Problem::get_domain_decomposition_by_distance_from_value(int v, int val, vector<vector<int> >& vals, vector<int>& len_from_val) const {
	// Distance from the given value. Calculated in BFS style.
	vector<int> first, open;

	int dom_size = get_variable_domain(v);

	len_from_val.assign(dom_size,-1);
	len_from_val[val] = 0;
	int counted = 1;

	first.push_back(val);
	get_dtg_successors(v,val,open);
//	dtgs[v]->get_successors(val,open);
	vals.push_back(first);
	vals.push_back(open);
	for (size_t i=0; i < open.size(); ++i) {
		len_from_val[open[i]] = 1;
		counted++;
	}

	// Open keeps the front (all values of distance k)
	while ((counted < dom_size) && (open.size() > 0)) {
		// The open list for next iteration
		vector<int> next;
		for (size_t i=0; i < open.size(); ++i) {
			// For each value of distance k we develop all its successors
			// If those successors were not developed sooner, then they are
			// of distance k+1
			vector<int> new_open;
			get_dtg_successors(v,open[i],new_open);
//			dtgs[v]->get_successors(open[i],new_open);
			for (size_t j=0; j < new_open.size(); ++j) {
				if (-1 == len_from_val[new_open[j]]) {
					// Updating the distance
					len_from_val[new_open[j]] = len_from_val[open[i]] + 1;
					next.push_back(new_open[j]);
					counted++;
				}
			}
		}
		vals.push_back(next);
		open = next;
	}

}

void Problem::get_domain_values_by_distance_to_goal(int v, vector<vector<int> >& vals, vector<int>& len_to_goal) const {

	int g_v = get_goal_val(v);
	vector<int> first, open;
	const vector<int> &doms = get_variable_domains();
	int dom_size = doms[v];

	len_to_goal.assign(dom_size,-1);
	len_to_goal[g_v] = 0;
	int counted = 1;

//	vector<int> pred[dom_size];
	vector<vector<int> > pred;
	pred.resize(dom_size);
	for (int i=0;i<dom_size;i++) {
		vector<int> succ;
		get_dtg_successors(v,i,succ);
//		dtgs[v]->get_successors(i,succ);

		for (size_t j=0; j < succ.size(); ++j){
			pred[succ[j]].push_back(i);
		}
	}


	first.push_back(g_v);

	open = pred[g_v];
	vals.push_back(first);
	vals.push_back(open);

	for (size_t i=0; i < open.size(); ++i) {
		if (-1 == len_to_goal[open[i]]) {
			len_to_goal[open[i]] = 1;
			counted++;
		}
	}

	// Open keeps the front (all values of distance k)
	while ((counted < dom_size) && (open.size() > 0)) {
		// The open list for next iteration
		vector<int> next;
		for (size_t i=0; i < open.size(); ++i) {
			// For each value of distance k we develop all its successors
			// If those successors were not developed sooner, then they are
			// of distance k+1
			vector<int> new_open = pred[open[i]];
			for (size_t j=0; j < new_open.size(); ++j) {
				if (-1 == len_to_goal[new_open[j]]) {
					// Updating the distance
					len_to_goal[new_open[j]] = len_to_goal[open[i]] + 1;
					next.push_back(new_open[j]);
					counted++;
				}
			}
		}
		vals.push_back(next);
		open = next;
	}
}


void Problem::get_cycle_free_paths_by_length(int v, int length, vector<vector<DOperator*> >& paths) const {

	// Returns all the paths to goal (if defined) of length <= the given bound
	if (length > 2){
		cout << "The greatest implemented bound is 2" << endl;    // To be implemented in the future
		exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}
	int g_v = get_goal_val(v);
	if (-1==g_v) return;     // No goal

	const vector<DOperator*> &A_v = get_var_actions(v);


	// Getting the paths of length 1.
	for (size_t a=0; a < A_v.size(); ++a) {
		if (g_v == A_v[a]->get_post_val(v)) {
			vector<DOperator*> ops;
			ops.push_back(A_v[a]);
			paths.push_back(ops);
		}
	}
	if (length == 1) return;

	// This is for the ternary case only, general algorithm will be implemented in the future
	int sz = paths.size();
	for (int i=0; i < sz; i++) {
		// Going over the paths of length 1
		vector<DOperator*> ops = paths[i];
		int new_g = ops[0]->get_pre_val(v);
		if (-1 == new_g) {
			// We don't need to extend this path further.
			continue;
		}
		for (size_t a=0; a < A_v.size(); ++a) {
			if (new_g == A_v[a]->get_post_val(v)) {
				// Checking for loops - in general case cycle free paths.
				if (g_v != A_v[a]->get_pre_val(v)) {
					vector<DOperator*> new_ops;
					new_ops.push_back(A_v[a]); // In general case - insert new operator first
					new_ops.push_back(ops[0]);
					paths.push_back(new_ops);
				}
			}
		}
	}
}

bool Problem::is_interfere(vector<Prevail>& pre1, vector<Prevail>& pre2) const {
	// Check if two preconditions interfere
	for (size_t i=0; i < pre1.size(); ++i) {
		for (size_t j=0; j < pre2.size(); ++j) {
			if (pre1[i].var == pre2[j].var && pre1[i].prev != pre2[j].prev)
				return true;
		}
	}
	return false;
}

void Problem::create_operators_non_interfering_effects(const vector<DOperator*>& ops, vector<DOperator*>& to_ops) const {
	assert(to_ops.size() == 0);

	for (size_t i=0; i < ops.size(); ++i) {
		vector<DOperator*> splitted, to_remove;
		DOperator* to_split = ops[i];
		split_operator_interfering_effects(to_split, splitted);
		while (splitted.size() > 0) {
			assert(splitted.size() == 2);
			to_split = splitted[1];
			to_remove.push_back(to_split);
			to_ops.push_back(splitted[0]);
			splitted.clear();
			split_operator_interfering_effects(to_split, splitted);
		}
		if (to_remove.size() > 0)
			to_remove.pop_back();
		to_ops.push_back(to_split);
		for (size_t j=0; j < to_remove.size(); ++j) {
			delete to_remove[j];
		}
	}
}

void Problem::split_operator_interfering_effects(DOperator* op, vector<DOperator*>& ops) const {
	assert(ops.size() == 0);
	// Creates two new operators, enters both into ops. Need to take care of deleting them later
	// The first conditional effect that interferes with the rest is splitted.
	vector<PrePost> pp = op->get_pre_post();
	if (pp.size() < 2)
		return;
	for (size_t i=0; i < pp.size(); ++i) {
		// Check if condition interfere with all other
		bool interfere = true;
		for (size_t j=0; j < pp.size(); ++j) {
			if (i == j)
				continue;

			if (!is_interfere(pp[i].cond,pp[j].cond)) {
				interfere = false;
				break;
			}
		}

		if (interfere) {
			// Breaking the action into two
			vector<Prevail> pre = op->get_prevail();
			vector<PrePost> eff1, eff2;

			for (size_t j=0; j < pp.size(); ++j) {
				if (i == j) {
					eff1.push_back(pp[j]);
				} else {
					eff2.push_back(pp[j]);
				}
			}
			DOperator* new_op1 = new DOperator(op->is_axiom(),pre, eff1,op->get_name(),op->get_double_cost());
			new_op1->set_index(op->get_index());
			ops.push_back(new_op1);
			DOperator* new_op2 = new DOperator(op->is_axiom(),pre, eff2,op->get_name(),op->get_double_cost());
			new_op2->set_index(op->get_index());
			ops.push_back(new_op2);

			break;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////
/*
void Problem::compute_CG_distances_matrix() {
	if (CG_distances_matrix_computed)
		return;

	int num_vars = get_vars_number();

	CG_distances_matrix - new int*[num_vars];
	for (int i=0;i<num_vars;i++) {
		CG_distances_matrix[i] = new int[num_vars];
		for (int j=0;j<num_vars;j++) {
			CG_distances_matrix[i][j] = -1;
		}
	}
	for (int i=0;i<num_vars;i++) {
		CG_distances_matrix[i][i] = 0;
		const vector<int>& succ = get_causal_graph()->get_successors(i);
		for (size_t j=0; j < succ.size(); ++j) {
			CG_distances_matrix[i][succ[j]] = 1;
		}
	}
	for (int k=0;k<num_vars;k++) {
		for (int i=0;i<num_vars;i++) {
			for (int j=0;j<num_vars;j++) {
				if (CG_distances_matrix[i][k] == -1 || CG_distances_matrix[k][j] == -1)  // No shorter path this way
					continue;
				CG_distances_matrix[i][j] = min(CG_distances_matrix[i][j],CG_distances_matrix[i][k] + CG_distances_matrix[k][j]);
			}
		}
	}

	CG_distances_matrix_computed = true;
}

bool Problem::path_in_CG_from_to_exists(int a, int b) {
	compute_CG_distances_matrix();
	return (CG_distances_matrix[a][b] != -1);
}
*/

void Problem::compute_connected_variables_bounded_sets(int v, int bound, vector<vector<int> >& vals) const {

	set<set<int> > possible_sets;
	set<int> curr;
	compute_hat_variants(v, curr, bound, possible_sets);

	// For each possible hat, two checks are performed:
	// 1. Checking whether the hat does not consist definite leaf variables. If the variable
	//    is not connected to the rest of the hat and has no outgoing edge to c, then it is a definite leaf.
	// 2. If there exists a non goal variable, with no path from it to any goal variable or to the center,
	//    then the hat is not valid.

	set<set<int> > valid_sets;

	for (set<set<int> >::iterator it=possible_sets.begin(); it != possible_sets.end(); ++it) {
		if (chech_hat_for_leafs(v,(*it)))
			continue;

		if (chech_hat_connected_to_goals(v,(*it))) {
			// The set is valid, adding vector to the final result
			valid_sets.insert((*it));
		}

	}
	// Getting rid of the subsets of the existing sets - removing dominated patterns
	for (set<set<int> >::iterator it=valid_sets.begin(); it != valid_sets.end(); ++it) {
		// Chech whether subset of some other
		bool is_subset = false;
		for (set<set<int> >::iterator it2=valid_sets.begin(); it2 != valid_sets.end(); ++it2) {
			if (it == it2)
				continue;

			set<int> unionset = (*it2);
			unionset.insert((*it).begin(),(*it).end());
			if (unionset.size() == (*it2).size()) {
				is_subset = true;
				break;
			}
		}
		if (!is_subset) {
			vector<int> res;
			res.insert(res.begin(),(*it).begin(),(*it).end());
			vals.push_back(res);
		}
	}

}

bool Problem::chech_hat_for_leafs(int v, const set<int>& hat) const {
	// 1. Checking whether the hat does not consist definite leaf variables. If the variable
	//    is not connected to the rest of the hat and has no outgoing edge to c, then it is a definite leaf.
	for (set<int>::iterator it=hat.begin(); it != hat.end(); ++it) {
		int var = *it;
		const vector<int>& succ = get_causal_graph()->get_successors(var);
		bool arc_to_center = false;
		for (size_t i=0; i < succ.size(); ++i) {
			if (succ[i] == v) {
				arc_to_center = true;
				break;
			}
		}
		if (arc_to_center)
			continue;

		// Checking whether connected to other hat vars
		const vector<int>& pred = get_causal_graph()->get_predecessors(var);
		bool neigh_connected = false;
		for (size_t i=0; i < pred.size(); ++i) {
			if (hat.find(pred[i]) != hat.end()) {
				neigh_connected = true;
				break;
			}
		}
		if (neigh_connected)
			continue;
		for (size_t i=0; i < succ.size(); ++i) {
			if (hat.find(succ[i]) != hat.end()) {
				neigh_connected = true;
				break;
			}
		}
		if (neigh_connected)
			continue;

		return true;
	}
	return false;
}

bool Problem::chech_hat_connected_to_goals(int v, const set<int>& hat) const {

	// 2. If there exists a non goal variable, with no path from it to any goal variable or to the center,
	//    then the hat is not valid.
	// The algorithm starts from the set of goals and the center, adding predecessors, until fixpoint is reached.
	// if all the set is in, then the set is valid.
	vector<bool> curr_vec, iterating_vec;
	curr_vec.assign(get_vars_number(),false);
	iterating_vec.assign(get_vars_number(),false);

	set<int> connected, iterating;
	connected.insert(v);
	iterating.insert(v);
	iterating_vec[v] = true;

	for (set<int>::iterator it2=hat.begin(); it2 != hat.end(); ++it2) {
		curr_vec[*it2] =  true;
		if (is_goal_var(*it2)) {
			connected.insert(*it2);
			iterating.insert(*it2);
			iterating_vec[*it2] = true;
		}
	}
	// Iterating over the elements of the "connected" set, add new connected elements
	while (connected.size() <= hat.size()) {
		set<int> new_connected;
		for (set<int>::iterator it3=iterating.begin(); it3 != iterating.end(); ++it3) {
			const vector<int>& pred = get_causal_graph()->get_predecessors(*it3);
			for (size_t i=0; i < pred.size(); ++i) {
				if (curr_vec[pred[i]] && !iterating_vec[pred[i]]) {
					// New element found
					iterating_vec[pred[i]] = true;
					new_connected.insert(pred[i]);
				}
			}
		}
		// If new_connected is empty, then break - no new element is found.
		if (new_connected.size() == 0)
			break;
		// Moving new_connected into iterating, and adding them to the connected
		iterating = new_connected;
		connected.insert(new_connected.begin(),new_connected.end());
	}
	return (connected.size() > hat.size());
}



void Problem::compute_hat_variants(int c, set<int>& curr, int bound, set<set<int> >& vals) const {
//	cout << "------------------------------------------------------" << endl;
//	cout << "Current call with:" << endl;
//	cout << "c=" << c << endl;
//	cout << "curr =" ;
//	for (set<int>::iterator it=curr.begin(); it != curr.end(); ++it) {
//		cout << " " << *it;
//	}
//	cout << endl << "vals =" << endl;
//	for (set<set<int> >::iterator it=vals.begin(); it != vals.end(); ++it) {
//		for (set<int>::iterator it2=(*it).begin(); it2 != (*it).end(); ++it2) {
//			cout << " " << *it2;
//		}
//		cout << endl;
//	}
//	cout << "------------------------------------------------------" << endl;

	if (curr.size() > 0) {
		pair<set<set<int> >::iterator, bool> res = vals.insert(curr);
		if (!res.second) // Wasn't inserted, already existed.
			return;
	}
	if ((int)curr.size() == bound) // Bound is reached
		return;
//cout << "curr size = 0" << endl;
	// Creating a set of elements to add - neighbours of all elements currently in the set and the center, excluding those that are already there.
	set<int> toadd;
	const vector<int>& pred = get_causal_graph()->get_predecessors(c);
	toadd.insert(pred.begin(), pred.end());
	const vector<int>& succ = get_causal_graph()->get_successors(c);
	toadd.insert(succ.begin(), succ.end());
	for (set<int>::iterator it=curr.begin(); it != curr.end(); ++it) {
		const vector<int>& curr_pred = get_causal_graph()->get_predecessors(*it);
		toadd.insert(curr_pred.begin(), curr_pred.end());
		const vector<int>& curr_succ = get_causal_graph()->get_successors(*it);
		toadd.insert(curr_succ.begin(), curr_succ.end());
	}
	for (set<int>::iterator it=curr.begin(); it != curr.end(); ++it) {
		toadd.erase(*it);
	}
	toadd.erase(c);
//	cout << "Adding variables:" ;
//	for (set<int>::iterator it=toadd.begin(); it != toadd.end(); ++it) {
//		cout << " " <<*it;
//	}
//	cout << endl;

	for (set<int>::iterator it=toadd.begin(); it != toadd.end(); ++it) {
		// Recursive call with one more element added to the set
		curr.insert(*it);
		compute_hat_variants(c,curr,bound, vals);
		curr.erase(*it);
	}

}



/////////////////////////////////////////////////////////////////////////////////////////////
// Returns all cycle free paths from any value to the goal
// Currently used for getting the paths of the root of inverted fork task.
// For that it is sufficient to check if the effects are all on the same variable and split
// them into separate actions. Currently the behavior is more general...
// New single effect operators are created
void Problem::get_all_cycle_free_paths_to_goal(int v, vector<vector<DOperator*> >& paths) const {
	//TODO: This function is a mess, rewrite it.
	int g_v = get_goal_val(v);
	if (-1==g_v) return;     // No goal

	vector<DOperator*> to_ops;
//	cout << "Creating operators with non interfering effects from the operators" << endl;
	const vector<DOperator*>& ops = get_var_actions(v);
//	for (size_t i=0; i < ops.size(); ++i) {
//		ops[i]->dump();
//	}
//	cout << "Result:" << endl;
	create_operators_non_interfering_effects(ops, to_ops);
	for (size_t i=0; i < to_ops.size(); ++i) {
		// Simplifying operators
		if (to_ops[i]->is_single_effect())
			to_ops[i]->simplify_single_effect();
//		to_ops[i]->dump();
	}
//	cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
	get_all_cycle_free_paths_to_goal_ops(v, to_ops, paths);
//	cout << "Found paths:" << endl;
//	for (size_t i=0; i < paths.size(); ++i) {
//		cout << "Path " << i << endl;
//		for (size_t j=0; j < paths[i].size(); ++j) {
//			paths[i][j]->dump();
//		}
//		cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
//	}
//	cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;

}

void Problem::get_all_cycle_free_paths_to_goal_ops(int v, vector<DOperator*>& A_v, vector<vector<DOperator*> >& paths) const {

	int g_v = get_goal_val(v);
	if (0 == paths.size()) {
		// Getting the paths of length 1 - to start with.
		for (size_t a=0; a < A_v.size(); ++a) {
			if (g_v == A_v[a]->get_post_val(v)) {
				vector<DOperator*> ops;
				ops.push_back(A_v[a]);
				paths.push_back(ops);
			}
		}

	}

	for (size_t i=0; i < paths.size(); ++i) {
		// Going over all the paths found so far and trying to expand each one
		vector<DOperator*> ops = paths[i];
		if (variable_domain[v] - 1 == (int)ops.size()) {
			// We don't need to extend this path further.
			continue;
		}
		int new_g = ops[0]->get_pre_val(v);
		if (-1 == new_g) {
			// We don't need to extend this path further.
			continue;
		}
		for (size_t a=0; a < A_v.size(); ++a) {
			if (new_g != A_v[a]->get_post_val(v))
				continue;

			// Checking for loops - in general case cycle free paths.
			int new_pre = A_v[a]->get_pre_val(v);
			if (-1 != new_pre) {
				// Going over the path, checking if the new value is not an effect of any action
				bool is_loop = false;
				for (size_t j=0; j < ops.size(); ++j) {
					if (new_pre == ops[j]->get_post_val(v)) {
						is_loop = true;
						break;
					}
				}
				if (is_loop)
					continue;

				// Build a new path and try to extend it
				vector<DOperator*> new_path = ops;
				new_path.insert(new_path.begin(), A_v[a]);
				vector<vector<DOperator*> > new_paths;
				new_paths.push_back(new_path);
				get_all_cycle_free_paths_to_goal_ops(v, A_v, new_paths);

				// Add the new paths to the previously found
				for (size_t p=0; p < new_paths.size(); ++p) {
					paths.push_back(new_paths[p]);
				}
			} else {
				// Add the -1 ending path to the previously found
				vector<DOperator*> new_path = ops;
				new_path.insert(new_path.begin(), A_v[a]);
				paths.push_back(new_path);
			}
		}
	}
}



/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
// Returns the number of all cycle free paths from any value to the goal
int Problem::get_estimated_number_of_all_cycle_free_paths_to_goal(int v) const {
	// Estimation Algorithm from
	// B. Roberts and D. P. Kroese, Estimating the Number of s-t Paths in a Graph, Journal of Graph Algorithms and Applications, vol. 11, no. 1, pp. 195–214 (2007)
	int g_v = get_goal_val(v);
	if (-1==g_v) return 0;     // No goal

	const vector<DOperator*> &A_v = get_var_actions(v);

	int dom = get_variable_domain(v);

	int** A = new int *[dom];
	for( int i = 0 ; i < dom ; i++ )
		A[i] = new int [dom];

	for( int i = 0 ; i < dom ; i++ )
		for( int j = 0 ; j < dom ; j++ )
			A[i][j] = 0;

	// Building the adjacent matrix
	for (size_t a=0; a < A_v.size(); ++a) {
		int pre = A_v[a]->get_pre_val(v);
		int post = A_v[a]->get_post_val(v);

		int start_val = (-1 == pre) ? 0 : pre;
		int end_val = (-1 == pre) ? dom : pre;

		for (int val=start_val;val<=end_val;val++) {
			if (val != post) {
				A[val][post] ++;
			}
		}
	}

	for( int i = 0 ; i < dom ; i++ ) {
		for( int j = 0 ; j < dom ; j++ )
			cout << A[i][j] << " " ;

		cout << endl;
	}

	int num_paths = 0;
	int N = 100;
	for( int i = 0 ; i < dom ; i++ ) {
		// For each value in the domain, except the goal, estimate the number of paths to the goal
		if (i == g_v)
			continue;
		// Building an adjacent matrix for i - removing links to i
		vector<vector<int> > adj;
		adj.resize(dom);

		for( int j = 0 ; j < dom ; j++ ) {
			for( int k = 0 ; k < dom ; k++ ) {
				if (k == i)
					continue;
				if (A[j][k] > 0)
					adj[j].push_back(k);
			}
		}

		// Running N times, counting the total value.
		// Estimated number of paths from i is num_paths/N
		for (int step = 0; step < N; step++) {
			vector<vector<int> > new_adj = adj;
			int curr = i;
			int val = 1;
			while (curr != g_v) {
				int num_ch = new_adj[curr].size();
				if (num_ch == 0) {
					val = 0;
					break;
				}
				int tmp_val = 1;
				for( int ch = 0 ; ch < num_ch ; ch++ ) {
					int to = new_adj[curr][ch];
					tmp_val = tmp_val*A[curr][to];
				}
				val = val*tmp_val;
				int next = rand() % num_ch;
				curr = new_adj[curr][next];
				// Update adjacent matrix - remove links to new current
				for( int j = 0 ; j < dom ; j++ ) {
					for (size_t k=0; k < new_adj[j].size(); ++k) {
						if (new_adj[j][k] == curr) {
							new_adj[j].erase(new_adj[j].begin()+k);
							break;
						}
					}
				}
			}
			num_paths += val;
		}
	}

	for( int i = 0 ; i < dom ; i++ )
		delete [] A[i];

	delete [] A;

	return static_cast<int>(ceil(num_paths/N));
}

// Returns true if the number of all cycle free paths from any value to the goal is under the given bound
bool Problem::is_estimated_number_of_all_cycle_free_paths_to_goal_bounded(int v, int bound) const {
	// Estimation Algorithm from
	// B. Roberts and D. P. Kroese, Estimating the Number of s-t Paths in a Graph, Journal of Graph Algorithms and Applications, vol. 11, no. 1, pp. 195–214 (2007)
	int g_v = get_goal_val(v);
	if (-1==g_v) return 0;     // No goal

	const vector<DOperator*> &A_v = get_var_actions(v);

	int dom = get_variable_domain(v);

	int** A = new int *[dom];
	for( int i = 0 ; i < dom ; i++ )
		A[i] = new int [dom];

	for( int i = 0 ; i < dom ; i++ )
		for( int j = 0 ; j < dom ; j++ )
			A[i][j] = 0;

	// Building the adjacent matrix
	for (size_t a=0; a < A_v.size(); ++a) {
		int pre = A_v[a]->get_pre_val(v);
		int post = A_v[a]->get_post_val(v);

		int start_val = (-1 == pre) ? 0 : pre;
		int end_val = (-1 == pre) ? dom : pre;

		for (int val=start_val;val<=end_val;val++) {
			if (val != post) {
				A[val][post] ++;
			}
		}
	}

	for( int i = 0 ; i < dom ; i++ ) {
		for( int j = 0 ; j < dom ; j++ )
			cout << A[i][j] << " " ;

		cout << endl;
	}

	bool is_bounded = true;
	int num_paths = 0;
	int N = 100;
	for( int i = 0 ; i < dom ; i++ ) {
		// For each value in the domain, except the goal, estimate the number of paths to the goal
		if (i == g_v)
			continue;
		// Building an adjacent matrix for i - removing links to i
		vector<vector<int> > adj;
		adj.resize(dom);

		for( int j = 0 ; j < dom ; j++ ) {
			for( int k = 0 ; k < dom ; k++ ) {
				if (k == i)
					continue;
				if (A[j][k] > 0)
					adj[j].push_back(k);
			}
		}

		// Running N times, counting the total value.
		// Estimated number of paths from i is num_paths/N
		for (int step = 0; step < N; step++) {
			vector<vector<int> > new_adj = adj;
			int curr = i;
			int val = 1;
			while (curr != g_v) {
				int num_ch = new_adj[curr].size();
				if (num_ch == 0) {
					val = 0;
					break;
				}
				int tmp_val = 1;
				for( int ch = 0 ; ch < num_ch ; ch++ ) {
					int to = new_adj[curr][ch];
					tmp_val = tmp_val*A[curr][to];
				}
				val = val*tmp_val;
				int next = rand() % num_ch;
				curr = new_adj[curr][next];
				// Update adjacent matrix - remove links to new current
				for( int j = 0 ; j < dom ; j++ ) {
					for (size_t k=0; k < new_adj[j].size(); ++k) {
						if (new_adj[j][k] == curr) {
							new_adj[j].erase(new_adj[j].begin()+k);
							break;
						}
					}
				}
			}
			num_paths += val;
		}
		if (num_paths/N > bound) {
			is_bounded = false;
			break;
		}
	}

	for( int i = 0 ; i < dom ; i++ )
		delete [] A[i];

	delete [] A;

	return is_bounded;
}





void Problem::set_operators_to_uniform_cost() {
	for (size_t a=0; a < operators.size(); ++a) {
		operators[a]->set_double_cost(1.0);
	}
}


void Problem::increase_operators_cost() {
	for (size_t a=0; a < operators.size(); ++a) {
		operators[a]->set_double_cost(operators[a]->get_double_cost() + 1.0);
	}
}


bool Problem::check_nonconditional() {

	for (size_t it=0; it < operators.size(); ++it){
		const vector<PrePost> &pre = operators[it]->get_pre_post();
		for (size_t it2=0; it2 < pre.size(); ++it2)
			if (pre[it2].cond.size() > 0) return false;
	}
	return true;
}


bool Problem::has_goal_child(int var) const {

	const vector<int>& successors = get_causal_graph()->get_successors(var);

	for (size_t ch=0; ch < successors.size(); ++ch) {
		if (is_goal_var(successors[ch])) {
			return true;
		}
	}
	return false;
}


int Problem::get_var_index(string var_name) const {
	for (size_t var=0; var < variable_name.size(); ++var) {
		if (var_name==variable_name[var])
			return var;
	}
	return -1;
}





void Problem::print_conditional() const {

	for (size_t it=0; it < operators.size(); ++it){
		const vector<PrePost> &pre = operators[it]->get_pre_post();
		bool condeff = false;
		for (size_t it2=0; it2 < pre.size(); ++it2) {
			if (pre[it2].cond.size() > 0) {
				condeff = true;
			}
		}
		if (condeff) {
			operators[it]->dump();
		}
	}
}


void Problem::dump() const {
	cout << "Variables:" << endl;
	for (size_t i=0; i < variable_domain.size(); ++i)
		cout << "  " << i << ":  " << variable_name[i] << " " << variable_domain[i] << endl;

	cout << "Actions:" << endl;
	for (size_t k=0; k < operators.size(); ++k) {
		cout << "  ";

	    cout << operators[k]->get_name() << ":";
	    const std::vector<Prevail> &prevail = operators[k]->get_prevail();
	    for (size_t i=0; i < prevail.size(); ++i) {
	        cout << " [";
	        cout << get_variable_name(prevail[i].var) << ": " << prevail[i].prev;
	        cout << "]";
	    }
	    const std::vector<PrePost> &pre_post = operators[k]->get_pre_post();
	    for (size_t i=0; i < pre_post.size(); ++i) {
	        cout << " [";

	        cout << get_variable_name(pre_post[i].var) << ": " << pre_post[i].pre << " => " << pre_post[i].post;
	        const std::vector<Prevail> &cond = pre_post[i].cond;
	        if (cond.size() > 0) {
	            cout << " if";
	            for (size_t j=0; j < cond.size(); ++j) {
	                cout << " ";
	                cout << get_variable_name(cond[j].var) << ": " << cond[j].prev;
	            }
	        }
	        cout << "]";
	    }
	    cout << " " << operators[k]->get_double_cost();
	    cout << endl;


	}

	cout << "Initial State:" << endl;
	const SPState &init = get_initial_state();
	for (size_t i=0; i < init.size(); ++i)
		cout << "  var" << i << " = " << init[i] << endl;

	cout << "Goals:" << endl;
	for (size_t i=0; i < goal.size(); ++i)
		cout << "  " << goal[i].first << ":  " << goal[i].second << endl;
/*
	cg->dump();

	cout << "Domain Transition Graphs:" << endl;
		for (size_t i=0; i < dtgs.size(); ++i){
			//dtgs[i]->dump2();
			dtgs[i]->dump();
		}
*/
}

void Problem::dumpPrevail(Prevail pre) const {
	assert(pre.var >= 0);
	assert(pre.var < (int)variable_name.size());
	cout << "Variable " << pre.var << " out of " << variable_name.size() << endl;
    cout << get_variable_name(pre.var) << ": " << pre.prev;
}

void Problem::dumpPrePost(PrePost pp) const {
	assert(pp.var >= 0);
	assert(pp.var < (int)variable_name.size());
	cout << "Variable " << pp.var << " out of " << variable_name.size() << endl;
    cout << get_variable_name(pp.var) << ": " << pp.pre << " => " << pp.post;
    if (!pp.cond.empty()) {
        cout << " if";
        for (size_t i=0; i < pp.cond.size(); ++i) {
            cout << " ";
            dumpPrevail(pp.cond[i]);
        }
    }
}

void Problem::dumpAction(const DOperator* op) const {
    cout << op->get_name() << ":";
    const std::vector<Prevail> &prevail = op->get_prevail();

    for (size_t i=0; i < prevail.size(); ++i) {
    	cout << " [";
    	dumpPrevail(prevail[i]);
    	cout << "]";
    }
    const std::vector<PrePost> &pre_post = op->get_pre_post();
    for (size_t i=0; i < pre_post.size(); ++i) {
    	cout << " [";
    	dumpPrePost(pre_post[i]);
    	cout << "]";
    }
    cout << " " << op->get_double_cost();
    cout << endl;
}


void Problem::dump_SAS(const char* filename) const {
	ofstream os(filename);
	os << "begin_metric" << endl;
	os << 0 << endl;  // metric flag (always 0 — cost handled elsewhere)
	os << "end_metric" << endl;

	os << "begin_variables" << endl;
	os << variable_domain.size() << endl;
	for (size_t i=0; i < variable_domain.size(); ++i)
		os << variable_name[i] << " " << variable_domain[i] << " -1" << endl;
	os << "end_variables" << endl;

	os << "begin_state" << endl;
	const SPState &state = get_initial_state();
	for (size_t i=0; i < variable_domain.size(); ++i)
		os << state[i] << endl;
	os << "end_state" << endl;

	os << "begin_goal" << endl;
	os << goal.size() << endl;
	for (size_t i=0; i < goal.size(); ++i)
		os << goal[i].first << " " << goal[i].second << endl;
	os << "end_goal" << endl;

	os << operators.size() << endl;
	for (size_t i=0; i < operators.size(); ++i) {
		operators[i]->dump_SAS(os);
	}
	os << axioms.size() << endl;
	for (size_t i=0; i < axioms.size(); ++i) {
		axioms[i].dump_SAS(os);
	}
}

/*
void Problem::make_single_goal() {
	// Adding a dummy goal variable
	g_use_metric = true;
	string nm = "var99999";
	variable_domain.push_back(2);
	variable_name.push_back(nm);

	// Change initial state, add action, set goal
	int var_count = variable_domain.size();
	state_var_t* buf = new state_var_t[var_count];

	for(int i = 0; i < var_count-1; i++) {
		buf[i] = initial_state->get_buffer()[i];
	}
	buf[var_count-1] = 0;
	initial_state = new State(buf);

	vector<Prevail> prv, cond;
	for (size_t i=0; i < goal.size(); ++i) {
		prv.push_back(Prevail(goal[i].first,goal[i].second));
	}
	nm = "Reach " + nm;
	vector<PrePost> pre;
	pre.push_back(PrePost(var_count-1, 0, 1, cond));
	DOperator* op = new DOperator(false,prv,pre,nm,0);
	operators.push_back(op);

	goal.clear();
	goal.push_back(make_pair(var_count-1,1));
}
*/

void Problem::delete_operators(){
	if (!borrowed_ops) {

		for (size_t it=0; it < operators.size(); ++it){
			if (operators[it]) delete operators[it];
		}
	}
	operators.clear();
	for (size_t it=0; it < v_operators.size(); ++it){
		v_operators[it].clear();
	}
	v_operators.clear();
}

void Problem::delete_causal_graph() {
	if (cg)
		delete cg;
}


void Problem::dump_CG_dot(const char* filename) const {
	ofstream os(filename);
	os << "Digraph CG {" << endl;
    for (int i = 0; i < get_vars_number(); i++)
    	if (is_goal_var(i))
    		os << i << " [shape=doublecircle];" << endl;
    	else
    		os << i << " [shape=circle];" << endl;

    for (int i = 0; i < get_vars_number(); i++) {
		const vector<int>& succ = get_causal_graph()->get_successors(i);

        for (size_t j=0; j < succ.size(); ++j)
            os << i << " -> " << succ[j] << ";" << endl;
    }
    os << "}" << endl;
}
int Problem::get_max_domain_size() const {
	int max_size = 0;
    for (int i = 0; i < get_vars_number(); i++) {
    	if (variable_domain[i] > max_size)
    		max_size = variable_domain[i];
    }
    return max_size;
}
