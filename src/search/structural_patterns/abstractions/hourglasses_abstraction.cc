#include "../SP_globals.h"
#include "hourglasses_abstraction.h"
#include "../mappings/op_hash_var_proj_mapping.h"
#include "../mappings/op_hash_mapping.h"
#include "../mappings/var_proj_mapping.h"
#include <string>
// causal_graph: use task_utils

using namespace std;

HourglassesAbstraction::HourglassesAbstraction() : GeneralAbstraction() {
	is_singleton = false;
	is_empty = false;
	is_var_singleton = false;
	set_abstraction_type(HOURGLASS);
}

HourglassesAbstraction::HourglassesAbstraction(int v, const vector<int>& parent_vars) : GeneralAbstraction() {
	is_singleton = false;
	is_empty = false;
	is_var_singleton = false;
	set_variable_and_parents(v, parent_vars);
	set_abstraction_type(HOURGLASS);
}


HourglassesAbstraction::~HourglassesAbstraction() {
}


void HourglassesAbstraction::set_variable_and_parents(int v, const vector<int>& parent_vars) {
	var = v;
	parents = parent_vars;
}


int HourglassesAbstraction::get_root() const {
	return var;
}

bool HourglassesAbstraction::is_empty_abstraction() const {
	return is_empty;

}

bool HourglassesAbstraction::is_singleton_abstraction() const {
	return is_singleton;
}


void HourglassesAbstraction::create(const Problem* p) {
	if (get_abstraction_creation_status() == DONE)
		return;


	// Creating abstract variables, initial state.
	const vector<string> &var_name = p->get_variable_names();
	const vector<int> &var_domain = p->get_variable_domains();

	const vector<int> &successors = p->get_causal_graph()->get_successors(var);
	const vector<int> &predecessors = p->get_causal_graph()->get_predecessors(var);

	if (0 == successors.size() && 0 == predecessors.size()) {
		is_var_singleton = true;
	}

	int num_vars = var_name.size();
	vector<int> abs_vars;
	vector<int> orig_vars;
	orig_vars.push_back(var);

	// create the leafs, and the connection between original variables and abstract variables.
	// First abstract variable is the center, then hat, and then leafs
	is_parent.assign(num_vars, false);
	abs_parents.clear();
	for (size_t i=0; i < parents.size(); ++i) {
		is_parent[parents[i]] = true;
		abs_parents.push_back(orig_vars.size());
		orig_vars.push_back(parents[i]);
	}
	abs_leafs.clear();
	// Leafs are defined to be all goal successors that are not in the hat.
	for (size_t i=0; i < successors.size(); ++i) {
		if ((p->is_goal_var(successors[i])) &&
			(!is_parent[successors[i]])) {
			leafs.push_back(successors[i]);  //saving the leafs with goals
			abs_leafs.push_back(orig_vars.size());
			orig_vars.push_back(successors[i]);
		}
	}

	if (1==orig_vars.size()) {
		if (!p->is_goal_var(var)) {
			is_empty = true;
			return;
		}
		else
			is_singleton = true;
	}

	//Creating the empty mapping
	OpHashVarProjMapping* map = new OpHashVarProjMapping();
	map->set_original(p);
	map->set_original_vars(orig_vars);

	abs_vars.assign(num_vars,-1);

	int var_count = orig_vars.size();

	vector<string> new_var_name;
	vector<int> new_var_domain;

	new_var_name.resize(var_count);
	new_var_domain.resize(var_count);

//	cout << "Hourglass variables:" << endl;
	for(int i = 0; i < var_count; i++) {
		abs_vars[orig_vars[i]] = i;
		new_var_name[i] = var_name[orig_vars[i]];
		new_var_domain[i] = var_domain[orig_vars[i]];
//		cout << "Name: " << new_var_name[i] << ", domain: " << new_var_domain[i] << ", ID: " << orig_vars[i] << endl;
	}
	abs_root_var = abs_vars[var];

	map->set_abstract_vars(abs_vars);
	vector<int> init_state_buffer(var_count);

	for(int i = 0; i < var_count; i++) {
		init_state_buffer[i] = p->get_initial_state()[orig_vars[i]];
	}

	// Creating abstract goal
	vector<pair<int, int> > orig_goal;
	p->get_goal(orig_goal);
	vector<pair<int, int> > g;
	for (size_t i=0; i < orig_goal.size(); ++i){
		if (abs_vars[orig_goal[i].first] != -1){
			pair<int, int> new_goal;
			new_goal.first = abs_vars[orig_goal[i].first];
			new_goal.second = orig_goal[i].second;
			g.push_back(new_goal);
		}
	}

	// Creating abstract action
	const vector<DOperator*> &orig_ops = p->get_operators();

	vector<DOperator*> ops, axioms;
	vector<pair<DOperator*, DOperator*> > ops_to_add;

	// Adding axioms as zero-cost operators
	vector<DOperator> axi = p->get_axioms();
	for (size_t i=0; i < axi.size(); ++i){
		cout << "XXXXXXXX" << endl;
		axioms.push_back(&axi[i]);
	}

	abstract_actions(abs_vars,orig_ops,ops,ops_to_add);
	abstract_actions(abs_vars,axioms,ops,ops_to_add);

	vector<DOperator> no_axioms;
	Problem* abs_prob = new Problem(new_var_name, new_var_domain, init_state_buffer, g, ops, no_axioms);

	map->set_abstract(abs_prob);
	map->initialize();
//	abs_prob->dump();

//	cout << "Actions added to the problem" << endl;
	for (size_t i=0; i < ops_to_add.size(); ++i) {
		map->add_abs_operator(ops_to_add[i].first,ops_to_add[i].second);
//		ops_to_add[i].first->dump();
//		ops_to_add[i].second->dump();
	}

	set_mapping(map);

	assert (abs_root_var == 0);
	set_abstraction_creation_status(DONE);

}


void HourglassesAbstraction::abstract_action(const vector<int>& abs_vars, DOperator* op, vector<DOperator*>& abs_op){
	//TODO: REWRITE FOR HOURGLASSES!!!!
	// Warning! The current implementation is for non conditional actions only!
	// Decomposition details:
	//  1. If the action has effects on the parents, center, and leafs, split to a sequence of actions,
	//     first having the effects on each of the parents, then on the center, prevailed by the effects of the
	//     previous actions, and the rest having effect on each leaf, prevailed by the effect value of the center (if defined).
	//  2. For each action, all prevails on the leafs are removed.
	//  3. For each parent changing action, all prevails are removed.


	// An action for each variable in the effects is created
/*
	cout << "Abstracting action "<< endl;
	op->dump_no_name();
	cout << "Result (abstract variables [";
	for (size_t v=0; v < abs_vars.size(); ++v) {
		cout << " " << v << ":" << abs_vars[v];
	}
	cout << "])" << endl;
*/
	const vector<Prevail> &prv = op->get_prevail();
//	const vector<PrePost> &pre = op->get_pre_post();


	int root_prev = get_value_for_var(var,prv);
	int root_eff = -1;

	// Creating the first action in the sequence.
	vector<PrePost> center_prepost;

	if (root_prev == -1) {  // No prevail on root - may be an effect
		vector<PrePost> root_effs;
		op->get_var_pre_post(var, root_effs);
		// For non-conditional actions!!!
		if (root_effs.size() > 0) {
			assert(root_effs.size() == 1);
			root_eff = root_effs[0].post;
			// Adding representative for the center
			vector<Prevail> empty;
			center_prepost.push_back(PrePost(abs_root_var,root_effs[0].pre,root_eff,empty));
		}
	}
	vector<Prevail> center_prevail;

	// Going over the parent variables, creating an action for each, and a prevail for the center.
	for (size_t i=0; i < parents.size(); ++i) {
		int v = parents[i];
		if (abs_vars[v] < 0) { // The variable is not in the hourglass (can it even happen?)
			continue;
		}

		vector<PrePost> effs;
		op->get_var_pre_post(v, effs);

		if (effs.size() == 0) //  Is not affected by the action
			continue;

		assert(effs.size() == 1);
		vector<Prevail> empty, parent_prevail;
		vector<PrePost> parent_prepost;
		parent_prepost.push_back(PrePost(abs_vars[v],effs[0].pre,effs[0].post,empty));
		center_prevail.push_back(Prevail(abs_vars[v],effs[0].post));
		string nm;
#ifdef DEBUGMODE
		nm = op->get_name();
		const int max_str_len(42);
		char my_string[max_str_len+1];
		snprintf(my_string, max_str_len, "::0::%d::%#x",abs_vars[v], (unsigned int) this);
		nm += my_string;
#endif

		DOperator* parent_op = new DOperator(op->is_axiom(), parent_prevail, parent_prepost, nm, op->get_double_cost());
//		parent_op->dump_no_name();
		abs_op.push_back(parent_op);
	}

	if (center_prepost.size() > 0) { // Prepost is not empty - adding an action
		// Going over the prevails, adding those that correspond to the parents
		for (size_t i=0; i < prv.size(); ++i) {
			int v = prv[i].var;
			if (abs_vars[v] < 0) { // The variable is not in the hourglass
				continue;
			}
			if (!is_parent[v]) //Not a parent
				continue;

			center_prevail.push_back(Prevail(abs_vars[v],prv[i].prev));
		}

		string nm;
#ifdef DEBUGMODE
		nm = op->get_name();
		const int max_str_len(42);
		char my_string[max_str_len+1];
		snprintf(my_string, max_str_len, "::0::%d::%#x",abs_root_var, (unsigned int) this);
		nm += my_string;
#endif

		DOperator* first_op = new DOperator(op->is_axiom(), center_prevail, center_prepost, nm, op->get_double_cost());
//		first_op->dump_no_name();
		abs_op.push_back(first_op);
	}

	// Going over the leaf variables and creating a representative for each
	for (size_t i=0; i < leafs.size(); ++i) {
		int v = leafs[i];
		assert(abs_vars[v] >= 0 && abs_vars[v] != abs_root_var);

		vector<PrePost> effs;
		op->get_var_pre_post(v, effs);

		if (effs.size() == 0)
			continue;
		assert(effs.size() == 1);

//		cout << "Creating action for variable " << v <<":"<< abs_vars[v] << endl;
		vector<Prevail> new_prv;
		int new_prev_val = root_eff;
		if (new_prev_val == -1)  // If the action is not affecting the center variable
			new_prev_val = root_prev;
		if (new_prev_val != -1)  // If the action is either prevailed by the center or changing it
			new_prv.push_back(Prevail(abs_root_var,new_prev_val));

		assert(get_value_for_var(v,prv) == -1);  // No prevail is possible on v if non-conditional

		string nm;
#ifdef DEBUGMODE
		nm = op->get_name();
		const int max_str_len(42);
		char my_string[max_str_len+1];
		snprintf(my_string, max_str_len, "::0::%d::%#x",abs_vars[v], (unsigned int) this);
		nm += my_string;
#endif

		vector<PrePost> new_pre;
		// Creating effect
		vector<Prevail> empty;
		new_pre.push_back(PrePost(abs_vars[v], effs[0].pre, effs[0].post, empty));

		DOperator* new_op = new DOperator(op->is_axiom(), new_prv, new_pre, nm, op->get_double_cost());
//		new_op->dump_no_name();
		abs_op.push_back(new_op);
	}
//	cout << "========================================================================================" << endl;
}

