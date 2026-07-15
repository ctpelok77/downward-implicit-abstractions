#include "../SP_globals.h"
#include "forks_abstraction.h"
#include "../mappings/op_hash_var_proj_mapping.h"
#include "../mappings/op_hash_mapping.h"
#include "../mappings/var_proj_mapping.h"
#include <string>
// causal_graph: use task_utils

using namespace std;


ForksAbstraction::ForksAbstraction() : GeneralAbstraction() {
	is_singleton = false;
	is_empty = false;
	is_var_singleton = false;
	set_abstraction_type(FORK);
}

ForksAbstraction::ForksAbstraction(int v) : GeneralAbstraction() {
	is_singleton = false;
	is_empty = false;
	is_var_singleton = false;
	set_variable(v);
	set_abstraction_type(FORK);
}


ForksAbstraction::~ForksAbstraction() {
}


void ForksAbstraction::set_variable(int v) {
	var = v;
}


int ForksAbstraction::get_root() const {
	return var;
}
/*
void ForksAbstraction::get_leafs(vector<int>& vars) const {
	vars = leafs;
}
*/
bool ForksAbstraction::is_empty_abstraction() const {
	return is_empty;

}

bool ForksAbstraction::is_singleton_abstraction() const {
	return is_singleton;
}


void ForksAbstraction::create(const Problem* p) {
	if (get_abstraction_creation_status() == DONE)
		return;

	// Creating abstract variables, initial state.
	const vector<string> &var_name = p->get_variable_names();
	const vector<int> &var_domain = p->get_variable_domains();

	vector<int> successors = p->get_causal_graph()->get_successors(var);
	vector<int> predecessors = p->get_causal_graph()->get_predecessors(var);

	if (0 == successors.size() && 0 == predecessors.size()) {
		is_var_singleton = true;
	}

	vector<int> abs_vars;
	vector<int> orig_vars;
	orig_vars.push_back(var);
	for (size_t i=0; i < successors.size(); ++i) {
		if (p->is_goal_var(successors[i])) {
//			leafs.push_back(successors[i]);  //saving the leafs with goals
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

	abs_vars.assign(var_name.size(),-1);

	int var_count = orig_vars.size();

	vector<string> new_var_name;
	vector<int> new_var_domain;

	new_var_name.resize(var_count);
	new_var_domain.resize(var_count);

	for(int i = 0; i < var_count; i++) {
		abs_vars[orig_vars[i]] = i;
		new_var_name[i] = var_name[orig_vars[i]];
		new_var_domain[i] = var_domain[orig_vars[i]];
	}

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

//	int num_orig_ops = orig_ops.size();

	vector<DOperator*> ops, axioms;
	vector<pair<DOperator*, DOperator*> > ops_to_add;

	// Adding axioms as zero-cost operators
	vector<DOperator> axi = p->get_axioms();
	for (size_t i=0; i < axi.size(); ++i){
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

	abs_root_var = abs_vars[var];
	assert (abs_root_var == 0);
	set_abstraction_creation_status(DONE);
}


void ForksAbstraction::abstract_action(const vector<int>& abs_vars, DOperator* op, vector<DOperator*>& abs_op){
	// An action for each variable in the effects is created
//	cout << "Abstracting action "<< endl;
//	op->dump();
//	cout << "Result (abstract variables [";
//	for (size_t v=0; v < abs_vars.size(); ++v) {
//		cout << " " << v << ":" << abs_vars[v];
//	}
//	cout << "])" << endl;

	const vector<Prevail> &prv = op->get_prevail();
	int root_val = get_value_for_var(var,prv);

	vector<PrePost> root_effs;
	op->get_var_pre_post(var, root_effs);

	// Going over the variables and creating a representative for each
	for (int v=0; v < (int)abs_vars.size(); ++v) {
		if (abs_vars[v] < 0) { // The variable is not in fork
			continue;
		}

		vector<PrePost> effs;
		op->get_var_pre_post(v, effs);

		if (effs.size() == 0)
			continue;

//		cout << "Creating action for variable " << v <<":"<< abs_vars[v] << endl;
		vector<Prevail> new_prv;
		if ((int)v == var && root_effs.size() == 0) // Not changing root variable
				continue;

		if ((int)v != var) {
			// Creating root prevail for child changing action
			if (root_effs.size() == 1 && root_effs[0].cond.size() == 0) {
				// Depending on the effect of root changing action
				new_prv.push_back(Prevail(abs_vars[var], root_effs[0].post));
			} else {
				// Depending on the prevail, if defined
				if (root_val != -1) {
					// Creating prevail
					new_prv.push_back(Prevail(abs_vars[var], root_val));
				}
			}
		}
		int val_v = get_value_for_var(v,prv);
		if (val_v != -1) {
			// Creating prevail
			new_prv.push_back(Prevail(abs_vars[v], val_v));
		}
		string nm;
#ifdef DEBUGMODE
		nm = op->get_name();
		const int max_str_len(42);
		char my_string[max_str_len+1];
		snprintf(my_string, max_str_len, "::0::%d::%#x",abs_vars[v], (unsigned int) this);
		nm += my_string;
#endif

		vector<PrePost> new_pre;
		for (size_t p=0; p < effs.size(); ++p) {
			vector<Prevail> new_cond;
			int eff_val_v = get_value_for_var(v,effs[p].cond);
			if (-1 != eff_val_v) { // condition on the variable defined
				new_cond.push_back(Prevail(abs_vars[v], eff_val_v));
			}
			if (v != var) {
				int eff_val_root = get_value_for_var(var,effs[p].cond);
				if (root_val != -1) { // precondition on root defined
					assert(eff_val_root == -1 || root_val == eff_val_root);
				} else {
					if (-1 != eff_val_root) { // condition on root defined
						new_cond.push_back(Prevail(abs_vars[var], eff_val_root));
					}
				}
			}
			// Creating effect
			new_pre.push_back(PrePost(abs_vars[v], effs[p].pre, effs[p].post, new_cond));
		}

		DOperator* new_op = new DOperator(op->is_axiom(), new_prv, new_pre, nm, op->get_double_cost());
		abs_op.push_back(new_op);
	}
}

/*
int ForksAbstraction::root_prevail_index(DOperator* op) const {

	const vector<Prevail> &prv = op->get_prevail();
	for (size_t i=0; i < prv.size(); ++i)
		if (prv[i].var == var)
			return i;
	return -1;
}

int ForksAbstraction::root_prepost_index(DOperator* op) const {

	const vector<PrePost> &pre = op->get_pre_post();
	for (size_t i=0; i < pre.size(); ++i)
		if (pre[i].var == var)
			return i;
	return -1;
}

int ForksAbstraction::root_unconditional_prepost_index(DOperator* op) const {

	const vector<PrePost> &pre = op->get_pre_post();
	for (size_t i=0; i < pre.size(); ++i)
		if ((pre[i].var == var) && (pre[i].cond.size() == 0))
			return i;
	return -1;
}
*/
