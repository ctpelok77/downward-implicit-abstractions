#include "../SP_globals.h"
#include "iforks_abstraction.h"
#include "../mappings/op_hash_var_proj_mapping.h"
#include "../mappings/op_hash_mapping.h"
#include "../mappings/var_proj_mapping.h"
#include <string>
// causal_graph: use task_utils

using namespace std;

IforksAbstraction::IforksAbstraction() : GeneralAbstraction() {

	is_singleton = false;
	is_empty = false;
	is_var_singleton = false;
	set_abstraction_type(INVERTED_FORK);

}

IforksAbstraction::IforksAbstraction(int v) : GeneralAbstraction() {
	is_singleton = false;
	is_empty = false;
	is_var_singleton = false;
	var = v;
	set_abstraction_type(INVERTED_FORK);

}


IforksAbstraction::~IforksAbstraction() {
	// TODO Auto-generated destructor stub
}

int IforksAbstraction::get_root() const {
	return var;
}

void IforksAbstraction::get_parents(vector<int>& vars) const {
	vars = parents;
}

bool IforksAbstraction::is_empty_abstraction() const {
	return is_empty;

}

bool IforksAbstraction::is_singleton_abstraction() const {
	return is_singleton;

}


void IforksAbstraction::create(const Problem* p) {
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

	for (size_t i=0; i < predecessors.size(); ++i) {
		parents.push_back(predecessors[i]);  //saving the leafs with goals
		orig_vars.push_back(predecessors[i]);
	}

	if (1==orig_vars.size()) {
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
	for(int i = 0; i < var_count; i++)
		init_state_buffer[i] = p->get_initial_state()[orig_vars[i]];

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

	// Adding axioms as zero-cost operators
	vector<DOperator*> ops, axioms;
	vector<pair<DOperator*, DOperator*> > ops_to_add;

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

	for (size_t i=0; i < ops_to_add.size(); ++i)
		map->add_abs_operator(ops_to_add[i].first,ops_to_add[i].second);

	set_mapping(map);

	abs_root_var = abs_vars[var];
	assert (abs_root_var == 0);
	set_abstraction_creation_status(DONE);
}


void IforksAbstraction::abstract_action(const vector<int>& abs_vars, DOperator* op, vector<DOperator*>& abs_op){
	// An action for each variable in the effects is created

	const vector<Prevail> &prv = op->get_prevail();

	vector<PrePost> root_effs;
	op->get_var_pre_post(var, root_effs);
	vector<Prevail> child_prv;

	// Going over the parent variables and creating a representative for each
	// On a way prevail is created for the child variable
	for (size_t v=0; v < abs_vars.size(); ++v) {
		if (abs_vars[v] < 0) { // The variable is not in fork
			continue;
		}
		int prv_val = get_value_for_var(v, prv);
		if (-1 != prv_val) { // prevail on the variable defined
			child_prv.push_back(Prevail(abs_vars[v], prv_val));
		}

		if ((int)v == var)
			continue;

		vector<PrePost> effs;
		op->get_var_pre_post(v, effs);

		if (effs.size() == 0)
			continue;

//		cout << "Creating action for variable " << v <<":"<< abs_vars[v] << endl;
		vector<Prevail> new_prv;
		if (-1 != prv_val) { // prevail on the variable defined
			new_prv.push_back(Prevail(abs_vars[v], prv_val));
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
		int cond_val_child = -1;
		for (size_t p=0; p < effs.size(); ++p) {
			vector<Prevail> new_cond;
			int eff_val_v = get_value_for_var(v,effs[p].cond);
			if (-1 != eff_val_v) { // condition on the variable defined
				new_cond.push_back(Prevail(abs_vars[v], eff_val_v));
			}

			cond_val_child = get_value_for_var(var,effs[p].cond);
			// Creating effect
			new_pre.push_back(PrePost(abs_vars[v], effs[p].pre, effs[p].post, new_cond));
		}
		if (cond_val_child == -1 && new_pre.size() == 1 && new_pre[0].cond.size() == 0) {
			// Adding an effect as a prevail to the child changing action
			child_prv.push_back(Prevail(abs_vars[v], new_pre[0].post));
		}

		DOperator* new_op = new DOperator(op->is_axiom(), new_prv, new_pre, nm, op->get_double_cost());
		abs_op.push_back(new_op);
//		new_op->dump();
	}

	if (root_effs.size() == 0)
		return;

//	cout << "Creating action for variable " << var <<":"<< abs_vars[var] << endl;
	string nm;
#ifdef DEBUGMODE
	nm = op->get_name();
	const int max_str_len(42);
	char my_string[max_str_len+1];
	snprintf(my_string, max_str_len, "::0::%d::%#x",abs_vars[var], (unsigned int) this);
	nm += my_string;
#endif

	vector<PrePost> new_pre;
	for (size_t p=0; p < root_effs.size(); ++p) {
		vector<Prevail> new_cond;
		// Going over the parents, taking the condition value
		for (size_t v=0; v < abs_vars.size(); ++v) {
			if (abs_vars[v] < 0) { // The variable is not in fork
				continue;
			}

			int pre_val_v = get_value_for_var(abs_vars[v], child_prv);
			if (-1 != pre_val_v) { // prevail on the variable is defined
				continue;
			}

			int eff_val_v = get_value_for_var(v, root_effs[p].cond);
			if (-1 != eff_val_v) { // condition on the variable defined
				new_cond.push_back(Prevail(abs_vars[v], eff_val_v));
			}
		}

		// Creating effect
		new_pre.push_back(PrePost(abs_vars[var], root_effs[p].pre, root_effs[p].post, new_cond));
	}

	DOperator* new_op = new DOperator(op->is_axiom(), child_prv, new_pre, nm, op->get_double_cost());
	abs_op.push_back(new_op);
//	new_op->dump();
}

int IforksAbstraction::root_unconditional_prepost_index(DOperator* op) const{

	const vector<PrePost> &pre = op->get_pre_post();
	for (size_t i=0; i < pre.size(); ++i)
		if ((pre[i].var == var) && (pre[i].cond.size() == 0))
			return i;
	return -1;
}


int IforksAbstraction::root_prepost_index(DOperator* op) const {

	const vector<PrePost> &pre = op->get_pre_post();
	for (size_t i=0; i < pre.size(); ++i)
		if (pre[i].var == var)
			return i;
	return -1;
}
/*
int IforksAbstraction::get_value_for_var(int v, vector<Prevail>& prv) const {

	for (size_t ind=0; ind < prv.size(); ++ind) {
		if (prv[ind].var == v) {
			return prv[ind].prev;
		}
	}
	return -1;
}
*/
