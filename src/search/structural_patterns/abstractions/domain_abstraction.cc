#include "domain_abstraction.h"
#include "../mappings/op_hash_dom_abs_mapping.h"
#include "../mappings/op_hash_mapping.h"
#include "../mappings/var_proj_mapping.h"
#include <string>
#include "../d_operator.h"

using namespace std;

DomainAbstraction::DomainAbstraction() : GeneralAbstraction() {

}

DomainAbstraction::DomainAbstraction(Domain* new_dom) : GeneralAbstraction() {
//	set_domain(new_dom);
	set_root_var_and_domain(new_dom);
}


DomainAbstraction::~DomainAbstraction() {
}

//void DomainAbstraction::set_domain(Domain* new_dom){
void DomainAbstraction::set_root_var_and_domain(Domain* new_dom){
	dom = new_dom;
}

void DomainAbstraction::get_orig_values(int abs_val, vector<int>& ret) const {

	dom->get_orig_values(abs_val,ret);
//	return dom->get_orig_values(abs_val);
}


int DomainAbstraction::get_abs_value(int orig_val) const {
	if (orig_val == -1)
		return -1;
	return dom->get_abs_value(orig_val);
}

void DomainAbstraction::abstract_prevail(const vector<Prevail>& prevail, vector<Prevail>& prv) {
//	vector<Prevail> prv;
	for (size_t i=0; i < prevail.size(); ++i) {
		if (prevail[i].var == dom->get_var())
			prv.push_back(Prevail(prevail[i].var,get_abs_value(prevail[i].prev)));
		else
			prv.push_back(Prevail(prevail[i].var,prevail[i].prev));
	}
//	return prv;
}


void DomainAbstraction::abstract_prepost(const vector<PrePost>& prepost, vector<PrePost>& pre, vector<Prevail>& prv) {
	for (size_t it=0; it < prepost.size(); ++it) {
		// Get abstracted condition
		vector<Prevail> new_cond;
		abstract_prevail(prepost[it].cond, new_cond);
		int new_pre = prepost[it].pre;
		int new_post = prepost[it].post;

		if (prepost[it].var == dom->get_var()){
			new_pre = get_abs_value(prepost[it].pre);
			new_post = get_abs_value(prepost[it].post);
			int new_c = get_value_for_var(dom->get_var(), new_cond);  // Already an abstracted value
			assert(-1 == new_c || -1 == new_pre || new_c == new_pre);

			assert(new_post >= 0);
//			if (new_pre == new_post || new_c == new_post)
//				continue;
			// Changed to support structures beyond forks and iforks
			if (new_c == new_post)
				continue;
			if (new_pre == new_post) {

				// If there is no condition, adding a prevail
				if (new_cond.size() == 0)
					prv.push_back(Prevail(prepost[it].var, new_pre));

				continue;
			}
		}
		pre.push_back(PrePost(prepost[it].var,new_pre,new_post,new_cond));
	}
}





void DomainAbstraction::create(const Problem* p) {
	if (get_abstraction_creation_status() == DONE)
		return;

	//Creating the empty mapping
	OpHashDomAbsMapping* map = new OpHashDomAbsMapping();
	map->set_original(p);
	map->set_domain(dom);

	// Creating abstract variables, initial state.
	const vector<string> &var_name = p->get_variable_names();
	const vector<int> &var_domain = p->get_variable_domains();

	int var_count = var_name.size();
	const SPState &state = p->get_initial_state();

	vector<int> new_var_domain;
	new_var_domain.resize(var_count);
	vector<int> init_state_buffer(var_count);
	for(int i = 0; i < var_count; i++){
		if (i == dom->get_var()){
			new_var_domain[i] = dom->get_new_size();
			init_state_buffer[i] = dom->get_abs_value(state[i]);
		} else {
			new_var_domain[i] = var_domain[i];
			init_state_buffer[i] = state[i];
		}
	}

	// Creating abstract goal
	vector<pair<int, int> > new_goal;
	p->get_goal(new_goal);
	for (size_t i=0; i < new_goal.size(); ++i){
		if (dom->get_var() == new_goal[i].first)
			new_goal[i].second = get_abs_value(new_goal[i].second);
	}

	// Creating abstract action
	const vector<DOperator*> &orig_ops = p->get_operators();

	vector<DOperator*> ops, axioms;
//	int num_orig_ops = orig_ops.size();

	vector<pair<DOperator*, DOperator*> > ops_to_add;
	vector<DOperator> axi = p->get_axioms();
	for (size_t i=0; i < axi.size(); ++i){
		axioms.push_back(&axi[i]);
	}
	const vector<int> abs_vars;

	abstract_actions(abs_vars,orig_ops,ops,ops_to_add);
	abstract_actions(abs_vars,axioms,ops,ops_to_add);

	vector<DOperator> no_axioms;
	Problem* abs_prob = new Problem(var_name, new_var_domain, init_state_buffer, new_goal, ops, no_axioms);

	map->set_abstract(abs_prob);
	map->initialize();
//	abs_prob->dump();

//	cout << "Actions added to the problem - domain abstraction" << endl;
	for (size_t i=0; i < ops_to_add.size(); ++i) {
		map->add_abs_operator(ops_to_add[i].first,ops_to_add[i].second);
//		ops_to_add[i].first->dump();
//		ops_to_add[i].second->dump();
	}
	set_mapping(map);

	set_abstraction_creation_status(DONE);
}

// Creates abstract actions vector (freeing this vector is up to user)
void DomainAbstraction::abstract_action(const vector<int>& , DOperator* op, vector<DOperator*>& abs_op){

	vector<Prevail> prv;
	abstract_prevail(op->get_prevail(),prv);
	vector<PrePost> pre;
	abstract_prepost(op->get_pre_post(),pre, prv);
	if (0 == pre.size()) {
		return;
	}
	string nm;
#ifdef DEBUGMODE
	nm = op->get_name()+string("::");
	const int max_str_len(42);
	char my_string[max_str_len+1];
	snprintf(my_string, max_str_len, "%#x", (unsigned int) this);
	nm += my_string;
#endif

	DOperator* new_op = new DOperator(op->is_axiom(), prv, pre, nm, op->get_double_cost());
	abs_op.push_back(new_op);
}
