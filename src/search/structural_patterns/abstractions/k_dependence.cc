#include "k_dependence.h"
#include "../mappings/op_hash_dom_abs_mapping.h"
#include "../mappings/op_hash_mapping.h"
#include "../mappings/var_proj_mapping.h"
#include <string>
#include "../d_operator.h"

using namespace std;

KDependence::KDependence() : GeneralAbstraction() {

}

KDependence::KDependence(size_t k) : GeneralAbstraction() {
	set_constant(k);
}


KDependence::~KDependence() {
}


void KDependence::abstract_prevail(const vector<Prevail>& prevail, vector<Prevail>& prv) {
	if (prevail.size() <= bound) {
		// Copying everything
		for (size_t i=0; i < prevail.size(); ++i) {
			prv.push_back(Prevail(prevail[i].var,prevail[i].prev));
		}
		return;
	}
	// The case when we need to decide which prevails to keep
	// Calculating the vector of ranks, go over it k times, getting the maximal element.
	vector<int> rank_prevail;
	for (size_t i=0; i < prevail.size(); ++i) {
		int dist = get_initial_distances(prevail[i].var,prevail[i].prev);
		assert(dist != -1);
		rank_prevail.push_back(dist);
	}

	for(size_t i=0; i < bound; ++i) {
		// Find the maximal element, set its rank to -1
		int max_elem = -1;
		int max_elem_index = -1;
		for (size_t j=0; j < rank_prevail.size(); ++j) {
			if (max_elem < rank_prevail[j]) {
				// Update the max element
				max_elem = rank_prevail[j];
				max_elem_index = j;
			}
		}
		assert(max_elem_index > -1);
		rank_prevail[max_elem_index] = -1;
		prv.push_back(Prevail(prevail[max_elem_index].var,prevail[max_elem_index].prev));
	}
}

int KDependence::get_initial_distances(int v, int val) {
	if (initial_distances[v].size() > 0)
		return initial_distances[v][val];
	return -1;
}

void KDependence::create(const Problem* p) {
	if (get_abstraction_creation_status() == DONE)
		return;

	cout << "K bound is " << bound << endl;
	// Creating abstract variables, initial state.
	const vector<string> &var_name = p->get_variable_names();
	const vector<int> &var_domain = p->get_variable_domains();

	int var_count = var_name.size();
	const SPState &state = p->get_initial_state();
	vector<int> init_state_buffer(var_count);

	initial_distances.clear();
	for(int i = 0; i < var_count; i++){
		init_state_buffer[i] = state[i];
		vector<int> distances;
		initial_distances.push_back(distances);
	}

	// Basically, there is no need to calculate the distances for the variables
	//  that do not participate in prevails of size > k.
	const vector<DOperator*> &orig_ops = p->get_operators();
	set<int> vars_for_distances;
	for (size_t i=0; i < orig_ops.size(); ++i){
	    const vector<Prevail> &prv = orig_ops[i]->get_prevail();
	    if (prv.size() <= bound)
	    	continue;
		for (size_t p=0; p < prv.size(); ++p){
			vars_for_distances.insert(prv[p].var);
		}
	}

	for(set<int>::iterator it = vars_for_distances.begin(); it != vars_for_distances.end(); ++it){
		// Calculating initial distances for all variables in advance
		vector<vector<int> > domain_vals;
		cout << "Calculating distances from value " << init_state_buffer[(*it)] << " of " << (*it) << endl;
		p->get_domain_decomposition_by_distance_from_value((*it), init_state_buffer[(*it)], domain_vals, initial_distances[(*it)]);
	}
	cout << "Distances calculated" << endl;

	// Creating abstract goal
	vector<pair<int, int> > new_goal;
	p->get_goal(new_goal);
//	cout << "==================================================================" << endl;
//	p->dump();
	// Creating abstract action
	vector<DOperator> axi = p->get_axioms();

	vector<DOperator*> ops, axioms;

	vector<pair<DOperator*, DOperator*> > ops_to_add;
	for (size_t i=0; i < axi.size(); ++i) {
		axioms.push_back(&axi[i]);
	}
	const vector<int> abs_vars;

//	cout << "==================================================================" << endl;
	KDependence::abstract_actions(abs_vars,orig_ops,ops,ops_to_add);
//	cout << "==================================================================" << endl;
	KDependence::abstract_actions(abs_vars,axioms,ops,ops_to_add);
//	cout << "==================================================================" << endl;

	vector<DOperator> no_axioms;
	Problem* abs_prob = new Problem(var_name, var_domain, init_state_buffer, new_goal, ops, no_axioms);

	//Creating the empty mapping
	OpHashMapping* map = new OpHashMapping();
	map->set_original(p);

	map->set_abstract(abs_prob);
	map->initialize();
//	cout << "==================================================================" << endl;
//	abs_prob->dump();

//	cout << "Actions added to the problem - domain abstraction" << endl;
	for (size_t i=0; i < ops_to_add.size(); ++i) {
		map->add_abs_operator(ops_to_add[i].first,ops_to_add[i].second);
	}
	set_mapping(map);
	set_abstraction_creation_status(DONE);
}

// Creates abstract actions vector (freeing this vector is up to user)
void KDependence::abstract_action(const vector<int>& , DOperator* op, vector<DOperator*>& abs_op){
//	cout << "Entering here!!!!!!!!!!" << endl;
	vector<Prevail> prv;
	abstract_prevail(op->get_prevail(),prv);
	vector<PrePost> pre = op->get_pre_post();
//	cout << " Pre size: " << pre.size() << endl;
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
//	new_op->dump_no_name();
	abs_op.push_back(new_op);
}


void KDependence::abstract_actions(const vector<int>& abs_vars, const vector<DOperator*>& orig_ops,
		vector<DOperator*>& ops, vector<pair<DOperator*, DOperator*> >& ops_to_add){
	for (size_t it=0; it < orig_ops.size(); ++it){
		vector<DOperator*> abs_ops;
		KDependence::abstract_action(abs_vars,orig_ops[it],abs_ops);
		for (size_t abs_it=0; abs_it < abs_ops.size(); ++abs_it){
			ops.push_back(abs_ops[abs_it]);
			pair<DOperator*, DOperator*> to_add;
			to_add.first = orig_ops[it];
			to_add.second = abs_ops[abs_it];
			ops_to_add.push_back(to_add);
		}
	}
}
