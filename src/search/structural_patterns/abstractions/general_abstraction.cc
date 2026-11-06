#include "general_abstraction.h"
#include "../problem.h"
#include "../sp_abstract_task.h"
#include "../../abstract_task.h"

#include <cassert>

using namespace std;

GeneralAbstraction::GeneralAbstraction() : creation_status(NONE) {
}

GeneralAbstraction::~GeneralAbstraction() {
}

void GeneralAbstraction::create(const std::shared_ptr<AbstractTask> &task) {
	// Build a Problem from the AbstractTask and delegate to the virtual overload.
	// The Problem is owned by the Mapping (stored as get_mapping()->get_original()),
	// so we must NOT delete it after create() sets it there.
	// In branches where create() returns early without building a mapping (e.g.
	// is_empty), the Problem was never stored and we free it here.
	Problem *prob = new Problem(task);
	create(prob);
	if (get_mapping() == nullptr || get_mapping()->get_original() != prob)
		delete prob;
}


Mapping* GeneralAbstraction::get_mapping() const {
	return m;
}

void GeneralAbstraction::set_mapping(Mapping* map) {
	m = map;
}

AbstractionType GeneralAbstraction::get_abstraction_type() const {
	return abstraction_type;
}
void GeneralAbstraction::set_abstraction_type(AbstractionType type) {
	abstraction_type = type;
}

void GeneralAbstraction::abstract_actions(const vector<int>& abs_vars, const vector<DOperator*>& orig_ops,
		vector<DOperator*>& ops, vector<pair<DOperator*, DOperator*> >& ops_to_add){
	for (size_t it=0; it < orig_ops.size(); ++it){
		vector<DOperator*> abs_ops;
		abstract_action(abs_vars,orig_ops[it],abs_ops);
		for (size_t abs_it=0; abs_it < abs_ops.size(); ++abs_it){
			ops.push_back(abs_ops[abs_it]);
			pair<DOperator*, DOperator*> to_add;
			to_add.first = orig_ops[it];
			to_add.second = abs_ops[abs_it];
			ops_to_add.push_back(to_add);
		}
	}
}

Problem* GeneralAbstraction::build_abstract_problem(const Problem* p,
		const vector<string>& var_name,
		const vector<int>& var_domain,
		const vector<int>& init_state,
		const vector<pair<int, int> >& goal,
		const vector<DOperator*>& ops,
		const vector<pair<DOperator*, DOperator*> >& ops_to_add,
		vector<int> parent_vars,
		int abstracted_var,
		vector<int> value_map) {
	vector<DOperator> no_axioms;
	Problem* abs_prob = new Problem(var_name, var_domain, init_state, goal, ops, no_axioms);

	// abstract_actions pushes to ops and ops_to_add in lockstep.
	assert(ops.size() == ops_to_add.size());
	vector<int> parent_ops;
	parent_ops.reserve(ops_to_add.size());
	for (size_t i=0; i < ops_to_add.size(); ++i) {
		assert(ops_to_add[i].second == ops[i]);
		DOperator* orig = ops_to_add[i].first;
		int index = -1;
		for (int j = 0; j < p->get_actions_number(); ++j) {
			if (p->get_action_by_index(j) == orig) {
				index = j;
				break;
			}
		}
		parent_ops.push_back(index);  // -1: originates from a parent axiom
	}

	abs_prob->set_task(make_shared<SPAbstractTask>(
		p->get_task(), vector<string>(var_name), vector<int>(var_domain),
		vector<int>(init_state), goal, ops, std::move(parent_ops),
		std::move(parent_vars), abstracted_var, std::move(value_map)));

#ifndef NDEBUG
	// The task must describe exactly the same abstract problem.
	Problem check(abs_prob->get_task());
	assert(check.get_variable_domains() == abs_prob->get_variable_domains());
	assert(check.get_initial_state() == abs_prob->get_initial_state());
	vector<int> g1, g2;
	check.get_goal_vals(g1);
	abs_prob->get_goal_vals(g2);
	assert(g1 == g2);
	assert(check.get_actions_number() == abs_prob->get_actions_number());
	for (int i = 0; i < abs_prob->get_actions_number(); ++i) {
		const DOperator* a = abs_prob->get_action_by_index(i);
		const DOperator* b = check.get_action_by_index(i);
		assert(a->get_prevail() == b->get_prevail());
		assert(a->get_pre_post().size() == b->get_pre_post().size());
		for (size_t k = 0; k < a->get_pre_post().size(); ++k)
			assert(a->get_pre_post()[k] == b->get_pre_post()[k]);
		assert(a->get_double_cost() == b->get_double_cost());
	}
#endif
	return abs_prob;
}
