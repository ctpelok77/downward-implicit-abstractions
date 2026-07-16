#include "general_abstraction.h"
#include "../problem.h"
#include "../../abstract_task.h"

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
