#include "general_abstraction.h"

using namespace std;

GeneralAbstraction::GeneralAbstraction() : creation_status(NONE) {
}

GeneralAbstraction::~GeneralAbstraction() {
//	if (m)
//		delete m;
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
