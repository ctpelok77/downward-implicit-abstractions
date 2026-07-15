#include "op_hash_mapping.h"
#include "../../utils/system.h"

using namespace std;

OpHashMapping::OpHashMapping(const Problem* orig, Problem* abs) {

	set_original(orig);
	set_abstract(abs);

}

OpHashMapping::OpHashMapping() {

}

OpHashMapping::~OpHashMapping() {
	// TODO Auto-generated destructor stub

/*
	for (auto ii = beta_func.begin(); ii != beta_func.end(); ++ii){
		delete ii->second;
	}
	beta_func.clear();
*/
}

void OpHashMapping::initialize() {
	original_ops.resize(abstract->get_actions_number());

}
int OpHashMapping::get_abs_operators(DOperator* o, vector<DOperator*>& abs_ops) {

	int o_index = original->get_action_index(o);

	if (beta_func.find(o_index) == beta_func.end()) {
		return 0;
	}
	vector<DOperator*> ops = *(beta_func[o_index]);
	for (size_t i=0; i < ops.size(); ++i) {
		abs_ops.push_back(ops[i]);
	}
	return ops.size();
}


DOperator* OpHashMapping::get_orig_operator(DOperator* o) const {

	int orig_index = original_ops[abstract->get_action_index(o)];

	return original->get_action_by_index(orig_index);
}


void OpHashMapping::add_abs_operator(DOperator* o, DOperator* a) {

	int o_index = original->get_action_index(o);
	int a_index = abstract->get_action_index(a);

	if (beta_func.find(o_index) == beta_func.end()) {
		beta_func[o_index] = new vector<DOperator*>;
	}

	beta_func[o_index]->push_back(a);
	original_ops[a_index] = o_index;

}

int OpHashMapping::get_abstract_value(int original_variable, int original_value, int abstract_variable) const {
	assert(abstract_variable < original->get_vars_number());
	assert(original_variable == abstract_variable);
	(void)(original_variable);
	(void)(abstract_variable);
	/*
	if (original_variable != abstract_variable) {
		cout << "OpHashMapping: "<< original_variable << " != " << abstract_variable << endl;
		exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
	*/
	return original_value;
}

bool OpHashMapping::has_abs_operators(DOperator* o) const {
	int o_index = original->get_action_index(o);
	return (beta_func.find(o_index) != beta_func.end());
}
