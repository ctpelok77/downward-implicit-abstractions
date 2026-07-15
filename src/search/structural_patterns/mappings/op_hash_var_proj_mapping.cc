#include "op_hash_var_proj_mapping.h"

using namespace std;

OpHashVarProjMapping::OpHashVarProjMapping() {

}

OpHashVarProjMapping::~OpHashVarProjMapping() {
}
int OpHashVarProjMapping::get_abs_operators(DOperator* o, vector<DOperator*>& abs_ops) {
	return OpHashMapping::get_abs_operators(o,abs_ops);
}

DOperator* OpHashVarProjMapping::get_orig_operator(DOperator* o) const {
	return OpHashMapping::get_orig_operator(o);
}


int OpHashVarProjMapping::get_abstract_value(int original_variable, int original_value, int abstract_variable) const {
//	cout << "OpHashVarProjMapping: "<< original_variable << ", " << original_value << ", "<< abstract_variable << endl;
	return VarProjMapping::get_abstract_value(original_variable, original_value, abstract_variable);
}

bool OpHashVarProjMapping::has_abs_operators(DOperator* o) const {
	return OpHashMapping::has_abs_operators(o);
}
