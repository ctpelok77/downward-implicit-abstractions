#include "op_hash_dom_abs_mapping.h"

using namespace std;

OpHashDomAbsMapping::OpHashDomAbsMapping() {

}

OpHashDomAbsMapping::~OpHashDomAbsMapping() {
}
int OpHashDomAbsMapping::get_abs_operators(DOperator* o, vector<DOperator*>& abs_ops) {
	return OpHashMapping::get_abs_operators(o, abs_ops);
}

DOperator* OpHashDomAbsMapping::get_orig_operator(DOperator* o) const {
	return OpHashMapping::get_orig_operator(o);
}


int OpHashDomAbsMapping::get_abstract_value(int original_variable, int original_value, int abstract_variable) const {
//	cout << "OpHashDomAbsMapping: "<< original_variable << ", " << original_value << ", "<< abstract_variable << endl;
	return DomainAbstractionMapping::get_abstract_value(original_variable, original_value, abstract_variable);
}

bool OpHashDomAbsMapping::has_abs_operators(DOperator* o) const {
	return OpHashMapping::has_abs_operators(o);
}
