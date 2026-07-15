#include "composition_mapping.h"
#include <vector>

using namespace std;

CompositionMapping::CompositionMapping(Mapping* a, Mapping* b):from_map(a), to_map(b) {
	set_original(from_map->get_original());
	set_abstract(to_map->get_abstract());
}

CompositionMapping::~CompositionMapping() {
}


int CompositionMapping::get_abs_operators(DOperator* o, vector<DOperator*>& abs_ops){

	vector<DOperator*> ops;
	from_map->get_abs_operators(o,ops);

	int tot = 0;
	for (size_t i=0; i < ops.size(); ++i){
		tot += to_map->get_abs_operators(ops[i],abs_ops);
	}
	return tot;
}


DOperator* CompositionMapping::get_orig_operator(DOperator* o) const {

	return from_map->get_orig_operator(to_map->get_orig_operator(o));
}


int CompositionMapping::get_original_var(int abs) const {
	return from_map->get_original_var(to_map->get_original_var(abs));
}

int CompositionMapping::get_abstract_var(int orig) const {
	return to_map->get_abstract_var(from_map->get_abstract_var(orig));
}


int CompositionMapping::get_abstract_value(int original_variable, int original_value, int abstract_variable) const {

	assert(original_variable < from_map->get_original()->get_vars_number());
	assert(abstract_variable < to_map->get_abstract()->get_vars_number());

	int middle_variable = from_map->get_abstract_var(original_variable);
	assert(middle_variable == to_map->get_original_var(abstract_variable));

//	cout << "CompositionMappingFROM: "<< original_variable << ", " << original_value << ", "<< middle_variable << endl;

	int middle_value = from_map->get_abstract_value(original_variable, original_value, middle_variable);

//	cout << "CompositionMappingTO: "<< middle_variable << ", " << middle_value << ", "<< abstract_variable << endl;

	return to_map->get_abstract_value(middle_variable, middle_value, abstract_variable);
}

bool CompositionMapping::has_abs_operators(DOperator* o) const {

	if (!from_map->has_abs_operators(o))
		return false;

	vector<DOperator*> ops;
	from_map->get_abs_operators(o,ops);

	for (size_t i=0; i < ops.size(); ++i){
		if (to_map->has_abs_operators(ops[i]))
			return true;
	}
	return false;
}
