#include "mapping.h"

#include "../problem.h"
#include "../d_operator.h"
#include <vector>

using namespace std;

Mapping::Mapping(const Problem* orig, Problem* abs) {
	abstract = NULL;

	set_original(orig);
	set_abstract(abs);
}

Mapping::Mapping() {
	abstract = NULL;
}

Mapping::~Mapping() {
	if (abstract)
		delete abstract;
}

void Mapping::set_original(const Problem* p) {original = p;}

void Mapping::set_abstract(Problem* p) {
	if (abstract)
		delete abstract;

	abstract = p;
}

int Mapping::get_abstract_state_value(const SPState &state, int abstract_variable) const {
	int original_variable = get_original_var(abstract_variable);
	int original_value = state[original_variable];
	return get_abstract_value(original_variable, original_value, abstract_variable);
}

int Mapping::get_abs_operators_by_index(int op_index,
                                        std::vector<DOperator*> &abs_ops) {
	return get_abs_operators(original->get_action_by_index(op_index), abs_ops);
}

bool Mapping::has_abs_operators_by_index(int op_index) const {
	return has_abs_operators(original->get_action_by_index(op_index));
}
