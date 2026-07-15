#include "mapping.h"

#include "../../task_proxy.h"
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
