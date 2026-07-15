#include "dom_abs_mapping.h"
#include "../../utils/system.h"

using namespace std;

DomainAbstractionMapping::DomainAbstractionMapping() {
}

DomainAbstractionMapping::DomainAbstractionMapping(const Problem* orig, Problem* abs, Domain* d) {
	set_original(orig);
	set_abstract(abs);

	set_domain(d);
}

DomainAbstractionMapping::~DomainAbstractionMapping() {
}

void DomainAbstractionMapping::set_domain(Domain* d) {
	dom = d;
}

int DomainAbstractionMapping::get_abstract_value(int original_variable, int original_value, int abstract_variable) const {
	assert(original_variable == abstract_variable);
	(void)(original_variable);
	/*
	if (original_variable != abstract_variable) {
		cout << "DomainAbstractionMapping: "<< original_variable << " != " << abstract_variable << endl;
		exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
	*/
	if (abstract_variable == dom->get_var()) {
		int abstract_value = dom->get_abs_value(original_value);
//		cout << "Variable " << abstract_variable << " mapping value " << original_value << " to " << abstract_value << endl;
		return abstract_value;
	}
	return original_value;
}

