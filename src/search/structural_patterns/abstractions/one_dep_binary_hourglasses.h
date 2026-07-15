#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_ONE_DEP_BINARY_HOURGLASSES_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_ONE_DEP_BINARY_HOURGLASSES_H

#include "one_dep_hourglasses.h"
#include "domain_abstraction.h"

class OneDependentBinaryHourglassesAbstraction: public OneDependentHourglassesAbstraction {
	Domain* dom;

public:
	OneDependentBinaryHourglassesAbstraction();
	OneDependentBinaryHourglassesAbstraction(Domain* new_dom);
	OneDependentBinaryHourglassesAbstraction(OneDependentHourglassesAbstraction* f, Domain* new_dom);

	virtual ~OneDependentBinaryHourglassesAbstraction();

	virtual void create(const Problem* p);
	virtual void set_root_var_and_domain(Domain* new_dom);


};

#endif /* ONE_DEP_BINARY_HOURGLASSES_H */
