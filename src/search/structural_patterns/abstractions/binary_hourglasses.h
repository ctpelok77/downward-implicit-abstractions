#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_BINARY_HOURGLASSES_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_BINARY_HOURGLASSES_H

#include "hourglasses_abstraction.h"
#include "domain_abstraction.h"

class BinaryHourglassesAbstraction: public HourglassesAbstraction {
	Domain* dom;
public:
	BinaryHourglassesAbstraction();
	BinaryHourglassesAbstraction(Domain* new_dom);
	BinaryHourglassesAbstraction(HourglassesAbstraction* f, Domain* new_dom);

	virtual ~BinaryHourglassesAbstraction();

	virtual void create(const Problem* p);
	virtual void set_root_var_and_domain(Domain* new_dom);


};

#endif /* BINARY_HOURGLASSES_H */
