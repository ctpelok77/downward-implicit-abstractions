#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_BOUNDED_IFORKS_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_BOUNDED_IFORKS_H

#include "iforks_abstraction.h"
#include "domain_abstraction.h"


class BoundedIforksAbstraction: public IforksAbstraction {
	Domain* dom;

public:
	BoundedIforksAbstraction();
	BoundedIforksAbstraction(Domain* new_dom);
	BoundedIforksAbstraction(IforksAbstraction* ifork, Domain* abs_domain);

	virtual ~BoundedIforksAbstraction();

	virtual void create(const Problem* p);
//	void set_variable_and_domain(Domain* new_dom);
	virtual void set_root_var_and_domain(Domain* new_dom);

	Domain* get_domain();
};

#endif /* BOUNDED_IFORKS_H */
