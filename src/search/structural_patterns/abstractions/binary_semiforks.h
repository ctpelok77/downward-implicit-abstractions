#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_BINARY_SEMIFORKS_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_BINARY_SEMIFORKS_H

#include "semiforks_abstraction.h"
#include "domain_abstraction.h"

class BinarySemiForksAbstraction: public SemiForksAbstraction {
	Domain* dom;
public:
	BinarySemiForksAbstraction();
//	BinarySemiForksAbstraction(Domain* new_dom, vector<int>& hat_vars);
//	BinarySemiForksAbstraction(SemiForksAbstraction* f, Domain* new_dom, vector<int>& hat_vars);
	BinarySemiForksAbstraction(Domain* new_dom);
	BinarySemiForksAbstraction(SemiForksAbstraction* f, Domain* new_dom);

	virtual ~BinarySemiForksAbstraction();

	virtual void create(const Problem* p);
	virtual void set_root_var_and_domain(Domain* new_dom);


};

#endif /* BINARY_SEMIFORKS_H */
