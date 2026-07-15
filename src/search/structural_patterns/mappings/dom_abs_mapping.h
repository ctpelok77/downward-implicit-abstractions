#ifndef STRUCTURAL_PATTERNS_MAPPINGS_DOM_ABS_MAPPING_H
#define STRUCTURAL_PATTERNS_MAPPINGS_DOM_ABS_MAPPING_H

#include "mapping.h"
#include "../abstractions/domain.h"

class DomainAbstractionMapping: public virtual Mapping {
protected:
	Domain* dom;

public:
	DomainAbstractionMapping();
	DomainAbstractionMapping(const Problem* orig, Problem* abs, Domain* d);
	virtual ~DomainAbstractionMapping();

	void set_domain(Domain* d);
	virtual int get_abstract_value(int original_variable, int original_value, int abstract_variable) const;

	virtual int get_abs_operators(DOperator* o, std::vector<DOperator*>& abs_ops) = 0;
	virtual DOperator* get_orig_operator(DOperator* o) const = 0;

	virtual int get_original_var(int abs) const {return abs;}
	virtual int get_abstract_var(int orig) const {return orig;}

	virtual bool has_abs_operators(DOperator* o) const = 0;

};

#endif /* DOM_ABS_MAPPING_H */
