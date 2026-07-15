#ifndef STRUCTURAL_PATTERNS_MAPPINGS_COMPOSITION_MAPPING_H
#define STRUCTURAL_PATTERNS_MAPPINGS_COMPOSITION_MAPPING_H

#include "mapping.h"
#include <set>

class CompositionMapping: public Mapping {
	Mapping* from_map;
	Mapping* to_map;

public:
	CompositionMapping(Mapping* a, Mapping* b);
	virtual ~CompositionMapping();

	virtual int get_abstract_value(int original_variable, int original_value, int abstract_variable) const;

	virtual int get_abs_operators(DOperator* o, std::vector<DOperator*>& abs_ops);
	virtual DOperator* get_orig_operator(DOperator* o) const;

	virtual int get_original_var(int abs) const;
	virtual int get_abstract_var(int orig) const;

	virtual bool has_abs_operators(DOperator* o) const;

};

#endif /* COMPOSITION_MAPPING_H */
