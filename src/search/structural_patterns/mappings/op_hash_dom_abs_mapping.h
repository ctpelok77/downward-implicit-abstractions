#ifndef STRUCTURAL_PATTERNS_MAPPINGS_OP_HASH_DOM_ABS_MAPPING_H
#define STRUCTURAL_PATTERNS_MAPPINGS_OP_HASH_DOM_ABS_MAPPING_H

#include "dom_abs_mapping.h"
#include "op_hash_mapping.h"

class OpHashDomAbsMapping: public virtual DomainAbstractionMapping,
		public virtual OpHashMapping {
public:
	OpHashDomAbsMapping();
	virtual ~OpHashDomAbsMapping();

	virtual int get_abstract_value(int original_variable, int original_value, int abstract_variable) const;

	virtual int get_abs_operators(DOperator* o, std::vector<DOperator*>& abs_ops);

	virtual DOperator* get_orig_operator(DOperator* o) const;

	virtual int get_original_var(int abs) const {return OpHashMapping::get_original_var(abs);}
	virtual int get_abstract_var(int orig) const {return OpHashMapping::get_abstract_var(orig);}

	virtual bool has_abs_operators(DOperator* o) const;
};

#endif /* OP_HASH_DOM_ABS_MAPPING_H */
