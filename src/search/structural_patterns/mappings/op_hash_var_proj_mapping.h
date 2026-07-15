#ifndef STRUCTURAL_PATTERNS_MAPPINGS_OP_HASH_VAR_PROJ_MAPPING_H
#define STRUCTURAL_PATTERNS_MAPPINGS_OP_HASH_VAR_PROJ_MAPPING_H

#include "op_hash_mapping.h"
#include "var_proj_mapping.h"

class OpHashVarProjMapping: public virtual OpHashMapping, public virtual VarProjMapping {
public:
	OpHashVarProjMapping();
	virtual ~OpHashVarProjMapping();

	virtual int get_abstract_value(int original_variable, int original_value, int abstract_variable) const;

	virtual int get_abs_operators(DOperator* o, std::vector<DOperator*>& abs_ops);

	virtual DOperator* get_orig_operator(DOperator* o) const;

	virtual int get_original_var(int abs) const {return VarProjMapping::get_original_var(abs);}
	virtual int get_abstract_var(int orig) const {return VarProjMapping::get_abstract_var(orig);}

	virtual bool has_abs_operators(DOperator* o) const;

};

#endif /* OP_HASH_VAR_PROJ_MAPPING_H */
