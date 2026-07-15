#ifndef STRUCTURAL_PATTERNS_MAPPINGS_OP_HASH_MAPPING_H
#define STRUCTURAL_PATTERNS_MAPPINGS_OP_HASH_MAPPING_H

#include <unordered_map>
#include <vector>

#include "mapping.h"

class OpHashMapping: public virtual Mapping {

protected:

	std::vector<int> original_ops;

	std::unordered_map<int, std::vector<DOperator*>*> beta_func;

public:
	OpHashMapping(const Problem* orig, Problem* abs);
	OpHashMapping();
	virtual ~OpHashMapping();

	void initialize();

	virtual int get_abstract_value(int original_variable, int original_value, int abstract_variable) const;

	virtual int get_abs_operators(DOperator* o, std::vector<DOperator*>& abs_ops);

	virtual DOperator* get_orig_operator(DOperator* o) const;

	virtual void add_abs_operator(DOperator *o, DOperator* a);

	virtual int get_original_var(int abs) const {return abs;}
	virtual int get_abstract_var(int orig) const {return orig;}

	virtual bool has_abs_operators(DOperator* o) const;

};

#endif /* OP_HASH_MAPPING_H */
