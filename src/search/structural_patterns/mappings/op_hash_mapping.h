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

	virtual int get_abstract_value(int original_variable, int original_value, int abstract_variable) const override;

	virtual int get_abs_operators(DOperator* o, std::vector<DOperator*>& abs_ops) override;
	// Optimized overload: goes directly to the hash table without touching Problem*.
	virtual int get_abs_operators_by_index(int op_index,
	                                       std::vector<DOperator*> &abs_ops) override;
	virtual bool has_abs_operators_by_index(int op_index) const override;

	virtual DOperator* get_orig_operator(DOperator* o) const override;

	virtual void add_abs_operator(DOperator *o, DOperator* a);

	virtual int get_original_var(int abs) const override {return abs;}
	virtual int get_abstract_var(int orig) const override {return orig;}

	virtual bool has_abs_operators(DOperator* o) const override;

};

#endif /* OP_HASH_MAPPING_H */
