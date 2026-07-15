#ifndef STRUCTURAL_PATTERNS_MAPPINGS_VAR_PROJ_MAPPING_H
#define STRUCTURAL_PATTERNS_MAPPINGS_VAR_PROJ_MAPPING_H

#include "mapping.h"

class VarProjMapping: public virtual Mapping {

protected:
	std::vector<int> abstract_vars;      // Holds abstract variables for each original variable
	std::vector<int> original_vars;		// Holds original variables for each abstract variable
public:
	VarProjMapping(const Problem* orig, Problem* abs, const std::vector<int>& orig_vars,  const std::vector<int>& abs_vars);
	VarProjMapping();
	virtual ~VarProjMapping();

	virtual int get_abstract_value(int original_variable, int original_value, int abstract_variable) const;

	virtual int get_abs_operators(DOperator* o, std::vector<DOperator*>& abs_ops) = 0;
	virtual DOperator* get_orig_operator(DOperator* o) const = 0;

	std::vector<int> get_original_vars() const;
	void set_original_vars(const std::vector<int>& vars);

	std::vector<int> get_abstract_vars() const;
	void set_abstract_vars(const std::vector<int>& vars);

	virtual int get_original_var(int abs) const;
	virtual int get_abstract_var(int orig) const;

	virtual bool has_abs_operators(DOperator* o) const = 0;

};

#endif /* VAR_PROJ_MAPPING_H */
