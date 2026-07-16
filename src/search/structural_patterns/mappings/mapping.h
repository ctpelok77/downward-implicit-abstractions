#ifndef STRUCTURAL_PATTERNS_MAPPINGS_MAPPING_H
#define STRUCTURAL_PATTERNS_MAPPINGS_MAPPING_H

#include <vector>
#include "../problem.h"
#include "../d_operator.h"

class Mapping {

protected:

	const Problem* original;
	Problem* abstract;

public:
	const Problem* get_original() const {return original;}
	Problem* get_abstract() const {return abstract;}

	virtual int get_abstract_value(int original_variable, int original_value, int abstract_variable) const = 0;
	int get_abstract_state_value(const SPState &state, int abstract_variable) const;

	virtual int get_abs_operators(DOperator* o, std::vector<DOperator*>& abs_ops) = 0;
	virtual DOperator* get_orig_operator(DOperator* o) const = 0;

	// Index-based variant: avoids going through the original Problem* pointer.
	// Default delegates to the DOperator* overload; subclasses may override for
	// direct hash-table lookup without touching the Problem object.
	virtual int get_abs_operators_by_index(int op_index,
	                                       std::vector<DOperator*> &abs_ops);
	virtual bool has_abs_operators_by_index(int op_index) const;

	Mapping(const Problem* orig, Problem* abs);
	Mapping();
	virtual ~Mapping();

	void set_original(const Problem* p);
	void set_abstract(Problem* p);

	void remove_abstract_operators() {abstract->delete_operators();}

	virtual int get_original_var(int abs) const = 0;
	virtual int get_abstract_var(int orig) const = 0;

	virtual bool has_abs_operators(DOperator* o) const = 0;

};

#endif /* MAPPING_H */
