#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_HOURGLASSES_ABSTRACTION_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_HOURGLASSES_ABSTRACTION_H

#include "general_abstraction.h"

class HourglassesAbstraction: public virtual GeneralAbstraction {
protected:
	int var;
	int abs_root_var;

	std::vector<int> leafs;
	std::vector<int> parents;
	std::vector<bool> is_parent;
	std::vector<int> abs_parents; // Keeps the abstract variable indexes. The way it is currently constructed, that would be 1,2,...,k
	std::vector<int> abs_leafs; // Keeps the abstract variable indexes. The way it is currently constructed, that would be k+1,k+2,...,n


	bool is_singleton;
	bool is_empty;
	bool is_var_singleton;

public:
	HourglassesAbstraction();
	HourglassesAbstraction(int v, const std::vector<int>& parent_vars);
	virtual ~HourglassesAbstraction();

	virtual void create(const Problem* p);
	virtual void set_root_var_and_domain(Domain* new_dom) { var = new_dom->get_var();}
	virtual void set_pattern(std::vector<int>&) {};

	void set_variable_and_parents(int v, const std::vector<int>& parent_vars);

	int get_root() const;
	bool is_empty_abstraction() const;

	bool is_singleton_abstraction() const;


	virtual void abstract_action(const std::vector<int>& abs_vars, DOperator* op, std::vector<DOperator*>& abs_op);

	bool is_originally_singleton() const {return is_var_singleton;}
	virtual int get_abstract_root() const {return abs_root_var;}

	const std::vector<int>& get_leafs() const {return leafs;}
	const std::vector<int>& get_parents() const {return parents;}

	int get_num_leafs() const { return leafs.size(); }

	const std::vector<int>& get_abs_leaf_vars() const {return abs_leafs;}
	const std::vector<int>& get_abs_parent_vars() const {return abs_parents;}

};

#endif /* HOURGLASSES_ABSTRACTION_H */
