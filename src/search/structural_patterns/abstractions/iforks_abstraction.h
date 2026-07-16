#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_IFORKS_ABSTRACTION_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_IFORKS_ABSTRACTION_H

#include "general_abstraction.h"

class IforksAbstraction: public virtual GeneralAbstraction {
protected:
	int var;
	int abs_root_var;

	std::vector<int> parents; // TODO: check if it is used;

	bool is_singleton;
	bool is_empty;
	bool is_var_singleton;


public:
	IforksAbstraction();
	IforksAbstraction(int v);
	virtual ~IforksAbstraction();

	using GeneralAbstraction::create;
	virtual void create(const Problem* p);
	virtual void set_root_var_and_domain(Domain* new_dom) { var = new_dom->get_var();}
	virtual void set_pattern(std::vector<int>&) {};

	int get_root() const;
	void get_parents(std::vector<int>& vars) const;
	bool is_empty_abstraction() const;

	bool is_singleton_abstraction() const;

	virtual void abstract_action(const std::vector<int>& abs_vars, DOperator* op, std::vector<DOperator*>& abs_op);

	int root_prepost_index(DOperator* op) const;
	int root_unconditional_prepost_index(DOperator* op) const;

	bool is_originally_singleton() const {return is_var_singleton;}
	virtual int get_abstract_root() const {return abs_root_var;}

};

#endif /* IFORKS_ABSTRACTION_H */
