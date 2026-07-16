#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_FORKS_ABSTRACTION_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_FORKS_ABSTRACTION_H

#include "general_abstraction.h"

class ForksAbstraction: public virtual GeneralAbstraction {
protected:
	int var;
	int abs_root_var;

	bool is_singleton;
	bool is_empty;
	bool is_var_singleton;

public:
	ForksAbstraction();
	ForksAbstraction(int v);
	virtual ~ForksAbstraction();

	using GeneralAbstraction::create;
	virtual void create(const Problem* p);
	virtual void set_root_var_and_domain(Domain* new_dom) { var = new_dom->get_var();}
	virtual void set_pattern(std::vector<int>&) {};

	void set_variable(int v);

	int get_root() const;
	bool is_empty_abstraction() const;

	bool is_singleton_abstraction() const;


	virtual void abstract_action(const std::vector<int>& abs_vars, DOperator* op, std::vector<DOperator*>& abs_op);

	bool is_originally_singleton() const {return is_var_singleton;}
	virtual int get_abstract_root() const {return abs_root_var;}

};

#endif /* FORKS_ABSTRACTION_H */
