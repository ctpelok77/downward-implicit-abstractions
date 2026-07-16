#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_SEMIFORKS_ABSTRACTION_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_SEMIFORKS_ABSTRACTION_H

#include "general_abstraction.h"

class SemiForksAbstraction: public virtual GeneralAbstraction {
protected:
	int var;
	int abs_root_var;

	std::vector<int> leafs;
	std::vector<int> hat;
	std::vector<bool> is_hat;
	std::vector<int> abs_hat; // Keeps the abstract variable indexes. The way it is currently constructed, that would be 1,2,...,k
	std::vector<int> abs_leafs; // Keeps the abstract variable indexes. The way it is currently constructed, that would be k+1,k+2,...,n


	bool is_singleton;
	bool is_empty;
	bool is_var_singleton;

public:
	SemiForksAbstraction();
	SemiForksAbstraction(int v, const std::vector<int>& hat_vars);
	virtual ~SemiForksAbstraction();

	using GeneralAbstraction::create;
	virtual void create(const Problem* p);
	virtual void set_root_var_and_domain(Domain* new_dom) { var = new_dom->get_var();}
	virtual void set_pattern(std::vector<int>&) {};

	void set_variable_and_hat(int v, const std::vector<int>& hat_vars);

	int get_root() const;
	bool is_empty_abstraction() const;

	bool is_singleton_abstraction() const;


	virtual void abstract_action(const std::vector<int>& abs_vars, DOperator* op, std::vector<DOperator*>& abs_op);

	bool is_originally_singleton() const {return is_var_singleton;}
	virtual int get_abstract_root() const {return abs_root_var;}

	const std::vector<int>& get_leafs() const {return leafs;}
	const std::vector<int>& get_hat() const {return hat;}

	int get_num_leafs() const { return leafs.size(); }

	const std::vector<int>& get_abs_leaf_vars() const {return abs_leafs;}
	const std::vector<int>& get_abs_hat_vars() const {return abs_hat;}

//	int get_num_abs_leaf_vars() const;
//	int get_num_abs_hat_vars() const;

};

#endif /* SEMIFORKS_ABSTRACTION_H */
