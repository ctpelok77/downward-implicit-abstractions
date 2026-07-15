#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_DOMAIN_ABSTRACTION_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_DOMAIN_ABSTRACTION_H

#include "general_abstraction.h"


class DomainAbstraction: public GeneralAbstraction {
	Domain* dom;
public:
	DomainAbstraction();
	DomainAbstraction(Domain* new_dom);
	virtual ~DomainAbstraction();

	virtual void create(const Problem* p);
//	void set_domain(Domain* new_dom);
	virtual void set_root_var_and_domain(Domain* new_dom);
	virtual void set_pattern(std::vector<int>&) {};
	void get_orig_values(int abs_val, std::vector<int>& ret) const;
	int get_abs_value(int orig_val) const;

	void abstract_prevail(const std::vector<Prevail>& prevail, std::vector<Prevail>& prv);
	void abstract_prepost(const std::vector<PrePost>& prepost, std::vector<PrePost>& pre, std::vector<Prevail>& prv);
	virtual void abstract_action(const std::vector<int>& abs_vars, DOperator* op, std::vector<DOperator*>& abs_ops);

	virtual int get_abstract_root() const {return dom->get_var();}

};

#endif /* DOMAIN_ABSTRACTION_H */
