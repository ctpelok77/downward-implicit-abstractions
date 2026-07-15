#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_VARPROJECTION_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_VARPROJECTION_H

#include "general_abstraction.h"
#include <vector>
#include "../mappings/var_proj_mapping.h"
#include "../problem.h"


class VariablesProjection: public virtual GeneralAbstraction {
	std::vector<int> ptrn;
public:
	VariablesProjection(std::vector<int>& pattern);
	VariablesProjection();
	virtual ~VariablesProjection();
	virtual void create(const Problem* p);
	virtual void set_pattern(std::vector<int>& pattern);
	virtual void set_root_var_and_domain(Domain*) {};

	void abstract_prevail(std::vector<Prevail> prevail, std::vector<Prevail>& prv, const std::vector<int>& abs_vars);
	void abstract_prepost(std::vector<PrePost> prepost, std::vector<PrePost>& pre, const std::vector<int>& abs_vars);

	virtual void abstract_action(const std::vector<int>& abs_vars, DOperator* op, std::vector<DOperator*>& abs_op);
	virtual int get_abstract_root() const {return 0;}

};

#endif /* VARPROJECTION_H */
