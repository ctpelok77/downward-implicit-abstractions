#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_GENERAL_ABSTRACTION_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_GENERAL_ABSTRACTION_H

#include "../problem.h"
#include "../mappings/mapping.h"
#include "domain.h"
#include "../SP_globals.h"

#include <memory>

class AbstractTask;

class GeneralAbstraction {

private:
	Mapping* m;
	AbstractionType abstraction_type;
	AbstractionCreationStatus creation_status;

public:
	GeneralAbstraction();
	virtual ~GeneralAbstraction();

	virtual void create(const Problem* p) = 0;
	// Convenience overload: build a Problem from the shared AbstractTask and call create(Problem*).
	// This lets call sites in SP_heuristic pass the Heuristic-owned shared_ptr<AbstractTask>
	// instead of keeping a raw original_problem pointer around.
	void create(const std::shared_ptr<AbstractTask> &task);
	virtual void set_root_var_and_domain(Domain* new_dom) = 0;
	virtual void set_pattern(std::vector<int>& pattern) = 0;
	virtual void abstract_action(const std::vector<int>& abs_vars, DOperator* op, std::vector<DOperator*>& abs_op) = 0;
	virtual void abstract_actions(const std::vector<int>& abs_vars, const std::vector<DOperator*>& orig_ops,
			std::vector<DOperator*>& ops, std::vector<std::pair<DOperator*, DOperator*> >& ops_to_add);

	virtual Mapping* get_mapping() const;
	virtual void set_mapping(Mapping* map);

	virtual AbstractionType get_abstraction_type() const;
	virtual void set_abstraction_type(AbstractionType type);

	virtual AbstractionCreationStatus get_abstraction_creation_status() const { return creation_status; }
	virtual void set_abstraction_creation_status(AbstractionCreationStatus status) { creation_status = status; }

	virtual void remove_abstract_operators() {m->remove_abstract_operators();}
	virtual int get_abstract_root() const = 0;
};

#endif /* GENERAL_ABSTRACTION_H */
