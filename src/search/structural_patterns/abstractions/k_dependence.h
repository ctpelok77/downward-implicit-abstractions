#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_KDEPENDENCE_ABSTRACTION_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_KDEPENDENCE_ABSTRACTION_H

#include "general_abstraction.h"


class KDependence: public virtual GeneralAbstraction {
	size_t bound;
	std::vector<std::vector<int> > initial_distances;  // For deciding which preconditions to keep
	int get_initial_distances(int v, int val);

public:
	KDependence();
	KDependence(size_t k);
	virtual ~KDependence();

	virtual void create(const Problem* p);

	void abstract_prevail(const std::vector<Prevail>& prevail, std::vector<Prevail>& prv);
	virtual void abstract_action(const std::vector<int>& abs_vars, DOperator* op, std::vector<DOperator*>& abs_ops);

	void set_constant(size_t k) { bound = k; }
	size_t get_constant() const { return bound; }
	virtual void set_root_var_and_domain(Domain* ) {}
	virtual void set_pattern(std::vector<int>&) {}

	virtual void abstract_actions(const std::vector<int>& abs_vars, const std::vector<DOperator*>& orig_ops,
			std::vector<DOperator*>& ops, std::vector<std::pair<DOperator*, DOperator*> >& ops_to_add);
};

#endif /* KDEPENDENCE_ABSTRACTION_H */
