#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_ONE_DEP_HOURGLASSES_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_ONE_DEP_HOURGLASSES_H

#include "hourglasses_abstraction.h"
#include "k_dependence.h"
#include <iostream>

class OneDependentHourglassesAbstraction: public virtual HourglassesAbstraction, public virtual KDependence {
	bool is_K_dependent() const {
		std::cout << "Checking K dependence" << std::endl;
		return get_mapping()->get_abstract()->is_k_dependent(get_constant()); }
public:
	OneDependentHourglassesAbstraction();
	OneDependentHourglassesAbstraction(HourglassesAbstraction* f);

	virtual ~OneDependentHourglassesAbstraction();

	virtual void create(const Problem* p);

	virtual int get_abstract_root() const {return HourglassesAbstraction::get_abstract_root();}


	virtual Mapping* get_mapping() const { return HourglassesAbstraction::get_mapping(); }
	virtual void set_mapping(Mapping* map) { HourglassesAbstraction::set_mapping(map); }


	virtual AbstractionType get_abstraction_type() const { return HourglassesAbstraction::get_abstraction_type(); }
	virtual void set_abstraction_type(AbstractionType type) {HourglassesAbstraction::set_abstraction_type(type); }

	virtual AbstractionCreationStatus get_abstraction_creation_status() const { return HourglassesAbstraction::get_abstraction_creation_status(); }
	virtual void set_abstraction_creation_status(AbstractionCreationStatus status) { HourglassesAbstraction::set_abstraction_creation_status(status); }

	virtual void remove_abstract_operators() {
		HourglassesAbstraction::remove_abstract_operators();
		KDependence::remove_abstract_operators();
	}

	virtual void set_root_var_and_domain(Domain* ) {}
	virtual void set_pattern(std::vector<int>&) {}
	virtual void abstract_action(const std::vector<int>& , DOperator* , std::vector<DOperator*>& ) {}



};

#endif /* ONE_DEP_HOURGLASSES_H */
