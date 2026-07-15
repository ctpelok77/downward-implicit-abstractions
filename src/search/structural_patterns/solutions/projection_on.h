#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_PROJECTION_ON_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_PROJECTION_ON_H

#include "projection_gen.h"
#include "../abstractions/general_abstraction.h"

class Projection_ON: public Projection {
public:
	Projection_ON();
	Projection_ON(GeneralAbstraction* abs);
	Projection_ON(std::vector<int>& pattern);
	virtual ~Projection_ON();

	virtual double get_solution_value(const SPState &state) {
		solve();
		return Projection::get_solution_value(state);
	}
	virtual void remove_abstract_operators() {}

};

#endif /* PROJECTION_ON_H */
