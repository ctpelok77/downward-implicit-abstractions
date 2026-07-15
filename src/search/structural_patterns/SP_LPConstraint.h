#ifndef STRUCTURAL_PATTERNS_SPLPCONSTRAINT_H
#define STRUCTURAL_PATTERNS_SPLPCONSTRAINT_H

#include <map>
#include <vector>
#include <iostream>


struct ConstraintVar {
    int var;
    double val;
    ConstraintVar(int v, double l) : var(v), val(l) {}
    void dump(int index) { std::cout << val << " * x_" << var+index; }
};

class SPLPConstraint {

	std::map<int, double> c_vars;
    std::vector<ConstraintVar*> vars;

    double lb, ub;
    bool to_keep;
    int index;

public:
	SPLPConstraint();
	SPLPConstraint(SPLPConstraint* lpc, bool keep);
	SPLPConstraint(double l, double u, bool keep);
	virtual ~SPLPConstraint();

	void add_val(int var, double val);
	void finalize();
	const std::vector<ConstraintVar*>& get_vals() const;
	const std::map<int, double>& get_c_vars() const;

	double get_lb() {return lb;};
	double get_ub() {return ub;};
	bool tokeep() {return to_keep;};
	int get_index() {return index;};
	void set_index(int ind) {index = ind;};

	int get_num_vars() {return vars.size();};

    void free_mem();
    void delete_constr();
    void dump();


};

#endif /* LPCONSTRAINT_H */
