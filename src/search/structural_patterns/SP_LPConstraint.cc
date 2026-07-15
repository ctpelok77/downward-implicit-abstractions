#include "SP_LPConstraint.h"
#include <math.h>
#include <cfloat>
#include <cassert>
#include <limits>

using namespace std;

SPLPConstraint::SPLPConstraint() {
	lb = 0.0;
	ub = numeric_limits<double>::max();
	to_keep = true;
	index = 0;
}

SPLPConstraint::SPLPConstraint(SPLPConstraint* lpc, bool keep) {
	lb = lpc->get_lb();
	ub = lpc->get_ub();
	to_keep = keep;
	index = lpc->get_index();
	vars = lpc->get_vals();
	c_vars = lpc->get_c_vars();
}


SPLPConstraint::SPLPConstraint(double l, double u, bool keep): lb(l), ub(u), to_keep(keep) {
	index = 0;
}

SPLPConstraint::~SPLPConstraint() {
	delete_constr();
}

void SPLPConstraint::add_val(int var, double val) {
	assert(var>=0);
	if (c_vars.find(var) == c_vars.end()) {
		c_vars[var] = val;
	} else {
		c_vars[var] += val;
	}
}

void SPLPConstraint::finalize() {

	for (map<int,double>::iterator ii=c_vars.begin(); ii!=c_vars.end(); ++ii) {
	   vars.push_back(new ConstraintVar((*ii).first, (*ii).second));
	}
	c_vars.clear();
}

const vector<ConstraintVar*>& SPLPConstraint::get_vals() const {
	return vars;
}

const map<int, double>& SPLPConstraint::get_c_vars() const {
	return c_vars;
}


void SPLPConstraint::free_mem() {
	if (!to_keep) {
		delete_constr();
	}
}


void SPLPConstraint::delete_constr() {
	for (size_t i=0; i < vars.size(); ++i) {
		if (vars[i]) {
			delete vars[i];
		}
	}
	vars.clear();
}


void SPLPConstraint::dump() {
	if (0 == vars.size())
		return;
	cout << "  " << lb << " <= ";
	vars[0]->dump(index);
	for (size_t i=1; i < vars.size(); ++i) {
		cout << " + ";
		vars[i]->dump(index);
	}
	cout << " <= " << ub << endl;
}
