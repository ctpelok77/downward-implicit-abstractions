#include "domain.h"

using namespace std;

Domain::Domain() {

}

Domain::Domain(int v, int old_sz, int new_sz) {
	set_var(v, old_sz, new_sz);
}

Domain::~Domain() {

}

void Domain::set_var(int v, int old_sz, int new_sz) {
	var = v;
	old_size = old_sz;
	new_size = new_sz;
	new_domain.assign(old_size, -1);
//	old_domain.reserve(new_size);
}

void Domain::update_var(int v) {
	var = v;
}

int Domain::get_var() {
	return var;
}
int Domain::get_old_size() {
	return old_size;
}
int Domain::get_new_size() {
	return new_size;
}


void Domain::set_value(int orig_val, int abs_val) {
	new_domain[orig_val] = abs_val;
}


void Domain::get_orig_values(int abs_val, vector<int>& ret) const {

	for (int i = 0; i < old_size; i++)
		if (new_domain[i] == abs_val)
			ret.push_back(i);
}


int Domain::get_abs_value(int orig_val) const {
	assert(orig_val >= 0);
	assert(orig_val < old_size);
	return new_domain[orig_val];
}


void Domain::dump() const {
	cout<< "Domain for variable " << var << endl;
	for (int i = 0; i < old_size; i++)
		cout << "  "<< i << " => " << new_domain[i] << endl;
}

