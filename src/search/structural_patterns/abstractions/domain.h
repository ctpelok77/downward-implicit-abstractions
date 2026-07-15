#ifndef STRUCTURAL_PATTERNS_ABSTRACTIONS_DOMAIN_H
#define STRUCTURAL_PATTERNS_ABSTRACTIONS_DOMAIN_H

#include "../problem.h"
#include "../mappings/mapping.h"
#include <vector>

class Domain {
	int var;
	int old_size, new_size;
	std::vector<int> new_domain;
	std::vector<std::vector<int> > old_domain;
public:
	Domain();
	Domain(int v, int old_sz, int new_sz);
	virtual ~Domain();

	void set_var(int v, int old_sz, int new_sz);
	void update_var(int v);
	void set_value(int orig_val, int abs_val);
	int get_var();
	int get_old_size();
	int get_new_size();

	void get_orig_values(int abs_val, std::vector<int>& ret) const;
	int get_abs_value(int orig_val) const;

	void dump() const;

};


#endif
