#ifndef STRUCTURAL_PATTERNS_LP_SOLUTION_H
#define STRUCTURAL_PATTERNS_LP_SOLUTION_H
#include <vector>
#include <iostream>
#include <unordered_map>


class Solution {
private:
	std::unordered_map<int, double> sol;

public:
	Solution();
	virtual ~Solution();
	void set_solution(std::vector<double>& s);
	void set_solution(const double* s, int n_vars);
	double get_value(int var) const;
	void set_value(int var, double val);
	void remove_var(int var);
	void clear_solution();
	void dump(int ind = 0) const;
	int get_size() const;
};

#endif /* LP_SOLUTION_H */
