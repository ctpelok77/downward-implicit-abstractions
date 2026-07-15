#include "solution.h"
#include "SP_globals.h"
#include "../utils/system.h"
using namespace std;

Solution::Solution() {

}

Solution::~Solution() {
}


void Solution::set_solution(vector<double>& s){
	sol.clear();
	for (size_t i=0; i < s.size(); ++i) {
		sol[i] = s[i];
	}
}

void Solution::set_solution(const double* s, int n_vars){
	sol.clear();
	for (int i=0;i<n_vars;i++) {
		sol[i] = s[i];
	}
}

double Solution::get_value(int var) const {
	std::unordered_map<int, double>::const_iterator it = sol.find(var);
	if (it == sol.end()) {
		cout<< "--------->No value found for " << var << endl;
		exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
	return (*it).second;
}

void Solution::set_value(int var, double val) {
	sol[var] = val;
}

void Solution::remove_var(int var) {
	sol.erase(sol.find(var));
}

void Solution::clear_solution(){
	sol.clear();
}

void Solution::dump(int ind) const {
	cout << "Variable values" << endl;
	for(auto it = sol.begin(); it != sol.end(); it++){
		cout << "x_" << (*it).first + ind << " = " << (*it).second << endl;
	}
}

int Solution::get_size() const {
	return sol.size();
}

