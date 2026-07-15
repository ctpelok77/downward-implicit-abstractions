#include "solution_method.h"
#include <cfloat>
#include <limits>

using namespace std;
//using namespace __gnu_cxx;

SolutionMethod::SolutionMethod(int d_vars_mult) : d_vars_multiplier(d_vars_mult) {
	index = 0;
	active = true;
//	active = false;
	solution = NULL;
	abstraction = NULL;
	obj_func = NULL;
	use_cache = false;

}

SolutionMethod::~SolutionMethod() {
	if (solution)
		delete solution;
	if (abstraction)
		delete abstraction;

//	if (use_cache) {
//		delete state_cached_values;
//	}

	SolutionMethod::free_constraints();
}

void SolutionMethod::use_caching() {
	use_cache = true;
//    state_cache = new __gnu_cxx::hash_map<StateProxy, double>(0, __gnu_cxx::hash<StateProxy>(num_abs_vars), std::equal_to<StateProxy>(num_abs_vars));

}

void SolutionMethod::free_constraints() {
	int sz = static_LPConstraints.size();
	for (int i=0;i<sz;i++){
		delete static_LPConstraints[i];
    }
	static_LPConstraints.clear();
}


Mapping* SolutionMethod::get_mapping() const {
	return abstraction->get_mapping();
}

void SolutionMethod::set_mapping(Mapping* map) {
	abstraction->set_mapping(map);
}


AbstractionType SolutionMethod::get_abstraction_type() const {
	return abstraction->get_abstraction_type();
}

void SolutionMethod::set_abstraction_type(AbstractionType type) {
	abstraction->set_abstraction_type(type);
}


void SolutionMethod::set_solution(Solution* sol){
	if (solution)
		delete solution;
	solution = sol;
}

Solution* SolutionMethod::get_solution(){
	return solution;
}

GeneralAbstraction* SolutionMethod::get_abstraction() const {
	return abstraction;
}

void SolutionMethod::set_abstraction(GeneralAbstraction* abs) {
	abstraction = abs;
	num_abs_vars = abs->get_mapping()->get_abstract()->get_vars_number();
}

bool SolutionMethod::is_active() const {
	return active;
}

void SolutionMethod::activate() {
	active = true;
}

void SolutionMethod::deactivate() {
	active = false;
}

void SolutionMethod::set_objective() {
	obj_func = new SPLPConstraint();
	int h = h_var();
	obj_func->add_val(h, 1.0);
	obj_func->finalize();
}

void SolutionMethod::get_objective(vector<ConstraintVar*>& res) const {
	res = obj_func->get_vals();
}

void SolutionMethod::get_static_constraints(vector<SPLPConstraint*> &stat_constr) const {
	stat_constr = static_LPConstraints;
}

int SolutionMethod::append_constraints(const SPState &state, vector<SPLPConstraint*> &constr) {

	int res = 0;
	vector<SPLPConstraint*> static_constr;
	get_static_constraints(static_constr);
	int static_size = static_constr.size();
	for (int i = 0; i < static_size; i++) {
		static_constr[i]->set_index(get_abstraction_index());
		constr.push_back(static_constr[i]);
		res+=static_constr[i]->get_num_vars();
//		if (STATISTICS >= 2) {
//			cout << index << ":  ";
//			static_constr[i]->dump();
//		}
	}

	vector<SPLPConstraint*> dyn_constr;
	get_dynamic_constraints(state, dyn_constr);
	int dyn_size = dyn_constr.size();
	for (int i = 0; i < dyn_size; i++) {
		dyn_constr[i]->set_index(get_abstraction_index());
		constr.push_back(dyn_constr[i]);
		res += dyn_constr[i]->get_num_vars();
//		if (STATISTICS >= 2) {
//			cout << index << ":  ";
//			dyn_constr[i]->dump();
//		}
	}
	return res;
}



SPLPConstraint* SolutionMethod::set_x_eq_0_constraint(int x, bool tokeep) const {
	// x = 0
	SPLPConstraint* lpc = new SPLPConstraint(0.0,0.0,tokeep);
	lpc->add_val(x, 1.0);
	lpc->finalize();
	return lpc;
}

SPLPConstraint* SolutionMethod::set_x_eq_y_constraint(int x, int y, bool tokeep) const {
	// x = y
	SPLPConstraint* lpc = new SPLPConstraint(0.0,0.0,tokeep);

	lpc->add_val(x, -1.0);
	lpc->add_val(y, 1.0);
	lpc->finalize();
	return lpc;
}

SPLPConstraint* SolutionMethod::set_x_leq_y_constraint(int x, int y, bool tokeep) const {
	// x <= y
	SPLPConstraint* lpc = new SPLPConstraint(0.0,numeric_limits<double>::max(),tokeep);

	lpc->add_val(x, -1.0);
	lpc->add_val(y, 1.0);

	lpc->finalize();
	return lpc;
}

SPLPConstraint* SolutionMethod::set_x_leq_y_plus_z_constraint(int x, int y, int z, bool tokeep) const {
	// x <= y + z
	SPLPConstraint* lpc = new SPLPConstraint(0.0,numeric_limits<double>::max(),tokeep);

	lpc->add_val(x, -1.0);
	lpc->add_val(y, 1.0);
	lpc->add_val(z, 1.0);

	lpc->finalize();
	return lpc;
}


void SolutionMethod::dump() const {

	cout << "Objective function:" << endl;
	cout << "max " ;
	for (size_t i=0; i < obj_func->get_vals().size(); ++i)
		cout << " + " << obj_func->get_vals()[i]->val << " * x_" << obj_func->get_vals()[i]->var;
	cout << endl;

	cout << "Static Constraints:" << endl;
	vector<SPLPConstraint*> static_constr;
	get_static_constraints(static_constr);
	for (size_t i=0; i < static_constr.size(); ++i)
		static_constr[i]->dump();
}


int SolutionMethod::encode_state(const SPState &state) const {
	// Encode state as a single integer (Lehmer code / mixed-radix).
	const Problem *abs = get_mapping()->get_abstract();
	int code = 0;
	int multiplier = 1;
	for (size_t v=0; v < state.size(); ++v) {
		code += state[v] * multiplier;
		multiplier *= abs->get_variable_domain(v);
	}
	return code;
}

double SolutionMethod::get_cached_state_value(const SPState &abs_state) {
	// Returns -1 when the value does not exist
	auto it = state_cached_values.find(encode_state(abs_state));
	if (it == state_cached_values.end())
		return -1.0;
	return it->second;
}

void SolutionMethod::set_cached_state_value(const SPState &abs_state, double val) {
	state_cached_values[encode_state(abs_state)] = val;
}

SPState SolutionMethod::get_abstract_state(const SPState &state) {
	int num_abstract_variables = get_mapping()->get_abstract()->get_vars_number();
	vector<int> state_buffer(num_abstract_variables);
	for (int abstract_variable = 0; abstract_variable < num_abstract_variables; abstract_variable++) {
		state_buffer[abstract_variable] = get_mapping()->get_abstract_state_value(state, abstract_variable);
	}
	return get_mapping()->get_abstract()->get_state_by_values(state_buffer);
}

double SolutionMethod::sum_infinity(double num1, double num2) const {
	if (num1 < infinity && num2 < infinity)
		return num1 + num2;
	return infinity;
}

/*
void SolutionMethod::determine_relevant_operators(std::vector<bool> &relevant_operators) const {
	const vector<DOperator*> &ops = get_mapping()->get_original()->get_operators();
	relevant_operators.resize(ops.size(), false);
	for (size_t op_no = 0; op_no < ops.size(); ++op_no) {
		vector<DOperator*> abs_ops;
		get_mapping()->get_abs_operators(ops[op_no],abs_ops);
		relevant_operators[op_no] = (abs_ops.size() > 0);
	}
}


const std::vector<bool>& SolutionMethod::get_relevant_operators() const {
    if (cached_relevant_operators.empty()) {
        determine_relevant_operators(cached_relevant_operators);
    }
    return cached_relevant_operators;

}
*/

bool SolutionMethod::is_relevant(int op_no) const {
	const vector<DOperator*> &ops = get_mapping()->get_original()->get_operators();
	return get_mapping()->has_abs_operators(ops[op_no]);
}
