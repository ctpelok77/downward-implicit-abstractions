#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_SOLUTION_METHOD_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_SOLUTION_METHOD_H

#include "../abstractions/general_abstraction.h"
#include "../solution.h"
#include "../mappings/mapping.h"
#include "../problem.h"
#include "../SP_LPConstraint.h"
#include "../../utils/system.h"

#include <memory>
#include <unordered_map>

class AbstractTask;

class SolutionMethod {
    // Caching is keyed by a flat integer encoding of the abstract state values.
    // -1.0 sentinel means "not cached".
    std::unordered_map<int, double> state_cached_values;

protected:
	int index;
	Solution* solution;
	GeneralAbstraction* abstraction;
	bool active;

	SPLPConstraint* obj_func;
	std::vector<SPLPConstraint*> static_LPConstraints;

	int d_vars_multiplier; // For moving from LP to offline

	int num_abs_vars;
	bool use_cache;
	double sum_infinity(double num1, double num2) const;

public:
	SolutionMethod(int d_vars_mult = 2);
	virtual ~SolutionMethod();

	Mapping* get_mapping() const;
	void set_mapping(Mapping* map);

	AbstractionType get_abstraction_type() const;
	void set_abstraction_type(AbstractionType type);

	void set_root_var_and_domain(Domain* new_dom) {abstraction->set_root_var_and_domain(new_dom);}
	void set_pattern(std::vector<int>& pattern) {abstraction->set_pattern(pattern);}
	void create(const Problem* p) {abstraction->create(p);}
	void create(const std::shared_ptr<AbstractTask> &t) {abstraction->create(t);}

	int get_abstraction_index() const {return index;}
	void set_abstraction_index(int ind) {index = ind;}

	virtual double get_solution_value(const SPState &state) = 0;
	virtual void set_solution(Solution* sol);
	virtual Solution* get_solution();

	void use_caching();
	GeneralAbstraction* get_abstraction() const;

	void set_abstraction(GeneralAbstraction* abs);
	bool is_active() const;
	void activate();
	void deactivate();
	virtual void remove_abstract_operators() {abstraction->remove_abstract_operators();}

	virtual void initiate() = 0;
	virtual void solve() = 0;
	virtual int get_num_vars() const = 0;

	virtual void set_objective();
	virtual void set_static_constraints() {exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);}

	void get_objective(std::vector<ConstraintVar*>& res) const;
	void get_static_constraints(std::vector<SPLPConstraint*> &stat_constr) const;
	virtual void get_dynamic_constraints(const SPState &, std::vector<SPLPConstraint*> &) {exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);}
	int append_constraints(const SPState &state, std::vector<SPLPConstraint*> &constr);

	SPLPConstraint* set_x_eq_0_constraint(int x, bool tokeep) const;
	SPLPConstraint* set_x_eq_y_constraint(int x, int y, bool tokeep) const;
	SPLPConstraint* set_x_leq_y_constraint(int x, int y, bool tokeep) const;
	SPLPConstraint* set_x_leq_y_plus_z_constraint(int x, int y, int z, bool tokeep) const;

	virtual void dump() const;
	virtual int h_var() const {exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);}
	virtual int w_var(DOperator* ) const {exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);}

	void set_d_vars_multiplier(int val) {d_vars_multiplier = val;}
	virtual void set_default_number_of_variables() {}
	virtual void dump_number_of_variables() const { std::cout << "d_vars_multiplier: " << d_vars_multiplier << std::endl; }

	virtual void free_constraints();

	// Caching states
	double get_cached_state_value(const SPState &abs_state);
	void set_cached_state_value(const SPState &abs_state, double val);

	virtual void dump_solution() const {
		solution->dump(index);
	}

	SPState get_abstract_state(const SPState &state);

	// Returns all operators affecting this abstraction
//    const std::vector<bool> &get_relevant_operators() const;
//    void determine_relevant_operators(std::vector<bool> &relevant_operators) const;
    bool is_relevant(int op_no) const;

private:
    // Encode an SPState as a single integer key for the cache.
    // Only correct for small abstract problems (product of domains < INT_MAX).
    int encode_state(const SPState &state) const;
};






#endif /* SOLUTION_METHOD_H */
