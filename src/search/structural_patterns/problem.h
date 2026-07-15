#ifndef STRUCTURAL_PATTERNS_PROBLEM_H
#define STRUCTURAL_PATTERNS_PROBLEM_H

#include <memory>
#include <vector>
#include <set>
#include <string>

#include "d_operator.h"
#include "sp_causal_graph.h"

class AbstractTask;

/*
  Within structural_patterns, a "State" is simply a vector<int> of variable
  values.  This typedef decouples the abstract sub-problems from the new FD
  StateRegistry/State machinery; the translation to/from new-FD State objects
  happens in SP_heuristic (Phase 2/3).
*/
using SPState = std::vector<int>;

class Problem {

	bool nonconditional_task;
	bool borrowed_ops;

	std::vector<std::string> variable_name;
	std::vector<int> variable_domain;

	std::vector<std::pair<int, int>> goal;
	std::vector<DOperator*> operators;
	SPCausalGraph* cg;
	std::vector<int> goal_vars;
	std::vector<std::vector<DOperator*>> v_operators;
	std::vector<DOperator> axioms;

	int** CG_distances_matrix;
	bool CG_distances_matrix_computed;
	std::vector<int> initial_state_values;

private:
	void get_all_cycle_free_paths_to_goal_ops(int v, std::vector<DOperator*>& A_v, std::vector<std::vector<DOperator*> >& paths) const;
	void remove_conditional_effects(DOperator* op, std::vector<DOperator*>& ops);
	bool check_nonconditional();
	Problem(const Problem& p);
	bool is_interfere(std::vector<Prevail>& pre1, std::vector<Prevail>& pre2) const;
	void create_operators_non_interfering_effects(const std::vector<DOperator*>& ops, std::vector<DOperator*>& to_ops) const ;
	void split_operator_interfering_effects(DOperator* op, std::vector<DOperator*>& ops) const;
	void compute_hat_variants(int c, std::set<int>& curr, int k, std::set<std::set<int> >& vals) const;
	bool chech_hat_connected_to_goals(int v, const std::set<int>& hat) const;
	bool chech_hat_for_leafs(int v, const std::set<int>& hat) const;

public:
	Problem();
	explicit Problem(const std::shared_ptr<AbstractTask> &task);

	Problem(const std::vector<std::string>& var_name,
			const std::vector<int>& var_domain,
			const std::vector<int>& init_state_buffer,
			const std::vector<std::pair<int, int> >& g,
			const std::vector<DOperator*>& ops,
			const std::vector<DOperator>& axioms);

	virtual ~Problem();

	void create_problem(const std::vector<std::string>& var_name,
			const std::vector<int>& var_domain,
			const std::vector<std::pair<int, int> >& g,
			const std::vector<DOperator*>& ops, const std::vector<DOperator>& axioms);

	SPState get_state_by_values(const std::vector<int>& values) const { return values; }
	const SPState& get_initial_state() const { return initial_state_values; }
	bool is_goal(const SPState &state) const;

	void generate_state_transition_graph(std::vector<std::vector<int> >& states) const;
	void get_applicable_states(DOperator* op, std::vector<int> & vals) const;

    const std::vector<DOperator*> &get_var_actions(int var) const {return v_operators[var];}
    const std::vector<std::string> &get_variable_names() const {return variable_name;}
	const std::vector<int> &get_variable_domains() const {return variable_domain;}
	const std::vector<DOperator*> &get_operators() const {return operators;}

	void get_goal(std::vector<std::pair<int, int> >& g) const {g = goal;}
	void get_goal_vals(std::vector<int>& g) const {g = goal_vars;}
	std::string get_variable_name(int var) const {return variable_name[var];}
	int get_variable_domain(int var) const {return variable_domain[var];}
	int get_vars_number() const {return variable_name.size();}
	int get_max_domain_size() const;
	int get_actions_number() const {return operators.size();}
	const std::vector<DOperator>& get_axioms() const{return axioms;}
	SPCausalGraph* get_causal_graph() const {return cg;}

	void set_causal_graph(SPCausalGraph* causalg) {cg = causalg;}

	void get_dtg_successors(int v, int val, std::vector<int>& vals) const;
	void get_domain_decomposition_by_distance_from_value(int v, int val, std::vector<std::vector<int> >& vals, std::vector<int>& len_from_val) const;
	void get_domain_values_by_distance_to_goal(int v, std::vector<std::vector<int> >& vals, std::vector<int>& len_to_goal) const;
	void get_cycle_free_paths_by_length(int v, int length, std::vector<std::vector<DOperator*> >& paths) const;

	void compute_connected_variables_bounded_sets(int v, int bound, std::vector<std::vector<int> >& vals) const;


	void get_all_cycle_free_paths_to_goal(int v, std::vector<std::vector<DOperator*> >& paths) const;
	int get_estimated_number_of_all_cycle_free_paths_to_goal(int v) const;
	bool is_estimated_number_of_all_cycle_free_paths_to_goal_bounded(int v, int bound) const;

	void fill_DTG(int var);

	int get_action_index(const DOperator* a) const {return a->get_index();}

	DOperator* get_action_by_index(int index) const {return operators[index];}

	bool is_nonconditional() const {return nonconditional_task;}

	void print_conditional() const;

	bool has_goal_child(int var) const;

	int get_goal_val(int var) const {return goal_vars[var];}
	bool is_goal_var(int var) const {return (-1 != get_goal_val(var));}
//	bool path_in_CG_from_to_exists(int a, int b); // Warning! Creates adjacency matrix for the Causal Graph (once)
	int get_var_index(std::string var_name) const;

 	void set_operators_to_uniform_cost();
 	void increase_operators_cost();

 	void delete_operators();
 	void delete_causal_graph();

 	int get_mem_size() const ;
 	void dumpPrevail(Prevail pre) const;
 	void dumpPrePost(PrePost pp) const;
 	void dumpAction(const DOperator* op) const;

    void dump() const;
    void dump_SAS(const char* filename) const;
    void make_single_goal();

    void dump_CG_dot(const char* filename) const;
    bool is_k_dependent(size_t k) const {
    	for (size_t i=0; i < operators.size(); ++i) {
    		if (k < operators[i]->get_prevail().size())
    			return false;
    	}
    	return true;
    }
};

#endif /* PROBLEM_H */
