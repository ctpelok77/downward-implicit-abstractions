#ifndef STRUCTURAL_PATTERNS_SPHEURISTIC_H
#define STRUCTURAL_PATTERNS_SPHEURISTIC_H

#include <memory>
#include <set>
#include <vector>

#include "abstractions/iforks_abstraction.h"
#include "abstractions/forks_abstraction.h"
#include "abstractions/semiforks_abstraction.h"
#include "abstractions/one_dep_hourglasses.h"
#include "abstractions/one_dep_binary_hourglasses.h"
#include "../heuristic.h"
#include "problem.h"
#include "abstractions/general_abstraction.h"
#include "abstractions/var_projection.h"
#include "mappings/var_proj_mapping.h"
#include "mappings/mapping.h"
#include "abstractions/domain_abstraction.h"
#include "solutions/solution_method.h"
#include "SP_globals.h"
#include "../utils/logging.h"
#include "../task_utils/causal_graph.h"

#include <iostream>

class Problem;

namespace plugins {
class Options;
class Feature;
}

class SPHeuristic: public Heuristic {
    DecompositionStrategy strategy;
    SingletonStrategy singletons_strategy;
    CostPartitioning cost_partitioning;
    VariablesSelectionStrategy selected_ensemble;
    std::vector<SolutionMethod*> ensemble;
    std::vector<double> pattern_weights;
    bool use_caching;
    int semifork_hat_bound_size;

    // Cached goal values: goal_vals_[v] == -1 if v is not a goal variable,
    // otherwise the required goal value.  Populated in the constructor.
    std::vector<int> goal_vals_;

protected:
    // Helpers that use task_proxy instead of original_problem for structural
    // queries (variable count, domain size, CG, goals).  These are used during
    // ensemble construction so that those paths no longer depend on Problem.
    int sp_var_count() const { return task_proxy.get_variables().size(); }
    int sp_var_domain(int v) const {
        return task_proxy.get_variables()[v].get_domain_size();
    }
    bool sp_is_goal_var(int v) const { return goal_vals_[v] != -1; }
    bool sp_has_goal_child(int v) const {
        const causal_graph::CausalGraph &cg = task_proxy.get_causal_graph();
        for (int s : cg.get_successors(v))
            if (sp_is_goal_var(s)) return true;
        return false;
    }
    bool sp_is_goal(const SPState &sp) const {
        for (int v = 0; v < (int)goal_vals_.size(); ++v)
            if (goal_vals_[v] != -1 && sp[v] != goal_vals_[v])
                return false;
        return true;
    }

    // DTG-based helpers: compute Domain Transition Graph successors for variable
    // v from value val, and BFS backward distances to the goal value.
    // These replace original_problem->get_dtg_successors / get_domain_values_by_distance_to_goal.
    void sp_dtg_successors(int var, int val, std::vector<int> &succ) const;
    void sp_domain_values_by_distance_to_goal(int var,
                                              std::vector<std::vector<int>> &vals,
                                              std::vector<int> &dist_to_goal) const;

    // CG-based hat enumeration: replaces original_problem->compute_connected_variables_bounded_sets.
    void sp_compute_hat_variants(int center, std::set<int> &curr, int bound,
                                 std::set<std::set<int>> &result) const;
    bool sp_hat_has_leaf(int center, const std::set<int> &hat) const;
    bool sp_hat_connected_to_goals(int center, const std::set<int> &hat) const;
    void sp_compute_connected_variables_bounded_sets(int v, int bound,
                                                     std::vector<std::vector<int>> &hats) const;

    int SIZEOFPATTERNLIMIT;
    int PERCENTAGEOFENSEMBLE;
    int STATISTICS;

    void create_binary_forks();
    void create_bounded_inverted_forks();
    void create_binary_forks_and_bounded_iforks();
    void create_binary_semiforks();
    void create_binary_hourglasses();

    void create_binary_fork(int v, int dom_size);
    void create_bounded_inverted_fork(int v, int dom_size);
    void create_pattern(std::vector<int>& pattern);
    void create_singleton(int var);
    void create_binary_semifork(int v, const std::vector<int>& hat, int dom_size);
    void create_binary_hourglass(int v, const std::vector<int>& parents, int dom_size);

    void create_binary_forkLOO(int v, int dom_size);
    void create_bounded_inverted_forkDGV(int v, int bound, int dom_size);
    void create_inverted_fork_all_paths(int v, int dom_size);
    void create_bounded_inverted_fork_check_paths(int v, int bound, int paths_bound, int dom_size);
    void create_binary_semiforkLOO(int v, const std::vector<int>& hat, int dom_size);
    void create_binary_hourglassLOO(int v, const std::vector<int>& parents, int dom_size);

    void apply_uniform_representatives_cost();
    void apply_full_representatives_cost();

    Domain* create_LOO_domain(int var, int to_leave, int dom_size) const;
    Domain* create_id_domain(int var, int dom_size) const;
    Domain* create_DGV_domain(int var, std::vector<int>& distances, int lb, int ub) const;

    void create_ensemble_from_file(std::istream &in);
    void set_smart_representatives_cost();
    void set_smart_uniform_representatives_cost();

    void apply_uniform_ensemble_representatives_cost();
    void apply_weighted_uniform_representatives_cost();

    virtual void add_ensemble_member(SolutionMethod* membr) { ensemble.push_back(membr); }

    virtual SolutionMethod* add_pattern(std::vector<int>& pattern);

    virtual SolutionMethod* add_binary_fork(GeneralAbstraction* abs);
    virtual SolutionMethod* add_bounded_inverted_fork(GeneralAbstraction* abs);
    virtual SolutionMethod* add_pattern(GeneralAbstraction* abs);
    virtual SolutionMethod* add_binary_semifork(GeneralAbstraction* abs);
    virtual SolutionMethod* add_binary_hourglass(GeneralAbstraction* abs);

    virtual SolutionMethod* add_binary_fork(ForksAbstraction* f, Domain* abs_domain);
    virtual SolutionMethod* add_bounded_inverted_fork(IforksAbstraction* ifork, Domain* abs_domain);
    virtual SolutionMethod* add_binary_semifork(SemiForksAbstraction* sfork, Domain* abs_domain);
    virtual SolutionMethod* add_binary_hourglass(OneDependentHourglassesAbstraction* hg, Domain* abs_domain);
    virtual SolutionMethod* add_binary_semifork(OneDependentHourglassesAbstraction* hg, Domain* abs_domain);

    int get_num_active_members();

    void deactivate_all();

    // Convert a new-FD State to an SPState (vector<int>).
    SPState to_sp_state(const State &state) const;

public:
    // Primary constructor: takes shared_ptr<AbstractTask> from new FD plugin system.
    SPHeuristic(
        const std::shared_ptr<AbstractTask> &transform,
        bool cache_estimates,
        const std::string &description,
        utils::Verbosity verbosity,
        DecompositionStrategy strategy,
        SingletonStrategy singletons,
        CostPartitioning cost_part,
        VariablesSelectionStrategy var_strategy,
        int pattern_max_size,
        int ensemble_pct,
        int statistics,
        bool cache,
        int semifork_bound);

    // Secondary constructor: supply an already-constructed Problem (for LP heuristic path).
    SPHeuristic(
        const std::shared_ptr<AbstractTask> &transform,
        bool cache_estimates,
        const std::string &description,
        utils::Verbosity verbosity,
        DecompositionStrategy strategy,
        SingletonStrategy singletons,
        CostPartitioning cost_part,
        VariablesSelectionStrategy var_strategy,
        int pattern_max_size,
        int ensemble_pct,
        int statistics,
        bool cache,
        int semifork_bound,
        const Problem* prob);

    virtual ~SPHeuristic();

    virtual void print_statistics() const { };
    void create_ensemble();
    virtual int compute_heuristic(const State &ancestor_state) override;
    void get_ensemble_values(const State &state, std::vector<double>& vals);

    bool is_heuristic_applicable() const;

    void set_selected_ensemble_strategy(VariablesSelectionStrategy str) { selected_ensemble = str;}
    VariablesSelectionStrategy get_selected_ensemble_strategy() {return selected_ensemble;}
    void set_strategy(DecompositionStrategy str) { strategy = str;}
    DecompositionStrategy get_strategy() {return strategy;}
    void set_singletons_strategy(SingletonStrategy s) {singletons_strategy = s;}
    SingletonStrategy get_singletons_strategy() {return singletons_strategy;}

    void set_cost_partitioning_strategy(CostPartitioning part) {cost_partitioning = part;}
    CostPartitioning get_cost_partitioning_strategy() {return cost_partitioning;}

    int get_percentage_of_ensemble() {return PERCENTAGEOFENSEMBLE;}
    int get_minimal_size_for_non_projection_pattern() {return SIZEOFPATTERNLIMIT;}

    int get_num_active_members(int var);

    virtual void initialize();
    SolutionMethod* get_ensemble_member(int ind) {return ensemble[ind];}
    int get_ensemble_size() {return ensemble.size();}
    int get_num_vars();
    int get_num_active_vars();

    virtual int get_number_of_unsolved_states() const {
        exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
    }
    void solve_all();
    void solve_all_and_remove_operators();
    void check_cost_partition();
    void print_abstract_operators();

    void set_pattern_wieghts(std::vector<double>& w) { pattern_weights = w; }

    // Plugin helpers
    static void add_options_to_feature(plugins::Feature &feature);
    static std::tuple<
        DecompositionStrategy, SingletonStrategy, CostPartitioning,
        VariablesSelectionStrategy, int, int, int, bool, int>
    get_sp_arguments_from_options(const plugins::Options &opts);
};


#endif /* SPHEURISTIC_H */
