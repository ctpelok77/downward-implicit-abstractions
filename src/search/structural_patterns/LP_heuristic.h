#ifndef STRUCTURAL_PATTERNS_LPHEURISTIC_H
#define STRUCTURAL_PATTERNS_LPHEURISTIC_H

#include <vector>
#include <memory>

#include "abstractions/forks_abstraction.h"
#include "abstractions/general_abstraction.h"
#include "abstractions/iforks_abstraction.h"
#include "abstractions/semiforks_abstraction.h"
#include "abstractions/domain_abstraction.h"

#include "SP_heuristic.h"
#include "SP_LPConstraint.h"
#include "SP_OSI_solver.h"

class LPHeuristic : public SPHeuristic {
    int unsolved_states;
    SP_OSI_solver *lp_prob;

protected:
    int compute_Optimal_heuristic(const SPState &state);
    int compute_Additive_heuristic(const SPState &state);

    void build_LP_objective(std::vector<ConstraintVar *> &obj_func);
    void build_LP_objective(int ind, std::vector<ConstraintVar *> &obj_func);
    int build_LP_constraints(std::vector<SPLPConstraint *> &constr, const SPState &state);
    int build_LP_constraints(std::vector<SPLPConstraint *> &constr, int ind, const SPState &state);

    int generate_general_cost_constraints(std::vector<SPLPConstraint *> &constr);
    int generate_uniform_cost_constraints(std::vector<SPLPConstraint *> &constr);
    void calculate_representatives_cost(const std::vector<DOperator *> &ops,
                                        std::vector<double> &costs);
    void update_costs_from_solution(Solution *sol);

    int generate_cost_constraints(std::vector<SPLPConstraint *> &constr, int ind,
                                  const std::vector<DOperator *> &ops,
                                  std::vector<double> &costs);

    virtual SolutionMethod *add_pattern(std::vector<int> &pattern) override;

    virtual SolutionMethod *add_binary_fork(GeneralAbstraction *abs) override;
    virtual SolutionMethod *add_bounded_inverted_fork(GeneralAbstraction *abs) override;
    virtual SolutionMethod *add_pattern(GeneralAbstraction *abs) override;
    virtual SolutionMethod *add_binary_semifork(GeneralAbstraction *abs) override;

    virtual SolutionMethod *add_binary_fork(ForksAbstraction *fork, Domain *abs_domain) override;
    virtual SolutionMethod *add_bounded_inverted_fork(IforksAbstraction *ifork, Domain *abs_domain) override;
    virtual SolutionMethod *add_binary_semifork(SemiForksAbstraction *sfork, Domain *abs_domain) override;

public:
    LPHeuristic(
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
        lp::LPSolverType lp_solver_type);

    virtual ~LPHeuristic();

    virtual int compute_heuristic(const State &ancestor_state) override;

    virtual int get_number_of_unsolved_states() const override { return unsolved_states; }
    virtual void print_statistics() const override;

    static void add_options_to_feature(plugins::Feature &feature);
};

#endif /* STRUCTURAL_PATTERNS_LPHEURISTIC_H */
