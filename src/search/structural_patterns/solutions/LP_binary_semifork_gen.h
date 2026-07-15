#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_SEMIFORK_GEN_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_SEMIFORK_GEN_H

#include "binary_semiforks_gen.h"
#include "../abstractions/semiforks_abstraction.h"
#include <cfloat>

/*
 * LP construction for binary semi-forks.
 * Ported from old FD; only change is State → SPState throughout.
 */

class LPBinarySemiFork : public BinarySemiFork {

    virtual SPLPConstraint *set_h_constraint(int sigma, const SPState &eval_state) const;
    void set_static_constraints_fork();
    void set_static_constraints_hat();
    void solve_step(const std::vector<std::vector<int>> &states, int changes);
    void set_distances(std::vector<int> free_vars, std::vector<int> state,
                       const std::vector<int> &doms, DOperator *op,
                       int changes, int theta);
    void set_base_goal_distances(std::vector<int> free_vars, std::vector<int> state,
                                  const std::vector<int> &doms, int theta);

    void dump_m_constraint(const SPState &state, int count, int theta) const;
    void dump_d_constraint(int v, int val, int sz, int theta) const;
    void dump_p_constraint(int v, int val1, int val2, int theta) const;
    void dump_w_constraint(const DOperator *op) const;

public:
    LPBinarySemiFork();
    LPBinarySemiFork(GeneralAbstraction *abs);
    LPBinarySemiFork(SemiForksAbstraction *f, Domain *abs_domain);
    virtual ~LPBinarySemiFork();

    virtual void initiate() override;
    virtual void set_static_constraints() override;
    virtual void get_dynamic_constraints(const SPState &state,
                                         std::vector<SPLPConstraint *> &dyn_constr) override;
    virtual int w_var(DOperator *a) const override;

    virtual void remove_abstract_operators() override {}

    int m_var(const SPState &state, int changes, int theta) const;
};

#endif /* STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_SEMIFORK_GEN_H */
