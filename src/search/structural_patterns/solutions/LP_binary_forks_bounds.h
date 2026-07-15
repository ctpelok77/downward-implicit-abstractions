#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORKS_BOUNDS_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORKS_BOUNDS_H

#include "LP_binary_fork_gen.h"
#include "../abstractions/binary_forks.h"
#include "binary_forks_on.h"
#include <cfloat>

/*
 * Offline-bounds LP fork: computes per-state sigma bounds via offline solve,
 * then solves an LP per state.
 */

class LPBinaryForks_b : public LPBinaryFork {
    virtual SPLPConstraint *set_h_constraint(int sigma, const SPState &eval_state) const override;

public:
    LPBinaryForks_b();
    LPBinaryForks_b(GeneralAbstraction *abs);
    LPBinaryForks_b(ForksAbstraction *f, Domain *abs_domain);
    virtual ~LPBinaryForks_b();

    virtual void initiate() override;
    virtual void set_static_constraints() override;
    virtual void get_dynamic_constraints(const SPState &state,
                                         std::vector<SPLPConstraint *> &dyn_constr) override;

    SPLPConstraint *set_h_constraint(int sigma, int root_zero, bool tokeep) const;
    SPLPConstraint *set_p_constraint(int v, int val, int pre, int post, int prv,
                                     int w_ind, bool tokeep) const;
    SPLPConstraint *set_d_constraint(int v, int val_0, int val_1, int sz,
                                     int root_zero, bool tokeep) const;

    virtual int w_v(int var, int val1, int val2, int root_val) const;

    void set_bounds_for_state(const SPState &state);
    void set_general_bounds();
    int get_domain_bound(int v, int g_v, int root_zero) const;
    int get_domain_bound(int v, int g_v, const SPState &eval_state) const;
};

#endif /* STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORKS_BOUNDS_H */
