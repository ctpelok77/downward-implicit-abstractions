#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORK_GEN_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORK_GEN_H

#include "binary_forks_gen.h"
#include <cfloat>

/*
 * LP construction for binary-root-domain forks.
 * Ported from old FD; only change is State → SPState throughout.
 */

class LPBinaryFork : public BinaryFork {

    virtual SPLPConstraint *set_h_constraint(int sigma, const SPState &eval_state) const;

protected:
    std::vector<SPLPConstraint *> dynamic_LPConstraints[2];

public:
    LPBinaryFork();
    LPBinaryFork(GeneralAbstraction *abs);
    LPBinaryFork(ForksAbstraction *f, Domain *abs_domain);
    virtual ~LPBinaryFork();

    virtual void initiate() override;
    virtual void set_static_constraints() override;
    virtual void get_dynamic_constraints(const SPState &state,
                                         std::vector<SPLPConstraint *> &dyn_constr) override;
    virtual int w_var(DOperator *a) const override;

    virtual void remove_abstract_operators() override {}
    virtual void free_constraints() override;
};

#endif /* STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORK_GEN_H */
