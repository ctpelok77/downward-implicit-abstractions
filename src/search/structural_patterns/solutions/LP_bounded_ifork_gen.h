#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_LP_BOUNDED_IFORK_GEN_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_LP_BOUNDED_IFORK_GEN_H

#include "bounded_iforks_gen.h"
#include "../abstractions/bounded_iforks.h"
#include <cfloat>
#include "ifork_root_path.h"

/*
 * LP construction for bounded inverted forks.
 * Ported from old FD; only change is State → SPState throughout.
 */

class LPBoundedIfork : public BoundedIfork {
private:
    void precalculate_path(IforkRootPath *path);

public:
    LPBoundedIfork();
    LPBoundedIfork(GeneralAbstraction *abs);
    LPBoundedIfork(IforksAbstraction *ifork, Domain *abs_domain);
    virtual ~LPBoundedIfork();

    virtual void set_static_constraints() override;
    virtual void initiate() override;
    virtual void get_dynamic_constraints(const SPState &state,
                                         std::vector<SPLPConstraint *> &dyn_constr) override;

    virtual int h_var() const override;
    virtual int w_var(DOperator *a) const override;

    virtual void remove_abstract_operators() override {}
};

#endif /* STRUCTURAL_PATTERNS_SOLUTIONS_LP_BOUNDED_IFORK_GEN_H */
