#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_LP_BOUNDED_IFORKS_OFF_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_LP_BOUNDED_IFORKS_OFF_H

#include "LP_bounded_ifork_gen.h"
#include "../abstractions/bounded_iforks.h"
#include <cfloat>
#include "ifork_root_path.h"

/*
 * Offline LP bounded inverted fork.
 */

class LPBoundedIforks_OFF : public LPBoundedIfork {

public:
    LPBoundedIforks_OFF();
    LPBoundedIforks_OFF(GeneralAbstraction *abs);
    LPBoundedIforks_OFF(IforksAbstraction *ifork, Domain *abs_domain);
    virtual ~LPBoundedIforks_OFF();

    virtual void set_objective() override;
    virtual void set_static_constraints() override;
    virtual void get_dynamic_constraints(const SPState &, std::vector<SPLPConstraint *> &) override {
        exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
    }
    virtual void solve() override { solve_internal(false); }
};

#endif /* STRUCTURAL_PATTERNS_SOLUTIONS_LP_BOUNDED_IFORKS_OFF_H */
