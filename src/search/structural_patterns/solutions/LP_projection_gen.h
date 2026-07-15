#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_LP_PROJECTION_GEN_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_LP_PROJECTION_GEN_H

#include "../abstractions/var_projection.h"
#include "projection_gen.h"
#include <cfloat>

/*
 * LP construction for pattern projections.
 * Ported from old FD; only change is State → SPState throughout.
 */

class LPProjection : public Projection {

public:
    LPProjection();
    LPProjection(GeneralAbstraction *abs);
    LPProjection(std::vector<int> &pattern);
    virtual ~LPProjection();

    virtual void initiate() override;
    virtual void set_static_constraints() override;
    virtual void get_dynamic_constraints(const SPState &state,
                                         std::vector<SPLPConstraint *> &dyn_constr) override;

    virtual void add_distance_constraints(std::vector<int> free_vars,
                                          std::vector<int> state,
                                          const std::vector<int> &doms,
                                          DOperator *op);
    virtual void add_goal_constraints(std::vector<int> free_vars,
                                      std::vector<int> state,
                                      const std::vector<int> &doms);

    virtual void remove_abstract_operators() override {}

    virtual int h_var() const override;
    virtual int w_var(DOperator *a) const override;
};

#endif /* STRUCTURAL_PATTERNS_SOLUTIONS_LP_PROJECTION_GEN_H */
