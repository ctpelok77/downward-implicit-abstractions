#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_LP_PROJECTION_OFF_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_LP_PROJECTION_OFF_H

#include "../abstractions/var_projection.h"
#include "LP_projection_gen.h"
#include <cfloat>

/*
 * Offline LP projection.
 */

class LPProjection_OFF : public LPProjection {

public:
    LPProjection_OFF();
    LPProjection_OFF(GeneralAbstraction *abs);
    LPProjection_OFF(std::vector<int> &pattern);
    virtual ~LPProjection_OFF();

    virtual void set_objective() override;
    virtual void get_dynamic_constraints(const SPState &, std::vector<SPLPConstraint *> &) override {
        exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
    }

    virtual void add_distance_constraints(std::vector<int> free_vars,
                                          std::vector<int> state,
                                          const std::vector<int> &doms,
                                          DOperator *op) override;
    virtual void add_goal_constraints(std::vector<int> free_vars,
                                      std::vector<int> state,
                                      const std::vector<int> &doms) override;
};

#endif /* STRUCTURAL_PATTERNS_SOLUTIONS_LP_PROJECTION_OFF_H */
