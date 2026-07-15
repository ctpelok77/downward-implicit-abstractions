#ifndef OPERATOR_COUNTING_IMPLICIT_ABSTRACTIONS_CONSTRAINTS_H
#define OPERATOR_COUNTING_IMPLICIT_ABSTRACTIONS_CONSTRAINTS_H

#include "constraint_generator.h"
#include "../utils/logging.h"

#include <memory>
#include <string>
#include <vector>

class AbstractTask;
class SPHeuristic;
class SolutionMethod;

namespace operator_counting {
/*
  Operator-counting constraint generator based on implicit abstractions
  (acyclic causal-graph decompositions / structural patterns).

  One permanent constraint per active ensemble member:
      h_abs(s) <= sum_{o relevant to abs} Count_o

  The lower bound of each constraint is updated per state.
*/
class ImplicitAbstractionsConstraints : public ConstraintGenerator {
    std::shared_ptr<AbstractTask> task;
    SPHeuristic *heur;
    std::vector<SolutionMethod *> active_members;
    int constraint_offset;

public:
    explicit ImplicitAbstractionsConstraints(
        const std::shared_ptr<AbstractTask> &task,
        bool cache_estimates,
        const std::string &description,
        utils::Verbosity verbosity,
        int strategy,
        int singletons,
        int cost_partitioning,
        int var_strategy,
        int pattern_max_size,
        int ensemble_pct,
        int statistics,
        bool cache,
        int semifork_bound);

    ~ImplicitAbstractionsConstraints();

    virtual void initialize_constraints(
        const std::shared_ptr<AbstractTask> &task,
        lp::LinearProgram &lp) override;

    virtual bool update_constraints(
        const State &state, lp::LPSolver &lp_solver) override;
};
}

#endif /* OPERATOR_COUNTING_IMPLICIT_ABSTRACTIONS_CONSTRAINTS_H */
