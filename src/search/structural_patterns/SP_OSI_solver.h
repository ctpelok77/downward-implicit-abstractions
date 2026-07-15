#ifndef STRUCTURAL_PATTERNS_SP_OSI_SOLVER_H
#define STRUCTURAL_PATTERNS_SP_OSI_SOLVER_H

#include "SP_LPConstraint.h"
#include "solution.h"
#include "../lp/lp_solver.h"

#include <vector>

/*
 * SP_OSI_solver: replaces the old OSI/COIN solver wrapper.
 * Internally uses new FD's lp::LPSolver.
 *
 * Interface kept identical to the old class so that LPHeuristic needs
 * minimal changes.
 */
class SP_OSI_solver {
    lp::LPSolver lp_solver;
    int cols;   // number of LP variables
    int rows;   // number of constraint rows

public:
    explicit SP_OSI_solver(lp::LPSolverType solver_type);
    ~SP_OSI_solver() = default;

    // Must be called before solve() to set dimensions.
    void set_size(int n_cols, int n_rows, int /*n_nonzeros*/);

    // initialize() is a no-op in the new implementation (kept for API compat).
    void initialize() {}

    // Free transient memory after a solve. No-op in the new implementation.
    void free_mem() {}

    /*
     * Build and solve the LP described by obj_func / constr.
     * Returns the maximised objective value, or -1 on failure.
     * Fills *sol with the primal solution if sol != nullptr.
     */
    double solve(std::vector<ConstraintVar *> &obj_func,
                 std::vector<SPLPConstraint *> &constr,
                 Solution *sol);
};

#endif /* STRUCTURAL_PATTERNS_SP_OSI_SOLVER_H */
