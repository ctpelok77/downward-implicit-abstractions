#include "SP_OSI_solver.h"

#include "../algorithms/named_vector.h"
#include "../utils/system.h"

#include <cassert>
#include <limits>
#include <vector>

using namespace std;

SP_OSI_solver::SP_OSI_solver(lp::LPSolverType solver_type)
    : lp_solver(solver_type), cols(0), rows(0) {
}

void SP_OSI_solver::set_size(int n_cols, int n_rows, int /*n_nonzeros*/) {
    cols = n_cols;
    rows = n_rows;
}

double SP_OSI_solver::solve(vector<ConstraintVar *> &obj_func,
                             vector<SPLPConstraint *> &constr,
                             Solution *sol) {
    assert(static_cast<int>(constr.size()) == rows);

    const double inf = numeric_limits<double>::infinity();

    // ------------------------------------------------------------------ //
    // Build LP variables: one per LP column.
    // All variables have lb=0, ub=inf; the objective coefficient is filled
    // from obj_func (sparse list of (var, coeff) pairs).
    // ------------------------------------------------------------------ //
    named_vector::NamedVector<lp::LPVariable> lp_vars;
    lp_vars.resize(cols, lp::LPVariable(0.0, inf, 0.0));

    for (ConstraintVar *cv : obj_func) {
        assert(cv->var >= 0 && cv->var < cols);
        lp_vars[cv->var].objective_coefficient = cv->val;
    }

    // ------------------------------------------------------------------ //
    // Build LP constraints from the SPLPConstraint list.
    // Each SPLPConstraint carries a sparse list of (local_var + index, coeff)
    // entries where index is the abstraction base offset stored in the
    // constraint itself.
    // ------------------------------------------------------------------ //
    named_vector::NamedVector<lp::LPConstraint> lp_constr;
    lp_constr.reserve(rows);

    for (int i = 0; i < rows; i++) {
        SPLPConstraint *sc = constr[i];
        double lb = sc->get_lb();
        double ub = sc->get_ub();
        // Map old ±DBL_MAX / ±infinity sentinels to solver infinity.
        if (lb <= -1e300) lb = -inf;
        if (ub >= 1e300)  ub = inf;

        lp_constr.emplace_back(lb, ub);
        lp::LPConstraint &row = lp_constr[i];

        const int base_index = sc->get_index();
        const vector<ConstraintVar *> &vals = sc->get_vals();
        for (ConstraintVar *cv : vals) {
            int col = cv->var + base_index;
            assert(col >= 0 && col < cols);
            row.insert(col, cv->val);
        }
    }

    // ------------------------------------------------------------------ //
    // Load, maximise, and extract solution.
    // ------------------------------------------------------------------ //
    lp::LinearProgram lp(
        lp::LPObjectiveSense::MAXIMIZE,
        std::move(lp_vars),
        std::move(lp_constr),
        inf);

    lp_solver.load_problem(lp);
    lp_solver.solve();

    if (!lp_solver.has_optimal_solution()) {
        if (lp_solver.is_infeasible())
            return inf;         // Infeasible → dead end (caller checks for >=LP_INFINITY)
        return -1.0;            // Unbounded or numerical failure
    }

    double val = lp_solver.get_objective_value();
    if (sol) {
        vector<double> primal = lp_solver.extract_solution();
        sol->set_solution(primal.data(), cols);
    }
    return val;
}
