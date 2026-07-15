#include "LP_heuristic.h"

#include "abstractions/var_projection.h"
#include "abstractions/general_abstraction.h"
#include "abstractions/forks_abstraction.h"
#include "abstractions/iforks_abstraction.h"
#include "abstractions/semiforks_abstraction.h"
#include "mappings/var_proj_mapping.h"
#include "problem.h"
#include "SP_LPConstraint.h"
#include "SP_globals.h"
#include "solutions/LP_projection_gen.h"
#include "solutions/LP_binary_forks.h"
#include "solutions/LP_binary_forks_bounds.h"
#include "solutions/LP_bounded_ifork_gen.h"
#include "solutions/LP_binary_semifork_gen.h"

#include "../lp/lp_solver.h"
#include "../plugins/plugin.h"
#include "../utils/logging.h"
#include "../utils/system.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

LPHeuristic::LPHeuristic(
        const shared_ptr<AbstractTask> &transform,
        bool cache_estimates,
        const string &description,
        utils::Verbosity verbosity,
        DecompositionStrategy strat,
        SingletonStrategy sing,
        CostPartitioning cost_part,
        VariablesSelectionStrategy var_strat,
        int pattern_max_size,
        int ens_pct,
        int stats,
        bool cache,
        int sfbound,
        lp::LPSolverType lp_solver_type)
    : SPHeuristic(transform, cache_estimates, description, verbosity,
                  strat, sing, cost_part, var_strat,
                  pattern_max_size, ens_pct, stats, cache, sfbound),
      unsolved_states(0),
      lp_prob(new SP_OSI_solver(lp_solver_type)) {
}

LPHeuristic::~LPHeuristic() {
    delete lp_prob;
}

void LPHeuristic::print_statistics() const {
    cout << "Unsolved LP in " << unsolved_states << " states" << endl;
}

int LPHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    SPState sp = to_sp_state(state);

    if (original_problem->is_goal(sp))
        return 0;

    int res;
    if (get_cost_partitioning_strategy() == OPTIMAL) {
        res = compute_Optimal_heuristic(sp);
    } else if (get_cost_partitioning_strategy() == UNIFORM) {
        res = compute_Additive_heuristic(sp);
    } else {
        cout << "NO COST PARTITIONING!!!" << endl;
        exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
    }

    if (unsolved_states > 0) {
        cout << "Unsolved LP in " << unsolved_states << " states" << endl;
        exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
    }

    return res;
}

int LPHeuristic::compute_Optimal_heuristic(const SPState &state) {
    int n_cols = get_num_active_vars();
    vector<SPLPConstraint *> constr;
    vector<ConstraintVar *> obj_func;

    build_LP_objective(obj_func);
    int num_nonzeros = build_LP_constraints(constr, state);
    num_nonzeros += generate_general_cost_constraints(constr);

    int n_rows = (int)constr.size();
    lp_prob->set_size(n_cols, n_rows, num_nonzeros);
    lp_prob->initialize();

    Solution *sol = new Solution();
    double val = lp_prob->solve(obj_func, constr, sol);

    update_costs_from_solution(sol);

    lp_prob->free_mem();
    delete sol;

    for (SPLPConstraint *c : constr) {
        if (!c->tokeep()) {
            c->free_mem();
            delete c;
        }
    }
    for (ConstraintVar *cv : obj_func)
        delete cv;

    if (val >= LP_INFINITY)
        return DEAD_END;

    if (val < 0.0) {
        unsolved_states++;
        return 0;
    }

    if (val > 0.0)
        return (int)ceil(val - 0.01);

    return 0;
}

int LPHeuristic::compute_Additive_heuristic(const SPState &state) {
    double total_val = 0.0;
    const vector<DOperator *> &ops = original_problem->get_operators();

    vector<double> costs;
    calculate_representatives_cost(ops, costs);

    for (int ind = 0; ind < get_ensemble_size(); ind++) {
        SolutionMethod *membr = get_ensemble_member(ind);
        if (!membr->is_active())
            continue;

        int n_cols = membr->get_num_vars();
        vector<SPLPConstraint *> constr;
        vector<ConstraintVar *> obj_func;

        build_LP_objective(ind, obj_func);
        int num_nonzeros = build_LP_constraints(constr, ind, state);
        num_nonzeros += generate_cost_constraints(constr, ind, ops, costs);

        int n_rows = (int)constr.size();
        lp_prob->set_size(n_cols, n_rows, num_nonzeros);
        lp_prob->initialize();

        double val = lp_prob->solve(obj_func, constr, nullptr);

        lp_prob->free_mem();
        for (SPLPConstraint *c : constr) {
            if (!c->tokeep()) {
                c->free_mem();
                delete c;
            }
        }
        for (ConstraintVar *cv : obj_func)
            delete cv;

        if (val >= LP_INFINITY)
            return DEAD_END;

        total_val += val;
    }

    if (total_val > 0.0)
        return (int)ceil(total_val - 0.01);

    unsolved_states++;
    return 0;
}

void LPHeuristic::build_LP_objective(vector<ConstraintVar *> &obj_func) {
    for (int ind = 0; ind < get_ensemble_size(); ind++) {
        if (!get_ensemble_member(ind)->is_active())
            continue;
        build_LP_objective(ind, obj_func);
    }
}

void LPHeuristic::build_LP_objective(int ind, vector<ConstraintVar *> &obj_func) {
    SolutionMethod *membr = get_ensemble_member(ind);
    vector<ConstraintVar *> obj;
    membr->get_objective(obj);

    for (ConstraintVar *cv : obj) {
        obj_func.push_back(
            new ConstraintVar(membr->get_abstraction_index() + cv->var, cv->val));
    }
}

int LPHeuristic::build_LP_constraints(vector<SPLPConstraint *> &constr,
                                       const SPState &state) {
    int num_nonzeros = 0;
    for (int ind = 0; ind < get_ensemble_size(); ind++) {
        if (!get_ensemble_member(ind)->is_active())
            continue;
        num_nonzeros += build_LP_constraints(constr, ind, state);
    }
    return num_nonzeros;
}

int LPHeuristic::build_LP_constraints(vector<SPLPConstraint *> &constr,
                                       int ind, const SPState &state) {
    return get_ensemble_member(ind)->append_constraints(state, constr);
}

int LPHeuristic::generate_general_cost_constraints(vector<SPLPConstraint *> &constr) {
    int res = 0;
    const vector<DOperator *> &ops = original_problem->get_operators();
    int num_ops = (int)ops.size();

    for (int a_i = 0; a_i < num_ops; a_i++) {
        SPLPConstraint *lpc = new SPLPConstraint(0.0, ops[a_i]->get_double_cost(), false);

        int tmp_res = 0;
        for (int ind = 0; ind < get_ensemble_size(); ind++) {
            SolutionMethod *membr = get_ensemble_member(ind);
            if (!membr->is_active())
                continue;
            int index = membr->get_abstraction_index();
            Mapping *map = membr->get_mapping();
            vector<DOperator *> abs_ops;
            map->get_abs_operators(ops[a_i], abs_ops);
            int num_abs_ops = (int)abs_ops.size();
            for (int j = 0; j < num_abs_ops; j++) {
                int w_ind = index + membr->w_var(abs_ops[j]);
                lpc->add_val(w_ind, 1.0);
            }
            tmp_res += num_abs_ops;
        }

        if (tmp_res > 0) {
            lpc->finalize();
            constr.push_back(lpc);
        } else {
            delete lpc;
        }
        res += tmp_res;
    }
    return res;
}

int LPHeuristic::generate_uniform_cost_constraints(vector<SPLPConstraint *> &constr) {
    int res = 0;
    const vector<DOperator *> &ops = original_problem->get_operators();
    vector<double> costs;
    calculate_representatives_cost(ops, costs);

    for (int ind = 0; ind < get_ensemble_size(); ind++) {
        if (!get_ensemble_member(ind)->is_active())
            continue;
        res += generate_cost_constraints(constr, ind, ops, costs);
    }
    return res;
}

void LPHeuristic::calculate_representatives_cost(const vector<DOperator *> &ops,
                                                  vector<double> &costs) {
    int num_ops = (int)ops.size();
    for (int a_i = 0; a_i < num_ops; a_i++) {
        vector<DOperator *> abs_ops;
        for (int ind = 0; ind < get_ensemble_size(); ind++) {
            SolutionMethod *membr = get_ensemble_member(ind);
            if (!membr->is_active())
                continue;
            membr->get_mapping()->get_abs_operators(ops[a_i], abs_ops);
        }
        int num_actions = (int)abs_ops.size();
        if (num_actions > 0)
            costs.push_back(ops[a_i]->get_double_cost() / num_actions);
        else
            costs.push_back(LP_INFINITY);
    }
}

void LPHeuristic::update_costs_from_solution(Solution *sol) {
    for (int ind = 0; ind < get_ensemble_size(); ind++) {
        SolutionMethod *membr = get_ensemble_member(ind);
        if (!membr->is_active())
            continue;

        int index = membr->get_abstraction_index();
        Mapping *map = membr->get_mapping();
        const vector<DOperator *> &abs_ops = map->get_abstract()->get_operators();

        int num_abs_ops = (int)abs_ops.size();
        for (int j = 0; j < num_abs_ops; j++) {
            int w_ind = index + membr->w_var(abs_ops[j]);
            double c = sol->get_value(w_ind);
            if (c < 0.0000001)
                c = 0.0;
            abs_ops[j]->set_double_cost(c);
        }
    }
}

int LPHeuristic::generate_cost_constraints(vector<SPLPConstraint *> &constr, int ind,
                                            const vector<DOperator *> &ops,
                                            vector<double> &costs) {
    int res = 0;
    int num_ops = (int)ops.size();
    SolutionMethod *membr = get_ensemble_member(ind);
    int index = membr->get_abstraction_index();
    Mapping *map = membr->get_mapping();

    for (int a_i = 0; a_i < num_ops; a_i++) {
        vector<DOperator *> abs_ops;
        map->get_abs_operators(ops[a_i], abs_ops);
        int num_abs_ops = (int)abs_ops.size();
        for (int j = 0; j < num_abs_ops; j++) {
            int w_ind = membr->w_var(abs_ops[j]);
            SPLPConstraint *lpc = new SPLPConstraint(costs[a_i], costs[a_i], false);
            lpc->add_val(index + w_ind, 1.0);
            lpc->finalize();
            constr.push_back(lpc);
        }
        res += num_abs_ops;
    }
    return res;
}

// ---------------------------------------------------------------------------
// add_* overrides — instantiate LP variants of each solution method.
// ---------------------------------------------------------------------------
SolutionMethod *LPHeuristic::add_binary_fork(GeneralAbstraction *abs) {
    return new LPBinaryForks_b(abs);
}
SolutionMethod *LPHeuristic::add_bounded_inverted_fork(GeneralAbstraction *abs) {
    return new LPBoundedIfork(abs);
}
SolutionMethod *LPHeuristic::add_pattern(GeneralAbstraction *abs) {
    return new LPProjection(abs);
}
SolutionMethod *LPHeuristic::add_binary_semifork(GeneralAbstraction *abs) {
    return new LPBinarySemiFork(abs);
}
SolutionMethod *LPHeuristic::add_binary_fork(ForksAbstraction *fork, Domain *abs_domain) {
    return new LPBinaryForks_b(fork, abs_domain);
}
SolutionMethod *LPHeuristic::add_bounded_inverted_fork(IforksAbstraction *ifork, Domain *abs_domain) {
    return new LPBoundedIfork(ifork, abs_domain);
}
SolutionMethod *LPHeuristic::add_binary_semifork(SemiForksAbstraction *sfork, Domain *abs_domain) {
    return new LPBinarySemiFork(sfork, abs_domain);
}

SolutionMethod *LPHeuristic::add_pattern(vector<int> &pattern) {
    LPProjection *ptrn = new LPProjection(pattern);
    ptrn->create(original_problem);
    ptrn->set_abstraction_type(PATTERN);
    return ptrn;
}

// ---------------------------------------------------------------------------
// Plugin registration
// ---------------------------------------------------------------------------
void LPHeuristic::add_options_to_feature(plugins::Feature &feature) {
    SPHeuristic::add_options_to_feature(feature);
    lp::add_lp_solver_option_to_feature(feature);
}

class LPHeuristicFeature
    : public plugins::TypedFeature<Evaluator, LPHeuristic> {
public:
    LPHeuristicFeature() : TypedFeature("implicit_abstractions_lp") {
        document_title("Implicit Abstractions LP heuristic");
        document_synopsis(
            "Admissible heuristic based on acyclic causal-graph decompositions "
            "with LP-based cost partitioning.");
        LPHeuristic::add_options_to_feature(*this);
        document_language_support("action costs", "supported");
        document_language_support("conditional effects", "partially supported");
        document_language_support("axioms", "partially supported");
        document_property("admissible", "yes");
        document_property("consistent", "no");
        document_property("safe", "yes");
        document_property("preferred operators", "no");
    }

    virtual shared_ptr<LPHeuristic> create_component(
        const plugins::Options &opts) const override {
        return plugins::make_shared_from_arg_tuples<LPHeuristic>(
            get_heuristic_arguments_from_options(opts),
            SPHeuristic::get_sp_arguments_from_options(opts),
            lp::get_lp_solver_arguments_from_options(opts));
    }
};

static plugins::FeaturePlugin<LPHeuristicFeature> _lp_plugin;
