#include "implicit_abstractions_constraints.h"

#include "../lp/lp_solver.h"
#include "../plugins/plugin.h"
#include "../structural_patterns/SP_heuristic.h"
#include "../structural_patterns/SP_globals.h"
#include "../structural_patterns/solutions/solution_method.h"
#include "../task_proxy.h"
#include "../utils/logging.h"

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

using namespace std;

namespace operator_counting {

ImplicitAbstractionsConstraints::ImplicitAbstractionsConstraints(
        const shared_ptr<AbstractTask> &task_,
        bool cache_estimates,
        const string &description,
        utils::Verbosity verbosity,
        int strategy,
        int singletons,
        int /*cost_partitioning*/,
        int var_strategy,
        int pattern_max_size,
        int ensemble_pct,
        int statistics,
        bool cache,
        int semifork_bound)
    : task(task_),
      constraint_offset(0) {
    // Build the SPHeuristic with POST_HOC_OPTIMIZATION cost partitioning.
    // The constraint generator owns the heuristic and its ensemble.
    heur = new SPHeuristic(
        task_,
        cache_estimates,
        description,
        verbosity,
        DecompositionStrategy(strategy),
        SingletonStrategy(singletons),
        CostPartitioning(POST_HOC_OPTIMIZATION),   // always POST_HOC for constraints
        VariablesSelectionStrategy(var_strategy),
        pattern_max_size,
        ensemble_pct,
        statistics,
        cache,
        semifork_bound);

    heur->initialize();

    for (int i = 0; i < heur->get_ensemble_size(); ++i) {
        SolutionMethod *member = heur->get_ensemble_member(i);
        if (member->is_active())
            active_members.push_back(member);
    }

    cout << "Generated " << active_members.size()
         << " implicit abstraction constraints" << endl;
}

ImplicitAbstractionsConstraints::~ImplicitAbstractionsConstraints() {
    delete heur;
}

void ImplicitAbstractionsConstraints::initialize_constraints(
        const shared_ptr<AbstractTask> &/*task_arg*/,
        lp::LinearProgram &lp) {
    // One permanent constraint per active abstraction member:
    //   h_abs(s)  <=  sum_{o relevant to abs} Count_o
    // Lower bound is 0 initially; updated per state in update_constraints.
    TaskProxy task_proxy(*task);
    named_vector::NamedVector<lp::LPConstraint> &constraints =
        lp.get_constraints();
    constraint_offset = constraints.size();

    for (SolutionMethod *member : active_members) {
        constraints.emplace_back(0.0, lp.get_infinity());
        lp::LPConstraint &constr = constraints.back();
        int num_ops = task_proxy.get_operators().size();
        for (int op_id = 0; op_id < num_ops; ++op_id) {
            if (member->is_relevant(op_id)) {
                double cost = task_proxy.get_operators()[op_id].get_cost();
                constr.insert(op_id, cost);
            }
        }
    }
}

bool ImplicitAbstractionsConstraints::update_constraints(
        const State &ancestor_state, lp::LPSolver &lp_solver) {
    // Convert new-FD State to SPState (vector<int>)
    int n = task->get_num_variables();
    SPState sp(n);
    for (int v = 0; v < n; ++v)
        sp[v] = ancestor_state[v].get_value();

    for (int i = 0; i < (int)active_members.size(); ++i) {
        int constraint_id = constraint_offset + i;
        SolutionMethod *member = active_members[i];
        double h = member->get_solution_value(sp);
        if (h == infinity) {
            return true;   // dead end
        }
        lp_solver.set_constraint_lower_bound(constraint_id, h);
    }
    return false;
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

class ImplicitAbstractionsConstraintsFeature
    : public plugins::TypedFeature<ConstraintGenerator,
                                   ImplicitAbstractionsConstraints> {
public:
    ImplicitAbstractionsConstraintsFeature()
        : TypedFeature("implicit_abstractions_constraints") {
        document_title("Implicit abstractions operator-counting constraints");
        document_synopsis(
            "Generates one operator-counting constraint per implicit abstraction "
            "(fork / inverted-fork / projection), enforcing h_abs(s) <= "
            "sum_{o relevant} Count_o. For details see Pommerening et al., IJCAI 2013 "
            "and Katz & Domshlak, JAIR 2010.");

        SPHeuristic::add_options_to_feature(*this);

        document_language_support("action costs", "supported");
        document_language_support("conditional effects", "partially supported");
        document_language_support("axioms", "partially supported");
        document_property("preferred operators", "no");
    }

    virtual shared_ptr<ImplicitAbstractionsConstraints> create_component(
        const plugins::Options &opts) const override {
        return plugins::make_shared_from_arg_tuples<
            ImplicitAbstractionsConstraints>(
            get_heuristic_arguments_from_options(opts),
            SPHeuristic::get_sp_arguments_from_options(opts));
    }
};

static plugins::FeaturePlugin<ImplicitAbstractionsConstraintsFeature> _plugin;

}  // namespace operator_counting
