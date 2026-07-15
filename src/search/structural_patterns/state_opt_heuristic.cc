#include "state_opt_heuristic.h"
#include "LP_heuristic.h"

#include "problem.h"
// globals removed — use TaskProxy

//#include "solutions/projection_gen.h"
//#include "solutions/binary_forks_gen.h"
//#include "solutions/bounded_iforks_gen.h"

#include <vector>

StateOptimalHeuristic::StateOptimalHeuristic(const Options &options, const State& state)
    : LPHeuristic(options), eval_state(state), solved(false) {
    CostPartitioning LP_cost_partitioning = CostPartitioning(options.get_enum("cost_partitioning"));
    exit_if_unsolved = false;
    if (LP_cost_partitioning == INITIAL_OPTIMAL) {
    	exit_if_unsolved = true;
    }
    LP_MAX_TIME_BOUND = options.get<int>("LP_timeout");
}

StateOptimalHeuristic::StateOptimalHeuristic(const Options &options, const State& state, const Problem* prob)
    : LPHeuristic(options, prob), eval_state(state), solved(false) {
    CostPartitioning LP_cost_partitioning = CostPartitioning(options.get_enum("cost_partitioning"));
	exit_if_unsolved = false;
    if (LP_cost_partitioning == INITIAL_OPTIMAL) {
    	exit_if_unsolved = true;
    }
    LP_MAX_TIME_BOUND = options.get<int>("LP_timeout");
}

StateOptimalHeuristic::~StateOptimalHeuristic() {
}

void StateOptimalHeuristic::initialize() {
	if (original_problem->is_goal(eval_state)) {
		cout << "Cannot create optimal partition for goal state!" << endl;
		return;
	}
	LPHeuristic::initialize();

	// Solving LP only if the number of LP variables is less than the bound set.
	cout << "------------->Start solving" << endl;
	int res =
		compute_Optimal_heuristic(eval_state);
	cout << "------------->End solving, solution value is " << res << endl;

	for (int ind=0;ind<get_ensemble_size();ind++) {
		// Freeing the allocated memory
		get_ensemble_member(ind)->free_constraints();
	}

	if (get_number_of_unsolved_states() > 0) { // If unsolved within the bound
		cout << "Unsolved LP for the given state:" << endl;
		eval_state.dump_fdr();
		if (exit_if_unsolved)
			exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
		return;
	}

	solved = true;

	for (int ind=0;ind<get_ensemble_size();ind++) {
		if (STATISTICS >= 1) {
			get_ensemble_member(ind)->get_mapping()->get_abstract()->dump();
		}
		//This part is a result of different number of d variables between online and offline versions
		if (get_ensemble_member(ind)->get_abstraction_type() == FORK || get_ensemble_member(ind)->get_abstraction_type() == SEMIFORK) {
			get_ensemble_member(ind)->set_d_vars_multiplier(2);
			get_ensemble_member(ind)->set_default_number_of_variables();
		}
	}

	check_cost_partition();
	solve_all_and_remove_operators();
}

