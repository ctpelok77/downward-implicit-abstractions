#include "SP_heuristic.h"
#include "LP_heuristic.h"
#include "SP_globals.h"
#include "abstractions/var_projection.h"
#include <vector>
#include "problem.h"
#include "abstractions/general_abstraction.h"
#include "mappings/var_proj_mapping.h"
#include <iostream>
#include "solutions/projection_gen.h"
#include "solutions/binary_forks_gen.h"
#include "solutions/bounded_iforks_gen.h"
#include "solutions/binary_semiforks_gen.h"
#include "solutions/binary_one_dep_hourglasses_gen.h"

#include "../plugins/plugin.h"
#include "../utils/logging.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <limits>

using namespace std;

// ---------------------------------------------------------------------------
// Helper: initialise all the strategy/option fields from the explicit args.
// ---------------------------------------------------------------------------
static void init_sp_fields(SPHeuristic * /*self*/,
                           int &SIZEOFPATTERNLIMIT,
                           int &PERCENTAGEOFENSEMBLE,
                           int &STATISTICS,
                           VariablesSelectionStrategy &selected_ensemble,
                           bool &use_caching,
                           DecompositionStrategy &strategy,
                           SingletonStrategy &singletons_strategy,
                           CostPartitioning &cost_partitioning,
                           int &semifork_hat_bound_size,
                           DecompositionStrategy strat,
                           SingletonStrategy sing,
                           CostPartitioning cost_part,
                           VariablesSelectionStrategy var_strat,
                           int pattern_max_size, int ens_pct,
                           int stats, bool cache, int sfbound) {
    strategy            = strat;
    singletons_strategy = sing;
    cost_partitioning   = cost_part;
    selected_ensemble   = var_strat;
    SIZEOFPATTERNLIMIT  = pattern_max_size;
    PERCENTAGEOFENSEMBLE= ens_pct;
    STATISTICS          = stats;
    use_caching         = cache;
    semifork_hat_bound_size = sfbound;
}

SPHeuristic::SPHeuristic(
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
        int sfbound)
    : Heuristic(transform, cache_estimates, description, verbosity) {
    external_problem = false;
    original_problem = new Problem(transform);
    init_sp_fields(this,
        SIZEOFPATTERNLIMIT, PERCENTAGEOFENSEMBLE, STATISTICS,
        selected_ensemble, use_caching, strategy, singletons_strategy,
        cost_partitioning, semifork_hat_bound_size,
        strat, sing, cost_part, var_strat,
        pattern_max_size, ens_pct, stats, cache, sfbound);
    initialize();
}

SPHeuristic::SPHeuristic(
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
        const Problem* prob)
    : Heuristic(transform, cache_estimates, description, verbosity),
      original_problem(prob) {
    external_problem = true;
    init_sp_fields(this,
        SIZEOFPATTERNLIMIT, PERCENTAGEOFENSEMBLE, STATISTICS,
        selected_ensemble, use_caching, strategy, singletons_strategy,
        cost_partitioning, semifork_hat_bound_size,
        strat, sing, cost_part, var_strat,
        pattern_max_size, ens_pct, stats, cache, sfbound);
    initialize();
}


SPHeuristic::~SPHeuristic() {

	for (int ind=0;ind<get_ensemble_size();ind++) {
		cout << "Deleting ensemble member " << ind << endl;
		delete get_ensemble_member(ind);
	}
	ensemble.clear();
	if (!external_problem)
		delete original_problem;
}

// Convert new-FD State to SPState (internal vector<int>)
SPState SPHeuristic::to_sp_state(const State &fdstate) const {
    int n = original_problem->get_vars_number();
    SPState sp(n);
    for (int v = 0; v < n; ++v)
        sp[v] = fdstate[v].get_value();
    return sp;
}

void SPHeuristic::get_ensemble_values(const State &ancestor_state, vector<double>& vals){
    State state = convert_ancestor_state(ancestor_state);
    SPState sp = to_sp_state(state);

    assert(vals.size() == 0);
    vals.assign(get_ensemble_size(), 0.0);

    if (original_problem->is_goal(sp)) {
        return;
    }

    for (int ind=0;ind<get_ensemble_size();ind++) {
        SolutionMethod* member = get_ensemble_member(ind);
        if (!member->is_active())
            continue;
        vals[ind] = get_ensemble_member(ind)->get_solution_value(sp);
    }
}


int SPHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    SPState sp = to_sp_state(state);

    if (original_problem->is_goal(sp)) {
        return 0;
    }
    double total = 0.0;
    for (int ind=0;ind<get_ensemble_size();ind++) {
        SolutionMethod* member = get_ensemble_member(ind);
        if (!member->is_active())
            continue;

        double sol = member->get_solution_value(sp);
        if (sol == infinity) {
            return DEAD_END;
        }
        total+=sol;
    }
    return (int)ceil(total-0.0001);
}


void SPHeuristic::create_ensemble() {

	// Creating the ensemble by the given strategy
	if(strategy == FORKS_ONLY) {
		if (selected_ensemble == MIXED_LANDMARK_FORKS_NON_LANDMARK_INVERTED_FORKS ||
				selected_ensemble == MIXED_NON_LANDMARK_FORKS_LANDMARK_INVERTED_FORKS) {
			cout << "Wrong variables strategy for forks only."<< endl;
			exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}
		create_binary_forks();
		return;
	}
	if(strategy == INVERTED_FORKS_ONLY) {
		if (selected_ensemble == MIXED_LANDMARK_FORKS_NON_LANDMARK_INVERTED_FORKS ||
				selected_ensemble == MIXED_NON_LANDMARK_FORKS_LANDMARK_INVERTED_FORKS) {
			cout << "Wrong variables strategy for inverted forks only."<< endl;
			exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}
		create_bounded_inverted_forks();
		return;
	}
	if(strategy == BOTH_FORKS_AND_INVERTED_FORKS) {
		create_binary_forks_and_bounded_iforks();
		return;
	}
	if(strategy == SEMIFORKS_ONLY) {
		if (selected_ensemble != ALL_VARIABLES) {
			cout << "Wrong variables strategy for semiforks only."<< endl;
			exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}
		create_binary_semiforks();
		return;
	}
	if(strategy == HOURGLASSES_ONLY) {
		if (selected_ensemble != ALL_VARIABLES) {
			cout << "Wrong variables strategy for hourglasses only."<< endl;
			exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}
		create_binary_hourglasses();
		return;
	}
	// Shouldn't happen
	cout << "NO STRATEGY SELECTED!!!" << endl;
	exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
}

void SPHeuristic::create_ensemble_from_file(istream &//in
		) {
	cout << "Ensemble creation from file option is currently disabled." << endl;
	exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
/*
	const vector<int> &var_dom = original_problem->get_variable_domains();
    int var_num = original_problem->get_vars_number();

    check_magic(in, "begin_strategy");
    int count;
    in >> count;
    if (count != var_num) {
    	cout << "The total number of variables does not match" << endl;
    	exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
    for(int v = 0; v < count; v++) {
		string name;
		in >> name;

		int var = original_problem->get_var_index(name);
		if (-1 == var) {
	    	cout << "The variable name " << name << " doesn't exist." << endl;
	    	exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	    }
		int var_strategy;  // 0 - nothing, 1 - fork only, 2 - inverted fork only, 3 - both
		in >> var_strategy;
		if (var_strategy < 0 || var_strategy > 3) {
			cout << "Unknown strategy for variable " << var << endl;
			exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}
		if (var_strategy == 1 || (var_strategy == 3 && original_problem->has_goal_child(var))) {
			//creating fork for variable i
			create_binary_fork(var,var_dom[var], var_strategy == 1);
		}
		if (var_strategy == 2 || var_strategy == 3) {
			//creating inverted fork for variable i
			create_bounded_inverted_fork(var,var_dom[var]);
		}
	}
    check_magic(in, "end_strategy");
*/
}

bool SPHeuristic::is_heuristic_applicable() const {
	return original_problem->is_nonconditional();
}


///////////////////////////////////////////////////////////////////////////////
// There are multiple possible strategies for creating an ensemble.
// We are focusing on three primary possibilities - (i) creating forks only,
// (ii) creating inverted forks only, and (iii) creating both forks and inverted forks.
/* Old version (before 26/5/2010)
void SPHeuristic::create_binary_forks()
{
	// Creating the ensemble consisting of forks only (binary root domains)
	int var_count = original_problem->get_vars_number();
	const vector<int> &var_dom = original_problem->get_variable_domains();

	for (int v = 0; v < var_count; v++) {
		create_binary_fork(v,var_dom[v], true);
	}
}


void SPHeuristic::create_bounded_inverted_forks() {

	// Creating the ensemble - inverted forks (or singletons) are created for each
	// goal variable.
	int var_count = original_problem->get_vars_number();
	const vector<int> &var_dom = original_problem->get_variable_domains();

	for (int v = 0; v < var_count; v++) {
		create_bounded_inverted_fork(v,var_dom[v]);
	}
}



void SPHeuristic::create_binary_forks_and_bounded_iforks() {

	// Creating the ensemble
	int var_count = original_problem->get_vars_number();
	const vector<int> &var_dom = original_problem->get_variable_domains();

	for (int v = 0; v < var_count; v++) {
		if (original_problem->has_goal_child(v)) {
			create_binary_fork(v,var_dom[v], false);
		}
		create_bounded_inverted_fork(v,var_dom[v]);
	}
}




///////////////////////////////////////////////////////////////
void SPHeuristic::create_binary_fork(int v, int dom_size, bool create_singletones) {
	// Each variable is either a singleton or appears in some fork.
	// By default we use binarization "Leave One Out"

	vector<int> successors = original_problem->get_causal_graph()->get_successors(v);
//	vector<int> predecessors = original_problem->get_causal_graph()->get_predecessors(v);

	if (original_problem->has_goal_child(v)) {
		// Create fork (or PDB)
		if (SIZEOFPATTERNLIMIT > successors.size()) {
			vector<int> pattern = successors;
			pattern.insert(pattern.begin(),v);
			create_pattern(pattern);
			return;
		}
		// Size exceeds the limit
		create_binary_forkLOO(v, dom_size);
	}
	// No fork to create. If this is goal variable, and it doesn't appear
	// on any other fork (meaning if it has no parents), then singleton is created
	if (!create_singletones)
		return;

	if ((original_problem->is_goal_var(v))
//			&& (dom_size > 2) // Only if performing the domain abstraction
//			&& (0 == predecessors.size())
											) {  // The variable is singleton
		create_singleton(v);
	}
}


void SPHeuristic::create_bounded_inverted_fork(int v, int dom_size) {
// We use a ternarization by distance to goal.

//	vector<int> successors = original_problem->get_causal_graph()->get_successors(v);
	vector<int> predecessors = original_problem->get_causal_graph()->get_predecessors(v);

	if (original_problem->is_goal_var(v)) {
		if (0 < predecessors.size()) {
			// Create inverted fork (or PDB)
			if (SIZEOFPATTERNLIMIT > predecessors.size()) {
				vector<int> pattern = predecessors;
				pattern.insert(pattern.begin(),v);
				create_pattern(pattern);
			} else {
				create_bounded_inverted_forkDGV(v, 3, dom_size);

//				if (dom_size > 3) { // Only if performing the domain abstraction
					create_singleton(v);
//				}
			}
		} else {
			// This is a goal variable with no parents. Singleton is now created
			create_singleton(v);
		}
	}

}

void SPHeuristic::create_singleton(int var) {
	vector<int> pattern;
	pattern.push_back(var);
	create_pattern(pattern);
}


*/


// New version (26/5/2010)
void SPHeuristic::create_binary_forks()
{
	// Creating the ensemble consisting of forks only (binary root domains)
	int var_count = original_problem->get_vars_number();

	const vector<int> &var_dom = original_problem->get_variable_domains();

	for (int v = 0; v < var_count; v++) {

		create_binary_fork(v,var_dom[v]);

		// Singletons
		vector<int> predecessors = original_problem->get_causal_graph()->get_predecessors(v);
		if (!original_problem->has_goal_child(v)) {
			if (0 < predecessors.size() && singletons_strategy == NECESSARY)
				continue;
			create_singleton(v);
		} else {
			if (0 == predecessors.size()) {
				// no pred, goal succ
				if (singletons_strategy == NECESSARY ||
						singletons_strategy == BY_DEFINITION)
					continue;
				if (var_dom[v] > 2)
					create_singleton(v);
			}
		}
	}
}


void SPHeuristic::create_bounded_inverted_forks() {

	// Creating the ensemble - inverted forks (or singletons) are created for each
	// goal variable.
	int var_count = original_problem->get_vars_number();

	const vector<int> &var_dom = original_problem->get_variable_domains();

	for (int v = 0; v < var_count; v++) {
		create_bounded_inverted_fork(v,var_dom[v]);
		// Singletons
		vector<int> predecessors = original_problem->get_causal_graph()->get_predecessors(v);

		if (!original_problem->has_goal_child(v)) {
			if (0 == predecessors.size()) {
				// no pred, no goal succ
				create_singleton(v);
			} else {
				// pred, no goal succ
				if (singletons_strategy == NECESSARY ||
						singletons_strategy == BY_DEFINITION)
					continue;
				if (var_dom[v] > 2)
					create_singleton(v);
			}
		} else {
			if (0 == predecessors.size()) {
				// no pred, goal succ
				if (singletons_strategy == NECESSARY)
					continue;
				create_singleton(v);
			}
		}
	}
}


void SPHeuristic::create_binary_forks_and_bounded_iforks() {

	// Creating the ensemble
	int var_count = original_problem->get_vars_number();

	const vector<int> &var_dom = original_problem->get_variable_domains();

	for (int v = 0; v < var_count; v++) {

		create_binary_fork(v,var_dom[v]);
		create_bounded_inverted_fork(v,var_dom[v]);

		// Singletons
		vector<int> predecessors = original_problem->get_causal_graph()->get_predecessors(v);

		if (!original_problem->has_goal_child(v)) {
			if (0 < predecessors.size() && singletons_strategy == NECESSARY)
				continue;
			create_singleton(v);
		} else {
			if (0 == predecessors.size()) {
				// no pred, goal succ
				if (singletons_strategy == NECESSARY)
					continue;
				create_singleton(v);
			}
		}

	}
}

void SPHeuristic::create_binary_semiforks() {
	original_problem->dump_CG_dot("cg.dot");
	// Creating the ensemble consisting of semiforks only (binary root domains)
	int var_count = original_problem->get_vars_number();
	const vector<int> &var_dom = original_problem->get_variable_domains();
	cout << "Creating an ensemble consisting of semiforks" << endl;
	for (int v = 0; v < var_count; v++) {
		// Getting the hats from the problem given the bound and initialize the semifork for each hat.
		vector<vector<int> > hats;
		original_problem->compute_connected_variables_bounded_sets(v, semifork_hat_bound_size, hats);
//		/*
		for (size_t i=0; i < hats.size(); ++i) {

			create_binary_semifork(v, hats[i], var_dom[v]);
		}
//		*/
		// Creating fork for the semiforks with no hat
		if (hats.size() == 0) {
			create_binary_fork(v, var_dom[v]);
		}
//		else {
//			create_binary_semifork(v, hats[hats.size() - 1], var_dom[v]);
//		}

		/* Singleton part is temporary disabled. Need to check if fits semiforks anyway

		// Singletons
		const vector<int>& predecessors = original_problem->get_causal_graph()->get_predecessors(v);
		if (!original_problem->has_goal_child(v)) {
			if (0 < predecessors.size() && singletons_strategy == NECESSARY)
				continue;
			create_singleton(v);
		} else {
			if (0 == predecessors.size()) {
				// no pred, goal succ
				if (singletons_strategy == NECESSARY ||
						singletons_strategy == BY_DEFINITION)
					continue;
				if (var_dom[v] > 2)
					create_singleton(v);
			}
		}
		*/
	}
}


void SPHeuristic::create_binary_hourglasses() {
	original_problem->dump_CG_dot("cg.dot");
	// Creating the ensemble consisting of hourglasses only (binary root domains)
	int var_count = original_problem->get_vars_number();
	const vector<int> &var_dom = original_problem->get_variable_domains();
	cout << "Creating an ensemble consisting of hourglasses" << endl;
	for (int v = 0; v < var_count; v++) {
		int dom_v = var_dom[v];
		const vector<int>& predecessors = original_problem->get_causal_graph()->get_predecessors(v);
		const vector<int>& successors = original_problem->get_causal_graph()->get_successors(v);
		set<int> parents;
		vector<int> leafs;
		parents.insert(predecessors.begin(), predecessors.end());
		for (size_t i=0; i < successors.size(); ++i) {
			if (original_problem->is_goal_var(successors[i])) {
				parents.erase(successors[i]);
				leafs.push_back(successors[i]);
			}
		}
		if (parents.size() == 0) {
			continue;
			cout << "Variable " << v << " has no parents, attempting at creating fork" << endl;
			create_binary_fork(v, dom_v);
			continue;
		}
		if (leafs.size() == 0) {
			continue;
			cout << "Variable " << v << " has no goal children, attempting at creating inverted fork" << endl;
			create_bounded_inverted_fork(v, dom_v);
			continue;
		}
		vector<int> vec_par;
		vec_par.insert(vec_par.begin(), parents.begin(), parents.end());
		create_binary_hourglass(v, vec_par, dom_v);

		/* Singleton part is temporary disabled. Need to check if fits semiforks anyway

		// Singletons
		const vector<int>& predecessors = original_problem->get_causal_graph()->get_predecessors(v);
		if (!original_problem->has_goal_child(v)) {
			if (0 < predecessors.size() && singletons_strategy == NECESSARY)
				continue;
			create_singleton(v);
		} else {
			if (0 == predecessors.size()) {
				// no pred, goal succ
				if (singletons_strategy == NECESSARY ||
						singletons_strategy == BY_DEFINITION)
					continue;
				if (var_dom[v] > 2)
					create_singleton(v);
			}
		}
		*/
	}
}

///////////////////////////////////////////////////////////////
void SPHeuristic::create_binary_fork(int v, int dom_size) {
	// By default we use binarization "Leave One Out"
	if (!original_problem->has_goal_child(v))
		return;

	const vector<int>& successors = original_problem->get_causal_graph()->get_successors(v);

	// Create fork (or PDB)
	if (SIZEOFPATTERNLIMIT > (int)successors.size()) {
		vector<int> pattern = successors;
		pattern.insert(pattern.begin(),v);
		create_pattern(pattern);
		return;
	}
	// Size exceeds the limit
	create_binary_forkLOO(v, dom_size);
}


void SPHeuristic::create_bounded_inverted_fork(int v, int dom_size) {
// We use a ternarization by distance to goal.
	if (!original_problem->is_goal_var(v))
		return;

	const vector<int>& predecessors = original_problem->get_causal_graph()->get_predecessors(v);

	if (0 == predecessors.size())
		return;

	// Create inverted fork (or PDB)
	if (SIZEOFPATTERNLIMIT > (int)predecessors.size()) {
		vector<int> pattern = predecessors;
		pattern.insert(pattern.begin(),v);
		create_pattern(pattern);
	} else {
		create_bounded_inverted_forkDGV(v, IFORKDOMBOUND, dom_size);
		return;
//		int var_num = original_problem->get_var_actions(v).size();
		// Bounding by |A_v|^{IFORKDOMBOUND - 1}
//		create_bounded_inverted_fork_check_paths(v, IFORKDOMBOUND, pow(var_num, IFORKDOMBOUND-1), dom_size);
		create_bounded_inverted_fork_check_paths(v, IFORKDOMBOUND, MAX_NUM_PATHS, dom_size);
	}
}

void SPHeuristic::create_singleton(int var) {
	if (!original_problem->is_goal_var(var))
		return;

	vector<int> pattern;
	pattern.push_back(var);
	create_pattern(pattern);
}

void SPHeuristic::create_binary_semifork(int v, const vector<int>& hat, int dom_size) {
	// By default we use binarization "Leave One Out"
	const vector<int>& successors = original_problem->get_causal_graph()->get_successors(v);

	set<int> total_vars;
	for (size_t i=0; i < successors.size(); ++i) {
		if (original_problem->is_goal_var(successors[i]))
			total_vars.insert(successors[i]);
	}
	total_vars.insert(hat.begin(),hat.end());

	// Create semifork (or PDB)
//	if (SIZEOFPATTERNLIMIT > total_vars.size() || total_vars.size() == hat.size()) {
	if (SIZEOFPATTERNLIMIT > (int)total_vars.size()) {
			cout << "Under the bound - creating pattern" << endl;

		vector<int> pattern;
		pattern.push_back(v);
		pattern.insert(pattern.begin()+1,total_vars.begin(),total_vars.end());
		create_pattern(pattern);
		return;
	}
	if (total_vars.size() == hat.size()) {
//		cout << "No goal children - exiting" << endl;
		return;
	}

	// Size exceeds the limit
	create_binary_semiforkLOO(v, hat, dom_size);
}

void SPHeuristic::create_binary_hourglass(int v, const vector<int>& parents, int dom_size) {
	// By default we use binarization "Leave One Out"
	const vector<int>& successors = original_problem->get_causal_graph()->get_successors(v);

	set<int> total_vars;
	for (size_t i=0; i < successors.size(); ++i) {
		if (original_problem->is_goal_var(successors[i]))
			total_vars.insert(successors[i]);
	}
	total_vars.insert(parents.begin(),parents.end());

	// Create hourglass (or PDB)
//	if (SIZEOFPATTERNLIMIT > total_vars.size() || total_vars.size() == hat.size()) {
	if (SIZEOFPATTERNLIMIT > (int)total_vars.size()) {
			cout << "Under the bound - creating pattern" << endl;

		vector<int> pattern;
		pattern.push_back(v);
		pattern.insert(pattern.begin()+1,total_vars.begin(),total_vars.end());
		create_pattern(pattern);
		return;
	}
	if (total_vars.size() == parents.size()) {
		cout << "No goal children - exiting" << endl;
		return;
	}

	// Size exceeds the limit
	create_binary_hourglassLOO(v, parents, dom_size);
}
///////////////////////////////////////////////////////////////////////////////////
void SPHeuristic::create_binary_forkLOO(int v, int dom_size) {
	cout << "Fork for variable " << v;// << endl;

	// Creating Fork first
	ForksAbstraction* f = new ForksAbstraction(v);
	f->create(original_problem);

	cout << " over " << f->get_mapping()->get_abstract()->get_vars_number() << " variables." << endl;

	if (dom_size == 2) {
		// Setting the domain mapping to identity.
		Domain* abs_domain = create_LOO_domain(v,0,dom_size);
		// Creating the abstraction
		SolutionMethod* member = add_binary_fork(f, abs_domain);
		add_ensemble_member(member);
//		cout << "Adding member " << member << endl;
		return;
	}
	// If the original domain is not binary.
	for (int val = 0; val < dom_size; val++) {
		Domain* abs_domain = create_LOO_domain(v,val,dom_size);
		// Creating the abstraction
//		add_ensemble_member(add_binary_fork(f, abs_domain));
		SolutionMethod* member = add_binary_fork(f, abs_domain);
		add_ensemble_member(member);
//		cout << "Adding member " << member << endl;
	}
}


void SPHeuristic::create_bounded_inverted_forkDGV(int v, int bound, int dom_size) {
	cout << "Inverted Fork for variable " << v;// << endl;


	// Creating Inverted Fork first
	IforksAbstraction* ifork = new IforksAbstraction(v);
	ifork->create(original_problem);

	cout << " over " << ifork->get_mapping()->get_abstract()->get_vars_number() << " variables." << endl;
	cout << "Var: " << v << ", domain: " << dom_size << ", bound: " << bound << endl;

	if (dom_size <= bound) {
		// Setting the domain mapping to identity.
		Domain* abs_domain = create_id_domain(v,dom_size);
		// Creating the abstraction
		add_ensemble_member(add_bounded_inverted_fork(ifork, abs_domain));
		return;
	}
	// If the original domain is bigger than a given bound.
	vector<vector<int> > dom_vals;
	vector<int> distances;
	original_problem->get_domain_values_by_distance_to_goal(v,dom_vals,distances);

//	int has_dead_end = 0;
	int lb = 0, ub = 0;
	for (size_t i=0; i < distances.size(); ++i) {
//		if(-1 == distances[i])  {
//			has_dead_end = 1;
//			continue;
//		}
		if(ub < distances[i])  {
			ub = distances[i];
		}
	}
//	ub+= has_dead_end; // Adding one for dead end values

	while (ub > lb) {
		// Setting the domain mapping to distance from initial value.
		int upper = min(lb+bound-1,ub);
		cout << "Bounds [" <<lb << "," << upper << "]" << endl;
		Domain* abs_domain = create_DGV_domain(v,distances,lb,upper);

		lb += bound-1;
		// Creating the abstraction
		add_ensemble_member(add_bounded_inverted_fork(ifork, abs_domain));
	}
}

void SPHeuristic::create_inverted_fork_all_paths(int v, int dom_size) {
	cout << "Inverted Fork for variable " << v; // << " with no bounds" << endl;

	// Creating Inverted Fork first
	IforksAbstraction* ifork = new IforksAbstraction(v);
	ifork->create(original_problem);

	Problem* abs = ifork->get_mapping()->get_abstract();

	cout << " over " << abs->get_vars_number() << " variables, with no bounds." << endl;

	cout << "Total number of paths "
		 << abs->get_estimated_number_of_all_cycle_free_paths_to_goal(0)
		 << endl;
	// Setting the domain mapping to identity.
	Domain* abs_domain = create_id_domain(v,dom_size);
	// Creating the abstraction
	add_ensemble_member(add_bounded_inverted_fork(ifork, abs_domain));
}



void SPHeuristic::create_bounded_inverted_fork_check_paths(int v, int bound, int paths_bound, int dom_size) {
	cout << "Inverted Fork for variable " << v;// << endl;

	// Creating Inverted Fork first
	IforksAbstraction* ifork = new IforksAbstraction(v);
	ifork->create(original_problem);

	Problem* abs = ifork->get_mapping()->get_abstract();

	cout << " over " << abs->get_vars_number() << " variables." << endl;

	cout << "Var: " << v << ", domain: " << dom_size << ", bound: " << bound;


	if (dom_size <= bound) {
		cout << endl;
		// Setting the domain mapping to identity.
		Domain* abs_domain = create_id_domain(v,dom_size);
		// Creating the abstraction
		add_ensemble_member(add_bounded_inverted_fork(ifork, abs_domain));
		return;
	}

	if (abs->is_estimated_number_of_all_cycle_free_paths_to_goal_bounded(0,paths_bound)) {
		cout << ", paths: under " << paths_bound << endl;
		// Setting the domain mapping to identity.
		Domain* abs_domain = create_id_domain(v,dom_size);
		// Creating the abstraction
		add_ensemble_member(add_bounded_inverted_fork(ifork, abs_domain));
		return;
	}
	cout << endl << "Too many paths, domain decomposition is performed" << endl;

	// If the original domain is bigger than a given bound.
	vector<vector<int> > dom_vals;
	vector<int> distances;
	original_problem->get_domain_values_by_distance_to_goal(v,dom_vals,distances);

	int lb = 0, ub = 0;
	for (size_t i=0; i < distances.size(); ++i) {
		if(ub < distances[i])  {
			ub = distances[i];
		}
	}

	while (ub > lb) {
		// Setting the domain mapping to distance from initial value.
		int upper = min(lb+bound-1,ub);
		cout << "Bounds [" <<lb << "," << upper << "]" << endl;
		Domain* abs_domain = create_DGV_domain(v,distances,lb,upper);

		lb += bound-1;
		// Creating the abstraction
		add_ensemble_member(add_bounded_inverted_fork(ifork, abs_domain));
	}
}


void SPHeuristic::create_pattern(vector<int>& pattern) {
	// Creating the abstraction
	cout << "Pattern for variables ";
	for (size_t i=0; i < pattern.size(); ++i)
		cout <<"  " << pattern[i];
	cout << endl;
	add_ensemble_member(add_pattern(pattern));
}

void SPHeuristic::create_binary_semiforkLOO(int v, const vector<int>& hat, int dom_size) {
//	cout << "SemiFork for variable " << v << endl;
	cout << "SemiFork with the center " << v << " and hat consisting of";
	for (size_t j=0; j < hat.size(); ++j) {
		cout << " " << hat[j] ;
	}
//	cout << endl;

	// Creating SemiFork first
	SemiForksAbstraction* sf = new SemiForksAbstraction(v,hat);
	sf->create(original_problem);

	cout << " over the total of " << sf->get_mapping()->get_abstract()->get_vars_number() << " variables." << endl;

	if (dom_size == 2) {
		// Setting the domain mapping to identity.
		Domain* abs_domain = create_LOO_domain(v,0,dom_size);
		// Creating the abstraction
		SolutionMethod* member = add_binary_semifork(sf, abs_domain);
		add_ensemble_member(member);
//		cout << "Adding member " << member << endl;
		return;
	}
	// If the original domain is not binary.
	for (int val = 0; val < dom_size; val++) {
		Domain* abs_domain = create_LOO_domain(v,val,dom_size);
		// Creating the abstraction
		SolutionMethod* member = add_binary_semifork(sf, abs_domain);
		add_ensemble_member(member);
//		cout << "Adding member " << member << endl;
	}
}


void SPHeuristic::create_binary_hourglassLOO(int v, const vector<int>& parents, int dom_size) {
//	cout << "Hourglass for variable " << v << endl;
	cout << "Hourglass with the center " << v  << " and parents";
	for (size_t j=0; j < parents.size(); ++j) {
		cout << " " << parents[j] ;
	}

	// Creating Hourglass first
	HourglassesAbstraction* hg = new HourglassesAbstraction(v, parents);
	hg->create(original_problem);

	cout << " over the total of " << hg->get_mapping()->get_abstract()->get_vars_number() << " variables." << endl;

//	cout << "-----------------------------------------------------------------------------" << endl << "Abstract hourglass:" << endl;
//	hg->get_mapping()->get_abstract()->dump();

	// Now, making it one dependent. No need to create any more.
//	cout << "Now, making it one dependent" << endl;
	OneDependentHourglassesAbstraction* ohg = new OneDependentHourglassesAbstraction(hg);
//	cout << "Done making it one dependent" << endl;

	if (dom_size == 2) {
		// Setting the domain mapping to identity.
		Domain* abs_domain = create_LOO_domain(v,0,dom_size);
		// Creating the abstraction
		SolutionMethod* member;
		// If there is only one parents, it is solved with the semiforks solution
		if (parents.size() == 1)
			member = add_binary_semifork(ohg, abs_domain);
		else
			member = add_binary_hourglass(ohg, abs_domain);
		add_ensemble_member(member);
//		cout << "Adding member " << member << endl;
		return;
	}
	// If the original domain is not binary.
	for (int val = 0; val < dom_size; val++) {
		Domain* abs_domain = create_LOO_domain(v,val,dom_size);
		// Creating the abstraction
		SolutionMethod* member;
		// If there is only one parents, it is solved with the semiforks solution
		if (parents.size() == 1)
			member = add_binary_semifork(ohg, abs_domain);
		else
			member = add_binary_hourglass(ohg, abs_domain);
		add_ensemble_member(member);
//		cout << "Adding member " << member << endl;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void SPHeuristic::apply_uniform_representatives_cost() {

	// Setting the cost of the representatives in each pattern
	// to be uniformly partitioned between ALL the representatives.
	const vector<DOperator*> &ops = original_problem->get_operators();

	int num_ops = ops.size();
	for (int a_i = 0; a_i < num_ops; a_i++) {

		// Counting the number of actions
		vector<DOperator*> abs_ops;
		for (int ind = 0; ind < get_ensemble_size(); ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			if (!member->is_active())
				continue;

			member->get_mapping()->get_abs_operators(ops[a_i],abs_ops);
		}
		int num_actions = abs_ops.size();
		if (num_actions > 0) {
			double abs_cost = ops[a_i]->get_double_cost() / num_actions;
			// Update costs
			for (int i=0;i<num_actions;i++) {
				abs_ops[i]->set_double_cost(abs_cost);
			}
		}
	}
}

void SPHeuristic::apply_uniform_ensemble_representatives_cost() {

	// Setting the cost of the representatives in each pattern
	// to be uniformly partitioned between ensembles, and then between ALL the representatives.
	const vector<DOperator*> &ops = original_problem->get_operators();

	int num_ops = ops.size();
	int num_ensemble_members = get_num_active_members();
	for (int a_i = 0; a_i < num_ops; a_i++) {
		for (int ind = 0; ind < get_ensemble_size(); ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			if (!member->is_active())
				continue;

			vector<DOperator*> abs_ops;
			member->get_mapping()->get_abs_operators(ops[a_i],abs_ops);
			int num_abs_ops = abs_ops.size();
			double abs_cost = ops[a_i]->get_double_cost() / (num_ensemble_members * num_abs_ops);

			for (int i=0;i<num_abs_ops;i++) {
				abs_ops[i]->set_double_cost(abs_cost);
			}

		}
	}
}

void SPHeuristic::apply_weighted_uniform_representatives_cost() {

	// Setting the cost of the representatives in each pattern
	// to be partitioned between ALL the representatives according to the predefined weights
	const vector<DOperator*> &ops = original_problem->get_operators();

	int num_ops = ops.size();
	for (int a_i = 0; a_i < num_ops; a_i++) {
		int num_abs_ops = 0;
		double total_weight = 0.0;
		// Counting the number of actions
		vector<vector<DOperator*> > abs_ops;
		int ensemble_size = get_ensemble_size();
		for (int ind = 0; ind < ensemble_size; ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			vector<DOperator*> tmp_ops;
			if (member->is_active()) {
				member->get_mapping()->get_abs_operators(ops[a_i],tmp_ops);
				num_abs_ops += tmp_ops.size();
				if (tmp_ops.size() > 0) {
					total_weight += pattern_weights[ind];
//					cout << ind << ":" << tmp_ops.size() << ":" << pattern_weights[ind] << "  ";
				}
			}
			abs_ops.push_back(tmp_ops);
		}

//		cout << endl << "Num abs ops: " << num_abs_ops << endl;
		if (num_abs_ops == 0)
			continue;

		double abs_cost = ops[a_i]->get_double_cost() / total_weight;
		cout << "===============================================" << endl;
		ops[a_i]->dump();
		cout << "-----------------------------------------------" << endl;
//		cout<< "Abs cost: " << abs_cost << endl;
		for (int ind = 0; ind < ensemble_size; ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			if (!member->is_active())
				continue;

			if (abs_ops[ind].size() == 0)
				continue;

			// Going over ensemble members, setting actions cost
			double weighted_abs_cost = (abs_cost * pattern_weights[ind])/abs_ops[ind].size();
//			cout<< "Weighted abs cost: " << weighted_abs_cost << endl;
			for (size_t i=0; i < abs_ops[ind].size(); ++i) {
				abs_ops[ind][i]->set_double_cost(weighted_abs_cost);
				abs_ops[ind][i]->dump();
			}
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setting costs in each abstraction to the cost of the original action - partitioning inside between the representatives uniformly

void SPHeuristic::apply_full_representatives_cost() {

	const vector<DOperator*> &ops = original_problem->get_operators();

	int num_ops = ops.size();
	for (int a_i = 0; a_i < num_ops; a_i++) {

		for (int ind = 0; ind < get_ensemble_size(); ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			if (!member->is_active())
				continue;

			vector<DOperator*> abs_ops;
			member->get_mapping()->get_abs_operators(ops[a_i],abs_ops);
			int num_representatives = abs_ops.size();
			if (num_representatives > 0) {
				double abs_cost = ops[a_i]->get_double_cost() / num_representatives;
				// Update costs
				for (int i=0;i<num_representatives;i++) {
					abs_ops[i]->set_double_cost(abs_cost);
				}
			}
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void SPHeuristic::print_abstract_operators() {

	const vector<DOperator*> &ops = original_problem->get_operators();

	int num_ops = ops.size();
	cout << "Printing operators and their abstract representatives" << endl;
	for (int a_i = 0; a_i < num_ops; a_i++) {
		ops[a_i]->dump();
		for (int ind = 0; ind < get_ensemble_size(); ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			if (!member->is_active())
				continue;

			vector<DOperator*> abs_ops;
			member->get_mapping()->get_abs_operators(ops[a_i],abs_ops);

			int num_actions = abs_ops.size();
			for (int i=0;i<num_actions;i++) {
				cout << member->get_abstraction_index() <<
				", " << member->w_var(abs_ops[i]) << " (" <<
				member->get_abstraction_index() + member->w_var(abs_ops[i])
				        << "): ";
				abs_ops[i]->dump();
			}
		}
	}
}




Domain* SPHeuristic::create_LOO_domain(int var, int to_leave, int dom_size) const {

	Domain* abs_domain = new Domain(var,dom_size,2);
	for (int k =0; k < dom_size; k++) {
		if (to_leave == k) {
			abs_domain->set_value(k,0);
		} else {
			abs_domain->set_value(k,1);
		}
	}
	return abs_domain;
}



Domain* SPHeuristic::create_DGV_domain(int var, vector<int>& distances, int lb, int ub) const {

	int dom_size = distances.size();
	int new_size = ub - lb + 1;
	Domain* abs_domain = new Domain(var,dom_size,new_size);
	for (size_t k=0; k < distances.size(); ++k) {
		// TODO: in general, distance < 0 should be removed from the problem
		if ((distances[k] > ub) || (distances[k] < 0)) {
			abs_domain->set_value(k,ub - lb);
			continue;
		}
		if (distances[k] < lb) {
			abs_domain->set_value(k,0);
			continue;
		}
		abs_domain->set_value(k,distances[k] - lb);
	}
//	abs_domain->dump();
	return abs_domain;
}


Domain* SPHeuristic::create_id_domain(int var, int dom_size) const {

	Domain* abs_domain = new Domain(var,dom_size,dom_size);
	for (int k =0; k < dom_size; k++) {
		abs_domain->set_value(k,k);
	}

	return abs_domain;
}


void SPHeuristic::initialize() {
	if (STATISTICS >= 1) {
		original_problem->dump();
	}
	if (0 == get_ensemble_size()) {
//        cout << "Creating ensemble " << std::endl;

		create_ensemble();
//		cout << "Done Creating ensemble " << std::endl;
		// print statistics
		cout << "SAS variables :: " << original_problem->get_vars_number() << endl;
		cout << "Ensemble size :: " << get_ensemble_size() << endl;
	}
	if (0 == get_ensemble_size()) {
		cout << "Ensemble is empty!!! Check creation options." << endl;
		exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}
	// Activating by a given percentage
	deactivate_all();

    //srand(time(0));  // Initialize random number generator.
    // Selecting patterns from ensemble to participate in computing heuristic
    int counter = 0;
    while (counter == 0) {
    	for (int it = 0; it < get_ensemble_size(); it++) {
    		SolutionMethod* member = get_ensemble_member(it);
   			int pstg = (rand() % 100) + 1;
   			if (pstg <= PERCENTAGEOFENSEMBLE) {
//   				member->set_abstraction_index(counter);
   				member->activate();
   				counter++;
//   				cout << "Counter: " << counter << endl;
    		}
    	}
    }

	if (get_cost_partitioning_strategy() == UNIFORM) {
		// Setting the cost partitioning
		apply_uniform_representatives_cost();
		//	apply_smart_representatives_cost();
		//	apply_smart_uniform_representatives_cost();
//		cout << "Done setting UNIFORM cost partitioning" << endl;
	} else if (get_cost_partitioning_strategy() == ENSEMBLE_UNIFORM) {
		// Setting the cost partitioning
		cout << "Setting UNIFORM among ensemble members cost partitioning" << endl;
		apply_uniform_ensemble_representatives_cost();
	} else if (get_cost_partitioning_strategy() == GREEDY_SAMPLE) {
		// Setting the cost partitioning
		cout << "Setting cost partitioning according to greedy sample" << endl;
		apply_weighted_uniform_representatives_cost();
		cout << "Done setting cost partitioning according to greedy sample" << endl;
		check_cost_partition();
	} else if (get_cost_partitioning_strategy() == POST_HOC_OPTIMIZATION) {
		// In this case every abstraction gets the full cost
		apply_full_representatives_cost();
	}

	if (use_caching) {
		cout << "Using caching for abstract states" << endl;
    	for (int it = 0; it < get_ensemble_size(); it++) {
    		SolutionMethod* member = get_ensemble_member(it);
    		member->use_caching();
    	}
	}

	int ens_size = ensemble.size();
	int disk_size = 0;
	int tot_vars = 0;
	int ens_types[MAX_ABSTRACTION_TYPE+1] = {};
	int abs_ind_counter = 0;
	for (int ind=0;ind<ens_size;ind++) {
		SolutionMethod* member = get_ensemble_member(ind);
		if (!member->is_active()) {
			member->remove_abstract_operators();  // Implemented only in the offline version
			continue;
		}
		if (STATISTICS >= 1) {
			cout << "Ensemble member "<< ind << endl;
			member->get_mapping()->get_abstract()->dump();
		}
		if (STATISTICS >= 2) {
			cout << "Initiating ensemble member " << ind << " " << std::endl;
		}
		member->initiate();
		if (STATISTICS >= 2) {
			cout << "Done initiating ensemble member " << ind << " " << std::endl;
		}
		if ((get_cost_partitioning_strategy() == INITIAL_OPTIMAL) ||
				(get_cost_partitioning_strategy() == OPTIMAL)) {
		#ifdef USE_LP
			member->set_objective();
			member->set_static_constraints();
		#else
				cout << "No LP Solver defined in this version" << endl;
				exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		#endif
		} else {
			if (STATISTICS >= 2) {
				cout << "Solving ensemble member " << ind << " " << std::endl;
			}
			member->solve();
			if (STATISTICS >= 2) {
				cout << "Done Solving, removing abstract operators " << std::endl;
			}
			// The solution currently exists, no need for the abstract actions.
			member->remove_abstract_operators();  // Implemented only in the offline version
			if (STATISTICS >= 2) {
				cout << "Done removing abstract operators " << std::endl;
			}
		}
		tot_vars += member->get_num_vars();
		disk_size += member->get_solution()->get_size();
		ens_types[member->get_abstraction_type()]++;

		// Setting abstraction index for LP
		member->set_abstraction_index(abs_ind_counter);
		abs_ind_counter += member->get_num_vars();

	}

	cout << "*Forks* in Ensemble :: " << ens_types[FORK] << endl;
	cout << "*Inverted Forks* in Ensemble :: " << ens_types[INVERTED_FORK] << endl;
	cout << "*Projections* in Ensemble :: " << ens_types[PATTERN] << endl;
	cout << "*Semiforks* in Ensemble :: " << ens_types[SEMIFORK] << endl;
	cout << "*Hourglasses* in Ensemble :: " << ens_types[HOURGLASS] << endl;

	/*
	// Activating by a given percentage
	deactivate_all();

    //srand(time(0));  // Initialize random number generator.
    // Selecting patterns from ensemble to participate in computing heuristic
    int counter = 0;
    while (counter == 0) {
    	for (int it = 0; it < get_ensemble_size(); it++) {
    		SolutionMethod* member = get_ensemble_member(it);
   			int pstg = (rand() % 100) + 1;
   			if (pstg <= PERCENTAGEOFENSEMBLE) {
   				member->set_abstraction_index(counter);
   				member->activate();
   				counter += member->get_num_vars();
    		}
    	}
    }
    */
	if (STATISTICS >= 1) {
		cout << "The size of the SPDB (number of entries) is " << disk_size << " out of " << tot_vars << endl;
	}
	cout << "DONE INITIALIZING!!!!!!!!" << endl;
}


//////////////////////////////////////////////////////////////////////////////////
void SPHeuristic::set_smart_representatives_cost() {
	//TODO: Adapt to conditional effects before using

	// The costs are uniformly partitioned between the patterns, inside each pattern first the whole cost
	// is given to the root, then the minimum is found and the rest is partitioned uniformly between others.
	const vector<DOperator*> &ops = original_problem->get_operators();

	int num_ops = ops.size();
	vector<double> pattern_costs;
	pattern_costs.assign(num_ops,0.0);
	for (int a_i = 0; a_i < num_ops; a_i++) {
		// Counting the number of actions
		vector<DOperator*> abs_ops;
		int num_ptrns=0;
		for (int ind = 0; ind < get_ensemble_size(); ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			if (!member->is_active())
				continue;

			int num_rep = member->get_mapping()->get_abs_operators(ops[a_i],abs_ops);
			if (num_rep > 0)
				num_ptrns++;
		}
		if (num_ptrns > 0) {
			double abs_cost = ops[a_i]->get_double_cost() / num_ptrns;
			int act_ind = ops[a_i]->get_index();
			assert(act_ind<num_ops);
			pattern_costs[act_ind] = abs_cost;
		}
	}

	for (int ind = 0; ind < get_ensemble_size(); ind++){
		SolutionMethod* member = get_ensemble_member(ind);
		if (!member->is_active())
			continue;

		Mapping* map = member->get_mapping();

		if (member->get_abstraction_type() == FORK) {
			const vector<DOperator*> &A_r = map->get_abstract()->get_var_actions(0);

			// Dividing root changing actions into two sets, by the post value
			double min0 = numeric_limits<double>::max();
			double min1 = numeric_limits<double>::max();

			for (size_t a=0; a < A_r.size(); ++a) {
				int act_ind = map->get_orig_operator(A_r[a])->get_index();
				double c = pattern_costs[act_ind];
				if (0 == A_r[a]->get_post_val(0)) {
					min0 = min(min0,c);
				} else {
					min1 = min(min1,c);
				}
			}
			// Set the costs of the actions in this pattern
			for (int a_i = 0; a_i < num_ops; a_i++) {
				// Counting the number of actions
				vector<DOperator*> abs_ops;
				int num_rep = map->get_abs_operators(ops[a_i],abs_ops);
				if (num_rep == 0) {
					continue;
				}

				int root_ind = -1; // will hold the index of the root changing representative.
				for (int i=0;i<num_rep;i++) {
					if(-1 < abs_ops[i]->get_post_val(0)) {
						root_ind = i;
						break;  // There are no two representatives both changing the root of binary fork.
					}
				}

				int act_ind = ops[a_i]->get_index();
				double cost_to_share = pattern_costs[act_ind];

				if (-1 == root_ind) {
					//  All get the same cost
					for (int i=0;i<num_rep;i++) {
						abs_ops[i]->set_double_cost(cost_to_share / num_rep);
					}
				} else {
					// The root representative gets the minimum, all others get the rest
					if( 0 == abs_ops[root_ind]->get_post_val(0)) {
						abs_ops[root_ind]->set_double_cost(min0);
						cost_to_share -= min0;
					} else {
						abs_ops[root_ind]->set_double_cost(min1);
						cost_to_share -= min1;
					}
					for (int i=0;i<num_rep;i++) {
						if (i != root_ind) {
							abs_ops[i]->set_double_cost(cost_to_share / (num_rep-1));
						}
					}
				}
			}
			continue;
		}
		if (member->get_abstraction_type() == INVERTED_FORK) {

			continue;
		}
		if (member->get_abstraction_type() == PATTERN) {

			continue;
		}
	}
}



void SPHeuristic::set_smart_uniform_representatives_cost() {
	//TODO: Adapt to conditional effects before using

	// The costs are uniformly partitioned between the patterns, inside each pattern first the whole cost
	// is given to the root, then the minimum is found and the rest is partitioned uniformly between others.
	const vector<DOperator*> &ops = original_problem->get_operators();

	int num_ops = ops.size();
	vector<double> rep_costs;
	rep_costs.assign(num_ops,0.0);
	for (int a_i = 0; a_i < num_ops; a_i++) {
		// Counting the number of actions
		vector<DOperator*> abs_ops;
		for (int ind = 0; ind < get_ensemble_size(); ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			if (!member->is_active())
				continue;

			member->get_mapping()->get_abs_operators(ops[a_i],abs_ops);
		}
		int num_rep = abs_ops.size();
		if (num_rep > 0) {
			double abs_cost = ops[a_i]->get_double_cost() / num_rep;
			int act_ind = ops[a_i]->get_index();
			assert(act_ind<num_ops);
			rep_costs[act_ind] = abs_cost;
		}
	}

	for (int ind = 0; ind < get_ensemble_size(); ind++){
		SolutionMethod* member = get_ensemble_member(ind);
		if (!member->is_active())
			continue;

		Mapping* map = member->get_mapping();

		if (member->get_abstraction_type() == FORK) {
			const vector<DOperator*> &A_r = map->get_abstract()->get_var_actions(0);

			// Dividing root changing actions into two sets, by the post value
			double min0 = numeric_limits<double>::max();
			double min1 = numeric_limits<double>::max();

			for (size_t a=0; a < A_r.size(); ++a) {
				int act_ind = map->get_orig_operator(A_r[a])->get_index();
				double c = rep_costs[act_ind];
				if (0 == A_r[a]->get_post_val(0)) {
					min0 = min(min0,c);
				} else {
					min1 = min(min1,c);
				}
			}
			// Set the costs of the actions in this pattern
			for (int a_i = 0; a_i < num_ops; a_i++) {
				// Counting the number of actions
				vector<DOperator*> abs_ops;
				int num_rep = map->get_abs_operators(ops[a_i],abs_ops);
				if (num_rep == 0) {
					continue;
				}

				int root_ind = -1; // will hold the index of the root changing representative.
				for (int i=0;i<num_rep;i++) {
					if(-1 < abs_ops[i]->get_post_val(0)) {
						root_ind = i;
						break;  // There are no two representatives both changing the root of binary fork.
					}
				}

				int act_ind = ops[a_i]->get_index();
				double cost_to_share = rep_costs[act_ind] * num_rep;

				if (-1 == root_ind) {
					//  All get the same cost
					for (int i=0;i<num_rep;i++) {
						abs_ops[i]->set_double_cost(cost_to_share / num_rep);
					}
				} else {
					// The root representative gets the minimum, all others get the rest
					if( 0 == abs_ops[root_ind]->get_post_val(0)) {
						abs_ops[root_ind]->set_double_cost(min0);
						cost_to_share -= min0;
					} else {
						abs_ops[root_ind]->set_double_cost(min1);
						cost_to_share -= min1;
					}
					for (int i=0;i<num_rep;i++) {
						if (i != root_ind) {
							abs_ops[i]->set_double_cost(cost_to_share / (num_rep-1));
						}
					}
				}
			}
			continue;
		}
		if (member->get_abstraction_type() == INVERTED_FORK) {

			continue;
		}
		if (member->get_abstraction_type() == PATTERN) {

			continue;
		}
	}
}


void SPHeuristic::deactivate_all() {
	for (int ind = 0; ind < get_ensemble_size(); ind++){
		get_ensemble_member(ind)->deactivate();
	}
}

int SPHeuristic::get_num_vars() {
	int res = 0;
	for (int ind = 0; ind < get_ensemble_size(); ind++){
		res += get_ensemble_member(ind)->get_num_vars();
	}
	return res;
}

int SPHeuristic::get_num_active_vars() {
	int res = 0;
	for (int ind = 0; ind < get_ensemble_size(); ind++){
		if (get_ensemble_member(ind)->is_active())
			res += get_ensemble_member(ind)->get_num_vars();
	}
	return res;
}


int SPHeuristic::get_num_active_members(int var) {
	int res = 0;
	for (int ind = 0; ind < get_ensemble_size(); ind++){
		if (get_ensemble_member(ind)->is_active()) {
			if (-1 != get_ensemble_member(ind)->get_mapping()->get_abstract_var(var))
				res++;
		}
	}
	return res;
}

int SPHeuristic::get_num_active_members() {
	int res = 0;
	for (int ind = 0; ind < get_ensemble_size(); ind++){
		if (get_ensemble_member(ind)->is_active()) {
				res++;
		}
	}
	return res;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
SolutionMethod* SPHeuristic::add_binary_fork(GeneralAbstraction* abs) {
	return new BinaryFork(abs);
}

SolutionMethod* SPHeuristic::add_bounded_inverted_fork(GeneralAbstraction* abs) {
	return new BoundedIfork(abs);
}

SolutionMethod* SPHeuristic::add_pattern(GeneralAbstraction* abs) {
	return new Projection(abs);
}

SolutionMethod* SPHeuristic::add_binary_semifork(GeneralAbstraction* abs) {
//	return new BinaryForks_OFF(fork, abs_domain);
	return new BinarySemiFork(abs);
}
SolutionMethod* SPHeuristic::add_binary_hourglass(GeneralAbstraction* abs) {
	return new BinaryOneDepHourglass(abs);
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
// Creating multiple domain abstraction using the same base abstraction
SolutionMethod* SPHeuristic::add_binary_fork(ForksAbstraction* fork, Domain* abs_domain) {
//	return new BinaryForks_OFF(fork, abs_domain);
	return new BinaryFork(fork, abs_domain);
}

SolutionMethod* SPHeuristic::add_bounded_inverted_fork(IforksAbstraction* ifork, Domain* abs_domain) {
	return new BoundedIfork(ifork, abs_domain);
}

SolutionMethod* SPHeuristic::add_binary_semifork(SemiForksAbstraction* sfork, Domain* abs_domain) {
	return new BinarySemiFork(sfork, abs_domain);
}

SolutionMethod* SPHeuristic::add_binary_hourglass(OneDependentHourglassesAbstraction* hg, Domain* abs_domain) {
	return new BinaryOneDepHourglass(hg, abs_domain);
}

SolutionMethod* SPHeuristic::add_binary_semifork(OneDependentHourglassesAbstraction* sfork, Domain* abs_domain) {
	return new BinarySemiFork(sfork, abs_domain);
}


/////////////////////////////////////////////////////////////////////////////////


SolutionMethod* SPHeuristic::add_pattern(vector<int>& pattern) {
//	Projection_OFF* ptrn = new Projection_OFF(pattern);
	Projection* ptrn = new Projection(pattern);
	ptrn->create(original_problem);
	ptrn->set_abstraction_type(PATTERN);
	return ptrn;
}


void SPHeuristic::solve_all() {
	for (int ind = 0; ind < get_ensemble_size(); ind++){
		get_ensemble_member(ind)->solve();
	}
}

void SPHeuristic::solve_all_and_remove_operators() {
	for (int ind = 0; ind < get_ensemble_size(); ind++){
//		cout << "Solving " << ind << endl;
		get_ensemble_member(ind)->solve();
//		cout << "Solution for ensemble member " << ind << endl;
//		get_ensemble_member(ind)->dump_solution();
//		cout << "Removing operators" << endl;
		get_ensemble_member(ind)->remove_abstract_operators();  // Implemented only in the offline version
//		cout << "Done Removing operators" << endl;
	}
}

void SPHeuristic::check_cost_partition() {

	// Checking the cost of the actions
	// to be correctly partitioned between the representatives.
	const vector<DOperator*> &ops = original_problem->get_operators();

	int num_ops = ops.size();
	for (int a_i = 0; a_i < num_ops; a_i++) {

		// Counting the number of actions
		vector<vector<DOperator*> > abs_ops;
		double tot_cost = 0.0;
		int num_abs = 0;
		for (int ind = 0; ind < get_ensemble_size(); ind++){
			SolutionMethod* member = get_ensemble_member(ind);
			if (!member->is_active())
				continue;

			vector<DOperator*> abs_operators;
			num_abs += member->get_mapping()->get_abs_operators(ops[a_i],abs_operators);
//			cout << "Inc num abs operators for ensemble " << ind << " is " << num_abs << endl;
			abs_ops.push_back(abs_operators);
			for (size_t i=0; i < abs_operators.size(); ++i) {
				tot_cost += abs_operators[i]->get_double_cost();
			}
		}
		double orig_cost = ops[a_i]->get_double_cost();
		if (tot_cost > orig_cost + 0.0000001) {
			cout << "Problem with cost partition of the operator"<< endl;
			ops[a_i]->dump();
			cout << "Into "<< num_abs << " representatives"<< endl;
			for (int ind = 0; ind < get_ensemble_size(); ind++){
				SolutionMethod* member = get_ensemble_member(ind);
				if (!member->is_active())
					continue;

				for (size_t i=0; i < abs_ops[ind].size(); ++i) {
					cout << member->get_abstraction_index() <<
					", " << member->w_var(abs_ops[ind][i]) << " (" <<
					member->get_abstraction_index() + member->w_var(abs_ops[ind][i])
					        << "): ";
					abs_ops[ind][i]->dump();
				}
			}
		}
	}
}

// ---------------------------------------------------------------------------
// New plugin API (Phase 5)
// ---------------------------------------------------------------------------

void SPHeuristic::add_options_to_feature(plugins::Feature &feature) {
    feature.add_option<int>("strategy",
        "forks-decomposition strategy: 0=FORKS_ONLY, 1=INVERTED_FORKS_ONLY, "
        "2=BOTH, 3=SEMIFORKS_ONLY, 4=HOURGLASSES_ONLY", "0");
    feature.add_option<int>("singletons",
        "singleton creation strategy: 0=NECESSARY, 1=BY_DEFINITION, "
        "2=COMPENSATE_FOR_DOMAIN_DECOMPOSITION", "0");
    feature.add_option<int>("cost_partitioning",
        "cost partitioning: 0=UNIFORM, 1=ENSEMBLE_UNIFORM, 2=OPTIMAL, "
        "3=INITIAL_OPTIMAL, 4=GREEDY_SAMPLE, 5=POST_HOC_OPTIMIZATION", "0");
    feature.add_option<int>("var_strategy",
        "variable selection: 0=LANDMARK_ONLY, 1=NON_LANDMARK_ONLY, "
        "2=ALL_VARIABLES, 3=MIXED_LM_FORKS, 4=MIXED_NLM_FORKS", "2");
    feature.add_option<int>("pattern_max_size",
        "maximum size for patterns (0 = unlimited)", "0");
    feature.add_option<int>("ensemble_percentage",
        "percentage of ensemble size to use (1-100)", "100");
    feature.add_option<int>("statistics", "verbose output level", "0");
    feature.add_option<bool>("cache", "caching abstract states", "false");
    feature.add_option<int>("semifork_bound",
        "bound for semifork hat size (excluding the center)", "1");
    add_heuristic_options_to_feature(feature, "implicit_abstractions");
}

tuple<DecompositionStrategy, SingletonStrategy, CostPartitioning,
      VariablesSelectionStrategy, int, int, int, bool, int>
SPHeuristic::get_sp_arguments_from_options(const plugins::Options &opts) {
    return make_tuple(
        DecompositionStrategy(opts.get<int>("strategy")),
        SingletonStrategy(opts.get<int>("singletons")),
        CostPartitioning(opts.get<int>("cost_partitioning")),
        VariablesSelectionStrategy(opts.get<int>("var_strategy")),
        opts.get<int>("pattern_max_size"),
        opts.get<int>("ensemble_percentage"),
        opts.get<int>("statistics"),
        opts.get<bool>("cache"),
        opts.get<int>("semifork_bound"));
}

class SPHeuristicFeature
    : public plugins::TypedFeature<Evaluator, SPHeuristic> {
public:
    SPHeuristicFeature() : TypedFeature("implicit_abstractions") {
        document_title("Implicit Abstractions heuristic");
        document_synopsis(
            "Admissible heuristic based on acyclic causal-graph decompositions "
            "(implicit abstractions / structural patterns).");
        SPHeuristic::add_options_to_feature(*this);
        document_language_support("action costs", "supported");
        document_language_support("conditional effects", "partially supported");
        document_language_support("axioms", "partially supported");
        document_property("admissible", "yes");
        document_property("consistent", "no");
        document_property("safe", "yes");
        document_property("preferred operators", "no");
    }

    virtual shared_ptr<SPHeuristic> create_component(
        const plugins::Options &opts) const override {
        return plugins::make_shared_from_arg_tuples<SPHeuristic>(
            get_heuristic_arguments_from_options(opts),
            SPHeuristic::get_sp_arguments_from_options(opts));
    }
};

static plugins::FeaturePlugin<SPHeuristicFeature> _plugin;
