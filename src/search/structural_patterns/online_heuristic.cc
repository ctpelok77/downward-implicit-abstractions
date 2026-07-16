#include "SP_globals.h"
#include "online_heuristic.h"
#include "abstractions/var_projection.h"
#include <vector>
#include "problem.h"
// globals removed — use TaskProxy
#include "abstractions/general_abstraction.h"
#include "mappings/var_proj_mapping.h"
#include <iostream>
#include "solutions/projection_on.h"
#include "solutions/binary_forks_on.h"
#include "solutions/bounded_iforks_on.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
// causal_graph: use task_utils

using namespace std;

OnlineHeuristic::OnlineHeuristic(const Options &options)
    : SPHeuristic(options) {

}

OnlineHeuristic::OnlineHeuristic(const Options &options, const Problem* prob)
    : SPHeuristic(options, prob) {
}


OnlineHeuristic::~OnlineHeuristic() {
}

///////////////////////////////////////////////////////////////////////////////

SolutionMethod* OnlineHeuristic::add_binary_fork(GeneralAbstraction* abs) {
	return new BinaryForks_ON(abs);
}

SolutionMethod* OnlineHeuristic::add_bounded_inverted_fork(GeneralAbstraction* abs) {
	return new BoundedIforks_ON(abs);
}

SolutionMethod* OnlineHeuristic::add_pattern(GeneralAbstraction* abs) {
	return new Projection_ON(abs);
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
// Creating multiple domain abstraction using the same base abstraction
SolutionMethod* OnlineHeuristic::add_binary_fork(ForksAbstraction* fork, Domain* abs_domain) {
	return new BinaryForks_ON(fork, abs_domain);
}

SolutionMethod* OnlineHeuristic::add_bounded_inverted_fork(IforksAbstraction* ifork, Domain* abs_domain) {
	return new BoundedIforks_ON(ifork, abs_domain);
}

/////////////////////////////////////////////////////////////////////////////////

SolutionMethod* OnlineHeuristic::add_pattern(vector<int>& pattern) {
	Projection_ON* ptrn = new Projection_ON(pattern);
	ptrn->create(task);
	ptrn->set_abstraction_type(PATTERN);
	return ptrn;
}

//////////////////////////////////////////////////////////////////////////////////
