#ifndef STRUCTURAL_PATTERNS_STATEOPTHEURISTIC_H
#define STRUCTURAL_PATTERNS_STATEOPTHEURISTIC_H

#include "LP_heuristic.h"

// option_parser removed — use plugins/plugin.h

class Problem;
class State;

class StateOptimalHeuristic: public LPHeuristic {
	const State& eval_state;
	bool solved;
	bool exit_if_unsolved;
protected:
	int num_LP_vars(const State& state);

public:
	StateOptimalHeuristic(const Options &options, const State& state);
	StateOptimalHeuristic(const Options &options, const State& state, const Problem* prob);
	virtual ~StateOptimalHeuristic();

    virtual void initialize();
	virtual int compute_heuristic(const State& state) { return SPHeuristic::compute_heuristic(state);}

	bool is_LP_solved() const { return solved; }
};

#endif /* STATEOPTHEURISTIC_H */
