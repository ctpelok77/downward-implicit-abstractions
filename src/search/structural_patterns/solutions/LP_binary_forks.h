#ifndef STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORKS_H
#define STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORKS_H

#include "LP_binary_fork_gen.h"
#include "../abstractions/general_abstraction.h"

/*
 * Online LP fork: calls offline solve() during initiate() to tighten bounds,
 * then solves an LP per state.
 */

class LPBinaryForks : public LPBinaryFork {

public:
    LPBinaryForks();
    LPBinaryForks(GeneralAbstraction *abs);
    virtual ~LPBinaryForks();

    virtual void initiate() override;
};

#endif /* STRUCTURAL_PATTERNS_SOLUTIONS_LP_BINARY_FORKS_H */
