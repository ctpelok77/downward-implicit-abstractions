#include "LP_binary_forks.h"
#include "../mappings/mapping.h"
#include <math.h>

using namespace std;

LPBinaryForks::LPBinaryForks() : LPBinaryFork() {}
LPBinaryForks::LPBinaryForks(GeneralAbstraction *abs) : LPBinaryFork(abs) {}
LPBinaryForks::~LPBinaryForks() {}

void LPBinaryForks::initiate() {
    LPBinaryFork::initiate();
    BinaryFork::solve();

    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();

    int root_goal = abs->get_goal_val(0);
    for (size_t v=1; v < doms.size(); ++v) {
        for (int val = 0; val < doms[v]; val++) {
            for (int theta = 0; theta < doms[0]; theta++) {
                int u     = get_sigma_upper_bound(v, val, theta);
                int d_ind = d_var(v, val, u, theta);
                int upper_b = (int)solution->get_value(d_ind) + 1;

                if (-1 != root_goal) {
                    if (theta == root_goal) {
                        if ((upper_b % 2) == 0) upper_b++;
                    } else {
                        if ((upper_b % 2) == 1) upper_b++;
                    }
                }
                set_sigma_upper_bound(v, val, theta, upper_b);
            }
        }
    }
}
