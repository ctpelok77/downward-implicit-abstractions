#include "LP_projection_off.h"
#include "../mappings/mapping.h"
#include "../SP_LPConstraint.h"
#include <limits>

using namespace std;

LPProjection_OFF::LPProjection_OFF() : LPProjection() {}
LPProjection_OFF::LPProjection_OFF(GeneralAbstraction *abs) : LPProjection(abs) {}
LPProjection_OFF::LPProjection_OFF(vector<int> &pattern) : LPProjection(pattern) {}
LPProjection_OFF::~LPProjection_OFF() {}

void LPProjection_OFF::set_objective() {
    obj_func = new SPLPConstraint();
    // Maximise sum of all d(s)
    for (int d_ind = 0; d_ind < get_num_vars(); d_ind++)
        obj_func->add_val(d_ind, 1.0);
    obj_func->finalize();
}

void LPProjection_OFF::add_distance_constraints(vector<int> free_vars, vector<int> state,
                                                  const vector<int> &doms, DOperator *op) {
    if (free_vars.empty()) {
        const vector<PrePost> &pre = op->get_pre_post();
        vector<int> next_state = state;
        for (size_t i=0; i < pre.size(); ++i)
            next_state[pre[i].var] = pre[i].post;

        int d_ind0 = d_var(state);
        int d_ind1 = d_var(next_state);
        if (d_ind0 == d_ind1) return;

        double c = op->get_double_cost();
        SPLPConstraint *lpc = new SPLPConstraint(-numeric_limits<double>::max(), c, true);
        lpc->add_val(d_ind0, 1.0);
        lpc->add_val(d_ind1, -1.0);
        lpc->finalize();
        static_LPConstraints.push_back(lpc);
        return;
    }
    int free_var = free_vars.back();
    free_vars.pop_back();
    for (int i = 0; i < doms[free_var]; i++) {
        state[free_var] = i;
        add_distance_constraints(free_vars, state, doms, op);
    }
}

void LPProjection_OFF::add_goal_constraints(vector<int> free_vars, vector<int> state,
                                              const vector<int> &doms) {
    if (free_vars.empty()) {
        int d_ind = d_var(state);
        static_LPConstraints.push_back(set_x_eq_0_constraint(d_ind, true));
        return;
    }
    int free_var = free_vars.back();
    free_vars.pop_back();
    for (int i = 0; i < doms[free_var]; i++) {
        state[free_var] = i;
        add_goal_constraints(free_vars, state, doms);
    }
}
