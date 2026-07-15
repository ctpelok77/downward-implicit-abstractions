#include "LP_projection_gen.h"
#include "../mappings/mapping.h"
#include "../SP_LPConstraint.h"
#include <math.h>
#include <cassert>

using namespace std;

LPProjection::LPProjection() : Projection() {}
LPProjection::LPProjection(GeneralAbstraction *abs) : Projection(abs) {}
LPProjection::LPProjection(vector<int> &pattern) : Projection(pattern) {}
LPProjection::~LPProjection() {}

void LPProjection::initiate() {
    Projection::initiate();
    const Problem *abs = get_mapping()->get_abstract();
    number_of_w_var_variables = abs->get_actions_number();
}

void LPProjection::set_static_constraints() {
    const Problem *abs = get_mapping()->get_abstract();

    vector<vector<int>> states;
    abs->generate_state_transition_graph(states);
    const vector<DOperator *> &ops = abs->get_operators();
    const vector<int> &doms = abs->get_variable_domains();

    // (I) d(s') <= d(s") + w(a)  for all <s",a,s'>
    for (size_t it=0; it < ops.size(); ++it) {
        if (ops[it]->is_redundant())
            continue;
        vector<int> generated = states[it];
        vector<int> free_vars;
        for (size_t i=0; i < generated.size(); ++i) {
            if (generated[i] < 0)
                free_vars.push_back(i);
        }
        add_distance_constraints(free_vars, generated, doms, ops[it]);
    }

    // (II) h <= d(G)  for all goals G
    vector<int> goals;
    abs->get_goal_vals(goals);
    vector<int> free_vars;
    for (size_t i=0; i < goals.size(); ++i) {
        if (goals[i] < 0)
            free_vars.push_back(i);
    }
    add_goal_constraints(free_vars, goals, doms);
}

void LPProjection::get_dynamic_constraints(const SPState &state,
                                            vector<SPLPConstraint *> &dyn_constr) {
    // Only dynamic constraint: d(s_I) = 0
    int d_ind = d_var(get_abstract_state(state));
    dyn_constr.push_back(set_x_eq_0_constraint(d_ind, false));
}

void LPProjection::add_distance_constraints(vector<int> free_vars, vector<int> state,
                                             const vector<int> &doms, DOperator *op) {
    if (free_vars.empty()) {
        const vector<PrePost> &pre = op->get_pre_post();
        vector<int> next_state = state;
        for (size_t i=0; i < pre.size(); ++i)
            next_state[pre[i].var] = pre[i].post;

        int d_ind0 = d_var(state);
        int d_ind1 = d_var(next_state);
        if (d_ind0 == d_ind1) return;

        int w_ind = w_var(op);
        static_LPConstraints.push_back(
            set_x_leq_y_plus_z_constraint(d_ind1, d_ind0, w_ind, true));
        return;
    }
    int free_var = free_vars.back();
    free_vars.pop_back();
    for (int i = 0; i < doms[free_var]; i++) {
        state[free_var] = i;
        add_distance_constraints(free_vars, state, doms, op);
    }
}

void LPProjection::add_goal_constraints(vector<int> free_vars, vector<int> state,
                                         const vector<int> &doms) {
    if (free_vars.empty()) {
        int d_ind = d_var(state);
        int h_ind = h_var();
        static_LPConstraints.push_back(set_x_leq_y_constraint(h_ind, d_ind, true));
        return;
    }
    int free_var = free_vars.back();
    free_vars.pop_back();
    for (int i = 0; i < doms[free_var]; i++) {
        state[free_var] = i;
        add_goal_constraints(free_vars, state, doms);
    }
}

int LPProjection::h_var() const {
    return number_of_d_variables;
}

int LPProjection::w_var(DOperator *a) const {
    const Problem *abs = get_mapping()->get_abstract();
    return number_of_d_variables + number_of_h_variables + abs->get_action_index(a);
}
