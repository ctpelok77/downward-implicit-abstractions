#include "LP_binary_semifork_gen.h"
#include "../mappings/mapping.h"
#include "../SP_LPConstraint.h"
#include <math.h>
#include <limits>
#include <cassert>
#include <iostream>

using namespace std;

LPBinarySemiFork::LPBinarySemiFork() : BinarySemiFork() {}
LPBinarySemiFork::LPBinarySemiFork(GeneralAbstraction *abs) : BinarySemiFork(abs) {}
LPBinarySemiFork::LPBinarySemiFork(SemiForksAbstraction *f, Domain *abs_domain)
    : BinarySemiFork(f, abs_domain) {}
LPBinarySemiFork::~LPBinarySemiFork() {}

void LPBinarySemiFork::initiate() {
    cout << "Initiating semifork LP" << endl;
    BinarySemiFork::initiate();
    BinarySemiFork::solve();
    number_of_w_var_variables = get_mapping()->get_abstract()->get_actions_number();
    number_of_m_variables     = 2 * num_hat_states * sigma_size;
    cout << "Done initiating semifork LP" << endl;
}

// ---------------------------------------------------------------------------
// Static constraints (fork part)
// ---------------------------------------------------------------------------
void LPBinarySemiFork::set_static_constraints_fork() {
    cout << "Setting static constraints for the fork part" << endl;
    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();

    for (size_t leaf_ind=0; leaf_ind < leaf_vars.size(); ++leaf_ind) {
        int v = leaf_vars[leaf_ind];
        int dom_size = doms[v];
        int g_v = abs->get_goal_val(v);
        const vector<DOperator *> &A_v = abs->get_var_actions(v);
        int A_v_size = (int)A_v.size();

        for (int val_0 = 0; val_0 < dom_size; val_0++) {
            for (int theta = 0; theta < doms[get_abs_root()]; theta++) {
                int p_ind = p_var(leaf_ind, val_0, val_0, theta);
                static_LPConstraints.push_back(set_x_eq_0_constraint(p_ind, true));

                if (1 == get_sigma_lower_bound(leaf_ind, val_0, theta)) {
                    int d_ind0 = d_var(leaf_ind, val_0, 1, theta);
                    int p_ind0 = p_var(leaf_ind, val_0, g_v, theta);
                    static_LPConstraints.push_back(set_x_eq_y_constraint(d_ind0, p_ind0, true));
                }

                for (int val_1 = 0; val_1 < dom_size; val_1++) {
                    int first_support = max(get_sigma_lower_bound(leaf_ind, val_1, theta), 2);
                    int last_support  = get_sigma_upper_bound(leaf_ind, val_1, theta);
                    for (int sz = first_support; sz <= last_support; sz++) {
                        if (get_sigma_lower_bound(leaf_ind, val_0, 1 - theta) >= sz)
                            continue;
                        int d_ind0  = d_var(leaf_ind, val_1, sz, theta);
                        int upper_b = min(get_sigma_upper_bound(leaf_ind, val_0, 1 - theta), sz - 1);
                        assert(upper_b > 0);
                        int d_ind1 = d_var(leaf_ind, val_0, upper_b, 1 - theta);
                        int p_ind2 = p_var(leaf_ind, val_1, val_0, theta);
                        static_LPConstraints.push_back(
                            set_x_leq_y_plus_z_constraint(d_ind0, d_ind1, p_ind2, true));
                    }
                }
            }

            for (int a = 0; a < A_v_size; a++) {
                int w_ind0 = w_var(A_v[a]);
                vector<PrePost> pre_v;
                A_v[a]->get_explicit_pre_post_for_var(v, pre_v);
                assert((int)pre_v.size() > 0);
                for (size_t p=0; p < pre_v.size(); ++p) {
                    assert(pre_v[p].cond.size() == 0 ||
                           (pre_v[p].cond.size() == 1 && pre_v[p].cond[0].var == 0));
                    int start_theta = (0 == (int)pre_v[p].cond.size()) ? 0 : pre_v[p].cond[0].prev;
                    int end_theta   = (0 == (int)pre_v[p].cond.size()) ? doms[0] - 1 : pre_v[p].cond[0].prev;
                    if (val_0 == pre_v[p].post)
                        continue;
                    for (int theta = start_theta; theta <= end_theta; theta++) {
                        int p_ind0 = p_var(leaf_ind, val_0, pre_v[p].post, theta);
                        if (-1 == pre_v[p].pre) {
                            static_LPConstraints.push_back(
                                set_x_leq_y_constraint(p_ind0, w_ind0, true));
                        } else {
                            int p_ind1 = p_var(leaf_ind, val_0, pre_v[p].pre, theta);
                            static_LPConstraints.push_back(
                                set_x_leq_y_plus_z_constraint(p_ind0, p_ind1, w_ind0, true));
                        }
                    }
                }
            }
        }
    }
    cout << "End setting static constraints for the fork part" << endl;
}

// ---------------------------------------------------------------------------
// Static constraints (hat/semifork part)
// ---------------------------------------------------------------------------
void LPBinarySemiFork::set_static_constraints_hat() {
    cout << "Setting static constraints for the hat part" << endl;
    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();

    vector<vector<int>> states;
    abs->generate_state_transition_graph(states);

    for (int change = 0; change < sigma_size; change++)
        solve_step(states, change);

    cout << "End setting static constraints for the hat part" << endl;
}

void LPBinarySemiFork::set_static_constraints() {
    set_static_constraints_fork();
    set_static_constraints_hat();
}

// ---------------------------------------------------------------------------
// solve_step / set_distances / set_base_goal_distances
// ---------------------------------------------------------------------------
void LPBinarySemiFork::solve_step(const vector<vector<int>> &states, int changes) {
    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();
    const vector<DOperator *> &ops = abs->get_operators();
    const vector<int> &doms = abs->get_variable_domains();

    for (size_t it=0; it < ops.size(); ++it) {
        if (ops[it]->is_redundant())
            continue;
        bool to_skip = true;
        const vector<PrePost> &pre = ops[it]->get_pre_post();
        for (size_t i=0; i < pre.size(); ++i) {
            if (pre[i].var == get_abs_root() || is_in_hat[pre[i].var]) {
                to_skip = false;
                break;
            }
        }
        if (to_skip)
            continue;
        vector<int> generated = states[it];
        vector<int> free_vars;
        for (size_t i=0; i < hat_vars.size(); ++i) {
            if (generated[hat_vars[i]] < 0)
                free_vars.push_back(hat_vars[i]);
        }
        for (int theta = 0; theta < doms[get_abs_root()]; theta++)
            set_distances(free_vars, generated, doms, ops[it], changes, theta);
    }

    if (changes == 0) {
        vector<int> goals;
        abs->get_goal_vals(goals);
        vector<int> free_vars;
        for (size_t i=0; i < goals.size(); ++i) {
            if (goals[i] < 0 && is_in_hat[i])
                free_vars.push_back(i);
        }
        int start_theta = 0;
        int end_theta   = doms[get_abs_root()] - 1;
        if (goals[get_abs_root()] != -1) {
            start_theta = goals[get_abs_root()];
            end_theta   = goals[get_abs_root()];
        }
        for (int theta = start_theta; theta <= end_theta; theta++)
            set_base_goal_distances(free_vars, goals, doms, theta);
    }
}

void LPBinarySemiFork::set_distances(vector<int> free_vars, vector<int> state,
                                      const vector<int> &doms, DOperator *op,
                                      int changes, int theta) {
    if (free_vars.empty()) {
        const vector<PrePost> &pre = op->get_pre_post();
        int new_theta = theta;
        vector<int> next_state = state;
        for (size_t i=0; i < pre.size(); ++i) {
            if (is_in_hat[pre[i].var])
                next_state[pre[i].var] = pre[i].post;
            if (get_abs_root() == pre[i].var) {
                new_theta = pre[i].post;
                assert(new_theta == 1 - theta);
            }
        }
        int m_ind0 = m_var(state, changes, theta);
        int w_ind  = w_var(op);
        if (theta != new_theta && changes > 0) {
            int m_prev = m_var(next_state, changes - 1, new_theta);
            static_LPConstraints.push_back(
                set_x_leq_y_plus_z_constraint(m_ind0, m_prev, w_ind, true));
            return;
        }
        int m_ind1 = m_var(next_state, changes, new_theta);
        if (m_ind0 == m_ind1) return;
        static_LPConstraints.push_back(
            set_x_leq_y_plus_z_constraint(m_ind0, m_ind1, w_ind, true));
        return;
    }
    int free_var  = free_vars.back();
    free_vars.pop_back();
    int dom_size = doms[free_var];
    for (int i = 0; i < dom_size; i++) {
        state[free_var] = i;
        set_distances(free_vars, state, doms, op, changes, theta);
    }
}

void LPBinarySemiFork::set_base_goal_distances(vector<int> free_vars, vector<int> state,
                                                const vector<int> &doms, int theta) {
    if (free_vars.empty()) {
        int m_ind = m_var(state, 0, theta);
        static_LPConstraints.push_back(set_x_eq_0_constraint(m_ind, true));
        return;
    }
    int free_var  = free_vars.back();
    free_vars.pop_back();
    int dom_size = doms[free_var];
    for (int i = 0; i < dom_size; i++) {
        state[free_var] = i;
        set_base_goal_distances(free_vars, state, doms, theta);
    }
}

// ---------------------------------------------------------------------------
// Dynamic constraints
// ---------------------------------------------------------------------------
void LPBinarySemiFork::get_dynamic_constraints(const SPState &state,
                                                vector<SPLPConstraint *> &dyn_constr) {
    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();
    const SPState &abs_state = get_abstract_state(state);

    int root_zero = abs_state[get_abs_root()];
    int root_goal = abs->get_goal_val(get_abs_root());

    int lower_b = 0;
    int upper_b = 0;
    for (size_t leaf_ind=0; leaf_ind < leaf_vars.size(); ++leaf_ind) {
        int v       = leaf_vars[leaf_ind];
        int val     = abs_state[v];
        int lower_v = get_sigma_lower_bound(leaf_ind, val, root_zero);
        int upper_v = get_sigma_upper_bound(leaf_ind, val, root_zero);
        lower_b = (lower_b < lower_v) ? lower_v : lower_b;
        upper_b = (upper_b < upper_v) ? upper_v : upper_b;
    }

    if (-1 != root_goal) {
        if (root_zero == root_goal) {
            if ((lower_b % 2) == 0) lower_b++;
            if ((upper_b % 2) == 0) upper_b++;
        } else {
            if ((lower_b % 2) == 1) lower_b++;
            if ((upper_b % 2) == 1) upper_b++;
        }
    }

    for (int sigma = lower_b; sigma <= upper_b; sigma += sigma_step)
        dyn_constr.push_back(set_h_constraint(sigma, abs_state));
}

SPLPConstraint *LPBinarySemiFork::set_h_constraint(int sigma, const SPState &eval_state) const {
    int root_zero = eval_state[get_abs_root()];
    SPLPConstraint *lpc = new SPLPConstraint(0.0, numeric_limits<double>::max(), false);

    int h_ind = h_var();
    lpc->add_val(h_ind, -1.0);

    for (size_t leaf_ind=0; leaf_ind < leaf_vars.size(); ++leaf_ind) {
        int v     = leaf_vars[leaf_ind];
        int val   = eval_state[v];
        int fixed = min(get_sigma_upper_bound(leaf_ind, val, root_zero), sigma);
        assert(fixed > 0);
        int d_ind = d_var(leaf_ind, val, fixed, root_zero);
        lpc->add_val(d_ind, 1.0);
    }
    int m_ind = m_var(eval_state, sigma - 1, root_zero);
    lpc->add_val(m_ind, 1.0);

    lpc->finalize();
    return lpc;
}

// ---------------------------------------------------------------------------
// Variable index helpers
// ---------------------------------------------------------------------------
int LPBinarySemiFork::w_var(DOperator *a) const {
    const Problem *abs = get_mapping()->get_abstract();
    return number_of_d_variables + number_of_p_variables + number_of_h_variables +
           number_of_w_r_variables + number_of_w_v_variables + abs->get_action_index(a);
}

int LPBinarySemiFork::m_var(const SPState &state, int changes, int theta) const {
    int ret = 0;
    for (size_t i=0; i < hat_vars.size(); ++i)
        ret += state[hat_vars[i]] * multiplier[i];
    return number_of_d_variables + number_of_p_variables + number_of_h_variables +
           number_of_w_r_variables + number_of_w_v_variables + number_of_w_var_variables +
           2 * sigma_size * ret + 2 * changes + theta;
}

// ---------------------------------------------------------------------------
// Debug helpers
// ---------------------------------------------------------------------------
void LPBinarySemiFork::dump_m_constraint(const SPState &state, int count, int theta) const {
    cout << "m(" << state[hat_vars[0]];
    for (size_t i=1; i < hat_vars.size(); ++i)
        cout << " " << state[hat_vars[i]];
    cout << "," << count << "," << theta << ")";
}
void LPBinarySemiFork::dump_d_constraint(int v, int val, int sz, int theta) const {
    cout << "d(" << v << "," << val << "," << sz << "," << theta << ")";
}
void LPBinarySemiFork::dump_p_constraint(int v, int val1, int val2, int theta) const {
    cout << "p(" << v << "," << val1 << "," << val2 << "," << theta << ")";
}
void LPBinarySemiFork::dump_w_constraint(const DOperator *op) const {
    cout << "w(a" << op->get_index() << ")";
}
