#include "LP_binary_forks_bounds.h"
#include "../mappings/mapping.h"
#include <math.h>
#include <cassert>
#include <limits>

using namespace std;

LPBinaryForks_b::LPBinaryForks_b() : LPBinaryFork() { set_d_vars_multiplier(1); }
LPBinaryForks_b::LPBinaryForks_b(GeneralAbstraction *abs) : LPBinaryFork(abs) { set_d_vars_multiplier(1); }
LPBinaryForks_b::LPBinaryForks_b(ForksAbstraction *f, Domain *abs_domain)
    : LPBinaryFork(f, abs_domain) { set_d_vars_multiplier(1); }
LPBinaryForks_b::~LPBinaryForks_b() {}

void LPBinaryForks_b::initiate() {
    LPBinaryFork::initiate();
    number_of_w_v_variables = 2 * num_leafs * sigma_size * sigma_size;
}

void LPBinaryForks_b::set_bounds_for_state(const SPState &state) {
    BinaryFork::solve(state);

    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();
    const SPState &abs_state = get_abstract_state(state);

    int root_zero = abs_state[0];
    int root_goal = abs->get_goal_val(0);

    for (size_t v=1; v < doms.size(); ++v) {
        for (int val = 0; val < doms[v]; val++) {
            int upper_b = get_domain_bound(v, val, abs_state);
            if (-1 != root_goal) {
                if (root_zero == root_goal) {
                    if ((upper_b % 2) == 0) upper_b++;
                } else {
                    if ((upper_b % 2) == 1) upper_b++;
                }
            }
            set_sigma_upper_bound(v, val, root_zero, upper_b);
        }
    }
}

int LPBinaryForks_b::get_domain_bound(int v, int g_v, const SPState &eval_state) const {
    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();
    const vector<DOperator *> &ops = abs->get_var_actions(v);
    int root_dom = abs->get_variable_domain(0);
    int dom = abs->get_variable_domain(v);

    bool **in_arcs  = new bool *[dom];
    bool **out_arcs = new bool *[dom];
    bool *in_uncond = new bool[dom];
    for (int val = 0; val < dom; val++) {
        in_arcs[val]  = new bool[root_dom]();
        out_arcs[val] = new bool[root_dom]();
        in_uncond[val] = false;
    }

    for (size_t a=0; a < ops.size(); ++a) {
        vector<PrePost> pre_v;
        ops[a]->get_explicit_pre_post_for_var(v, pre_v);
        assert((int)pre_v.size() > 0);
        for (size_t p=0; p < pre_v.size(); ++p) {
            assert(pre_v[p].cond.size() == 0 ||
                   (pre_v[p].cond.size() == 1 && pre_v[p].cond[0].var == 0));
            int pre_b = (-1 == pre_v[p].pre) ? 0 : pre_v[p].pre;
            int pre_e = (-1 == pre_v[p].pre) ? dom - 1 : pre_v[p].pre;
            if ((int)pre_v[p].cond.size() == 0) {
                in_uncond[pre_v[p].post] = true;
            } else {
                in_arcs[pre_v[p].post][pre_v[p].cond[0].prev] = true;
                for (int val = pre_b; val <= pre_e; val++)
                    out_arcs[val][pre_v[p].cond[0].prev] = true;
            }
        }
    }

    int s_v = (int)eval_state[v];
    int root_zero = (int)eval_state[0];
    int ret = 1;
    for (int val = 0; val < dom; val++) {
        if (val == s_v) {
            if (out_arcs[val][1 - root_zero]) ret++;
        } else if (val != g_v) {
            if ((in_arcs[val][0] && out_arcs[val][1]) ||
                (in_arcs[val][1] && out_arcs[val][0]) ||
                (in_uncond[val] && (out_arcs[val][0] || out_arcs[val][1])))
                ret++;
        }
    }
    for (int val = 0; val < dom; val++) {
        delete [] in_arcs[val];
        delete [] out_arcs[val];
    }
    delete [] in_arcs;
    delete [] out_arcs;
    delete [] in_uncond;
    return ret;
}

void LPBinaryForks_b::set_general_bounds() {
    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();
    int root_goal = abs->get_goal_val(0);

    for (size_t v=1; v < doms.size(); ++v) {
        for (int val = 0; val < doms[v]; val++) {
            for (int theta = 0; theta < doms[0]; theta++) {
                int lower_b = 1;
                int upper_b = doms[v];
                if (-1 != root_goal) {
                    if (theta == root_goal) {
                        if ((upper_b % 2) == 0) upper_b++;
                    } else {
                        if ((upper_b % 2) == 1) upper_b++;
                    }
                }
                set_sigma_lower_bound(v, val, theta, lower_b);
                set_sigma_upper_bound(v, val, theta, upper_b);
            }
        }
    }
}

int LPBinaryForks_b::get_domain_bound(int v, int g_v, int root_zero) const {
    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();
    const vector<DOperator *> &ops = abs->get_var_actions(v);
    int root_dom = abs->get_variable_domain(0);
    int dom = abs->get_variable_domain(v);

    bool **in_arcs  = new bool *[dom];
    bool **out_arcs = new bool *[dom];
    bool *in_uncond = new bool[dom];
    for (int val = 0; val < dom; val++) {
        in_arcs[val]  = new bool[root_dom]();
        out_arcs[val] = new bool[root_dom]();
        in_uncond[val] = false;
    }

    for (size_t a=0; a < ops.size(); ++a) {
        vector<PrePost> pre_v;
        ops[a]->get_explicit_pre_post_for_var(v, pre_v);
        assert((int)pre_v.size() > 0);
        for (size_t p=0; p < pre_v.size(); ++p) {
            assert(pre_v[p].cond.size() == 0 ||
                   (pre_v[p].cond.size() == 1 && pre_v[p].cond[0].var == 0));
            int pre_b = (-1 == pre_v[p].pre) ? 0 : pre_v[p].pre;
            int pre_e = (-1 == pre_v[p].pre) ? dom - 1 : pre_v[p].pre;
            if ((int)pre_v[p].cond.size() == 0) {
                in_uncond[pre_v[p].post] = true;
            } else {
                in_arcs[pre_v[p].post][pre_v[p].cond[0].prev] = true;
                for (int val = pre_b; val <= pre_e; val++)
                    out_arcs[val][pre_v[p].cond[0].prev] = true;
            }
        }
    }

    const SPState &abs_state = abs->get_initial_state();
    int s_v = abs_state[v];
    int ret = 1;
    for (int val = 0; val < dom; val++) {
        if (val == s_v) {
            if (out_arcs[val][1 - root_zero]) ret++;
        } else if (val != g_v) {
            if ((in_arcs[val][0] && out_arcs[val][1]) ||
                (in_arcs[val][1] && out_arcs[val][0]) ||
                (in_uncond[val] && (out_arcs[val][0] || out_arcs[val][1])))
                ret++;
        }
    }
    for (int val = 0; val < dom; val++) {
        delete [] in_arcs[val];
        delete [] out_arcs[val];
    }
    delete [] in_arcs;
    delete [] out_arcs;
    delete [] in_uncond;
    return ret;
}

void LPBinaryForks_b::set_static_constraints() {
    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();

    int var_num = (int)doms.size();
    for (int v = 1; v < var_num; v++) {
        int dom_size = doms[v];

        for (int val_0 = 0; val_0 < dom_size; val_0++) {
            for (int theta = 0; theta < doms[0]; theta++) {
                int p_ind = p_var(v, val_0, val_0, theta);
                static_LPConstraints.push_back(set_x_eq_0_constraint(p_ind, true));
            }
        }

        // Build action count table to decide whether to use auxiliary variables
        int ***actions = new int **[doms[0]];
        for (int theta = 0; theta < doms[0]; theta++) {
            actions[theta] = new int *[dom_size];
            for (int val_0 = 0; val_0 < dom_size; val_0++) {
                actions[theta][val_0] = new int[dom_size]();
            }
        }

        const vector<DOperator *> &A_v = abs->get_var_actions(v);
        int A_v_size = (int)A_v.size();

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

                if (-1 == pre_v[p].pre) {
                    for (int val_0 = 0; val_0 < dom_size; val_0++) {
                        for (int theta = start_theta; theta <= end_theta; theta++) {
                            static_LPConstraints.push_back(
                                set_p_constraint(v, val_0, pre_v[p].pre, pre_v[p].post,
                                                 theta, w_ind0, true));
                        }
                    }
                } else {
                    for (int theta = start_theta; theta <= end_theta; theta++)
                        actions[theta][pre_v[p].pre][pre_v[p].post]++;
                }
            }
        }

        const int threshold = 3;
        for (int a = 0; a < A_v_size; a++) {
            int w_ind0 = w_var(A_v[a]);
            vector<PrePost> pre_v;
            A_v[a]->get_explicit_pre_post_for_var(v, pre_v);
            assert((int)pre_v.size() > 0);
            for (size_t p=0; p < pre_v.size(); ++p) {
                if (-1 == pre_v[p].pre)
                    continue;
                int start_theta = (0 == (int)pre_v[p].cond.size()) ? 0 : pre_v[p].cond[0].prev;
                int end_theta   = (0 == (int)pre_v[p].cond.size()) ? doms[0] - 1 : pre_v[p].cond[0].prev;
                for (int theta = start_theta; theta <= end_theta; theta++) {
                    if (actions[theta][pre_v[p].pre][pre_v[p].post] > threshold) {
                        int w_ind = w_v(v, pre_v[p].pre, pre_v[p].post, theta);
                        static_LPConstraints.push_back(
                            set_x_leq_y_constraint(w_ind, w_ind0, true));
                    } else {
                        for (int val_0 = 0; val_0 < dom_size; val_0++) {
                            static_LPConstraints.push_back(
                                set_p_constraint(v, val_0, pre_v[p].pre, pre_v[p].post,
                                                 theta, w_ind0, true));
                        }
                    }
                }
            }
        }

        for (int val_0 = 0; val_0 < dom_size; val_0++) {
            for (int val_1 = 0; val_1 < dom_size; val_1++) {
                for (int theta = 0; theta < doms[0]; theta++) {
                    if (actions[theta][val_0][val_1] > threshold) {
                        for (int val = 0; val < dom_size; val++) {
                            int w_ind = w_v(v, val_0, val_1, theta);
                            static_LPConstraints.push_back(
                                set_p_constraint(v, val, val_0, val_1, theta, w_ind, true));
                        }
                    }
                }
            }
        }

        for (int theta = 0; theta < doms[0]; theta++) {
            for (int val_0 = 0; val_0 < dom_size; val_0++)
                delete [] actions[theta][val_0];
            delete [] actions[theta];
        }
        delete [] actions;
    }

    const vector<DOperator *> &A_r = abs->get_var_actions(0);
    int w_ind0 = w_r(0);
    int w_ind1 = w_r(1);

    for (size_t a=0; a < A_r.size(); ++a) {
        vector<PrePost> pre_r;
        A_r[a]->get_explicit_pre_post_for_var(0, pre_r);
        assert((int)pre_r.size() > 0);
        for (size_t p=0; p < pre_r.size(); ++p) {
            assert(pre_r[p].cond.size() == 0);
            int pre  = pre_r[p].pre;
            int post = pre_r[p].post;
            if (pre == post)
                continue;
            if (0 == post) {
                int w_ind3 = w_var(A_r[a]);
                dynamic_LPConstraints[0].push_back(set_x_leq_y_constraint(w_ind1, w_ind3, true));
                dynamic_LPConstraints[1].push_back(set_x_leq_y_constraint(w_ind0, w_ind3, true));
            } else {
                int w_ind2 = w_var(A_r[a]);
                dynamic_LPConstraints[0].push_back(set_x_leq_y_constraint(w_ind0, w_ind2, true));
                dynamic_LPConstraints[1].push_back(set_x_leq_y_constraint(w_ind1, w_ind2, true));
            }
        }
    }
}

void LPBinaryForks_b::get_dynamic_constraints(const SPState &state,
                                               vector<SPLPConstraint *> &dyn_constr) {
    set_bounds_for_state(state);

    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();
    const SPState &abs_state = get_abstract_state(state);
    const vector<int> &doms = abs->get_variable_domains();
    int root_zero = abs_state[0];

    LPBinaryFork::get_dynamic_constraints(state, dyn_constr);

    for (int v = 1; v <= num_leafs; v++) {
        int dom_size = doms[v];
        int s_v = abs_state[v];

        for (int val_0 = 0; val_0 < dom_size; val_0++) {
            if (1 == get_sigma_lower_bound(v, val_0, root_zero))
                dyn_constr.push_back(set_d_constraint(v, val_0, s_v, 1, root_zero, false));

            int first_support = max(get_sigma_lower_bound(v, val_0, root_zero), 2);
            int last_support  = get_sigma_upper_bound(v, val_0, root_zero);
            for (int sz = first_support; sz <= last_support; sz++) {
                for (int val_1 = 0; val_1 < dom_size; val_1++) {
                    if (get_sigma_lower_bound(v, val_1, root_zero) >= sz)
                        continue;
                    dyn_constr.push_back(set_d_constraint(v, val_0, val_1, sz, root_zero, false));
                }
            }
        }
    }
}

SPLPConstraint *LPBinaryForks_b::set_d_constraint(int v, int val_0, int val_1, int sz,
                                                    int root_zero, bool tokeep) const {
    assert(sz > 0);
    int d_ind0 = d_var(v, val_0, sz);
    int p_ind  = p_var(v, val_1, val_0, (1 + root_zero + sz) % 2);

    if (sz > 1) {
        int upper_b = min(get_sigma_upper_bound(v, val_1, root_zero), sz - 1);
        int d_ind1  = d_var(v, val_1, upper_b);
        return set_x_leq_y_plus_z_constraint(d_ind0, d_ind1, p_ind, tokeep);
    }
    return set_x_eq_y_constraint(d_ind0, p_ind, tokeep);
}

SPLPConstraint *LPBinaryForks_b::set_p_constraint(int v, int val, int pre, int post,
                                                    int prv, int w_ind, bool tokeep) const {
    assert(prv != -1);
    int p_ind0 = p_var(v, val, post, prv);
    if (-1 != pre) {
        int p_ind1 = p_var(v, val, pre, prv);
        return set_x_leq_y_plus_z_constraint(p_ind0, p_ind1, w_ind, tokeep);
    }
    return set_x_leq_y_constraint(p_ind0, w_ind, tokeep);
}

SPLPConstraint *LPBinaryForks_b::set_h_constraint(int sigma, const SPState &eval_state) const {
    int root_zero = eval_state[0];
    return set_h_constraint(sigma, root_zero, false);
}

SPLPConstraint *LPBinaryForks_b::set_h_constraint(int sigma, int root_zero, bool tokeep) const {
    const Problem *abs = get_mapping()->get_abstract();
    SPLPConstraint *lpc = new SPLPConstraint(0.0, numeric_limits<double>::max(), tokeep);

    int h_ind = h_var();
    lpc->add_val(h_ind, -1.0);

    for (int v = 1; v <= num_leafs; v++) {
        int g_v   = abs->get_goal_val(v);
        int fixed = min(get_sigma_upper_bound(v, g_v, root_zero), sigma);
        assert(fixed > 0);
        int d_ind = d_var(v, g_v, fixed);
        lpc->add_val(d_ind, 1.0);
    }

    if (sigma == 1) {
        lpc->finalize();
        return lpc;
    }

    double sig = ((double)sigma - 1) / 2;
    int w_coeff0 = (int)ceil(sig);
    int w_ind0   = w_r(0);
    lpc->add_val(w_ind0, w_coeff0);
    if (sigma > 2) {
        int w_ind1   = w_r(1);
        int w_coeff1 = (int)floor(sig);
        lpc->add_val(w_ind1, w_coeff1);
    }

    lpc->finalize();
    return lpc;
}

int LPBinaryForks_b::w_v(int var, int val1, int val2, int root_val) const {
    return number_of_d_variables + number_of_p_variables + number_of_h_variables +
           number_of_w_r_variables +
           num_leafs * sigma_size * sigma_size * root_val +
           (var - 1) * sigma_size * sigma_size + val1 * sigma_size + val2;
}
