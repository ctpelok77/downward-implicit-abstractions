#include "LP_binary_fork_gen.h"
#include "../mappings/mapping.h"
#include <math.h>
#include <limits>
#include <cassert>

using namespace std;

LPBinaryFork::LPBinaryFork() : BinaryFork() {}
LPBinaryFork::LPBinaryFork(GeneralAbstraction *abs) : BinaryFork(abs) {}
LPBinaryFork::LPBinaryFork(ForksAbstraction *f, Domain *abs_domain) : BinaryFork(f, abs_domain) {}

LPBinaryFork::~LPBinaryFork() {
    free_constraints();
}

void LPBinaryFork::free_constraints() {
    SolutionMethod::free_constraints();

    for (size_t i=0; i < dynamic_LPConstraints[0].size(); ++i)
        delete dynamic_LPConstraints[0][i];
    dynamic_LPConstraints[0].clear();

    for (size_t i=0; i < dynamic_LPConstraints[1].size(); ++i)
        delete dynamic_LPConstraints[1][i];
    dynamic_LPConstraints[1].clear();
}

void LPBinaryFork::initiate() {
    BinaryFork::initiate();
    number_of_w_var_variables = get_mapping()->get_abstract()->get_actions_number();
}

void LPBinaryFork::set_static_constraints() {
    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();

    int var_num = (int)doms.size();
    for (int v = 1; v < var_num; v++) {
        int dom_size = doms[v];
        int g_v = abs->get_goal_val(v);
        const vector<DOperator *> &A_v = abs->get_var_actions(v);
        int A_v_size = (int)A_v.size();

        for (int val_0 = 0; val_0 < dom_size; val_0++) {
            for (int theta = 0; theta < doms[0]; theta++) {
                // p(v, val_0, val_0, theta) = 0
                int p_ind = p_var(v, val_0, val_0, theta);
                static_LPConstraints.push_back(set_x_eq_0_constraint(p_ind, true));

                if (1 == get_sigma_lower_bound(v, val_0, theta)) {
                    int d_ind0 = d_var(v, val_0, 1, theta);
                    int p_ind0 = p_var(v, val_0, g_v, theta);
                    static_LPConstraints.push_back(set_x_eq_y_constraint(d_ind0, p_ind0, true));
                }

                for (int val_1 = 0; val_1 < dom_size; val_1++) {
                    int first_support = max(get_sigma_lower_bound(v, val_1, theta), 2);
                    int last_support = get_sigma_upper_bound(v, val_1, theta);
                    for (int sz = first_support; sz <= last_support; sz++) {
                        if (get_sigma_lower_bound(v, val_0, 1 - theta) >= sz)
                            continue;
                        int d_ind0 = d_var(v, val_1, sz, theta);
                        int upper_b = min(get_sigma_upper_bound(v, val_0, 1 - theta), sz - 1);
                        assert(upper_b > 0);
                        int d_ind1 = d_var(v, val_0, upper_b, 1 - theta);
                        int p_ind2 = p_var(v, val_1, val_0, theta);
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
                        int p_ind0 = p_var(v, val_0, pre_v[p].post, theta);
                        if (-1 == pre_v[p].pre) {
                            static_LPConstraints.push_back(
                                set_x_leq_y_constraint(p_ind0, w_ind0, true));
                        } else {
                            int p_ind1 = p_var(v, val_0, pre_v[p].pre, theta);
                            static_LPConstraints.push_back(
                                set_x_leq_y_plus_z_constraint(p_ind0, p_ind1, w_ind0, true));
                        }
                    }
                }
            }
        }
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

void LPBinaryFork::get_dynamic_constraints(const SPState &state,
                                            vector<SPLPConstraint *> &dyn_constr) {
    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();
    const SPState &abs_state = get_abstract_state(state);

    int root_zero = abs_state[0];
    int root_goal = abs->get_goal_val(0);

    dyn_constr = dynamic_LPConstraints[root_zero];

    int lower_b = 0;
    int upper_b = 0;
    for (int v = 1; v <= num_leafs; v++) {
        int val    = abs_state[v];
        int lower_v = get_sigma_lower_bound(v, val, root_zero);
        int upper_v = get_sigma_upper_bound(v, val, root_zero);
        lower_b = (lower_b < lower_v) ? lower_v : lower_b;
        upper_b = (upper_b < upper_v) ? upper_v : upper_b;
    }

    int step = 1;
    if (-1 != root_goal) {
        step = 2;
        if (root_zero == root_goal) {
            if ((lower_b % 2) == 0) lower_b++;
            if ((upper_b % 2) == 0) upper_b++;
        } else {
            if ((lower_b % 2) == 1) lower_b++;
            if ((upper_b % 2) == 1) upper_b++;
        }
    }

    for (int sigma = lower_b; sigma <= upper_b; sigma += step)
        dyn_constr.push_back(set_h_constraint(sigma, abs_state));
}

SPLPConstraint *LPBinaryFork::set_h_constraint(int sigma, const SPState &eval_state) const {
    int root_zero = eval_state[0];
    SPLPConstraint *lpc = new SPLPConstraint(0.0, numeric_limits<double>::max(), false);

    int h_ind = h_var();
    lpc->add_val(h_ind, -1.0);

    for (int v = 1; v <= num_leafs; v++) {
        int val   = eval_state[v];
        int fixed = min(get_sigma_upper_bound(v, val, root_zero), sigma);
        assert(fixed > 0);
        int d_ind = d_var(v, val, fixed, root_zero);
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

int LPBinaryFork::w_var(DOperator *a) const {
    const Problem *abs = get_mapping()->get_abstract();
    return number_of_d_variables + number_of_p_variables + number_of_h_variables +
           number_of_w_r_variables + number_of_w_v_variables + abs->get_action_index(a);
}
