#include "LP_bounded_ifork_gen.h"
#include "../mappings/mapping.h"
#include <math.h>
#include "../SP_LPConstraint.h"
#include <limits>
#include <cassert>

using namespace std;

LPBoundedIfork::LPBoundedIfork() : BoundedIfork() {}
LPBoundedIfork::LPBoundedIfork(GeneralAbstraction *abs) : BoundedIfork(abs) {}
LPBoundedIfork::LPBoundedIfork(IforksAbstraction *ifork, Domain *abs_domain)
    : BoundedIfork(ifork, abs_domain) {}
LPBoundedIfork::~LPBoundedIfork() {}

void LPBoundedIfork::initiate() {
    BoundedIfork::initiate();

    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();

    number_of_h_variables    = 1;
    number_of_w_var_variables = abs->get_actions_number();

    int root_dom = doms[0];

    vector<vector<DOperator *>> paths;
    abs->get_all_cycle_free_paths_to_goal(0, paths);
    for (size_t i=0; i < paths.size(); ++i) {
        IforkRootPath *new_path = new IforkRootPath((int)doms.size());
        new_path->set_path(paths[i]);
        precalculate_path(new_path);
        vector<int> vals;
        new_path->get_applicable_vals(root_dom, vals);
        assert(!vals.empty());
        root_paths_values[vals[0]].push_back(new_path);
        for (size_t j=1; j < vals.size(); ++j)
            root_paths_values[vals[j]].push_back(new IforkRootPath(new_path));
    }

    vector<DOperator *> emp_path;
    IforkRootPath *empty_path = new IforkRootPath((int)doms.size());
    empty_path->set_path(emp_path);
    precalculate_path(empty_path);

    int root_goal = abs->get_goal_val(0);
    root_paths_values[root_goal].push_back(empty_path);
}

void LPBoundedIfork::precalculate_path(IforkRootPath *path) {
    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();

    SPLPConstraint *lpc0 = new SPLPConstraint(0.0, numeric_limits<double>::max(), true);
    int h_ind = h_var();
    lpc0->add_val(h_ind, -1.0);

    int num_vars = (int)doms.size();
    for (int v = 1; v < num_vars; v++) {
        vector<int> supp;
        path->get_path_support(v, supp);
        int g_v = abs->get_goal_val(v);
        if (-1 != g_v)
            supp.push_back(g_v);
        if (!supp.empty()) {
            path->set_first_needed(v, supp[0]);
            for (size_t i=1; i < supp.size(); ++i) {
                int d_ind = d_var(v, supp[i - 1], supp[i]);
                lpc0->add_val(d_ind, 1.0);
            }
        }
    }

    const vector<DOperator *> &ops = path->get_path();
    for (size_t i=0; i < ops.size(); ++i) {
        int w_ind = w_var(ops[i]);
        lpc0->add_val(w_ind, 1.0);
    }
    path->set_LP_constraint(lpc0);
}

void LPBoundedIfork::set_static_constraints() {
    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();

    int var_num = (int)doms.size();
    for (int v = 1; v < var_num; v++) {
        int dom_size = doms[v];
        const vector<DOperator *> &A_v = abs->get_var_actions(v);
        int A_v_size = (int)A_v.size();

        for (int val_0 = 0; val_0 < dom_size; val_0++) {
            int d_ind0 = d_var(v, val_0, val_0);
            static_LPConstraints.push_back(set_x_eq_0_constraint(d_ind0, true));

            for (int a = 0; a < A_v_size; a++) {
                vector<PrePost> pre_v;
                A_v[a]->get_explicit_pre_post_for_var(v, pre_v);
                assert((int)pre_v.size() > 0);
                for (size_t p=0; p < pre_v.size(); ++p) {
                    assert(pre_v[p].cond.size() == 0);
                    int pre  = pre_v[p].pre;
                    int post = pre_v[p].post;
                    if (pre == post)
                        continue;
                    int w_ind0 = w_var(A_v[a]);
                    int d_ind1 = d_var(v, val_0, post);
                    if (-1 != pre) {
                        int d_ind2 = d_var(v, val_0, pre);
                        static_LPConstraints.push_back(
                            set_x_leq_y_plus_z_constraint(d_ind1, d_ind2, w_ind0, true));
                    } else {
                        static_LPConstraints.push_back(
                            set_x_leq_y_constraint(d_ind1, w_ind0, true));
                    }
                }
            }
        }
    }
}

void LPBoundedIfork::get_dynamic_constraints(const SPState &state,
                                              vector<SPLPConstraint *> &dyn_constr) {
    Mapping *map = get_mapping();
    const Problem *abs = map->get_abstract();
    const SPState &abs_state = get_abstract_state(state);
    const vector<int> &doms = abs->get_variable_domains();
    int root_zero = abs_state[0];
    int num_vars  = (int)doms.size();

    for (size_t p=0; p < root_paths_values[root_zero].size(); ++p) {
        IforkRootPath *path = root_paths_values[root_zero][p];
        SPLPConstraint *lpc0 = new SPLPConstraint(path->get_LP_constraint(), false);

        for (int v = 1; v < num_vars; v++) {
            int from = abs_state[v];
            int to   = path->get_first_needed(v);
            if ((-1 != to) && (from != to)) {
                int d_ind = d_var(v, from, to);
                lpc0->add_val(d_ind, 1.0);
            }
        }
        lpc0->finalize();
        dyn_constr.push_back(lpc0);
    }
}

int LPBoundedIfork::h_var() const {
    return number_of_d_variables;
}

int LPBoundedIfork::w_var(DOperator *a) const {
    const Problem *abs = get_mapping()->get_abstract();
    return number_of_d_variables + number_of_h_variables + abs->get_action_index(a);
}
