#include "LP_bounded_iforks_off.h"
#include "../SP_globals.h"
#include "../mappings/mapping.h"
#include "../SP_LPConstraint.h"
#include <limits>
#include <cassert>

using namespace std;

LPBoundedIforks_OFF::LPBoundedIforks_OFF() : LPBoundedIfork() {}
LPBoundedIforks_OFF::LPBoundedIforks_OFF(GeneralAbstraction *abs) : LPBoundedIfork(abs) {}
LPBoundedIforks_OFF::LPBoundedIforks_OFF(IforksAbstraction *ifork, Domain *abs_domain)
    : LPBoundedIfork(ifork, abs_domain) {}
LPBoundedIforks_OFF::~LPBoundedIforks_OFF() {}

void LPBoundedIforks_OFF::set_objective() {
    const Problem *abs = get_mapping()->get_abstract();
    const vector<int> &doms = abs->get_variable_domains();
    obj_func = new SPLPConstraint();

    int var_num = (int)doms.size();
    for (int v = 1; v < var_num; v++) {
        int dom_size = doms[v];
        for (int val_0 = 0; val_0 < dom_size; val_0++) {
            for (int val_1 = 0; val_1 < dom_size; val_1++) {
                if (val_0 != val_1) {
                    int d_ind = d_var(v, val_0, val_1);
                    obj_func->add_val(d_ind, 1.0);
                }
            }
        }
    }
    obj_func->finalize();
}

void LPBoundedIforks_OFF::set_static_constraints() {
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
                double c = A_v[a]->get_double_cost();
                vector<PrePost> pre_v;
                A_v[a]->get_explicit_pre_post_for_var(v, pre_v);
                assert((int)pre_v.size() > 0);
                for (size_t p=0; p < pre_v.size(); ++p) {
                    assert(pre_v[p].cond.size() == 0);
                    int pre  = pre_v[p].pre;
                    int post = pre_v[p].post;
                    if (pre == post)
                        continue;

                    SPLPConstraint *lpc1;
                    if (-1 == pre) {
                        lpc1 = new SPLPConstraint(0.0, c, true);
                    } else {
                        lpc1 = new SPLPConstraint(-numeric_limits<double>::max(), c, true);
                        int d_ind1 = d_var(v, val_0, pre);
                        lpc1->add_val(d_ind1, -1.0);
                    }
                    int d_ind2 = d_var(v, val_0, post);
                    lpc1->add_val(d_ind2, 1.0);
                    lpc1->finalize();
                    static_LPConstraints.push_back(lpc1);
                }
            }
        }
    }
}
