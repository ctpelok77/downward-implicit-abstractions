#ifndef STRUCTURAL_PATTERNS_D_OPERATOR_H
#define STRUCTURAL_PATTERNS_D_OPERATOR_H

#include "operator.h"
#include <algorithm>
#include <set>
#include <fstream>


int get_value_for_var(int v, const std::vector<Prevail>& prv);

class DOperator: public Operator {
    int index;
    double double_cost;
public:
    DOperator(const Operator &op);
    DOperator(const DOperator &op);
    DOperator& operator=(const DOperator&) = default;
    DOperator(bool is_axiom,
             std::vector<Prevail> prv,
             std::vector<PrePost> pre,
             std::string nm,
             double c);

    double get_double_cost() const {return double_cost;}
    void set_double_cost(double c) {double_cost=c;}
    int get_index() const {return index;}
    void set_index(int ind) {index = ind;}

    void get_var_pre_post(int v, std::vector<PrePost>& pp) const;
    int get_eff_for_var_val(int v, int val) const;
    void get_explicit_pre_post_for_var(int v, std::vector<PrePost>& pp) const;
    bool is_single_effect() { return (get_pre_post().size() == 1);}
    bool is_redundant() const;
    int get_prevail_val(int v) const;
    int get_pre_val(int v) const;
    int get_post_val(int v) const;
    void dump_SAS(std::ofstream& os) const;
    void dump_SAS(std::ofstream& os, const Prevail& prv) const;
    void dump_SAS(std::ofstream& os, const PrePost& pp) const;
};


#endif
