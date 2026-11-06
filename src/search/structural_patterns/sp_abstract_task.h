#ifndef STRUCTURAL_PATTERNS_SP_ABSTRACT_TASK_H
#define STRUCTURAL_PATTERNS_SP_ABSTRACT_TASK_H

#include "../tasks/delegating_task.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

class DOperator;

/*
  Abstract task of one implicit abstraction stage (projection onto a subset
  of the parent's variables, operator splitting per Def. 6, domain
  abstraction of one variable, or k-dependence), as a task derived from its
  parent.

  Variables, initial state, goals and operators are stored explicitly.
  Abstract variable i is parent variable parent_vars[i] (identity if
  parent_vars is empty); if abstracted_var >= 0, its values are mapped
  through value_map. Abstract operator i originates from parent operator
  parent_ops[i] (-1 if it originates from a parent axiom). The task has no
  axioms.
*/
class SPAbstractTask : public tasks::DelegatingTask {
    struct ExplicitEffect {
        FactPair fact;
        std::vector<FactPair> conditions;
    };

    struct ExplicitOperator {
        std::vector<FactPair> preconditions;
        std::vector<ExplicitEffect> effects;
        int cost;
        std::string name;
    };

    std::vector<std::string> variable_names;
    std::vector<int> domain_sizes;
    std::vector<int> initial_state_values;
    std::vector<FactPair> goals;
    std::vector<ExplicitOperator> operators;

    std::vector<int> parent_vars;
    int abstracted_var;
    std::vector<int> value_map;
    std::vector<int> parent_ops;

    const ExplicitOperator &get_operator(int index, bool is_axiom) const;

public:
    SPAbstractTask(
        const std::shared_ptr<AbstractTask> &parent,
        std::vector<std::string> &&variable_names,
        std::vector<int> &&domain_sizes,
        std::vector<int> &&initial_state_values,
        const std::vector<std::pair<int, int>> &goals,
        const std::vector<DOperator *> &ops,
        std::vector<int> &&parent_ops,
        std::vector<int> &&parent_vars,
        int abstracted_var = -1,
        std::vector<int> &&value_map = {});

    virtual int get_num_variables() const override;
    virtual std::string get_variable_name(int var) const override;
    virtual int get_variable_domain_size(int var) const override;
    virtual int get_variable_axiom_layer(int var) const override;
    virtual int get_variable_default_axiom_value(int var) const override;
    virtual std::string get_fact_name(const FactPair &fact) const override;
    virtual bool are_facts_mutex(
        const FactPair &fact1, const FactPair &fact2) const override;

    virtual int get_operator_cost(int index, bool is_axiom) const override;
    virtual std::string get_operator_name(
        int index, bool is_axiom) const override;
    virtual int get_num_operators() const override;
    virtual int get_num_operator_preconditions(
        int index, bool is_axiom) const override;
    virtual FactPair get_operator_precondition(
        int op_index, int fact_index, bool is_axiom) const override;
    virtual int get_num_operator_effects(
        int op_index, bool is_axiom) const override;
    virtual int get_num_operator_effect_conditions(
        int op_index, int eff_index, bool is_axiom) const override;
    virtual FactPair get_operator_effect_condition(
        int op_index, int eff_index, int cond_index,
        bool is_axiom) const override;
    virtual FactPair get_operator_effect(
        int op_index, int eff_index, bool is_axiom) const override;
    virtual int convert_operator_index_to_parent(int index) const override;

    virtual int get_num_axioms() const override;

    virtual int get_num_goals() const override;
    virtual FactPair get_goal_fact(int index) const override;

    virtual std::vector<int> get_initial_state_values() const override;
    virtual void convert_state_values_from_parent(
        std::vector<int> &values) const override;
};

#endif
