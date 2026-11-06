#include "sp_abstract_task.h"

#include "d_operator.h"

#include "../utils/system.h"

#include <cassert>
#include <cmath>

using namespace std;

SPAbstractTask::SPAbstractTask(
    const shared_ptr<AbstractTask> &parent,
    vector<string> &&variable_names,
    vector<int> &&domain_sizes,
    vector<int> &&initial_state_values,
    const vector<pair<int, int>> &goals,
    const vector<DOperator *> &ops,
    vector<int> &&parent_ops,
    vector<int> &&parent_vars,
    int abstracted_var,
    vector<int> &&value_map)
    : DelegatingTask(parent),
      variable_names(std::move(variable_names)),
      domain_sizes(std::move(domain_sizes)),
      initial_state_values(std::move(initial_state_values)),
      parent_vars(std::move(parent_vars)),
      abstracted_var(abstracted_var),
      value_map(std::move(value_map)),
      parent_ops(std::move(parent_ops)) {
    assert(this->parent_ops.size() == ops.size());
    for (const pair<int, int> &goal : goals)
        this->goals.emplace_back(goal.first, goal.second);

    operators.reserve(ops.size());
    for (const DOperator *op : ops) {
        double cost = op->get_double_cost();
        if (cost != floor(cost)) {
            cerr << "SPAbstractTask: non-integer operator cost " << cost
                 << " at construction time" << endl;
            utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
        }
        ExplicitOperator explicit_op;
        explicit_op.cost = static_cast<int>(cost);
        explicit_op.name = op->get_name();
        for (const Prevail &prv : op->get_prevail())
            explicit_op.preconditions.emplace_back(prv.var, prv.prev);
        for (const PrePost &pp : op->get_pre_post()) {
            if (pp.pre != -1)
                explicit_op.preconditions.emplace_back(pp.var, pp.pre);
            ExplicitEffect eff{FactPair(pp.var, pp.post), {}};
            for (const Prevail &cond : pp.cond)
                eff.conditions.emplace_back(cond.var, cond.prev);
            explicit_op.effects.push_back(std::move(eff));
        }
        operators.push_back(std::move(explicit_op));
    }
}

const SPAbstractTask::ExplicitOperator &SPAbstractTask::get_operator(
    int index, bool is_axiom) const {
    assert(!is_axiom);
    (void)is_axiom;
    assert(index >= 0 && index < static_cast<int>(operators.size()));
    return operators[index];
}

int SPAbstractTask::get_num_variables() const {
    return domain_sizes.size();
}

string SPAbstractTask::get_variable_name(int var) const {
    return variable_names[var];
}

int SPAbstractTask::get_variable_domain_size(int var) const {
    return domain_sizes[var];
}

int SPAbstractTask::get_variable_axiom_layer(int) const {
    return -1;
}

int SPAbstractTask::get_variable_default_axiom_value(int var) const {
    return initial_state_values[var];
}

string SPAbstractTask::get_fact_name(const FactPair &fact) const {
    if (fact.var == abstracted_var)
        return variable_names[fact.var] + "=" + to_string(fact.value);
    int var = parent_vars.empty() ? fact.var : parent_vars[fact.var];
    return parent->get_fact_name(FactPair(var, fact.value));
}

bool SPAbstractTask::are_facts_mutex(const FactPair &, const FactPair &) const {
    return false;
}

int SPAbstractTask::get_operator_cost(int index, bool is_axiom) const {
    return get_operator(index, is_axiom).cost;
}

string SPAbstractTask::get_operator_name(int index, bool is_axiom) const {
    return get_operator(index, is_axiom).name;
}

int SPAbstractTask::get_num_operators() const {
    return operators.size();
}

int SPAbstractTask::get_num_operator_preconditions(
    int index, bool is_axiom) const {
    return get_operator(index, is_axiom).preconditions.size();
}

FactPair SPAbstractTask::get_operator_precondition(
    int op_index, int fact_index, bool is_axiom) const {
    return get_operator(op_index, is_axiom).preconditions[fact_index];
}

int SPAbstractTask::get_num_operator_effects(int op_index, bool is_axiom) const {
    return get_operator(op_index, is_axiom).effects.size();
}

int SPAbstractTask::get_num_operator_effect_conditions(
    int op_index, int eff_index, bool is_axiom) const {
    return get_operator(op_index, is_axiom).effects[eff_index].conditions.size();
}

FactPair SPAbstractTask::get_operator_effect_condition(
    int op_index, int eff_index, int cond_index, bool is_axiom) const {
    return get_operator(op_index, is_axiom)
        .effects[eff_index].conditions[cond_index];
}

FactPair SPAbstractTask::get_operator_effect(
    int op_index, int eff_index, bool is_axiom) const {
    return get_operator(op_index, is_axiom).effects[eff_index].fact;
}

int SPAbstractTask::convert_operator_index_to_parent(int index) const {
    return parent_ops[index];
}

int SPAbstractTask::get_num_axioms() const {
    return 0;
}

int SPAbstractTask::get_num_goals() const {
    return goals.size();
}

FactPair SPAbstractTask::get_goal_fact(int index) const {
    return goals[index];
}

vector<int> SPAbstractTask::get_initial_state_values() const {
    return initial_state_values;
}

void SPAbstractTask::convert_state_values_from_parent(
    vector<int> &values) const {
    int num_vars = domain_sizes.size();
    vector<int> abstract_values(num_vars);
    for (int var = 0; var < num_vars; ++var) {
        int value = values[parent_vars.empty() ? var : parent_vars[var]];
        if (var == abstracted_var)
            value = value_map[value];
        abstract_values[var] = value;
    }
    values.swap(abstract_values);
}
