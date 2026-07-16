#include "projection_gen.h"
#include "../mappings/mapping.h"
#include "../SP_globals.h"
#include <math.h>
#include <queue>
#include <utility>
#include <vector>

using namespace std;

Projection::Projection() {
	set_abstraction(new VariablesProjection());
}

Projection::Projection(GeneralAbstraction* abs) {
	set_abstraction(abs);
}

Projection::Projection(vector<int>& pattern) {

//	set_abstraction(new VariablesProjection(pattern));
	abstraction = new VariablesProjection(pattern);
	num_abs_vars = pattern.size();

}

Projection::~Projection() {
	// TODO Auto-generated destructor stub
}

void Projection::initiate() {
	// No caching in projection
	use_cache = false;
	const Problem* abs = get_mapping()->get_abstract();
	const vector<int> &doms = abs->get_variable_domains();
	num_vars = abs->get_vars_number();

	int num_states = 1;
	for (size_t i=0; i < doms.size(); ++i) {
		multiplier.push_back(num_states);
		num_states *= doms[i];
	}
	set_solution(new Solution());

	number_of_d_variables = num_states;
	number_of_h_variables = 1;
	number_of_w_var_variables = 0;

}


// ---------------------------------------------------------------------------
// Helper: enumerate all ground states consistent with a partial state
// template (free variables = -1), calling `callback` for each.
// ---------------------------------------------------------------------------
static void enumerate_ground_states(
        vector<int>& state,
        vector<int>& free_vars,
        const vector<int>& doms,
        const function<void(const vector<int>&)>& callback) {
    if (free_vars.empty()) {
        callback(state);
        return;
    }
    int fv = free_vars.back();
    free_vars.pop_back();
    for (int v = 0; v < doms[fv]; ++v) {
        state[fv] = v;
        enumerate_ground_states(state, free_vars, doms, callback);
    }
    free_vars.push_back(fv);
}

void Projection::solve() {
    // Backward Dijkstra on the abstract state space.
    // For each state s, dist[s] = min cost to reach a goal from s.

    const Problem* abs = get_mapping()->get_abstract();
    const vector<DOperator*>& ops = abs->get_operators();
    const vector<int>& doms = abs->get_variable_domains();
    int num_states = number_of_d_variables;

    // --- Build reverse adjacency list ---
    // predecessors[succ] = list of (pred_rank, edge_cost)
    vector<vector<pair<int, double>>> predecessors(num_states);

    for (size_t it = 0; it < ops.size(); ++it) {
        const DOperator* op = ops[it];
        if (op->is_redundant())
            continue;
        double cost = op->get_double_cost();
        const vector<PrePost>& pre_post = op->get_pre_post();

        // Build applicable-state template: -1 = free, >=0 = required value
        vector<int> tmpl(num_vars, -1);
        for (const Prevail& prv : op->get_prevail())
            tmpl[prv.var] = prv.prev;
        for (const PrePost& pp : pre_post)
            if (pp.pre >= 0)
                tmpl[pp.var] = pp.pre;

        // Collect free variables in the template
        vector<int> free_vars;
        for (int i = 0; i < num_vars; ++i)
            if (tmpl[i] < 0)
                free_vars.push_back(i);

        // For each ground applicable state, compute successor and record edge
        enumerate_ground_states(tmpl, free_vars, doms,
                [&](const vector<int>& state) {
            vector<int> succ = state;
            for (const PrePost& pp : pre_post)
                succ[pp.var] = pp.post;
            int pred_rank = d_var(state);
            int succ_rank = d_var(succ);
            if (pred_rank != succ_rank)
                predecessors[succ_rank].emplace_back(pred_rank, cost);
        });
    }

    // --- Seed: all abstract goal states at dist = 0 ---
    vector<double> dist(num_states, infinity);
    // min-heap: (dist, state_rank)
    priority_queue<pair<double,int>,
                   vector<pair<double,int>>,
                   greater<pair<double,int>>> pq;

    vector<int> goals;
    abs->get_goal_vals(goals);  // goals[i] = required value, -1 = free
    vector<int> goal_free;
    for (int i = 0; i < num_vars; ++i)
        if (goals[i] < 0)
            goal_free.push_back(i);

    enumerate_ground_states(goals, goal_free, doms,
            [&](const vector<int>& state) {
        int rank = d_var(state);
        if (dist[rank] > 0.0) {
            dist[rank] = 0.0;
            pq.emplace(0.0, rank);
        }
    });

    // --- Dijkstra ---
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u])
            continue;  // stale entry
        for (auto [pred, cost] : predecessors[u]) {
            double nd = dist[u] + cost;
            if (nd < dist[pred]) {
                dist[pred] = nd;
                pq.emplace(nd, pred);
            }
        }
    }

    // --- Store results ---
    solution->clear_solution();
    for (int k = 0; k < num_states; ++k)
        solution->set_value(k, dist[k]);
}

double Projection::get_solution_value(const SPState &state) {

	const SPState &abs_state = get_abstract_state(state);
	double sol = solution->get_value(d_var(abs_state));
	return sol;

}



int Projection::d_var(const SPState &state) const {

	int ret = 0;
	for (int i = 0; i < num_vars; i++) {
		int s = state[i];
		int m = multiplier[i];
		ret += s*m;
//		cout << " " << s;
	}
//	cout << " = x_" << ret << endl;
	return ret;
}



int Projection::d_var(vector<int>& state) const {
	int ret = 0;
	for (size_t i=0; i < state.size(); ++i) {
		ret += state[i]*multiplier[i];
//		cout << " " << state[i];
	}
//	cout << " = x_" << ret << endl;
	return ret;
}


int Projection::get_num_vars() const {
	return number_of_d_variables + number_of_h_variables + number_of_w_var_variables;
}

