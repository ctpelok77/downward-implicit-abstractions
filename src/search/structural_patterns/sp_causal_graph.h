#ifndef STRUCTURAL_PATTERNS_SP_CAUSAL_GRAPH_H
#define STRUCTURAL_PATTERNS_SP_CAUSAL_GRAPH_H

/*
  Minimal self-contained causal graph for structural_patterns.
  Built from a vector<DOperator*> so it does not depend on any new-FD
  or old-FD global state.  Only successors and predecessors are
  maintained (pre→eff and eff→eff arcs), matching the interface used
  by Problem and SP_heuristic.
*/

#include "d_operator.h"
#include <algorithm>
#include <unordered_set>
#include <vector>

class SPCausalGraph {
    std::vector<std::vector<int>> successors_;
    std::vector<std::vector<int>> predecessors_;

    void add_arc(int from, int to) {
        if (from == to) return;
        successors_[from].push_back(to);
        predecessors_[to].push_back(from);
    }

    void finalize() {
        for (auto &v : successors_) {
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
        }
        for (auto &v : predecessors_) {
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
        }
    }

public:
    SPCausalGraph() = default;

    SPCausalGraph(int num_vars,
                  const std::vector<DOperator*> &ops,
                  const std::vector<DOperator> &axioms)
        : successors_(num_vars), predecessors_(num_vars) {
        auto process = [&](const DOperator *op) {
            const auto &prevail  = op->get_prevail();
            const auto &pre_post = op->get_pre_post();
            // pre→eff arcs (prevail → eff variables)
            for (const auto &prv : prevail)
                for (const auto &pp : pre_post)
                    add_arc(prv.var, pp.var);
            // pre→eff arcs (pre variable of prepost → eff variable)
            for (const auto &pp : pre_post)
                if (pp.pre != -1)
                    add_arc(pp.var, pp.var); // self — filtered by add_arc
            // eff→eff arcs
            for (size_t i = 0; i < pre_post.size(); ++i)
                for (size_t j = 0; j < pre_post.size(); ++j)
                    if (i != j)
                        add_arc(pre_post[i].var, pre_post[j].var);
            // condition→eff arcs for conditional effects
            for (const auto &pp : pre_post)
                for (const auto &cond : pp.cond)
                    add_arc(cond.var, pp.var);
        };
        for (const auto *op : ops) process(op);
        for (const auto &ax : axioms) process(&ax);
        finalize();
    }

    const std::vector<int> &get_successors(int var) const {
        return successors_[var];
    }
    const std::vector<int> &get_predecessors(int var) const {
        return predecessors_[var];
    }
    void dump() const {}
};

#endif /* STRUCTURAL_PATTERNS_SP_CAUSAL_GRAPH_H */
