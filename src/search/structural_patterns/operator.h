#ifndef STRUCTURAL_PATTERNS_OPERATOR_H
#define STRUCTURAL_PATTERNS_OPERATOR_H

/*
  Self-contained copy of the old FD Operator/Prevail/PrePost types, used
  internally by the structural patterns code.  The globals-dependent methods
  (is_applicable, get_operator_index) are removed; they are never called from
  within structural_patterns/.
*/

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

struct Prevail {
    int var;
    int prev;
    Prevail(int v, int p) : var(v), prev(p) {}

    bool operator==(const Prevail &other) const {
        return var == other.var && prev == other.prev;
    }
    bool operator!=(const Prevail &other) const { return !(*this == other); }
    bool operator<(const Prevail &other) const {
        return var < other.var || (var == other.var && prev < other.prev);
    }
    void dump() const {
        std::cout << "[" << var << " -> " << prev << "]";
    }
};

struct PrePost {
    int var;
    int pre, post;
    std::vector<Prevail> cond;
    PrePost() {}
    PrePost(int v, int pr, int po, const std::vector<Prevail> &co)
        : var(v), pre(pr), post(po), cond(co) {}

    bool operator==(const PrePost &other) const {
        if (var != other.var || pre != other.pre || post != other.post ||
            cond.size() != other.cond.size())
            return false;
        for (size_t i = 0; i < cond.size(); ++i)
            if (cond[i] != other.cond[i]) return false;
        return true;
    }
    bool operator<(const PrePost &other) const {
        if (var != other.var) return var < other.var;
        if (post != other.post) return post < other.post;
        if (pre != other.pre) return pre < other.pre;
        if (cond.size() != other.cond.size()) return cond.size() < other.cond.size();
        for (size_t i = 0; i < cond.size(); ++i)
            if (cond[i] != other.cond[i]) return cond[i] < other.cond[i];
        return true;
    }
    void dump() const {
        std::cout << "[" << var << ": " << pre << " -> " << post << "]";
    }
};

class Operator {
    bool is_an_axiom;
    std::vector<Prevail> prevail;
    std::vector<PrePost> pre_post;
    std::string name;
    int cost;

protected:
    void sort_and_remove_duplicates() {
        std::sort(prevail.begin(), prevail.end());
        for (int i = static_cast<int>(prevail.size()) - 1; i > 0; --i)
            if (prevail[i] == prevail[i - 1])
                prevail.erase(prevail.begin() + i);
        std::sort(pre_post.begin(), pre_post.end());
        for (int i = static_cast<int>(pre_post.size()) - 1; i > 0; --i)
            if (pre_post[i] == pre_post[i - 1])
                pre_post.erase(pre_post.begin() + i);
    }

public:
    Operator(bool is_axiom,
             std::vector<Prevail> prv,
             std::vector<PrePost> prp,
             std::string nm,
             int cst)
        : is_an_axiom(is_axiom),
          prevail(std::move(prv)),
          pre_post(std::move(prp)),
          name(std::move(nm)),
          cost(cst) {}

    void dump() const {
        std::cout << name << " cost=" << cost;
    }

    std::string get_name() const { return name; }
    bool is_axiom() const { return is_an_axiom; }
    const std::vector<Prevail> &get_prevail() const { return prevail; }
    const std::vector<PrePost> &get_pre_post() const { return pre_post; }
    int get_cost() const { return cost; }

    void simplify_single_effect() {
        assert(pre_post.size() == 1);
        if (pre_post[0].cond.empty()) return;
        int var  = pre_post[0].var;
        int new_pre = pre_post[0].pre;
        int post = pre_post[0].post;
        std::vector<Prevail> emp_cond;
        std::vector<Prevail> cond = pre_post[0].cond;
        for (size_t c = 0; c < cond.size(); ++c) {
            if (var == cond[c].var) {
                assert(new_pre == -1 || new_pre == cond[c].prev);
                new_pre = cond[c].prev;
                continue;
            }
            int new_prev = -1;
            for (size_t k = 0; k < prevail.size(); ++k) {
                if (prevail[k].var == cond[c].var) {
                    new_prev = prevail[k].prev;
                    break;
                }
            }
            if (-1 == new_prev) {
                prevail.push_back(cond[c]);
                continue;
            }
            assert(new_prev == cond[c].prev);
        }
        pre_post.pop_back();
        pre_post.push_back(PrePost(var, new_pre, post, emp_cond));
    }
};

#endif /* STRUCTURAL_PATTERNS_OPERATOR_H */
