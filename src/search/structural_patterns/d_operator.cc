#include "d_operator.h"
#include "../utils/system.h"

using namespace std;

DOperator::DOperator(const Operator &op):
	Operator(op.is_axiom(),
			 op.get_prevail(),
			 op.get_pre_post(),
			 op.get_name(),
			 op.get_cost())
{
    double_cost = (double) op.get_cost();
    sort_and_remove_duplicates();
}

DOperator::DOperator(const DOperator &op):
	Operator(op.is_axiom(),
				 op.get_prevail(),
				 op.get_pre_post(),
				 op.get_name(),
				 op.get_cost())
{
    double_cost = op.get_double_cost();
    sort_and_remove_duplicates();
}

DOperator::DOperator(bool is_axiom,
                     vector<Prevail> prv,
                     vector<PrePost> pre,
                     string nm,
                     double c):
           Operator(is_axiom,prv,pre,nm,0)
{
	double_cost = c;
    sort_and_remove_duplicates();
}

void DOperator::get_var_pre_post(int v, std::vector<PrePost>& pp) const {
	const std::vector<PrePost> &pre = get_pre_post();
	assert(pp.size() == 0);
	for (size_t i=0; i < pre.size(); ++i)
		if (pre[i].var == v)
			pp.push_back(pre[i]);
}

int DOperator::get_eff_for_var_val(int v, int val) const {
	// Works only with non-conditional effects
	const std::vector<Prevail> &prv = get_prevail();
	int prv_val = get_value_for_var(v, prv);
	std::vector<PrePost> pre;
	get_var_pre_post(v,pre);
	if (pre.size() == 0)
		return -1;
	if (pre.size() > 1) {
		std::cout << "Method get_eff_for_var_val works only with non-conditional effects" << std::endl;
		exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
	}

	if (-1 == prv_val) {
		prv_val = pre[0].pre;
	}
	if (prv_val != -1 && prv_val != val)
		return -1;

	return pre[0].post;
}

void DOperator::get_explicit_pre_post_for_var(int v, std::vector<PrePost>& pp) const {
	// Creates explicit preposts by expanding the condition using prevail and other
	// preconditions and updating the respective pre value.
	assert(pp.size() == 0);
	vector<Prevail> new_prev = get_prevail();

	const vector<PrePost> &pre = get_pre_post();
	vector<PrePost> pptemp;
	for (size_t i=0; i < pre.size(); ++i) {
		if (pre[i].var == v) {
			pptemp.push_back(pre[i]);
		} else {
    		if (pre[i].pre != -1)
    			new_prev.push_back(Prevail(pre[i].var, pre[i].pre));
		}
	}
	// sorting and removing duplicates
    ::sort(new_prev.begin(), new_prev.end());
	for(int i = new_prev.size() - 1; i > 0; i--){
		if (new_prev[i] == new_prev[i-1])
			new_prev.erase(new_prev.begin()+i);
	}
	// going over all relevant conditional effects and creating an explicit effect for each
	for (size_t i=0; i < pptemp.size(); ++i) {
    	int pre = pptemp[i].pre;
    	vector<Prevail> new_cond = new_prev;
    	// going over the condition, pushing it into new condition, updating pre, if at all
    	for (size_t j=0; j < pptemp[i].cond.size(); ++j) {
    		if (pptemp[i].cond[j].var == v) {
    			assert(-1 == pre || pre == pptemp[i].cond[j].prev);
    			pre = pptemp[i].cond[j].prev;
    		} else {
    			new_cond.push_back(pptemp[i].cond[j]);
    		}
    	}
    	// sorting and removing duplicates
        ::sort(new_cond.begin(), new_cond.end());
    	for(int j = new_cond.size() - 1; j > 0; j--){
    		if (new_cond[j] == new_cond[j-1])
    			new_cond.erase(new_cond.begin()+j);
    	}
    	pp.push_back(PrePost(v,pre,pptemp[i].post,new_cond));
	}
}



int get_value_for_var(int v, const vector<Prevail>& prv) {

	for (size_t ind=0; ind < prv.size(); ++ind) {
		if (prv[ind].var == v) {
			return prv[ind].prev;
		}
	}
	return -1;
}

bool DOperator::is_redundant() const {
	const std::vector<PrePost>& pre_post = get_pre_post();
    for (size_t i=0; i < pre_post.size(); ++i) {
        if (pre_post[i].pre != pre_post[i].post)
            return false;
    }
    return (0==pre_post.size());
}

int DOperator::get_prevail_val(int v) const {
	const std::vector<Prevail>& prevail = get_prevail();
    for (size_t i=0; i < prevail.size(); ++i)
        if (prevail[i].var == v) return prevail[i].prev;
    return -1;
}

int DOperator::get_pre_val(int v) const {
	const std::vector<PrePost>& pre_post = get_pre_post();
    for (size_t i=0; i < pre_post.size(); ++i)
        if (pre_post[i].var == v) return pre_post[i].pre;
    return -1;
}

int DOperator::get_post_val(int v) const {
	const std::vector<PrePost>& pre_post = get_pre_post();
    for (size_t i=0; i < pre_post.size(); ++i)
        if (pre_post[i].var == v) return pre_post[i].post;
    return -1;
}

void DOperator::dump_SAS(ofstream& os) const {
	const std::vector<Prevail>& prevail = get_prevail();
	const std::vector<PrePost>& pre_post = get_pre_post();

    if (is_axiom()) {
        os << "begin_rule" << endl;
        dump_SAS(os, pre_post[0]);
        os << "end_rule" << endl;
    } else {
        os << "begin_operator" << endl;
        os << get_name() << endl;
        os << prevail.size() << endl;
        for (size_t i=0; i < prevail.size(); ++i){
            dump_SAS(os, prevail[i]);
        }
        os << pre_post.size() << endl;
        for (size_t i=0; i < pre_post.size(); ++i){
            dump_SAS(os, pre_post[i]);
        }

        os << get_cost() << endl;

        os << "end_operator" << endl;
    }
}

void DOperator::dump_SAS(ofstream& os, const Prevail& prv) const {
    os << prv.var << " " << prv.prev << endl;
}

void DOperator::dump_SAS(ofstream& os, const PrePost& pp) const {
    os << pp.cond.size() << endl;
    for (size_t i=0; i < pp.cond.size(); ++i){
        dump_SAS(os, pp.cond[i]);
    }
    os << pp.var << " " << pp.pre << " " << pp.post << endl;
}
