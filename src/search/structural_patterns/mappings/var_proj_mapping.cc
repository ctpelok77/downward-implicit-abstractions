#include "var_proj_mapping.h"
#include "../../utils/system.h"

using namespace std;

VarProjMapping::VarProjMapping(const Problem* orig, Problem* abs, const vector<int>& orig_vars, const vector<int>& abs_vars) {

	set_original(orig);
	set_abstract(abs);

	set_original_vars(orig_vars);
	set_abstract_vars(abs_vars);


}

VarProjMapping::VarProjMapping() {

}

VarProjMapping::~VarProjMapping() {
	// TODO Auto-generated destructor stub
}

vector<int> VarProjMapping::get_original_vars() const {
	return original_vars;
}

void VarProjMapping::set_original_vars(const vector<int>& vars){
	original_vars = vars;
}

vector<int> VarProjMapping::get_abstract_vars() const {
	return abstract_vars;
}

void VarProjMapping::set_abstract_vars(const vector<int>& vars){
	abstract_vars = vars;
}

int VarProjMapping::get_original_var(int abs) const {
	return original_vars[abs];
}

int VarProjMapping::get_abstract_var(int orig) const{
	return abstract_vars[orig];
}

int VarProjMapping::get_abstract_value(int original_variable, int original_value, int abstract_variable) const {
	assert(abstract_variable < (int)original_vars.size());
	assert(original_variable == original_vars[abstract_variable]);
	(void)(original_variable);
	(void)(abstract_variable);
	/*
	if (original_variable != original_vars[abstract_variable]) {
		cout << "VarProjMapping: "<< original_variable << " != " << original_vars[abstract_variable] << endl;
		exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
	*/
	return original_value;
}
