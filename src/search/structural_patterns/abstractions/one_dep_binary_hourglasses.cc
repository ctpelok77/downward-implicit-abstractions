#include "one_dep_binary_hourglasses.h"
#include "k_dependence.h"

#include "domain_abstraction.h"
#include "domain.h"
#include "../mappings/composition_mapping.h"
#include "../mappings/op_hash_var_proj_mapping.h"
#include "../mappings/op_hash_dom_abs_mapping.h"
#include "../mappings/mapping.h"

using namespace std;

OneDependentBinaryHourglassesAbstraction::OneDependentBinaryHourglassesAbstraction()
		: OneDependentHourglassesAbstraction() {

}

OneDependentBinaryHourglassesAbstraction::OneDependentBinaryHourglassesAbstraction(Domain* new_dom)
		: OneDependentHourglassesAbstraction() {
	set_root_var_and_domain(new_dom);
}

OneDependentBinaryHourglassesAbstraction::OneDependentBinaryHourglassesAbstraction(
		OneDependentHourglassesAbstraction* f, Domain* new_dom) : OneDependentHourglassesAbstraction() {
//	cout << "Starting binarization" << endl;
	set_root_var_and_domain(new_dom);
	if (f->get_abstraction_creation_status() != DONE) {
		cout << "OneDependentHourglassesAbstraction is not created properly" << endl;
		return;
	}

	this->set_mapping(f->get_mapping());
	set_abstraction_type(f->get_abstraction_type());
	abs_parents = f->get_abs_parent_vars();
	abs_leafs = f->get_abs_leaf_vars();
	abs_root_var = f->get_abstract_root();

	if (f->is_empty_abstraction() || 2 == dom->get_old_size()) {
		set_abstraction_creation_status(DONE);
		return;
	}

	// Binarization according to new_dom
//	OpHashVarProjMapping* hg_map =  dynamic_cast<OpHashVarProjMapping*>(this->get_mapping());
	CompositionMapping* hg_map =  dynamic_cast<CompositionMapping*>(f->get_mapping());

	const Problem* abs_p = hg_map->get_abstract();
//	cout << "OneDependentHourglassesAbstraction abstract problem" << endl;
//	abs_p->dump();
	int new_var = hg_map->get_abstract_var(dom->get_var());
//	cout << "New abstract var: " << new_var << endl;
	dom->update_var(new_var);

	DomainAbstraction bin(dom);
//	cout << "Starting creation for binarization" << endl;
	bin.create(abs_p);
//	cout << "Ended creation for binarization" << endl;
	OpHashDomAbsMapping* bin_map =  dynamic_cast<OpHashDomAbsMapping*>(bin.get_mapping());

//	cout << "Mapping pointer1: " << hg_map << endl;
//	cout << "Mapping pointer2: " << bin_map << endl;

	CompositionMapping* map = new CompositionMapping(hg_map,bin_map);
	set_mapping(map);
	set_abstraction_creation_status(DONE);
}

OneDependentBinaryHourglassesAbstraction::~OneDependentBinaryHourglassesAbstraction() {
//	Mapping* map = get_mapping();
//	delete map;
}

void OneDependentBinaryHourglassesAbstraction::set_root_var_and_domain(Domain* new_dom){
	dom = new_dom;

}

void OneDependentBinaryHourglassesAbstraction::create(const Problem* p) {
	if (get_abstraction_creation_status() == DONE)
		return;

	OneDependentHourglassesAbstraction::create(p);
	if (is_empty)
		return;
	if (2 == dom->get_old_size())
		return;
	OpHashVarProjMapping* hg_map =  dynamic_cast<OpHashVarProjMapping*>(this->get_mapping());
	const Problem* abs_p = hg_map->get_abstract();

	int new_var = hg_map->get_abstract_var(dom->get_var());
	dom->update_var(new_var);

	DomainAbstraction bin(dom);
	bin.create(abs_p);
	Mapping* map = new CompositionMapping(hg_map,bin.get_mapping());
	this->set_mapping(map);
	set_abstraction_creation_status(DONE);
}

