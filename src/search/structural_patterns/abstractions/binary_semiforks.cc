#include "binary_semiforks.h"
#include "domain_abstraction.h"
#include "domain.h"
#include "../mappings/composition_mapping.h"
#include "../mappings/op_hash_var_proj_mapping.h"
#include "../mappings/mapping.h"

using namespace std;


BinarySemiForksAbstraction::BinarySemiForksAbstraction() : SemiForksAbstraction() {

}

//BinarySemiForksAbstraction::BinarySemiForksAbstraction(Domain* new_dom, vector<int>& hat_vars) : SemiForksAbstraction(new_dom->get_var(), hat_vars) {
BinarySemiForksAbstraction::BinarySemiForksAbstraction(Domain* new_dom) : SemiForksAbstraction() {
	set_root_var_and_domain(new_dom);
}

//BinarySemiForksAbstraction::BinarySemiForksAbstraction(SemiForksAbstraction* f, Domain* new_dom, vector<int>& hat_vars) : SemiForksAbstraction(new_dom->get_var(), hat_vars) {
BinarySemiForksAbstraction::BinarySemiForksAbstraction(SemiForksAbstraction* f, Domain* new_dom)
		: SemiForksAbstraction() {
	set_root_var_and_domain(new_dom);
	if (f->get_abstraction_creation_status() != DONE)
		return;

	set_mapping(f->get_mapping());
	set_abstraction_type(f->get_abstraction_type());
	abs_hat = f->get_abs_hat_vars();
	abs_leafs = f->get_abs_leaf_vars();
	abs_root_var = f->get_abstract_root();
	if (f->is_empty_abstraction())
		return;
	if (2 == dom->get_old_size())
		return;
	// Binarization according to new_dom
	OpHashVarProjMapping* fork_map =  dynamic_cast<OpHashVarProjMapping*>(get_mapping());
	const Problem* abs_p = fork_map->get_abstract();

	int new_var = fork_map->get_abstract_var(dom->get_var());
	dom->update_var(new_var);

	DomainAbstraction bin(dom);
	bin.create(abs_p);
	Mapping* map = new CompositionMapping(fork_map,bin.get_mapping());
	set_mapping(map);
	set_abstraction_creation_status(DONE);

}

BinarySemiForksAbstraction::~BinarySemiForksAbstraction() {
//	Mapping* map = get_mapping();
//	delete map;
}

void BinarySemiForksAbstraction::set_root_var_and_domain(Domain* new_dom){
	dom = new_dom;

}

void BinarySemiForksAbstraction::create(const Problem* p) {
//	SemiForksAbstraction::set_variable(dom->get_var());
	if (get_abstraction_creation_status() == DONE)
		return;
	SemiForksAbstraction::create(p);
	if (is_empty)
		return;
	if (2 == dom->get_old_size())
		return;
	OpHashVarProjMapping* semifork_map =  dynamic_cast<OpHashVarProjMapping*>(this->get_mapping());
	const Problem* abs_p = semifork_map->get_abstract();

	int new_var = semifork_map->get_abstract_var(dom->get_var());
	dom->update_var(new_var);

	DomainAbstraction bin(dom);
	bin.create(abs_p);
	Mapping* map = new CompositionMapping(semifork_map,bin.get_mapping());
	this->set_mapping(map);
	set_abstraction_creation_status(DONE);
}

