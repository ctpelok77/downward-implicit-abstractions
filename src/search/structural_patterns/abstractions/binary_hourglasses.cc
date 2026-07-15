#include "binary_hourglasses.h"
#include "domain_abstraction.h"
#include "domain.h"
#include "../mappings/composition_mapping.h"
#include "../mappings/op_hash_var_proj_mapping.h"
#include "../mappings/mapping.h"

using namespace std;


BinaryHourglassesAbstraction::BinaryHourglassesAbstraction() : HourglassesAbstraction() {

}

BinaryHourglassesAbstraction::BinaryHourglassesAbstraction(Domain* new_dom) : HourglassesAbstraction() {
	set_root_var_and_domain(new_dom);
}

BinaryHourglassesAbstraction::BinaryHourglassesAbstraction(HourglassesAbstraction* f, Domain* new_dom) : HourglassesAbstraction() {
	set_root_var_and_domain(new_dom);
	if (f->get_abstraction_creation_status() != DONE)
		return;
	set_mapping(f->get_mapping());
	set_abstraction_type(f->get_abstraction_type());
	abs_parents = f->get_abs_parent_vars();
	abs_leafs = f->get_abs_leaf_vars();
	abs_root_var = f->get_abstract_root();

	if (f->is_empty_abstraction())
		return;
	if (2 == dom->get_old_size())
		return;
	// Binarization according to new_dom
	OpHashVarProjMapping* hg_map =  dynamic_cast<OpHashVarProjMapping*>(get_mapping());
	const Problem* abs_p = hg_map->get_abstract();

	int new_var = hg_map->get_abstract_var(dom->get_var());
	dom->update_var(new_var);

	DomainAbstraction bin(dom);
	bin.create(abs_p);
	Mapping* map = new CompositionMapping(hg_map,bin.get_mapping());
	set_mapping(map);
	set_abstraction_creation_status(DONE);
}

BinaryHourglassesAbstraction::~BinaryHourglassesAbstraction() {
//	Mapping* map = get_mapping();
//	delete map;
}

void BinaryHourglassesAbstraction::set_root_var_and_domain(Domain* new_dom){
	dom = new_dom;

}

void BinaryHourglassesAbstraction::create(const Problem* p) {
	if (get_abstraction_creation_status() == DONE)
		return;

	HourglassesAbstraction::create(p);
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

