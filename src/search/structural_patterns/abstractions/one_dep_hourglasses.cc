#include "one_dep_hourglasses.h"
#include "k_dependence.h"
#include "hourglasses_abstraction.h"
#include "../mappings/composition_mapping.h"
#include "../mappings/op_hash_var_proj_mapping.h"
#include "../mappings/mapping.h"

using namespace std;

OneDependentHourglassesAbstraction::OneDependentHourglassesAbstraction()
		: HourglassesAbstraction(), KDependence(1) {

}

OneDependentHourglassesAbstraction::OneDependentHourglassesAbstraction(HourglassesAbstraction* f)
		: HourglassesAbstraction(), KDependence(1) {
	if (f->get_abstraction_creation_status() != DONE)
		return;

	set_mapping(f->get_mapping());
	set_abstraction_type(f->get_abstraction_type());
	abs_parents = f->get_abs_parent_vars();
	abs_leafs = f->get_abs_leaf_vars();
	abs_root_var = f->get_abstract_root();

	bool done = is_K_dependent();
	if (done) {
		cout << "The abstract problem is already 1-dependent." << endl;
	}

	if (f->is_empty_abstraction() || done) {
		set_abstraction_creation_status(DONE);
		return;
	}

	// 1-dependence according to parent distances
	OpHashVarProjMapping* hg_map =  dynamic_cast<OpHashVarProjMapping*>(get_mapping());
	const Problem* abs_p = hg_map->get_abstract();
//	cout << "HourglassesAbstraction abstract problem" << endl;
//	abs_p->dump();

//	cout << "Starting one dependence create method" << endl;
	KDependence::create(abs_p);
	OpHashMapping* kd_map =  dynamic_cast<OpHashMapping*>(get_mapping());
//	cout << "OneDependentHourglassesAbstraction Mapping pointer1: " << hg_map << endl;
//	cout << "OneDependentHourglassesAbstraction Mapping pointer2: " << kd_map << endl;

//	const Problem* abs_abs_p = kd_map->get_abstract();
//	cout << "After K-dependence: abstract problem" << endl;
//	abs_abs_p->dump();

	CompositionMapping* map = new CompositionMapping(hg_map,kd_map);
	this->set_mapping(map);
	set_abstraction_creation_status(DONE);
}

OneDependentHourglassesAbstraction::~OneDependentHourglassesAbstraction() {
//	Mapping* map = get_mapping();
//	delete map;
}

void OneDependentHourglassesAbstraction::create(const Problem* p) {
	if (get_abstraction_creation_status() == DONE)
		return;

	HourglassesAbstraction::create(p);
	if (is_empty || is_K_dependent()) {
		set_abstraction_creation_status(DONE);
		return;
	}

	OpHashVarProjMapping* hg_map =  dynamic_cast<OpHashVarProjMapping*>(this->HourglassesAbstraction::get_mapping());
	const Problem* abs_p = hg_map->get_abstract();

	KDependence::create(abs_p);
	OpHashMapping* kd_map =  dynamic_cast<OpHashMapping*>(get_mapping());

	CompositionMapping* map = new CompositionMapping(hg_map,kd_map);
	this->set_mapping(map);
	set_abstraction_creation_status(DONE);
}

