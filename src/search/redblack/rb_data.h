#ifndef REDBLACK_RB_DATA_H
#define REDBLACK_RB_DATA_H

#include "painting.h"
#include "int_packer.h"
#include "state_registry.h"
#include "../globals.h"

namespace redblack {
struct RBData {
	Painting painting;
	RBIntPacker int_packer;

	RBData(const Painting &painting) :
		painting(painting),
		int_packer(this->painting) {
		int_packer.initialize(g_variable_domain);
	}

	auto construct_state_registry(const std::vector<int> &initial_state_data) const -> std::unique_ptr<RBStateRegistry> {
		return std::make_unique<RBStateRegistry>(*g_root_task(), int_packer, *g_axiom_evaluator, initial_state_data);
	}
};
}

#endif
