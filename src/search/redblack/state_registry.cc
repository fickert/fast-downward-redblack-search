#include "state_registry.h"
#include "state.h"
#include "operator.h"
#include "../tasks/cost_adapted_task.h"
#include "../globals.h"

auto StateRegistryBase<redblack::RBState, redblack::RBOperator>::lookup_state(StateID) const -> redblack::RBState {
	assert(false && "implemented in child class");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}

auto StateRegistryBase<redblack::RBState, redblack::RBOperator>::get_successor_state(const redblack::RBState &, const redblack::RBOperator &) -> redblack::RBState {
	assert(false && "implemented in child class");
	utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
}


namespace redblack {

RBStateRegistry::RBStateRegistry(const AbstractTask &task, const RBIntPacker &state_packer,
	                             AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data,
	                             const std::vector<RBOperator> &operators, PackedStateBin *rb_initial_state_data)
	: StateRegistryBase<RBState, RBOperator>(task, state_packer, axiom_evaluator, initial_state_data),
	  painting(&state_packer.get_painting()),
	  initial_state_best_supporters(),
	  cached_best_supporters(task.get_num_variables()),
	  operators(operators) {
	if (rb_initial_state_data) {
		// TODO: make sure the passed initial state data matches the painting
		state_data_pool.push_back(rb_initial_state_data);
		StateID id = insert_id_or_pop_state();
		cached_initial_state = new RBState(lookup_state(id));
		// TODO: need initial state best supporters too?
	}
}

RBStateRegistry::~RBStateRegistry() {}

void RBStateRegistry::saturate_state(PackedStateBin *buffer, bool store_best_supporters) const {
	std::vector<std::vector<int>> lowest_cost(task.get_num_variables());

	for (size_t var = 0; var < task.get_num_variables(); ++var) {
		if (painting->is_red_var(var)) {
			lowest_cost[var].resize(task.get_variable_domain_size(var), -1);
			for (int val = 0; val < task.get_variable_domain_size(var); ++val)
				if (rb_state_packer().get_bit(buffer, var, val))
					lowest_cost[var][val] = 0;
			if (store_best_supporters)
				std::vector<OperatorID>(task.get_variable_domain_size(var), OperatorID(-1)).swap(cached_best_supporters[var]);
		}
	}

	bool change = true;
	while (change) {
		change = false;

		std::vector<OperatorID> applicable_ops;
		// Note: need to use a dummy id here
		g_successor_generator->generate_applicable_ops(RBState(buffer, *this, StateID(0), *painting, rb_state_packer()), applicable_ops, false);

		for (auto op_id : applicable_ops) {
			const auto &op = operators[op_id.get_index()];
#ifndef NDEBUG
			bool is_applicable = true;
			for (auto const &pre : op.get_black_preconditions()) {
				if (rb_state_packer().get(buffer, pre->var) != pre->val) {
					is_applicable = false;
					break;
				}
			}
			assert(is_applicable);
#endif
			bool changes_black = false;
			if (op.is_black()) {
				const auto &black_effs = op.get_black_effects();
				for (size_t eff = 0; eff < black_effs.size(); ++eff) {
					int var = black_effs[eff]->var;
					int val = black_effs[eff]->val;

					if (val != rb_state_packer().get(buffer, var)) {
						changes_black = true;
						break;
					}
				}
			}
			if (!changes_black) {
				int max_pre_cost = 0;
				const auto &red_pre = op.get_red_preconditions();
				for (size_t pre = 0; pre < red_pre.size(); ++pre) {
					int var = red_pre[pre]->var;
					int val = red_pre[pre]->val;

					assert(lowest_cost[var][val] >= 0);
					max_pre_cost = std::max(max_pre_cost, lowest_cost[var][val]);
				}

				max_pre_cost += task.get_operator_cost(op.get_id().get_index(), false); //get_adjusted_action_cost(*op, cost_type)

				const auto &red_effs = op.get_red_effects();
				for (size_t eff = 0; eff < red_effs.size(); ++eff) {
					int var = red_effs[eff]->var;
					int val = red_effs[eff]->val;
					// TODO only need to check for cheaper paths when storing best supporters
					if (lowest_cost[var][val] == -1 || max_pre_cost < lowest_cost[var][val]) {
						rb_state_packer().set_bit(buffer, var, val);
						lowest_cost[var][val] = max_pre_cost;
						change = true;
						if (store_best_supporters)
							cached_best_supporters[var][val] = op.get_id();
					}
				}
			}
		}
	}
}

auto RBStateRegistry::lookup_state(StateID id) const -> RBState {
	return RBState(state_data_pool[id.value], *this, id, *painting, rb_state_packer());
}

auto RBStateRegistry::get_initial_state() -> const RBState & {
	if (!cached_initial_state) {
		auto buffer = new PackedStateBin[get_bins_per_state()];
		// Avoid garbage values in half-full bins.
		std::fill_n(buffer, get_bins_per_state(), 0);
		for (size_t i = 0; i < initial_state_data.size(); ++i) {
			if (painting->is_red_var(i)) {
				rb_state_packer().init_zero(buffer, i);
				rb_state_packer().set_bit(buffer, i, initial_state_data[i]);
			} else {
				rb_state_packer().set(buffer, i, initial_state_data[i]);
			}
		}
		saturate_state(buffer, true);
		initial_state_best_supporters = get_stored_best_supporters();
		axiom_evaluator.evaluate(buffer, state_packer);
		state_data_pool.push_back(buffer);
		// buffer is copied by push_back
		delete[] buffer;
		StateID id = insert_id_or_pop_state();
		cached_initial_state = new RBState(lookup_state(id));
	}
	return *cached_initial_state;
}

auto RBStateRegistry::get_successor_state(const RBState &predecessor, const RBOperator &op) -> RBState {
	auto effect_does_fire = [](const auto &effect, const auto &state) {
		return std::all_of(std::begin(effect.conditions), std::end(effect.conditions), [&state](const auto &condition) {
			return state.has_fact(condition.var, condition.val);
		});
	};
	assert(op.is_applicable(predecessor));
	assert(!op.get_base_operator().is_axiom());
	assert(op.is_black());
	state_data_pool.push_back(predecessor.get_packed_buffer());
	PackedStateBin *buffer = state_data_pool[state_data_pool.size() - 1];
	for (const auto &effect : op.get_base_operator().get_effects()) {
		if (effect_does_fire(effect, predecessor)) {
			if (painting->is_black_var(effect.var))
				rb_state_packer().set(buffer, effect.var, effect.val);
			else
				rb_state_packer().set_bit(buffer, effect.var, effect.val);
		}
	}
	saturate_state(buffer);
	axiom_evaluator.evaluate(buffer, state_packer);
	auto id = insert_id_or_pop_state();
	return lookup_state(id);
}

}
