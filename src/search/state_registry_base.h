#ifndef STATE_REGISTRY_BASE_H
#define STATE_REGISTRY_BASE_H

#include "abstract_task.h"
#include "axioms.h"
#include "per_state_information.h"
#include "state_id.h"

#include "algorithms/int_packer.h"
#include "algorithms/segmented_vector.h"
#include "utils/hash.h"

#include <set>
#include <unordered_set>

/*
  Overview of classes relevant to storing and working with registered states.

  GlobalState
    This class is used for manipulating states.
    It contains the (uncompressed) variable values for fast access by the heuristic.
    A State is always registered in a StateRegistry and has a valid ID.
    States can be constructed from a StateRegistry by factory methods for the
    initial state and successor states.
    They never own the actual state data which is borrowed from the StateRegistry
    that created them.

  StateID
    StateIDs identify states within a state registry.
    If the registry is known, the ID is sufficient to look up the state, which
    is why IDs are intended for long term storage (e.g. in open lists).
    Internally, a StateID is just an integer, so it is cheap to store and copy.

  PackedStateBin (currently the same as unsigned int)
    The actual state data is internally represented as a PackedStateBin array.
    Each PackedStateBin can contain the values of multiple variables.
    To minimize allocation overhead, the implementation stores the data of many
    such states in a single large array (see SegmentedArrayVector).
    PackedStateBin arrays are never manipulated directly but through
    a global IntPacker object.

  -------------

  StateRegistry
    The StateRegistry allows to create states giving them an ID. IDs from
    different state registries must not be mixed.
    The StateRegistry also stores the actual state data in a memory friendly way.
    It uses the following class:

  SegmentedArrayVector<PackedStateBin>
    This class is used to store the actual (packed) state data for all states
    while avoiding dynamically allocating each state individually.
    The index within this vector corresponds to the ID of the state.

  PerStateInformation<T>
    Associates a value of type T with every state in a given StateRegistry.
    Can be thought of as a very compactly implemented map from GlobalState to T.
    References stay valid forever. Memory usage is essentially the same as a
    vector<T> whose size is the number of states in the registry.


  ---------------
  Usage example 1
  ---------------
  Problem:
    A search node contains a state together with some information about how this
    state was reached and the status of the node. The state data is already
    stored and should not be duplicated. Open lists should in theory store search
    nodes but we want to keep the amount of data stored in the open list to a
    minimum.

  Solution:

    SearchNodeInfo
      Remaining part of a search node besides the state that needs to be stored.

    SearchNode
      A SearchNode combines a StateID, a reference to a SearchNodeInfo and
      OperatorCost. It is generated for easier access and not intended for long
      term storage. The state data is only stored once an can be accessed
      through the StateID.

    SearchSpace
      The SearchSpace uses PerStateInformation<SearchNodeInfo> to map StateIDs to
      SearchNodeInfos. The open lists only have to store StateIDs which can be
      used to look up a search node in the SearchSpace on demand.

  ---------------
  Usage example 2
  ---------------
  Problem:
    In the LMcount heuristic each state should store which landmarks are
    already reached when this state is reached. This should only require
    additional memory, when the LMcount heuristic is used.

  Solution:
    The heuristic object uses a field of type PerStateInformation<std::vector<bool> >
    to store for each state and each landmark whether it was reached in this state.
*/


// For documentation on classes relevant to storing and working with registered
// states see the file state_registry.h.
template<class StateRegistryBaseType>
class StateBase {
protected:
    friend StateRegistryBaseType;
    template<class Entry, class StateType, class OperatorType>
    friend class PerStateInformation;

    // Values for vars are maintained in a packed state and accessed on demand.
    const PackedStateBin *buffer;

    // registry isn't a reference because we want to support operator=
    const StateRegistryBaseType *registry;
    StateID id;

    // Only used by the state registry.
    StateBase(
        const PackedStateBin *buffer, const StateRegistryBaseType &registry, StateID id);

    const PackedStateBin *get_packed_buffer() const {
        return buffer;
    }

    const StateRegistryBaseType &get_registry() const {
        return *registry;
    }
public:
    virtual ~StateBase() = default;

    StateID get_id() const {
        return id;
    }

    virtual int operator[](int var) const;

    virtual std::vector<int> get_values() const;

    virtual void dump_pddl() const = 0;
    virtual void dump_fdr() const = 0;
};


template<class StateType, class OperatorType>
class PerStateInformationBase;
class GlobalState;

using PackedStateBin = int_packer::IntPacker::Bin;

template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class StateRegistryBase {
    struct StateIDSemanticHash {
        const segmented_vector::SegmentedArrayVector<PackedStateBin> &state_data_pool;
        int state_size;
        StateIDSemanticHash(
            const segmented_vector::SegmentedArrayVector<PackedStateBin> &state_data_pool,
            int state_size)
            : state_data_pool(state_data_pool),
              state_size(state_size) {
        }

        size_t operator()(StateID id) const {
            const PackedStateBin *data = state_data_pool[id.value];
            utils::HashState hash_state;
            for (int i = 0; i < state_size; ++i) {
                hash_state.feed(data[i]);
            }
            return hash_state.get_hash64();
        }
    };

    struct StateIDSemanticEqual {
        const segmented_vector::SegmentedArrayVector<PackedStateBin> &state_data_pool;
        int state_size;
        StateIDSemanticEqual(
            const segmented_vector::SegmentedArrayVector<PackedStateBin> &state_data_pool,
            int state_size)
            : state_data_pool(state_data_pool),
              state_size(state_size) {
        }

        bool operator()(StateID lhs, StateID rhs) const {
            const PackedStateBin *lhs_data = state_data_pool[lhs.value];
            const PackedStateBin *rhs_data = state_data_pool[rhs.value];
            return std::equal(lhs_data, lhs_data + state_size, rhs_data);
        }
    };

    /*
      Hash set of StateIDs used to detect states that are already registered in
      this registry and find their IDs. States are compared/hashed semantically,
      i.e. the actual state data is compared, not the memory location.
    */
    using StateIDSet = std::unordered_set<StateID, StateIDSemanticHash, StateIDSemanticEqual>;

protected:
    /* TODO: The state registry still doesn't use the task interface completely.
             Fixing this is part of issue509. */
    /* TODO: AbstractTask is an implementation detail that is not supposed to
             leak. In the long run, we should store a TaskProxy here. */
    const AbstractTask &task;

    /* TODO: When we switch StateRegistry to the task interface, the next three
             members should come from the task. */
    const int_packer::IntPacker &state_packer;

    AxiomEvaluator &axiom_evaluator;
    const std::vector<int> &initial_state_data;
    const int num_variables;

    segmented_vector::SegmentedArrayVector<PackedStateBin> state_data_pool;
    StateIDSet registered_states;

	StateType *cached_initial_state;
    mutable std::set<PerStateInformationBase<StateType, OperatorType> *> subscribers;

    StateID insert_id_or_pop_state();
    int get_bins_per_state() const;
public:
    StateRegistryBase(
        const AbstractTask &task, const int_packer::IntPacker &state_packer,
        AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data);
    virtual ~StateRegistryBase();

    /* TODO: Ideally, this should return a TaskProxy. (See comment above the
             declaration of task.) */
    const AbstractTask &get_task() const {
        return task;
    }

    int get_num_variables() const {
        return num_variables;
    }

    int get_state_value(const PackedStateBin *buffer, int var) const {
        return state_packer.get(buffer, var);
    }

    /*
      Returns the state that was registered at the given ID. The ID must refer
      to a state in this registry. Do not mix IDs from from different registries.
    */
	virtual StateType lookup_state(StateID id) const;

    /*
      Returns a reference to the initial state and registers it if this was not
      done before. The result is cached internally so subsequent calls are cheap.
    */
    virtual const StateType &get_initial_state();

    /*
      Returns the state that results from applying op to predecessor and
      registers it if this was not done before. This is an expensive operation
      as it includes duplicate checking.
    */
	virtual StateType get_successor_state(const StateType &predecessor, const OperatorType &op);

    /*
      Returns the number of states registered so far.
    */
    size_t size() const {
        return registered_states.size();
    }

    int get_state_size_in_bytes() const;

    /*
      Remembers the given PerStateInformation. If this StateRegistry is
      destroyed, it notifies all subscribed PerStateInformation objects.
      The information stored in them that relates to states from this
      registry is then destroyed as well.
    */
    void subscribe(PerStateInformationBase<StateType, OperatorType> *psi) const;
    void unsubscribe(PerStateInformationBase<StateType, OperatorType> *psi) const;

    class const_iterator : public std::iterator<
                               std::forward_iterator_tag, StateID> {
        /*
          We intentionally omit parts of the forward iterator concept
          (e.g. default construction, copy assignment, post-increment)
          to reduce boilerplate. Supported compilers may complain about
          this, in which case we will add the missing methods.
        */

        friend class StateRegistryBase<StateType, OperatorType>;
        const StateRegistryBase<StateType, OperatorType> &registry;
        StateID pos;

        const_iterator(const StateRegistryBase<StateType, OperatorType> &registry, size_t start)
            : registry(registry), pos(start) {}
public:
        const_iterator &operator++() {
            ++pos.value;
            return *this;
        }

        bool operator==(const const_iterator &rhs) {
            assert(&registry == &rhs.registry);
            return pos == rhs.pos;
        }

        bool operator!=(const const_iterator &rhs) {
            return !(*this == rhs);
        }

        StateID operator*() {
            return pos;
        }

        StateID *operator->() {
            return &pos;
        }
    };

    const_iterator begin() const {
        return const_iterator(*this, 0);
    }

    const_iterator end() const {
        return const_iterator(*this, size());
    }
};

template<class StateType, class OperatorType>
StateRegistryBase<StateType, OperatorType>::StateRegistryBase(
    const AbstractTask &task, const int_packer::IntPacker &state_packer,
    AxiomEvaluator &axiom_evaluator, const std::vector<int> &initial_state_data)
    : task(task),
      state_packer(state_packer),
      axiom_evaluator(axiom_evaluator),
      initial_state_data(initial_state_data),
      num_variables(initial_state_data.size()),
      state_data_pool(get_bins_per_state()),
      registered_states(
          0,
          StateIDSemanticHash(state_data_pool, get_bins_per_state()),
          StateIDSemanticEqual(state_data_pool, get_bins_per_state())),
      cached_initial_state(0) {
}

template<class StateType, class OperatorType>
StateRegistryBase<StateType, OperatorType>::~StateRegistryBase() {
	for (auto subscriber : subscribers)
		subscriber->remove_state_registry(this);
    delete cached_initial_state;
}

template<class StateType, class OperatorType>
StateID StateRegistryBase<StateType, OperatorType>::insert_id_or_pop_state() {
    /*
      Attempt to insert a StateID for the last state of state_data_pool
      if none is present yet. If this fails (another entry for this state
      is present), we have to remove the duplicate entry from the
      state data pool.
    */
    StateID id(state_data_pool.size() - 1);
    std::pair<typename StateIDSet::iterator, bool> result = registered_states.insert(id);
    bool is_new_entry = result.second;
    if (!is_new_entry) {
        state_data_pool.pop_back();
    }
    assert(registered_states.size() == state_data_pool.size());
    return *result.first;
}

template<class StateType, class OperatorType>
const StateType &StateRegistryBase<StateType, OperatorType>::get_initial_state() {
    if (cached_initial_state == 0) {
        PackedStateBin *buffer = new PackedStateBin[get_bins_per_state()];
        // Avoid garbage values in half-full bins.
        std::fill_n(buffer, get_bins_per_state(), 0);
        for (size_t i = 0; i < initial_state_data.size(); ++i) {
            state_packer.set(buffer, i, initial_state_data[i]);
        }
        axiom_evaluator.evaluate(buffer, state_packer);
        state_data_pool.push_back(buffer);
        // buffer is copied by push_back
        delete[] buffer;
        StateID id = insert_id_or_pop_state();
        cached_initial_state = new StateType(lookup_state(id));
    }
    return *cached_initial_state;
}

template<class StateType, class OperatorType>
int StateRegistryBase<StateType, OperatorType>::get_bins_per_state() const {
    return state_packer.get_num_bins();
}

template<class StateType, class OperatorType>
int StateRegistryBase<StateType, OperatorType>::get_state_size_in_bytes() const {
    return get_bins_per_state() * sizeof(PackedStateBin);
}

template<class StateType, class OperatorType>
void StateRegistryBase<StateType, OperatorType>::subscribe(PerStateInformationBase<StateType, OperatorType> *psi) const {
    subscribers.insert(psi);
}

template<class StateType, class OperatorType>
void StateRegistryBase<StateType, OperatorType>::unsubscribe(PerStateInformationBase<StateType, OperatorType> *const psi) const {
    subscribers.erase(psi);
}


template<class StateRegistryBaseType>
StateBase<StateRegistryBaseType>::StateBase(
    const PackedStateBin *buffer, const StateRegistryBaseType &registry, StateID id)
    : buffer(buffer),
      registry(&registry),
      id(id) {
    assert(buffer);
    assert(id != StateID::no_state);
}

template<class StateRegistryBaseType>
int StateBase<StateRegistryBaseType>::operator[](int var) const {
    assert(var >= 0);
    assert(var < registry->get_num_variables());
    return registry->get_state_value(buffer, var);
}

template<class StateRegistryBaseType>
std::vector<int> StateBase<StateRegistryBaseType>::get_values() const {
    int num_variables = registry->get_num_variables();
    std::vector<int> values(num_variables);
    for (int var = 0; var < num_variables; ++var)
        values[var] = (*this)[var];
    return values;
}

#endif
