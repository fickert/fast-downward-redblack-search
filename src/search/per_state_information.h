#ifndef PER_STATE_INFORMATION_H
#define PER_STATE_INFORMATION_H

#include "algorithms/segmented_vector.h"
#include "utils/collections.h"

#include <cassert>
#include <unordered_map>

template<class StateType, class OperatorType>
class StateRegistryBase;
class GlobalState;
class GlobalOperator;

template<class StateType = GlobalState, class OperatorType = GlobalOperator>
class PerStateInformationBase {
    using StateRegistryType = StateRegistryBase<StateType, OperatorType>;
    friend StateRegistryType;
    virtual void remove_state_registry(StateRegistryType *registry) = 0;
public:
    PerStateInformationBase() {
    }
    virtual ~PerStateInformationBase() {}
};

/*
  PerStateInformation is used to associate information with states.
  PerStateInformation<Entry> logically behaves somewhat like an unordered map
  from states to objects of class Entry. However, lookup of unknown states is
  supported and leads to insertion of a default value (similar to the
  defaultdict class in Python).

  For example, landmark heuristics can use it to store the set of landmarks
  with every state reached during search, or search algorithms can use it to
  associate g values or creating operators with a state.

  Implementation notes: PerStateInformation is essentially implemented as a
  kind of two-level map:
    1. Find the correct SegmentedVector for the registry of the given state.
    2. Look up the associated entry in the SegmentedVector based on the ID of
       the state.
  It is common in many use cases that we look up information for states from
  the same registry in sequence. Therefore, to make step 1. more efficient, we
  remember (in "cached_registry" and "cached_entries") the results of the
  previous lookup and reuse it on consecutive lookups for the same registry.

  A PerStateInformation object subscribes to every StateRegistry for which it
  stores information. Once a StateRegistry is destroyed, it notifies all
  subscribed objects, which in turn destroy all information stored for states
  in that registry.
*/
template<class Entry, class StateType = GlobalState, class OperatorType = GlobalOperator>
class PerStateInformation : public PerStateInformationBase<StateType, OperatorType> {
    const Entry default_value;
    typedef std::unordered_map<const StateRegistryBase<StateType, OperatorType> *,
                               segmented_vector::SegmentedVector<Entry> * > EntryVectorMap;
    EntryVectorMap entries_by_registry;

    mutable const StateRegistryBase<StateType, OperatorType> *cached_registry;
    mutable segmented_vector::SegmentedVector<Entry> *cached_entries;

    /*
      Returns the SegmentedVector associated with the given StateRegistry.
      If no vector is associated with this registry yet, an empty one is created.
      Both the registry and the returned vector are cached to speed up
      consecutive calls with the same registry.
    */
    segmented_vector::SegmentedVector<Entry> *get_entries(const StateRegistryBase<StateType, OperatorType> *registry) {
        if (cached_registry != registry) {
            cached_registry = registry;
            typename EntryVectorMap::const_iterator it = entries_by_registry.find(registry);
            if (it == entries_by_registry.end()) {
                cached_entries = new segmented_vector::SegmentedVector<Entry>();
                entries_by_registry[registry] = cached_entries;
                registry->subscribe(this);
            } else {
                cached_entries = it->second;
            }
        }
        assert(cached_registry == registry && cached_entries == entries_by_registry[registry]);
        return cached_entries;
    }

    /*
      Returns the SegmentedVector associated with the given StateRegistry.
      Returns 0, if no vector is associated with this registry yet.
      Otherwise, both the registry and the returned vector are cached to speed
      up consecutive calls with the same registry.
    */
    const segmented_vector::SegmentedVector<Entry> *get_entries(const StateRegistryBase<StateType, OperatorType> *registry) const {
        if (cached_registry != registry) {
            typename EntryVectorMap::const_iterator it = entries_by_registry.find(registry);
            if (it == entries_by_registry.end()) {
                return 0;
            } else {
                cached_registry = registry;
                cached_entries = const_cast<segmented_vector::SegmentedVector<Entry> *>(it->second);
            }
        }
        assert(cached_registry == registry);
        return cached_entries;
    }

    // forbid copies and assignment
    PerStateInformation(const PerStateInformation<Entry, StateType> &) = delete;
    PerStateInformation &operator=(const PerStateInformation<Entry, StateType> &) = delete;
public:
    PerStateInformation()
        : default_value(),
          cached_registry(0),
          cached_entries(0) {
    }

    explicit PerStateInformation(const Entry &default_value_)
        : default_value(default_value_),
          cached_registry(0),
          cached_entries(0) {
    }

    ~PerStateInformation() {
        for (typename EntryVectorMap::iterator it = entries_by_registry.begin();
             it != entries_by_registry.end(); ++it) {
            it->first->unsubscribe(this);
            delete it->second;
        }
    }

    Entry &operator[](const StateType &state) {
        const auto *registry = &state.get_registry();
        segmented_vector::SegmentedVector<Entry> *entries = get_entries(registry);
        int state_id = state.get_id().value;
        size_t virtual_size = registry->size();
        assert(utils::in_bounds(state_id, *registry));
        if (entries->size() < virtual_size) {
            entries->resize(virtual_size, default_value);
        }
        return (*cached_entries)[state_id];
    }

    const Entry &operator[](const StateType &state) const {
        const StateRegistryBase<StateType, OperatorType> *registry = &state.get_registry();
        const segmented_vector::SegmentedVector<Entry> *entries = get_entries(registry);
        if (!entries) {
            return default_value;
        }
        int state_id = state.get_id().value;
        assert(utils::in_bounds(state_id, *registry));
        int num_entries = entries->size();
        if (state_id >= num_entries) {
            return default_value;
        }
        return (*entries)[state_id];
    }

    void remove_state_registry(StateRegistryBase<StateType, OperatorType> *registry) override {
        delete entries_by_registry[registry];
        entries_by_registry.erase(registry);
        if (registry == cached_registry) {
            cached_registry = 0;
            cached_entries = 0;
        }
    }
};

#endif
