#ifndef STATE_ID_H
#define STATE_ID_H

#include <ostream>

// For documentation on classes relevant to storing and working with registered
// states see the file state_registry.h.

namespace redblack {
class RBStateRegistry;
}

class StateID {
	template<class StateType, class OperatorType>
    friend class StateRegistryBase;
	friend class StateRegistry;
	friend class redblack::RBStateRegistry;
    friend std::ostream &operator<<(std::ostream &os, StateID id);
    template<class Entry, class StateType, class OperatorType>
    friend class PerStateInformation;

    int value;
    explicit StateID(int value_)
        : value(value_) {
    }

    // No implementation to prevent default construction
    StateID();
public:
    ~StateID() {
    }

    static const StateID no_state;

    bool operator==(const StateID &other) const {
        return value == other.value;
    }

    bool operator!=(const StateID &other) const {
        return !(*this == other);
    }

    size_t hash() const {
        return value;
    }
};

std::ostream &operator<<(std::ostream &os, StateID id);

namespace std {
template<>
struct hash<StateID> {
    size_t operator()(StateID id) const {
        return id.hash();
    }
};
}

#endif
