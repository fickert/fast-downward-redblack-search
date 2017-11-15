#ifndef OPTIONS_SYNERGY_H
#define OPTIONS_SYNERGY_H

#include <vector>

template<class StateType, class OperatorType>
class Heuristic;
class GlobalState;

namespace options {
class Synergy {
public:
    std::vector<Heuristic<GlobalState, GlobalOperator> *> heuristics;
};
}

#endif
