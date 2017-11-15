#ifndef OPEN_LISTS_TIEBREAKING_OPEN_LIST_H
#define OPEN_LISTS_TIEBREAKING_OPEN_LIST_H

#include "../open_list_factory.h"
#include "../option_parser_util.h"

namespace tiebreaking_open_list {
template<class Entry, class StateType, class OperatorType>
class TieBreakingOpenList : public OpenList<Entry> {
    using Bucket = std::deque<Entry>;

	std::map<const std::vector<int>, Bucket> buckets;
    int size;

	std::vector<Evaluator<StateType, OperatorType> *> evaluators;
    /*
      If allow_unsafe_pruning is true, we ignore (don't insert) states
      which the first evaluator considers a dead end, even if it is
      not a safe heuristic.
    */
    bool allow_unsafe_pruning;

    int dimension() const;

protected:
    virtual void do_insertion(EvaluationContext<StateType, OperatorType> &eval_context,
                              const Entry &entry) override;

public:
    explicit TieBreakingOpenList(const Options &opts);
    virtual ~TieBreakingOpenList() override = default;

    virtual Entry remove_min(std::vector<int> *key = nullptr) override;
    virtual bool empty() const override;
    virtual void clear() override;
    virtual void get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &hset) override;
    virtual bool is_dead_end(
        EvaluationContext<StateType, OperatorType> &eval_context) const override;
    virtual bool is_reliable_dead_end(
        EvaluationContext<StateType, OperatorType> &eval_context) const override;
};


template<class Entry, class StateType, class OperatorType>
TieBreakingOpenList<Entry, StateType, OperatorType>::TieBreakingOpenList(const Options &opts)
    : OpenList<Entry>(opts.get<bool>("pref_only")),
      size(0), evaluators(opts.get_list<Evaluator<StateType, OperatorType> *>("evals")),
      allow_unsafe_pruning(opts.get<bool>("unsafe_pruning")) {
}

template<class Entry, class StateType, class OperatorType>
void TieBreakingOpenList<Entry, StateType, OperatorType>::do_insertion(
    EvaluationContext<StateType, OperatorType> &eval_context, const Entry &entry) {
    std::vector<int> key;
    key.reserve(evaluators.size());
    for (Evaluator<StateType, OperatorType> *evaluator : evaluators)
        key.push_back(eval_context.get_heuristic_value_or_infinity(evaluator));

    buckets[key].push_back(entry);
    ++size;
}

template<class Entry, class StateType, class OperatorType>
Entry TieBreakingOpenList<Entry, StateType, OperatorType>::remove_min(std::vector<int> *key) {
    assert(size > 0);
    typename std::map<const std::vector<int>, Bucket>::iterator it;
    it = buckets.begin();
    assert(it != buckets.end());
    assert(!it->second.empty());
    --size;
    if (key) {
        assert(key->empty());
        *key = it->first;
    }
    Entry result = it->second.front();
    it->second.pop_front();
    if (it->second.empty())
        buckets.erase(it);
    return result;
}

template<class Entry, class StateType, class OperatorType>
bool TieBreakingOpenList<Entry, StateType, OperatorType>::empty() const {
    return size == 0;
}

template<class Entry, class StateType, class OperatorType>
void TieBreakingOpenList<Entry, StateType, OperatorType>::clear() {
    buckets.clear();
    size = 0;
}

template<class Entry, class StateType, class OperatorType>
int TieBreakingOpenList<Entry, StateType, OperatorType>::dimension() const {
    return evaluators.size();
}

template<class Entry, class StateType, class OperatorType>
void TieBreakingOpenList<Entry, StateType, OperatorType>::get_involved_heuristics(
    std::set<Heuristic<StateType, OperatorType> *> &hset) {
    for (Evaluator<StateType, OperatorType> *evaluator : evaluators)
        evaluator->get_involved_heuristics(hset);
}

template<class Entry, class StateType, class OperatorType>
bool TieBreakingOpenList<Entry, StateType, OperatorType>::is_dead_end(
    EvaluationContext<StateType, OperatorType> &eval_context) const {
    // TODO: Properly document this behaviour.
    // If one safe heuristic detects a dead end, return true.
    if (is_reliable_dead_end(eval_context))
        return true;
    // If the first heuristic detects a dead-end and we allow "unsafe
    // pruning", return true.
    if (allow_unsafe_pruning &&
        eval_context.is_heuristic_infinite(evaluators[0]))
        return true;
    // Otherwise, return true if all heuristics agree this is a dead-end.
    for (Evaluator<StateType, OperatorType> *evaluator : evaluators)
        if (!eval_context.is_heuristic_infinite(evaluator))
            return false;
    return true;
}

template<class Entry, class StateType, class OperatorType>
bool TieBreakingOpenList<Entry, StateType, OperatorType>::is_reliable_dead_end(
    EvaluationContext<StateType, OperatorType> &eval_context) const {
    for (Evaluator<StateType, OperatorType> *evaluator : evaluators)
        if (eval_context.is_heuristic_infinite(evaluator) &&
            evaluator->dead_ends_are_reliable())
            return true;
    return false;
}


template<class StateType, class OperatorType>
class TieBreakingOpenListFactory : public OpenListFactory<StateType, OperatorType> {
    Options options;
public:
    explicit TieBreakingOpenListFactory(const Options &options);
    virtual ~TieBreakingOpenListFactory() override = default;

    virtual std::unique_ptr<StateOpenList<StateType, OperatorType>> create_state_open_list() override;
    virtual std::unique_ptr<EdgeOpenList<StateType, OperatorType>> create_edge_open_list() override;
};

template<class StateType, class OperatorType>
TieBreakingOpenListFactory<StateType, OperatorType>::TieBreakingOpenListFactory(const Options &options)
    : options(options) {
}

template<class StateType, class OperatorType>
std::unique_ptr<StateOpenList<StateType, OperatorType>>
TieBreakingOpenListFactory<StateType, OperatorType>::create_state_open_list() {
    return utils::make_unique_ptr<TieBreakingOpenList<StateOpenListEntry, StateType, OperatorType>>(options);
}

template<class StateType, class OperatorType>
std::unique_ptr<EdgeOpenList<StateType, OperatorType>>
TieBreakingOpenListFactory<StateType, OperatorType>::create_edge_open_list() {
    return utils::make_unique_ptr<TieBreakingOpenList<EdgeOpenListEntry, StateType, OperatorType>>(options);
}


}

#endif
