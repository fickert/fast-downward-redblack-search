#ifndef OPEN_LISTS_STANDARD_SCALAR_OPEN_LIST_H
#define OPEN_LISTS_STANDARD_SCALAR_OPEN_LIST_H

#include "../open_list_factory.h"
#include "../option_parser_util.h"


/*
  Open list indexed by a single int, using FIFO tie-breaking.

  Implemented as a map from int to deques.
*/

namespace standard_scalar_open_list {
template<class Entry, class StateType, class OperatorType>
class StandardScalarOpenList : public OpenList<Entry, StateType, OperatorType> {
    typedef std::deque<Entry> Bucket;

    std::map<int, Bucket> buckets;
    int size;

    Evaluator<StateType, OperatorType> *evaluator;

protected:
    virtual void do_insertion(EvaluationContext<StateType, OperatorType> &eval_context,
                              const Entry &entry) override;

public:
    explicit StandardScalarOpenList(const Options &opts);
    StandardScalarOpenList(Evaluator<StateType, OperatorType> *eval, bool preferred_only);
    virtual ~StandardScalarOpenList() override = default;

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
StandardScalarOpenList<Entry, StateType, OperatorType>::StandardScalarOpenList(const Options &opts)
    : OpenList<Entry, StateType, OperatorType>(opts.get<bool>("pref_only")),
      size(0),
      evaluator(opts.get<Evaluator<StateType, OperatorType> *>("eval")) {
}

template<class Entry, class StateType, class OperatorType>
StandardScalarOpenList<Entry, StateType, OperatorType>::StandardScalarOpenList(
    Evaluator<StateType, OperatorType> *evaluator, bool preferred_only)
    : OpenList<Entry>(preferred_only),
      size(0),
      evaluator(evaluator) {
}

template<class Entry, class StateType, class OperatorType>
void StandardScalarOpenList<Entry, StateType, OperatorType>::do_insertion(
    EvaluationContext<StateType, OperatorType> &eval_context, const Entry &entry) {
    int key = eval_context.get_heuristic_value(evaluator);
    buckets[key].push_back(entry);
    ++size;
}

template<class Entry, class StateType, class OperatorType>
Entry StandardScalarOpenList<Entry, StateType, OperatorType>::remove_min(std::vector<int> *key) {
    assert(size > 0);
    auto it = buckets.begin();
    assert(it != buckets.end());
    if (key) {
        assert(key->empty());
        key->push_back(it->first);
    }

    Bucket &bucket = it->second;
    assert(!bucket.empty());
    Entry result = bucket.front();
    bucket.pop_front();
    if (bucket.empty())
        buckets.erase(it);
    --size;
    return result;
}

template<class Entry, class StateType, class OperatorType>
bool StandardScalarOpenList<Entry, StateType, OperatorType>::empty() const {
    return size == 0;
}

template<class Entry, class StateType, class OperatorType>
void StandardScalarOpenList<Entry, StateType, OperatorType>::clear() {
    buckets.clear();
    size = 0;
}

template<class Entry, class StateType, class OperatorType>
void StandardScalarOpenList<Entry, StateType, OperatorType>::get_involved_heuristics(
    std::set<Heuristic<StateType, OperatorType> *> &hset) {
    evaluator->get_involved_heuristics(hset);
}

template<class Entry, class StateType, class OperatorType>
bool StandardScalarOpenList<Entry, StateType, OperatorType>::is_dead_end(
    EvaluationContext<StateType, OperatorType> &eval_context) const {
    return eval_context.is_heuristic_infinite(evaluator);
}

template<class Entry, class StateType, class OperatorType>
bool StandardScalarOpenList<Entry, StateType, OperatorType>::is_reliable_dead_end(
    EvaluationContext<StateType, OperatorType> &eval_context) const {
    return is_dead_end(eval_context) && evaluator->dead_ends_are_reliable();
}


template<class StateType, class OperatorType>
class StandardScalarOpenListFactory : public OpenListFactory<StateType, OperatorType> {
    Options options;
public:
    explicit StandardScalarOpenListFactory(const Options &options);
    virtual ~StandardScalarOpenListFactory() override = default;

    virtual std::unique_ptr<StateOpenList<StateType, OperatorType>> create_state_open_list() override;
    virtual std::unique_ptr<EdgeOpenList<StateType, OperatorType>> create_edge_open_list() override;
};

template<class StateType, class OperatorType>
StandardScalarOpenListFactory<StateType, OperatorType>::StandardScalarOpenListFactory(
    const Options &options)
    : options(options) {
}

template<class StateType, class OperatorType>
std::unique_ptr<StateOpenList<StateType, OperatorType>>
StandardScalarOpenListFactory<StateType, OperatorType>::create_state_open_list() {
    return utils::make_unique_ptr<StandardScalarOpenList<StateOpenListEntry, StateType, OperatorType>>(options);
}

template<class StateType, class OperatorType>
std::unique_ptr<EdgeOpenList<StateType, OperatorType>>
StandardScalarOpenListFactory<StateType, OperatorType>::create_edge_open_list() {
    return utils::make_unique_ptr<StandardScalarOpenList<EdgeOpenListEntry, StateType, OperatorType>>(options);
}


}

#endif
