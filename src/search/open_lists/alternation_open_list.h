#ifndef OPEN_LISTS_ALTERNATION_OPEN_LIST_H
#define OPEN_LISTS_ALTERNATION_OPEN_LIST_H

#include "../open_list_factory.h"
#include "../option_parser_util.h"

namespace alternation_open_list {
template<class Entry, class StateType, class OperatorType>
class AlternationOpenList : public OpenList<Entry, StateType, OperatorType> {
    std::vector<std::unique_ptr<OpenList<Entry, StateType, OperatorType>>> open_lists;
    std::vector<int> priorities;

    const int boost_amount;
protected:
    virtual void do_insertion(EvaluationContext<StateType, OperatorType> &eval_context,
                              const Entry &entry) override;

public:
    explicit AlternationOpenList(const Options &opts);
    virtual ~AlternationOpenList() override = default;

    virtual Entry remove_min(std::vector<int> *key = nullptr) override;
    virtual bool empty() const override;
    virtual void clear() override;
    virtual void boost_preferred() override;
    virtual void get_involved_heuristics(std::set<Heuristic<StateType, OperatorType> *> &hset) override;
    virtual bool is_dead_end(
        EvaluationContext<StateType, OperatorType> &eval_context) const override;
    virtual bool is_reliable_dead_end(
        EvaluationContext<StateType, OperatorType> &eval_context) const override;
};


template<class Entry, class StateType, class OperatorType>
AlternationOpenList<Entry, StateType, OperatorType>::AlternationOpenList(const Options &opts)
    : boost_amount(opts.get<int>("boost")) {
    std::vector<std::shared_ptr<OpenListFactory<StateType, OperatorType>>> open_list_factories(
        opts.get_list<std::shared_ptr<OpenListFactory<StateType, OperatorType>>>("sublists"));
    open_lists.reserve(open_list_factories.size());
    for (const auto &factory : open_list_factories)
        open_lists.push_back(factory->create_open_list<Entry>());

    priorities.resize(open_lists.size(), 0);
}

template<class Entry, class StateType, class OperatorType>
void AlternationOpenList<Entry, StateType, OperatorType>::do_insertion(
    EvaluationContext<StateType, OperatorType> &eval_context, const Entry &entry) {
    for (const auto &sublist : open_lists)
        sublist->insert(eval_context, entry);
}

template<class Entry, class StateType, class OperatorType>
Entry AlternationOpenList<Entry, StateType, OperatorType>::remove_min(std::vector<int> *key) {
    if (key) {
        std::cerr << "not implemented -- see msg639 in the tracker" << std::endl;
        utils::exit_with(utils::ExitCode::UNSUPPORTED);
    }
    int best = -1;
    for (size_t i = 0; i < open_lists.size(); ++i) {
        if (!open_lists[i]->empty() &&
            (best == -1 || priorities[i] < priorities[best])) {
            best = i;
        }
    }
    assert(best != -1);
    const auto &best_list = open_lists[best];
    assert(!best_list->empty());
    ++priorities[best];
    return best_list->remove_min(nullptr);
}

template<class Entry, class StateType, class OperatorType>
bool AlternationOpenList<Entry, StateType, OperatorType>::empty() const {
    for (const auto &sublist : open_lists)
        if (!sublist->empty())
            return false;
    return true;
}

template<class Entry, class StateType, class OperatorType>
void AlternationOpenList<Entry, StateType, OperatorType>::clear() {
    for (const auto &sublist : open_lists)
        sublist->clear();
}

template<class Entry, class StateType, class OperatorType>
void AlternationOpenList<Entry, StateType, OperatorType>::boost_preferred() {
    for (size_t i = 0; i < open_lists.size(); ++i)
        if (open_lists[i]->only_contains_preferred_entries())
            priorities[i] -= boost_amount;
}

template<class Entry, class StateType, class OperatorType>
void AlternationOpenList<Entry, StateType, OperatorType>::get_involved_heuristics(
    std::set<Heuristic<StateType, OperatorType> *> &hset) {
    for (const auto &sublist : open_lists)
        sublist->get_involved_heuristics(hset);
}

template<class Entry, class StateType, class OperatorType>
bool AlternationOpenList<Entry, StateType, OperatorType>::is_dead_end(
    EvaluationContext<StateType, OperatorType> &eval_context) const {
    // If one sublist is sure we have a dead end, return true.
    if (is_reliable_dead_end(eval_context))
        return true;
    // Otherwise, return true if all sublists agree this is a dead-end.
    for (const auto &sublist : open_lists)
        if (!sublist->is_dead_end(eval_context))
            return false;
    return true;
}

template<class Entry, class StateType, class OperatorType>
bool AlternationOpenList<Entry, StateType, OperatorType>::is_reliable_dead_end(
    EvaluationContext<StateType, OperatorType> &eval_context) const {
    for (const auto &sublist : open_lists)
        if (sublist->is_reliable_dead_end(eval_context))
            return true;
    return false;
}


template<class StateType, class OperatorType>
class AlternationOpenListFactory : public OpenListFactory<StateType, OperatorType> {
    Options options;
public:
    explicit AlternationOpenListFactory(const Options &options);
    virtual ~AlternationOpenListFactory() override = default;

    virtual std::unique_ptr<StateOpenList<StateType, OperatorType>> create_state_open_list() override;
    virtual std::unique_ptr<EdgeOpenList<StateType, OperatorType>> create_edge_open_list() override;
};

template<class StateType, class OperatorType>
AlternationOpenListFactory<StateType, OperatorType>::AlternationOpenListFactory(const Options &options)
    : options(options) {
}

template<class StateType, class OperatorType>
std::unique_ptr<StateOpenList<StateType, OperatorType>>
AlternationOpenListFactory<StateType, OperatorType>::create_state_open_list() {
    return utils::make_unique_ptr<AlternationOpenList<StateOpenListEntry, StateType, OperatorType>>(options);
}

template<class StateType, class OperatorType>
std::unique_ptr<EdgeOpenList<StateType, OperatorType>>
AlternationOpenListFactory<StateType, OperatorType>::create_edge_open_list() {
    return utils::make_unique_ptr<AlternationOpenList<EdgeOpenListEntry, StateType, OperatorType>>(options);
}

}

#endif
