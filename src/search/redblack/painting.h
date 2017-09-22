#ifndef PAINTING_H
#define PAINTING_H

#include "../utils/rng.h"

#include <set>
#include <vector>
#include <memory>

namespace options {
class OptionParser;
class Options;
}

namespace redblack {

// TODO: move the construction of the paintings out of the painting class, either use static functions or a factory
// a) need a way to generate a painting from scratch
// b) incremental strategy (this is a separate problem)

// --> two FD plugins: a) painting, b) incrementalpaintingstrategy

using InternalPaintingType = std::vector<bool>;


class Painting {
	InternalPaintingType painting;

public:
	Painting(const InternalPaintingType &painting);
	Painting(InternalPaintingType &&painting);
	virtual ~Painting() = default;

	bool is_black_var(int var) const {
		return !painting[var];
	}

	bool is_red_var(int var) const {
		return painting[var];
	}

	auto get_painting() const -> const InternalPaintingType & { return painting; }
};



class PaintingFactory {
public:
	PaintingFactory(const options::Options &opts);
	virtual ~PaintingFactory() = default;

	virtual auto construct_painting() -> InternalPaintingType = 0;

	static void add_options_to_parser(options::OptionParser &parser);

protected:
	const bool force_cg_leaves_red;
	
	static auto get_all_red_painting() -> InternalPaintingType;
	static auto get_all_black_painting() -> InternalPaintingType;
	static auto get_cg_leaves_painting() -> InternalPaintingType;
};


// TODO rename this to something like FD-ordering
class CGTopFirstPaintingFactory : public PaintingFactory {
	const int num_black_vars;
	
public:
	CGTopFirstPaintingFactory(const options::Options &opts);

	auto construct_painting() -> InternalPaintingType override;
};


class CGBranchFirstPaintingFactory : public PaintingFactory {
	const int num_black_vars;
	std::vector<bool> scc_painted;
	std::vector<int> scc_offset_to_level;

	static auto get_connected_components(std::vector<int> variables) -> std::vector<std::set<int>>;
	auto paint_dfs_sccs(int cur_scc_offset, int starting_var_of_scc, std::vector<std::set<int>> sccs, InternalPaintingType &painting, int &already_black) -> bool;
	auto paint_succ_rec(int cur_node, InternalPaintingType &painting, int &already_black, std::set<int> &scc) -> bool;

public:
	CGBranchFirstPaintingFactory(const options::Options &opts);

	auto construct_painting()->InternalPaintingType override;
};


class IncSCCLvlPaintingFactory : public PaintingFactory {
	const int num_black_vars;
	const bool random_within_scc;
	const std::shared_ptr<utils::RandomNumberGenerator> rng;

	void randomly_paint_scc(InternalPaintingType &painting, const std::vector<int> &scc, std::size_t number_black);

public:
	IncSCCLvlPaintingFactory(const options::Options &opts);

	auto construct_painting() -> InternalPaintingType override;
};


class LeastConflictsPaintingFactory : public PaintingFactory {
	const int num_black_vars;
	const bool prefer_lvl;

public:
	LeastConflictsPaintingFactory(const options::Options &opts);

	auto construct_painting() -> InternalPaintingType override;
};

}


#endif
 
