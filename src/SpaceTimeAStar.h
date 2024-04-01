﻿#pragma once
#include "SingleAgentSolver.h"


class AStarNode: public LLNode
{
public:
	// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef pairing_heap< AStarNode*, compare<LLNode::compare_node> >::handle_type open_handle_t;
	typedef pairing_heap< AStarNode*, compare<LLNode::secondary_compare_node> >::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;

	AStarNode() : LLNode() {}
    AStarNode(const AStarNode& other) : LLNode(other) {} // copy everything except for handles
	AStarNode(int loc, int g_val, int h_val, LLNode* parent, int timestep, int num_of_conflicts) :
		LLNode(loc, g_val, h_val, parent, timestep, num_of_conflicts) {}


	~AStarNode() {}

	// The following is used by for generating the hash value of a nodes
	struct NodeHasher
	{
		size_t operator()(const AStarNode* n) const
		{
			size_t loc_hash = std::hash<int>()(n->location);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			return (loc_hash ^ (timestep_hash << 1));
		}
	};

	// The following is used for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-NULL and agree on the id and timestep
	struct eqnode
	{
		bool operator()(const AStarNode* s1, const AStarNode* s2) const
		{
			return (s1 == s2) || (s1 && s2 &&
                        s1->location == s2->location &&
                        s1->timestep == s2->timestep &&
						s1->wait_at_goal == s2->wait_at_goal &&
						s1->is_goal == s2->is_goal);
		}
	};
};


class SpaceTimeAStar: public SingleAgentSolver
{
public:
    // find path by time-space A* search
    // Returns a shortest path that does not collide with paths in the path table
    //Path findOptimalPath(const PathTable& path_table);
    //Path findOptimalPath(const ConstraintTable& constraint_table, const PathTableWC& path_table);
	// find path by time-space A* search
	// Returns a shortest path that satisfies the constraints of the give node  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is an underestimation of the length of the path in order to speed up the search.
	Path findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
						const vector<Path*>& paths, int agent, int lower_bound);

    // find path by time-space A* search
    // Returns a path that satisfies the constraint_table while
    // minimizing the number of conflicts with constraint_table, breaking ties by the path length.
    Path findPath(const ConstraintTable& constraint_table);

	pair<Path, int> findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
		const vector<Path*>& paths, int agent, int lowerbound, double w);  // return the path and the lowerbound


    int getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound);

	string getName() const { return "AStar"; }

	SpaceTimeAStar(const Instance& instance, int agent, bool pre_process=true):
		SingleAgentSolver(instance, agent, pre_process) {}

private:
	// define typedefs and handles for heap
	typedef pairing_heap< AStarNode*, compare<AStarNode::compare_node> > heap_open_t;
	typedef pairing_heap< AStarNode*, compare<AStarNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;

	// define typedef for hash_map
	typedef unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
	hashtable_t allNodes_table;

	// Updates the path datamember
	void updatePath(const LLNode* goal, vector<PathEntry> &path);
	void updateFocalList();
	inline AStarNode* popNode();
	inline void pushNode(AStarNode* node);
	void releaseNodes();

};
