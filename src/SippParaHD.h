#pragma once
#include <boost/functional/hash.hpp>
#include <mutex>

#include "ReservationTable.h"
#include "SIPP.h"
#include "SingleAgentSolver.h"
#include "Threading.h"
#include "completion_counter.h"
#include "hd_msg_buffer.hpp"
#include "robin_hood.h"

class SIPPNodeHD : public LLNode {
   public:
    uint64_t node_id = 0;
    // define a typedefs for handles to the heaps (allow up to quickly update a
    // node in the heap)
    typedef boost::heap::pairing_heap<
        SIPPNodeHD*, compare<SIPPNodeHD::compare_node>>::handle_type
        open_handle_t;
    typedef boost::heap::pairing_heap<
        SIPPNodeHD*, compare<SIPPNodeHD::secondary_compare_node>>::handle_type
        focal_handle_t;
    open_handle_t open_handle;
    focal_handle_t focal_handle;
    int high_generation;  // the upper bound with respect to generation
    int high_expansion;   // the upper bound with respect to expansion
    bool collision_v;
    SIPPNodeHD() : LLNode() {}
    SIPPNodeHD(int loc, int g_val, int h_val, SIPPNodeHD* parent, int timestep,
               int high_generation, int high_expansion, bool collision_v,
               int num_of_conflicts)
        : LLNode(loc, g_val, h_val, parent, timestep, num_of_conflicts),
          high_generation(high_generation),
          high_expansion(high_expansion),
          collision_v(collision_v) {}
    SIPPNodeHD(const SIPPNodeHD& other)
        : LLNode(other),
          high_generation(other.high_generation),
          high_expansion(other.high_expansion),
          collision_v(collision_v) {}
    ~SIPPNodeHD() {}

    void copy(const SIPPNodeHD& other)  // copy everything except for handles
    {
        LLNode::copy(other);
        high_generation = other.high_generation;
        high_expansion = other.high_expansion;
        collision_v = other.collision_v;
    }
    // The following is used by for generating the hash value of a nodes
    struct NodeHasher {
        std::size_t operator()(const SIPPNodeHD* n) const {
            size_t seed = 0;
            boost::hash_combine(seed, n->location);
            boost::hash_combine(seed, n->high_generation);
            // boost::hash_combine(seed, 1+((int)n->wait_at_goal)<<1);
            // boost::hash_combine(seed, 1+((int)n->is_goal)<<2);
            return seed;
        }
    };

    // The following is used for checking whether two nodes are equal
    // we say that two nodes, s1 and s2, are equal if
    // both are non-NULL and agree on the id and timestep
    struct eqnode {
        bool operator()(const SIPPNodeHD* n1, const SIPPNodeHD* n2) const {
            return (n1 == n2) || (n1 && n2 && n1->location == n2->location &&
                                  n1->wait_at_goal == n2->wait_at_goal &&
                                  n1->is_goal == n2->is_goal &&
                                  n1->high_generation == n2->high_generation);
            // max(n1->timestep, n2->timestep) <
            // min(get<1>(n1->interval), get<1>(n2->interval))); //overlapping
            // time intervals
        }
    };

	friend std::ostream& operator<<(std::ostream& os, const SIPPNodeHD& node)
	{
		os << 
			"id " << node.node_id << 
			" location " << node.location << 
			" timestep " << node.timestep << 
			" wait_at_goal " << node.wait_at_goal << 
			" (" << node.getFValW() << 
			" = " << node.g_val << "+" <<
			tl_single_agent_solver_f_w << "*" <<
			node.h_val << " ) with " << node.num_of_conflicts << " conflicts" <<
			" open " << node.in_openlist
			;
		return os;
	}

};

class SippParaHD : public SingleAgentSolver {
   public:
    // find path by SippParaHD
    // Returns a shortest path that satisfies the constraints of the give node
    // while minimizing the number of internal conflicts (that is conflicts with
    // known_paths for other agents found so far). lowerbound is an
    // underestimation of the length of the path in order to speed up the
    // search.
    // Path findOptimalPath(const PathTable& path_table) {return Path(); } //
    // TODO: To implement Path findOptimalPath(const ConstraintTable&
    // constraint_table, const PathTableWC& path_table);
    Path findOptimalPath(const HLNode& node,
                         const ConstraintTable& initial_constraints,
                         const vector<Path*>& paths, int agent, int lowerbound);
    pair<Path, int> findSuboptimalPath(
        const HLNode& node, const ConstraintTable& initial_constraints,
        const vector<Path*>& paths, int agent, int lowerbound,
        double w);  // return the path and the lowerbound
    Path findPath(const ConstraintTable&
                      constraint_table);  // return A path that minimizes
                                          // collisions, breaking ties by cost
    Path findPathSerial(const ConstraintTable& constraint_table);

    // return A path that minimizes collisions, breaking ties by cost
    Path findPathParallel( const ConstraintTable& constraint_table);  

    int getTravelTime(int start, int end,
                      const ConstraintTable& constraint_table, int upper_bound_cost);

    string getName() const { return "SippParaHD"; }

    SippParaHD(const Instance& instance, int agent, bool pre_process = true)
        : SingleAgentSolver(instance, agent, pre_process) {}

   private:
    friend class SippParaHDThread;

    // define typedefs and handles for heap
    typedef boost::heap::pairing_heap<
        SIPPNodeHD*, boost::heap::compare<LLNode::compare_node>>
        heap_open_t;
    typedef boost::heap::pairing_heap<
        SIPPNodeHD*, boost::heap::compare<LLNode::secondary_compare_node>>
        heap_focal_t;
    heap_open_t open_list;
    heap_focal_t focal_list;

    // define typedef for hash_map
    typedef boost::unordered_map<SIPPNodeHD*, list<SIPPNodeHD*>,
                                 SIPPNodeHD::NodeHasher, SIPPNodeHD::eqnode>
        hashtable_t;
    hashtable_t allNodes_table;
    list<SIPPNodeHD*> useless_nodes;
    // Path findNoCollisionPath(const ConstraintTable& constraint_table);

    // for parallel
    Path path_result;
    CompletionCounter* cc;
    std::atomic<bool> done_par;
    std::atomic<int> upper_bound_cost;
    std::atomic<int> upper_bound_collision;
	std::atomic<int> min_f_val_atomic; // minimal f value in OPEN
    pthread_barrier_t barrier;

    // hd_msg_buffer<SIPPNodeHD>* income_buffer = nullptr;
    vector<hd_msg_buffer<SIPPNodeHD>> income_buffer;
    vector<bool> terminate;
    int income_threshold;

    spinlock processlock;
    spinlock openlistlock;
    spinlock closelistlock;
    spinlock terminallock;

    std::mutex terminalLockG;
	vector<Mutex> terminalLocks;

    void updatePath(const LLNode* goal, std::vector<PathEntry>& path);

    inline void pushNodeToOpenAndFocal(SIPPNodeHD* node);
    inline void pushNodeToFocal(SIPPNodeHD* node);
    inline void eraseNodeFromLists(SIPPNodeHD* node);
    void updateFocalList();
    void releaseNodes();
    void printSearchTree() const;
    bool dominanceCheck(SIPPNodeHD* new_node);
    static int astar_helper(void* arg);

    void set_my_terminated(int myId);
    void unset_my_terminated(int myId);
    bool hasterminated();
};

class SippParaHDThread {
   public:
    int screen = 0;
    int tnum = 1;
    int myId = 0;
    int holding_time_par = 0;
    SippParaHD* p = nullptr;
    const ConstraintTable* constraint_table_par = nullptr;
    int num_expanded = 0;
    int num_generated = 0;
    int num_collisions = MAX_COST;
    uint64_t num_reopened = 0;
	Interval interval;

    // define typedefs and handles for heap
    typedef boost::heap::pairing_heap<
        SIPPNodeHD*, boost::heap::compare<LLNode::compare_node>>
        heap_open_t;
    typedef boost::heap::pairing_heap<
        SIPPNodeHD*, boost::heap::compare<LLNode::secondary_compare_node>>
        heap_focal_t;
    heap_open_t open_list;
    heap_focal_t focal_list;

    // define typedef for hash_map
    typedef boost::unordered_map<SIPPNodeHD*, list<SIPPNodeHD*>,
                                 SIPPNodeHD::NodeHasher, SIPPNodeHD::eqnode>
        hashtable_t;
    hashtable_t allNodes_table;
    list<SIPPNodeHD*> useless_nodes;
    // Path findNoCollisionPath(const ConstraintTable& constraint_table);

   public:
    int astar();

    int computeRecipient(SIPPNodeHD* node);
    void duplicate_node_reinsert(SIPPNodeHD* n, SIPPNodeHD* existing);

    void flush_receives(int id, std::vector<SIPPNodeHD*>& tmp);
    bool flush_sends(std::vector<std::vector<SIPPNodeHD*>>& outgo_buffer,
                     int id, int outgo_threshold);

    void send_next_node(std::vector<std::vector<SIPPNodeHD*>>& outgo_buffer,
                        SIPPNodeHD* node);

    inline SIPPNodeHD* popNode();
    inline void pushNode(SIPPNodeHD* node);

    void check_state();

    void updatePath(const LLNode* goal, std::vector<PathEntry>& path);

    inline void pushNodeToFocal(SIPPNodeHD* node);
    inline void eraseNodeFromLists(SIPPNodeHD* node);
    void releaseNodes();
    void printSearchTree() const;
    bool dominanceCheck(SIPPNodeHD* new_node);

	void printState();
	bool better_than_upper_bound(SIPPNodeHD* node);
};
