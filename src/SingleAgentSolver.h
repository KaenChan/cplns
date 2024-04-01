#pragma once
#include "Instance.h"
#include "ConstraintTable.h"

uint64_t rand_hash(uint64_t x); 
uint64_t rand_hash(int x, int y); 

extern thread_local float tl_single_agent_solver_f_w;

class LLNode // low-level node
{
public:
	int location = 0;
	int g_val = 0;
	int h_val = 0;
	LLNode* parent = nullptr;
	int timestep = 0;
	int num_of_conflicts = 0;
	bool in_openlist = false;
	bool wait_at_goal = false; // the action is to wait at the goal vertex or not. This is used for >lenghth constraints
    bool is_goal = false;
	// the following is used to comapre nodes in the OPEN list
	struct compare_node
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const LLNode* n1, const LLNode* n2) const
		{
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
            {
                if (n1->h_val == n2->h_val)
                {
					return rand_hash(n1->location, n2->location) ^ rand_hash(n1->timestep, n1->g_val) % 2;
                    // return rand() % 2 == 0;   // break ties randomly
                }
                return n1->h_val >= n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
            }
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

		// the following is used to compare nodes in the FOCAL list
	struct secondary_compare_node
	{
		bool operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
		{
			// if (std::abs(n1->num_of_conflicts - n2->num_of_conflicts) <= 2)
			if (n1->num_of_conflicts == n2->num_of_conflicts)
			{
				float w = tl_single_agent_solver_f_w;
                if (n1->g_val + w*n1->h_val == n2->g_val + w*n2->h_val)
                {
                    if (n1->h_val == n2->h_val)
                    {
						// return false;
						return rand_hash(n1->location, n2->location) ^ rand_hash(n1->timestep, n1->g_val) % 2;
                        // return rand() % 2 == 0;   // break ties randomly
                    }
                    return n1->h_val >= n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
                }
                return n1->g_val+w*n1->h_val >= n2->g_val+w*n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
			}
			return n1->num_of_conflicts >= n2->num_of_conflicts;  // n1 > n2 if it has more conflicts
		}
	};  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


	LLNode() {}
	LLNode(int location, int g_val, int h_val, LLNode* parent, int timestep, int num_of_conflicts) :
		location(location), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
		num_of_conflicts(num_of_conflicts) {}
	LLNode(const LLNode& other) { copy(other); }

	void copy(const LLNode& other)
	{
		location = other.location;
		g_val = other.g_val;
		h_val = other.h_val;
		parent = other.parent;
		timestep = other.timestep;
		num_of_conflicts = other.num_of_conflicts;
		wait_at_goal = other.wait_at_goal;
		is_goal = other.is_goal;
	}
    inline int getFVal() const { return g_val + h_val; }
    inline int getFValW() const { return g_val + tl_single_agent_solver_f_w * h_val; }
};

std::ostream& operator<<(std::ostream& os, const LLNode& node);

class SingleAgentSolver
{
public:
    uint64_t accumulated_num_expanded = 0;
    uint64_t accumulated_num_generated = 0;
    uint64_t accumulated_num_reopened = 0;
    uint64_t num_runs = 0;

	std::atomic<bool> *phase_collision_asynch_interrupt;
	std::atomic<bool> *phase_cost_asynch_interrupt;

    int num_collisions = MAX_COST;
	double runtime_build_CT = 0; // runtimr of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table

	int start_location;
	int goal_location;
	bool pre_process = true;
	vector<int> *p_my_heuristic = nullptr;  // this is the precomputed heuristic for this agent
	int compute_heuristic(int from, int to) const  // compute admissible heuristic between two locations
	{
		return max(get_DH_heuristic(from, to), instance.getManhattanDistance(from, to));
	}
	const Instance& instance;

    //virtual Path findOptimalPath(const PathTable& path_table) = 0;
    //virtual Path findOptimalPath(const ConstraintTable& constraint_table, const PathTableWC& path_table) = 0;
	virtual Path findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
		const vector<Path*>& paths, int agent, int lower_bound) = 0;
	virtual pair<Path, int> findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
		const vector<Path*>& paths, int agent, int lowerbound, double w) = 0;  // return the path and the lowerbound
    virtual Path findPath(const ConstraintTable& constraint_table) = 0;  // return the path
    void findMinimumSetofColldingTargets(vector<int>& goal_table,set<int>& A_target);
    virtual int getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound) = 0;
	virtual string getName() const = 0;

	list<int> getNextLocations(int curr) const; // including itself and its neighbors
	list<int> getNeighbors(int curr) const { return instance.getNeighbors(curr); }
    uint64_t getNumExpanded() const { return num_expanded; }
	// int getStartLocation() const {return instance.start_locations[agent]; }
	// int getGoalLocation() const {return instance.goal_locations[agent]; }

    SingleAgentSolver(const Instance& instance, int agent, bool pre_process)
        : instance(instance),  // agent(agent),
          start_location(instance.start_locations[agent]),
          goal_location(instance.goal_locations[agent]),
          pre_process(pre_process) {}

    void init() {
        if (pre_process) {
            p_my_heuristic = new vector<int>;
            compute_heuristics();
        }
    }

	virtual ~SingleAgentSolver() {
		if(pre_process)
			delete p_my_heuristic;
	}
    void reset()
    {
        if (num_generated > 0)
        {
            accumulated_num_expanded += num_expanded;
            accumulated_num_generated += num_generated;
            accumulated_num_reopened += num_reopened;
            num_runs++;
        }
        num_expanded = 0;
        num_generated = 0;
        num_reopened = 0;
    }
protected:
    uint64_t num_expanded = 0;
    uint64_t num_generated = 0;
    uint64_t num_reopened = 0;
	int min_f_val; // minimal f value in OPEN
	// int lower_bound; // Threshold for FOCAL
	double w = 1; // suboptimal bound

	void compute_heuristics();
	int get_DH_heuristic(int from, int to) const { 
		return abs(p_my_heuristic->at(from) - p_my_heuristic->at(to)); 
	}
};

