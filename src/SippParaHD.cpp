#include "SippParaHD.h"

#include "ReservationTable4Cost.h"
#include "ThreadPool.h"
#include "common.h"
#include "completion_counter.h"

int screen_const = 0;

static uint64_t num_of_created_llnode = 0;

Mutex llnodeid_mutex;

static thread_local double move_avg = 0;
static thread_local int last_choose = 0;
const double move_T = 100;
const double move_alpha = 2 / (1 + move_T);


uint64_t generate_node_id()
{
	llnodeid_mutex.lock();
	num_of_created_llnode++;
	llnodeid_mutex.unlock();
	return num_of_created_llnode;
}

void SippParaHD::updatePath(const LLNode* goal, vector<PathEntry>& path) {
    num_collisions = goal->num_of_conflicts;
    path.resize(goal->timestep + 1);
    // num_of_conflicts = goal->num_of_conflicts;

    const auto* curr = goal;
    while (curr->parent != nullptr)  // non-root node
    {
        const auto* prev = curr->parent;
        int t = prev->timestep + 1;
        while (t < curr->timestep) {
            path[t].location = prev->location;  // wait at prev location
            t++;
        }
        path[curr->timestep].location =
            curr->location;  // move to curr location
        curr = prev;
    }
    assert(curr->timestep == 0);
    path[0].location = curr->location;
}

// find path by A*
// Returns a path that minimizes the collisions with the paths in the path
// table, breaking ties by the length
Path SippParaHD::findPath(const ConstraintTable& constraint_table) {
    bool is_phase_cost = false;
    if(move_avg >= 50000) {
        if(last_choose != 1)
            printf("\t--> using parallel findPath move_avg %.1f\n", move_avg);
        last_choose = 1;
        return findPathParallel(constraint_table);
    } else { 
        if(last_choose != 2)
            printf("\t--> using serial findPath move_avg %.1f\n", move_avg);
        last_choose = 2;
        return findPathSerial(constraint_table);
    }
}

Path SippParaHD::findPathSerial(const ConstraintTable& constraint_table)
{
    double t1 = getTime();
	vector<int> &my_heuristic = *p_my_heuristic;
    reset();
    //Path path = findNoCollisionPath(constraint_table);
    //if (!path.empty())
    //    return path;
    ReservationTable reservation_table(constraint_table, goal_location);
    Path path;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return path;
    auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    auto last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);
    // generate start and add it to the OPEN & FOCAL list
    auto h = max(max(my_heuristic[start_location], holding_time), last_target_collision_time + 1);
    auto start = new SIPPNodeHD(start_location, 0, h, nullptr, 0, get<1>(interval), get<1>(interval),
                                get<2>(interval), get<2>(interval));
    pushNodeToFocal(start);

    int run_state = 0;
    // while (!focal_list.empty() && !phase_collision_asynch_interrupt->load() && !phase_cost_asynch_interrupt->load())
    while(true)
    {
        if(focal_list.empty()) {
            run_state = 51;
            break;
        }
        if(phase_collision_asynch_interrupt->load()) {
            run_state = 52;
            break;
        }
        if(phase_cost_asynch_interrupt->load()) {
            run_state = 53;
            break;
        }
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        // TODO: 先解决简单的路径,在解决难的
        // if(num_expanded > 10000)
        //     break;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->is_goal)
        {
            run_state = 1;
            updatePath(curr, path);
            break;
        }
        else if (curr->location == goal_location && // arrive at the goal location
                 !curr->wait_at_goal && // not wait at the goal location
                 curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            int future_collisions = constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep);
            if (future_collisions == 0)
            {
                run_state = 2;
                updatePath(curr, path);
                break;
            }
            // generate a goal node
            auto goal = new SIPPNodeHD(*curr);
            goal->is_goal = true;
            goal->h_val = 0;
            goal->num_of_conflicts += future_collisions;
            // try to retrieve it from the hash table
            if (dominanceCheck(goal))
                pushNodeToFocal(goal);
            else
                delete goal;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (auto & i : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                if (next_timestep + my_heuristic[next_location] > constraint_table.length_max)
                    break;
                auto next_collisions = curr->num_of_conflicts +
                                    (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) // wait time
                                      + (int)next_v_collision + (int)next_e_collision;
                auto next_h_val = max(my_heuristic[next_location], (next_collisions > 0?
                    holding_time : curr->getFVal()) - next_timestep); // path max
                // generate (maybe temporary) node
                auto next = new SIPPNodeHD(next_location, next_timestep, next_h_val, curr, next_timestep,
                                         next_high_generation, next_high_expansion, next_v_collision, next_collisions);
                // try to retrieve it from the hash table
                if (dominanceCheck(next))
                    pushNodeToFocal(next);
                else
                    delete next;
            }
        }  // end for loop that generates successors
        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
                get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            auto next_timestep = get<0>(interval);
            auto next_h_val = max(curr->h_val, (get<2>(interval) ? holding_time : curr->getFVal()) - next_timestep); // path max
            auto next_collisions = curr->num_of_conflicts +
                    (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) + (int)get<2>(interval);
            auto next = new SIPPNodeHD(curr->location, next_timestep, next_h_val, curr, next_timestep,
                                     get<1>(interval), get<1>(interval), get<2>(interval),
                                     next_collisions);
            next->wait_at_goal = (curr->location == goal_location);
            if (dominanceCheck(next))
                pushNodeToFocal(next);
            else
                delete next;
        }
        run_state = 3;
    }  // end while loop

    // printf("%d\n", num_expanded);
    double runtime = getTime() - t1;
    move_avg = move_alpha * num_expanded + (1 - move_alpha) * move_avg;
    // printf("tnum 1 num_epanded %ld avg %.1f time %.3fs\n", 
    //     num_expanded, move_avg, runtime);
    if(constraint_table.path_table_for_CT == nullptr) {
        if (path.empty())
        {
            printf("run_state %d focal_list %d\n", run_state, (int)focal_list.size());
            // printSearchTree();
            // exit(0);
        }
    }
    releaseNodes();
    return path;
}


// find path by A*
// Returns a path that minimizes the collisions with the paths in the path
// table, breaking ties by the length
Path SippParaHD::findPathParallel(const ConstraintTable& constraint_table) {
    reset();
    double t1 = getTime();
    // Path path = findNoCollisionPath(constraint_table);
    // if (!path.empty())
    //     return path;
    ReservationTable reservation_table(constraint_table, goal_location);
    Path path;
    Interval interval =
        reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0) return path;
    auto holding_time = constraint_table.getHoldingTime(
        goal_location, constraint_table.length_min);
    auto last_target_collision_time =
        constraint_table.getLastCollisionTimestep(goal_location);
    // generate start and add it to the OPEN & FOCAL list
    vector<int>& my_heuristic = *p_my_heuristic;
    auto h = max(max(my_heuristic[start_location], holding_time),
                 last_target_collision_time + 1);
    auto start =
        new SIPPNodeHD(start_location, 0, h, nullptr, 0, get<1>(interval),
                       get<1>(interval), get<2>(interval), get<2>(interval));
    // pushNodeToFocal(start);

    const int tnum_max = 4;
    static ThreadPool thread_pool(tnum_max);
    int tnum = 1;
    // if(move_avg >= 50000)
        tnum = tnum_max;
    start->node_id = generate_node_id();
    // if(income_buffer == nullptr)
    //     income_buffer = new hd_msg_buffer<SIPPNodeHD>[tnum];  //保存的是指针
    if(income_buffer.empty())
        for (int myId = 0; myId < tnum; myId++) 
            income_buffer.emplace_back();

    terminate.resize(tnum, false);
    terminalLocks.resize(tnum);

    cc = new CompletionCounter(tnum);
    upper_bound_cost = MAX_COST;
    upper_bound_collision = MAX_COST;
    done_par = false;
    num_of_created_llnode = 0;

    vector<SippParaHDThread*> sp_list;

    std::vector<std::future<int>> results;
    for (int myId = 0; myId < tnum; myId++) {
        SippParaHDThread* sp = new SippParaHDThread();
        sp->p = this;
        sp->tnum = tnum;
        sp->myId = myId;
        sp->holding_time_par = holding_time;
        sp->constraint_table_par = &constraint_table;
        sp->interval = interval;
        sp_list.push_back(sp);
    }

    int recv_id = sp_list[0]->computeRecipient(start);
    sp_list[recv_id]->pushNodeToFocal(start);
    // sp_list[recv_id]->allNodes_table.insert(start);

    for (int myId = 1; myId < tnum; myId++)
    	results.emplace_back(
    		thread_pool.enqueue(astar_helper, sp_list[myId])
    	);

    int r1 = astar_helper(sp_list[0]);   // for 1st thread, myid=0

    for (int i = 0; i < results.size(); i++) {
        int r2 = results[i].get();
    }

    for (int myId = 0; myId < tnum; myId++) {
        if (screen_const > 0)
            cout << myId << " " << sp_list[myId]->num_expanded << endl;
        num_expanded += sp_list[myId]->num_expanded;
        num_generated += sp_list[myId]->num_generated;
    }

    double runtime = getTime() - t1;
    move_avg = move_alpha * num_expanded + (1 - move_alpha) * move_avg;
    // printf("tnum %d num_epanded %ld avg %.1f time %.3fs\n", 
    //     tnum, num_expanded, move_avg, runtime);

    for (int myId = 0; myId < tnum; myId++) {
        sp_list[myId]->releaseNodes();
        delete sp_list[myId];
    }
    releaseNodes();
    // delete income_buffer;
    terminate.clear();
    delete cc;

    path = path_result;

    path_result.clear();
// exit(0);
    return path;
}

Path SippParaHD::findOptimalPath(const HLNode& node,
                                 const ConstraintTable& initial_constraints,
                                 const vector<Path*>& paths, int agent,
                                 int lowerbound) {
    return findSuboptimalPath(node, initial_constraints, paths, agent,
                              lowerbound, 1)
        .first;
}
// find path by SippParaHD
// Returns a shortest path that satisfies the constraints of the give node while
// minimizing the number of internal conflicts (that is conflicts with
// known_paths for other agents found so far). lowerbound is an underestimation
// of the length of the path in order to speed up the search.
pair<Path, int> SippParaHD::findSuboptimalPath(
    const HLNode& node, const ConstraintTable& initial_constraints,
    const vector<Path*>& paths, int agent, int lowerbound, double w) {
    //TODO
    return {Path(), min_f_val};
}

// TODO:: currently this is implemented in SippParaHD inefficiently
int SippParaHD::getTravelTime(int start, int end,
                              const ConstraintTable& constraint_table,
                              int upper_bound_cost) {
    reset();
    min_f_val = -1;  // this disables focal list
    int length = MAX_TIMESTEP;
    auto root = new SIPPNodeHD(start, 0, compute_heuristic(start, end), nullptr,
                               0, 1, 1, 0, 0);
    pushNodeToOpenAndFocal(root);
    auto static_timestep =
        constraint_table
            .getMaxTimestep();  // everything is static after this timestep
    while (!open_list.empty()) {
        auto curr = open_list.top();
        open_list.pop();
        if (curr->location == end) {
            length = curr->g_val;
            break;
        }
        list<int> next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations) {
            int next_timestep = curr->timestep + 1;
            int next_g_val = curr->g_val + 1;
            if (static_timestep <= curr->timestep) {
                if (curr->location == next_location) {
                    continue;
                }
                next_timestep--;
            }
            if (!constraint_table.constrained(next_location, next_timestep) &&
                !constraint_table.constrained(
                    curr->location, next_location,
                    next_timestep)) {  // if that grid is not blocked
                int next_h_val = compute_heuristic(next_location, end);
                if (next_g_val + next_h_val >=
                    upper_bound_cost)  // the cost of the path is larger than the
                                  // upper bound
                    continue;
                auto next = new SIPPNodeHD(
                    next_location, next_g_val, next_h_val, nullptr,
                    next_timestep, next_timestep + 1, next_timestep + 1, 0, 0);
                if (dominanceCheck(next))
                    pushNodeToOpenAndFocal(next);
                else
                    delete next;
            }
        }
    }
    releaseNodes();
    num_expanded = 0;
    num_generated = 0;
    num_reopened = 0;
    return length;
}

void SippParaHD::updateFocalList() {
    auto open_head = open_list.top();
    if (open_head->getFVal() > min_f_val) {
        int new_min_f_val = (int)open_head->getFVal();
        for (auto n : open_list) {
            if (n->getFVal() > w * min_f_val &&
                n->getFVal() <= w * new_min_f_val)
                n->focal_handle = focal_list.push(n);
        }
        min_f_val = new_min_f_val;
    }
}

inline void SippParaHD::pushNodeToOpenAndFocal(SIPPNodeHD* node) {
    num_generated++;
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    if (node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
    allNodes_table[node].push_back(node);
}
inline void SippParaHD::pushNodeToFocal(SIPPNodeHD* node) {
    num_generated++;
    allNodes_table[node].push_back(node);
    node->in_openlist = true;
    node->focal_handle =
        focal_list.push(node);  // we only use focal list; no open list is used
}
inline void SippParaHD::eraseNodeFromLists(SIPPNodeHD* node) {
    if (open_list.empty()) {  // we only have focal list
        focal_list.erase(node->focal_handle);
    } else if (focal_list.empty()) {  // we only have open list
        open_list.erase(node->open_handle);
    } else {  // we have both open and focal
        open_list.erase(node->open_handle);
        if (node->getFVal() <= w * min_f_val)
            focal_list.erase(node->focal_handle);
    }
}
void SippParaHD::releaseNodes() {
    open_list.clear();
    focal_list.clear();
    for (auto& node_list : allNodes_table) {
        for (auto n : node_list.second) delete n;
    }
    allNodes_table.clear();
    for (auto n : useless_nodes) delete n;
    useless_nodes.clear();
}

void SippParaHD::printSearchTree() const {
    vector<list<SIPPNodeHD*>> nodes;
    for (const auto& node_list : allNodes_table) {
        for (const auto& n : node_list.second) {
            if (nodes.size() <= n->timestep) nodes.resize(n->timestep + 1);
            nodes[n->timestep].emplace_back(n);
        }
    }
    cout << "Search Tree" << endl;
    for (int t = 0; t < nodes.size(); t++) {
        cout << "t=" << t << ":\t";
        for (const auto& n : nodes[t])
            cout << *n << "[" << n->timestep << "," << n->high_expansion
                 << "),c=" << n->num_of_conflicts << "\t";
        cout << endl;
    }
}

// return true iff we the new node is not dominated by any old node
bool SippParaHD::dominanceCheck(SIPPNodeHD* new_node) {
    auto ptr = allNodes_table.find(new_node);
    if (ptr == allNodes_table.end()) return true;
    // cout << ptr->second.size() << endl;
    for (auto old_node : ptr->second) {
        if (old_node->timestep <= new_node->timestep and
            old_node->num_of_conflicts <=
                new_node->num_of_conflicts) {  // the new node is dominated by
                                               // the old node
            return false;
        } else if (old_node->timestep >= new_node->timestep and
                   old_node->num_of_conflicts >=
                       new_node->num_of_conflicts)  // the old node is dominated
                                                    // by the new node
        {                                           // delete the old node
            if (old_node
                    ->in_openlist)  // the old node has not been expanded yet
                eraseNodeFromLists(
                    old_node);   // delete it from open and/or focal lists
            else                 // the old node has been expanded already
                num_reopened++;  // re-expand it
            ptr->second.remove(old_node);
            // delete old_node;
            useless_nodes.push_back(old_node);
            num_generated--;  // this is because we later will increase
                              // num_generated when we insert the new node into
                              // lists.
            return true;
        } else if (old_node->timestep < new_node->high_expansion and
                   new_node->timestep <
                       old_node->high_expansion) {  // intervals overlap --> we
                                                    // need to split the node to
                                                    // make them disjoint
            if (old_node->timestep <= new_node->timestep) {
                assert(old_node->num_of_conflicts > new_node->num_of_conflicts);
                old_node->high_expansion = new_node->timestep;
            } else  // i.e., old_node->timestep > new_node->timestep
            {
                assert(old_node->num_of_conflicts <=
                       new_node->num_of_conflicts);
                new_node->high_expansion = old_node->timestep;
            }
        }
    }
    return true;
}

int SippParaHD::astar_helper(void* arg) {
    SippParaHDThread* ptr = static_cast<SippParaHDThread*>(arg);
    return ptr->astar();
}

int SippParaHDThread::computeRecipient(SIPPNodeHD* node) {
    const int nblock = 8;
    int r = p->instance.getRowCoordinate(node->location) / nblock;
    int c = p->instance.getColCoordinate(node->location) / nblock;
    int w = p->instance.getCols() / nblock;
    int recv_id = int(r * w + c) % tnum;
    // int recv_id = int(r) % tnum;
    // int recv_id = int(node->location/32) % tnum;
    // int recv_id = int(node->high_generation) % tnum;
    return recv_id;
}

void SippParaHDThread::duplicate_node_reinsert(SIPPNodeHD* c,
                                               SIPPNodeHD* existing) {
    if (existing->getFVal() >
            c->getFVal() ||  // if f-val decreased through this new path
        (existing->getFVal() == c->getFVal() &&
         existing->num_of_conflicts >
             c->num_of_conflicts))  // or it remains the same but there's fewer
                                    // conflicts
    {
        if (!existing->in_openlist)  // if its in the closed list (reopen)
        {
            if (screen_const > 2)
                cout << "pe " << myId
                     << "    duplicate_node_reinsert, push new node c->node_id"
                     << c->node_id << " existing->node_id " << existing->node_id
                     << endl;
            existing->copy(*c);
            pushNode(existing);
        } else {
            if (screen_const > 2)
                cout << "pe " << myId
                     << "    duplicate_node_reinsert, update node c->node_id"
                     << c->node_id << " existing->node_id " << existing->node_id
                     << endl;
            existing->copy(*c);  // update existing node
            open_list.increase(
                existing->open_handle);  // increase because f-val improved
        }
    }
}

int SippParaHDThread::astar() {
    vector<int>& my_heuristic = *p->p_my_heuristic;
    // while (!open_list.empty())
    std::vector<std::vector<SIPPNodeHD*>> outgo_buffer;
    outgo_buffer.resize(tnum);
    // SIPPNodeHD* curr = new SIPPNodeHD();
    SIPPNodeHD* curr = nullptr;

    ReservationTable reservation_table(*constraint_table_par, p->goal_location);

    int run_state = 0;
    // while (!focal_list.empty() && !phase_collision_asynch_interrupt->load()
    // && !phase_cost_asynch_interrupt->load())
    while (true) {
        // check_state();
        if (p->phase_collision_asynch_interrupt->load()) {
            std::vector<SIPPNodeHD*> tmp;
            flush_receives(myId, tmp);
            run_state = 52;
            break;
        }
        if (p->phase_cost_asynch_interrupt->load()) {
            std::vector<SIPPNodeHD*> tmp;
            flush_receives(myId, tmp);
            run_state = 53;
            break;
        }
        // if(p->upper_bound_cost < MAX_COST) break;
        if (!p->income_buffer[myId].isempty()) {
            p->unset_my_terminated(myId);
            std::vector<SIPPNodeHD*> tmp;
            flush_receives(myId, tmp);
            for (int i = 0; i < tmp.size(); ++i) {
                // try to retrieve it from the hash table
                auto n = tmp[i];
                if (!better_than_upper_bound(n)) {
                    delete n;
                    continue;
                }
                if (dominanceCheck(n))
                    pushNodeToFocal(n);
                else
                    delete n;
            }
            tmp.clear();
        }
        while (!focal_list.empty() &&
               !better_than_upper_bound(focal_list.top())) {
            focal_list.top()->in_openlist = false;
            focal_list.pop();
        }
        if(screen_const > 1) {
            printState();
        }

        if (focal_list.empty()) {
            bool has_send = flush_sends(outgo_buffer, myId, 0);
            if (!has_send && p->income_buffer[myId].isempty()) {
                p->set_my_terminated(myId);
            }
            // num_of_no_node_null++;
            if (p->hasterminated()) {
                run_state = 51;
                if (screen_const > 0) printf("pe %d terminated\n", myId);
                break;
            }
            // usleep(100*1000);
            continue;  // ad hoc
        }
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        if(screen_const > 0)
            cout << "pe " << myId << " expand " << *curr << endl;
        // TODO: 先解决简单的路径,在解决难的
        // if(num_expanded > 10000)
        //     break;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->is_goal) {
            run_state = 1;
            p->processlock.lock();  // 不是很频繁，加锁应该影响不大
            p->upper_bound_cost = (int)curr->getFValW();
            p->upper_bound_collision = curr->num_of_conflicts;
            updatePath(curr, p->path_result);
            p->processlock.unlock();
            printState();
            continue;
        } else if (curr->location ==
                       p->goal_location &&  // arrive at the goal location
                   !curr->wait_at_goal &&   // not wait at the goal location
                   curr->timestep >=
                       holding_time_par)  // the agent can hold the goal
                                          // location afterward
        {
            int future_collisions =
                constraint_table_par->getFutureNumOfCollisions(curr->location,
                                                               curr->timestep);
            if (future_collisions == 0) {
                run_state = 2;
                p->processlock.lock();  // 不是很频繁，加锁应该影响不大
                p->upper_bound_cost = (int)curr->getFValW();
                p->upper_bound_collision = curr->num_of_conflicts;
                updatePath(curr, p->path_result);
                p->processlock.unlock();
                printState();
                continue;
            }
            // generate a goal node
            auto goal = new SIPPNodeHD(*curr);
            goal->node_id = generate_node_id();
            goal->is_goal = true;
            goal->h_val = 0;
            goal->num_of_conflicts += future_collisions;
            // try to retrieve it from the hash table
            send_next_node(outgo_buffer, goal);
        }

        for (int next_location : p->instance.getNeighbors(
                 curr->location))  // move to neighboring locations
        {
            for (auto& i : reservation_table.get_safe_intervals(
                     curr->location, next_location, curr->timestep + 1,
                     curr->high_expansion + 1)) {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion,
                    next_v_collision, next_e_collision) = i;
                if (next_timestep + my_heuristic[next_location] >
                    constraint_table_par->length_max)
                    break;
                auto next_collisions =
                    curr->num_of_conflicts +
                    (int)curr->collision_v *
                        max(next_timestep - curr->timestep - 1, 0)  // wait time
                    + (int)next_v_collision + (int)next_e_collision;
                auto next_h_val = max(
                    my_heuristic[next_location],
                    (next_collisions > 0 ? holding_time_par : curr->getFVal()) -
                        next_timestep);  // path max
                // generate (maybe temporary) node
                auto next = new SIPPNodeHD(
                    next_location, next_timestep, next_h_val, curr,
                    next_timestep, next_high_generation, next_high_expansion,
                    next_v_collision, next_collisions);
                next->node_id = generate_node_id();
                // try to retrieve it from the hash table
                send_next_node(outgo_buffer, next);
            }
        }  // end for loop that generates successors
        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location,
                                                 curr->high_expansion) and
            get<0>(interval) + curr->h_val <=
                reservation_table.constraint_table.length_max) {
            auto next_timestep = get<0>(interval);
            auto next_h_val =
                max(curr->h_val,
                    (get<2>(interval) ? holding_time_par : curr->getFVal()) -
                        next_timestep);  // path max
            auto next_collisions =
                curr->num_of_conflicts +
                (int)curr->collision_v *
                    max(next_timestep - curr->timestep - 1, 0) +
                (int)get<2>(interval);
            auto next = new SIPPNodeHD(curr->location, next_timestep,
                                       next_h_val, curr, next_timestep,
                                       get<1>(interval), get<1>(interval),
                                       get<2>(interval), next_collisions);
            next->node_id = generate_node_id();
            next->wait_at_goal = (curr->location == p->goal_location);
            send_next_node(outgo_buffer, next);
        }
        run_state = 3;
    }  // end while loop

    // printf("%d\n", num_expanded);
    if (constraint_table_par->path_table_for_CT == nullptr) {
        if (p->path_result.empty()) {
            printf("pe %d run_state %d focal_list %d\n", myId, run_state,
                   (int)focal_list.size());
            // printSearchTree();
            // exit(0);
        }
    }
    if(screen_const > 0) {
        printf("pe %d run_state %d focal_list %d\n", myId, run_state,
                (int)focal_list.size());
    }
    if (run_state < 52 && constraint_table_par->path_table_for_CAT != nullptr &&
        p->upper_bound_collision == MAX_COST) {
        printf("errror pe %d run_state %d focal_list %d collision %d cost %d\n",
               myId, run_state,
               (int)focal_list.size(), p->upper_bound_collision.load(),
               p->upper_bound_cost.load());
        exit(1);
    }
    return num_expanded;
}

void SippParaHDThread::flush_receives(int id, std::vector<SIPPNodeHD*>& tmp) {
    double t1 = getTime();
    int force_income = 0;
    if (p->income_buffer[id].size() >= p->income_threshold) {
        ++force_income;
        p->income_buffer[id].lock();
        tmp = p->income_buffer[id].pull_all_with_lock();
        p->income_buffer[id].release_lock();
    } else if (p->income_buffer[id].try_lock()) {
        tmp = p->income_buffer[id].pull_all_with_lock();
        p->income_buffer[id].release_lock();
    }
}

bool SippParaHDThread::flush_sends(
    std::vector<std::vector<SIPPNodeHD*>>& outgo_buffer, int id,
    int outgo_threshold) {
    int has_send = false;
    double t1 = getTime();
    for (int i = 0; i < tnum; ++i) {
        // least number of nodes to send
        if (i != id && outgo_buffer[i].size() > outgo_threshold) {
            if (p->income_buffer[i].try_lock()) {
                // acquired lock
                p->income_buffer[i].push_all_with_lock(outgo_buffer[i]);
                p->unset_my_terminated(i);
                p->income_buffer[i].release_lock();
                outgo_buffer[i].clear();
                has_send = true;
            }
        }
    }
    return has_send;
}

inline SIPPNodeHD* SippParaHDThread::popNode() {
    auto node = open_list.top();
    open_list.pop();
    node->in_openlist = false;
    num_expanded++;
    if (screen_const > 2)
        cout << "pe " << myId << "        popNode " << *node << " opensize "
             << open_list.size() << endl;
    return node;
}

inline void SippParaHDThread::pushNode(SIPPNodeHD* node) {
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
    if (screen_const > 2)
        cout << "pe " << myId << "        pushNode " << *node << " opensize "
             << open_list.size() << endl;
}

void SippParaHDThread::check_state() {
    // if (screen_const == 0) return;
    // p->openlistlock.lock();
    // for (auto node : allNodes_table) {
    //     if (node->in_openlist) {
    //         bool has = false;
    //         for (auto node2 : open_list)
    //             if (node == node2) has = true;
    //         if (!has) {
    //             cout << *node << endl;
    //             for (auto node2 : open_list) cout << *node2 << endl;
    //         }
    //         assert(has);
    //     }
    // }
    // p->openlistlock.unlock();
}

void SippParaHDThread::send_next_node(
    std::vector<std::vector<SIPPNodeHD*>>& outgo_buffer, SIPPNodeHD* next) {
    int recv_id = computeRecipient(next);
    if (screen_const > 0)
        cout << "pe " << myId << "    push c to recv_id " << recv_id << " "
             << *next << endl;

    if (recv_id == myId) {
        if (dominanceCheck(next))
            pushNodeToFocal(next);
        else
            delete next;
    } else {
        outgo_buffer[recv_id].push_back(next);
        flush_sends(outgo_buffer, myId, 0);
    }
}

inline void SippParaHDThread::pushNodeToFocal(SIPPNodeHD* node) {
    num_generated++;
    allNodes_table[node].push_back(node);
    node->in_openlist = true;
    node->focal_handle =
        focal_list.push(node);  // we only use focal list; no open list is used
    if(screen_const > 0)
        cout << "pe " << myId << "  pushNodeToFocal " << *node << " focal " << focal_list.size() << endl;
}
inline void SippParaHDThread::eraseNodeFromLists(SIPPNodeHD* node) {
    if (open_list.empty()) {  // we only have focal list
        focal_list.erase(node->focal_handle);
    } else if (focal_list.empty()) {  // we only have open list
        open_list.erase(node->open_handle);
    } else {  // we have both open and focal
        open_list.erase(node->open_handle);
        if (node->getFValW() <= p->w * p->min_f_val_atomic)
            focal_list.erase(node->focal_handle);
    }
}
void SippParaHDThread::releaseNodes() {
    open_list.clear();
    focal_list.clear();
    for (auto& node_list : allNodes_table) {
        for (auto n : node_list.second) delete n;
    }
    allNodes_table.clear();
    for (auto n : useless_nodes) delete n;
    useless_nodes.clear();
}

void SippParaHDThread::printSearchTree() const {
    vector<list<SIPPNodeHD*>> nodes;
    for (const auto& node_list : allNodes_table) {
        for (const auto& n : node_list.second) {
            if (nodes.size() <= n->timestep) nodes.resize(n->timestep + 1);
            nodes[n->timestep].emplace_back(n);
        }
    }
    cout << "Search Tree" << endl;
    for (int t = 0; t < nodes.size(); t++) {
        cout << "t=" << t << ":\t";
        for (const auto& n : nodes[t])
            cout << *n << "[" << n->timestep << "," << n->high_expansion
                 << "),c=" << n->num_of_conflicts << "\t";
        cout << endl;
    }
}

// return true iff we the new node is not dominated by any old node
bool SippParaHDThread::dominanceCheck(SIPPNodeHD* new_node) {
    auto ptr = allNodes_table.find(new_node);
    if (ptr == allNodes_table.end()) return true;
    // cout << ptr->second.size() << endl;
    for (auto old_node : ptr->second) {
        if (old_node->timestep <= new_node->timestep and
            old_node->num_of_conflicts <=
                new_node->num_of_conflicts) {  // the new node is dominated by
                                               // the old node
            return false;
        } else if (old_node->timestep >= new_node->timestep and
                   old_node->num_of_conflicts >=
                       new_node->num_of_conflicts)  // the old node is dominated
                                                    // by the new node
        {                                           // delete the old node
            if (old_node
                    ->in_openlist)  // the old node has not been expanded yet
                eraseNodeFromLists(
                    old_node);   // delete it from open and/or focal lists
            else                 // the old node has been expanded already
                num_reopened++;  // re-expand it
            ptr->second.remove(old_node);
            // delete old_node;
            useless_nodes.push_back(old_node);
            num_generated--;  // this is because we later will increase
                              // num_generated when we insert the new node into
                              // lists.
            return true;
        } else if (old_node->timestep < new_node->high_expansion and
                   new_node->timestep <
                       old_node->high_expansion) {  // intervals overlap --> we
                                                    // need to split the node to
                                                    // make them disjoint
            if (old_node->timestep <= new_node->timestep) {
                assert(old_node->num_of_conflicts > new_node->num_of_conflicts);
                old_node->high_expansion = new_node->timestep;
            } else  // i.e., old_node->timestep > new_node->timestep
            {
                assert(old_node->num_of_conflicts <=
                       new_node->num_of_conflicts);
                new_node->high_expansion = old_node->timestep;
            }
        }
    }
    return true;
}

void SippParaHDThread::updatePath(const LLNode* goal, vector<PathEntry>& path) {
    num_collisions = goal->num_of_conflicts;
    path.resize(goal->timestep + 1);
    // num_of_conflicts = goal->num_of_conflicts;

    const auto* curr = goal;
    while (curr->parent != nullptr)  // non-root node
    {
        const auto* prev = curr->parent;
        int t = prev->timestep + 1;
        while (t < curr->timestep) {
            path[t].location = prev->location;  // wait at prev location
            t++;
        }
        path[curr->timestep].location =
            curr->location;  // move to curr location
        curr = prev;
    }
    assert(curr->timestep == 0);
    path[0].location = curr->location;
}


void SippParaHDThread::printState() {
    if(screen_const <= 0) return;
    cout 
        << "pe " << myId
        << " UB collsion " << p->upper_bound_collision.load() 
        << " cost " << p->upper_bound_cost.load() 
        << " focal " << focal_list.size() 
        << " term ";
    for (int i = 0; i < p->terminate.size(); ++i) {
        cout << " " << p->terminate[i];
    }
    cout << endl;
    if(screen_const <= 1) return;
    heap_focal_t tmp = focal_list;
    int num = 0;
    while(!tmp.empty()) {
        cout << "  " << *tmp.top() << endl;
        tmp.pop();
        num++;
        if(num > 5) break;
    }
}

bool SippParaHDThread::better_than_upper_bound(SIPPNodeHD* node) {
    if(node->num_of_conflicts > p->upper_bound_collision.load())
        return false;
    else if(node->num_of_conflicts < p->upper_bound_collision.load())
        return true;
    else {   // collision equal
        if(node->getFValW() < p->upper_bound_cost.load())
            return true;
        else
            return false;
    }
}

void SippParaHD::unset_my_terminated(int myId) {
    std::unique_lock<std::mutex> lock(terminalLockG);
    bool change = false;
    terminalLocks[myId].lock();
    if (terminate[myId]) {
        change = true;
        terminate[myId] = false;
        cc->uncomplete();
    }
    terminalLocks[myId].unlock();
    if (screen_const > 1 && change) {
        // checkState();
    }
}

void SippParaHD::set_my_terminated(int myId) {
    std::unique_lock<std::mutex> lock(terminalLockG);
    bool change = false;
    terminalLocks[myId].lock();
    if (income_buffer[myId].isempty()) {
        if (!terminate[myId]) {
            change = true;
            terminate[myId] = true;
            cc->complete();
        }
    }
    terminalLocks[myId].unlock();
    if (screen_const > 1 && change) {
        // checkState();
    }
}

bool SippParaHD::hasterminated() {
    std::unique_lock<std::mutex> lock(terminalLockG);
    bool term = true;
    for (int i = 0; i < (int)terminalLocks.size(); i++) 
        terminalLocks[i].lock();
    term = cc->is_complete();
    for (int i = 0; i < (int)terminalLocks.size(); i++)
        terminalLocks[i].unlock();
    if (term && screen_const > 1) {
        // checkState();
    }
    return term;
}
