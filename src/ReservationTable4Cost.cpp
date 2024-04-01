#include "ReservationTable4Cost.h"


void ReservationTable4Cost::insert2SIT(int location, int t_min, int t_max)
{
	assert(t_min >= 0 and t_min < t_max and !sit[location].empty());
    for (auto it = sit[location].begin(); it != sit[location].end();)
    {
        if (t_min >= get<1>(*it))
			++it; 
        else if (t_max <= get<0>(*it))
            break;
        else if (get<0>(*it) < t_min && get<1>(*it) <= t_max)
        {
            (*it) = make_tuple(get<0>(*it), t_min);
			++it;
        }
        else if (t_min <= get<0>(*it) && t_max < get<1>(*it))
        {
            (*it) = make_tuple(t_max, get<1>(*it));
            break;
        }
        else if (get<0>(*it) < t_min && t_max < get<1>(*it))
        {
            sit[location].insert(it, make_tuple(get<0>(*it), t_min));
            (*it) = make_tuple(t_max, get<1>(*it));
            break;
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            it = sit[location].erase(it);
        }
    }
}


// update SIT at the given location
void ReservationTable4Cost::updateSIT(int location)
{
    assert(sit[location].empty());
    // length constraints for the goal location
    if (location == goal_location) // we need to divide the same intervals into 2 parts [0, length_min) and [length_min, length_max + 1)
    {
        if (constraint_table.length_min > constraint_table.length_max) // the location is blocked for the entire time horizon
        {
            sit[location].emplace_back(0, 0);
            return;
        }
        if (0 < constraint_table.length_min)
        {
            sit[location].emplace_back(0, constraint_table.length_min);
        }
        assert(constraint_table.length_min >= 0);
        sit[location].emplace_back(constraint_table.length_min, min(constraint_table.length_max + 1, MAX_TIMESTEP));
    }
    else
    {
        sit[location].emplace_back(0, min(constraint_table.length_max, MAX_TIMESTEP - 1) + 1);
    }
    // path table
    if (constraint_table.path_table_for_CT != nullptr and !constraint_table.path_table_for_CT->table.empty())
    {
        if (location < constraint_table.map_size) // vertex conflict
        {
            for (int t = 0; t < (int)constraint_table.path_table_for_CT->table[location].size(); t++)
            {
                if (constraint_table.path_table_for_CT->table[location][t] != NO_AGENT)
                {
                    insert2SIT(location, t, t+1);
                }
            }
            if (constraint_table.path_table_for_CT->goals[location] < MAX_TIMESTEP) // target conflict
                insert2SIT(location, constraint_table.path_table_for_CT->goals[location], MAX_TIMESTEP + 1);
        }
        else // edge conflict
        {
            auto from = location / constraint_table.map_size - 1;
            auto to = location % constraint_table.map_size;
            if (from != to)
            {
                int t_max = (int) min(constraint_table.path_table_for_CT->table[from].size(),
                                      constraint_table.path_table_for_CT->table[to].size() + 1);
                for (int t = 1; t < t_max; t++)
                {
                    if (constraint_table.path_table_for_CT->table[to][t - 1] != NO_AGENT and
                        constraint_table.path_table_for_CT->table[to][t - 1] ==
                        constraint_table.path_table_for_CT->table[from][t])
                    {
                        insert2SIT(location, t, t+1);
                    }
                }
            }
        }
    }

    // negative constraints
    const auto& it = constraint_table.ct.find(location);
    if (it != constraint_table.ct.end())
    {
        for (auto time_range : it->second)
            insert2SIT(location, time_range.first, time_range.second);
    }

    // positive constraints
    if (location < constraint_table.map_size)
    {
        for (auto landmark : constraint_table.landmarks)
        {
            if (landmark.second != location)
            {
                insert2SIT(location, landmark.first, landmark.first + 1);
            }
        }
    }
}


// return <upper_bound, low, high,  vertex collision, edge collision>
list<tuple<int, int, int, bool, bool>> ReservationTable4Cost::get_safe_intervals(int from, int to, int lower_bound, int upper_bound)
{
    list<tuple<int, int, int, bool, bool>> rst;
    if (lower_bound >= upper_bound)
        return rst;

    if (sit[to].empty()) {
        updateSIT(to);
        sit_data_size += sit[to].size();
    }

    // cout << sit[to].size() << endl;
    if(sit[to].size() > 0 && false) {
        cout << sit[to].size() << endl;
        for(auto a : sit[to])
            cout << get<0>(a) << " " << get<1>(a) << endl;
        cout << endl;
    }
    for(auto Interval2 : sit[to])
    {
        if (lower_bound >= get<1>(Interval2))
            continue;
        else if (upper_bound <= get<0>(Interval2))
            break;
        // the Interval2 overlaps with [lower_bound, upper_bound)
        auto t1 = get_earliest_arrival_time(from, to,
                max(lower_bound, get<0>(Interval2)), min(upper_bound, get<1>(Interval2)));
        if (t1 < 0) // the Interval2 is not reachable
            continue;
        else // the Interval2 does not have collisions
        { // so we need to check the move action has collisions or not
            auto t2 = get_earliest_no_collision_arrival_time(from, to, Interval2, t1, upper_bound);
            if (t1 == t2)
                rst.emplace_back(get<1>(Interval2), t1, get<1>(Interval2), false, false);
            else if (t2 < 0)
                rst.emplace_back(get<1>(Interval2), t1, get<1>(Interval2), false, true);
            else
            {
                rst.emplace_back(get<1>(Interval2), t1, t2, false, true);
                rst.emplace_back(get<1>(Interval2), t2, get<1>(Interval2), false, false);
            }
        }
    }
    return rst;
}

Interval ReservationTable4Cost::get_first_safe_interval(size_t location)
{
    if (sit[location].empty())
	    updateSIT(location);
    auto i = sit[location].front();
    Interval interval = Interval(get<0>(i), get<1>(i), false);
    return interval;
}

// find a safe Interval2 with t_min as given
bool ReservationTable4Cost::find_safe_interval(Interval& interval2, size_t location, int t_min)
{
	if (t_min >= min(constraint_table.length_max, MAX_TIMESTEP - 1) + 1)
		return false;
    if (sit[location].empty())
	    updateSIT(location);
    for( auto & i : sit[location])
    {
        if ((int)get<0>(i) <= t_min && t_min < (int)get<1>(i))
        {
            interval2 = Interval(t_min, get<1>(i), false);
            return true;
        }
        else if (t_min < (int)get<0>(i))
            break;
    }
    return false;
}

int ReservationTable4Cost::get_earliest_arrival_time(int from, int to, int lower_bound, int upper_bound) const
{
    for (auto t = lower_bound; t < upper_bound; t++)
    {
        if (!constraint_table.constrained(from, to, t))
            return t;
    }
    return -1;
}
int ReservationTable4Cost::get_earliest_no_collision_arrival_time(int from, int to, const Interval2& Interval2,
                                                             int lower_bound, int upper_bound) const
{
    for (auto t = max(lower_bound, get<0>(Interval2)); t < min(upper_bound, get<1>(Interval2)); t++)
    {
        if (!constraint_table.hasEdgeConflict(from, to, t))
            return t;
    }
    return -1;
}