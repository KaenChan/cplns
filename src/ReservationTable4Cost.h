// This is used by SIPP
#pragma once
#include "VectorErase.h"
#include "ConstraintTable.h"
#include "ReservationTable.h"

typedef tuple<int, int> Interval2; // [t_min, t_max)

class ReservationTable4Cost: public ReservationTableAbs
{
public:
    const ConstraintTable& constraint_table;

    ReservationTable4Cost(const ConstraintTable& constraint_table, int goal_location) :
        constraint_table(constraint_table), goal_location(goal_location), sit(constraint_table.map_size) {

        pconstraint_table = &constraint_table;
    }

    list<tuple<int, int, int, bool, bool> > get_safe_intervals(int from, int to, int lower_bound, int upper_bound);
    Interval get_first_safe_interval(size_t location);
    bool find_safe_interval(Interval& Interval2, size_t location, int t_min);

	// Safe Interval2 Table (SIT)
	// typedef vector< list<Interval2> > SIT2;
	typedef vector< VectorE_t<Interval2> > SIT2;
    SIT2 sit; // location -> [t_min, t_max), num_of_collisions
    int sit_data_size = 0;

private:
    int goal_location;
    void insert2SIT(int location, int t_min, int t_max);
	// void mergeIntervals(list<Interval2 >& intervals) const;
	void updateSIT(int location); // update SIT at the given location
    int get_earliest_arrival_time(int from, int to, int lower_bound, int upper_bound) const;
    int get_earliest_no_collision_arrival_time(int from, int to, const Interval2& Interval2,
                                               int lower_bound, int upper_bound) const;
};
