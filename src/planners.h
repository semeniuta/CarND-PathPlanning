//
// Created by Oleksandr Semeniuta on 2019-02-23.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
#include "types.h"
#include "helpers.h"


class PolynomialPathPlanner : public PathPlanner {

public:

  pp_output plan(const pp_input& in, const map_waypoints& wp) override;

};

class TrafficAwarePathPlanner : public PathPlanner {

public:

  pp_output plan(const pp_input& in, const map_waypoints& wp) override;

private:

  int current_lane_{1};
  bool too_close_{false};
  double target_velocity_{MAX_SPEED_MPH};
  bool start_{true};


};

class FrenetPathPlanner : public PathPlanner {

public:

  pp_output plan(const pp_input& in, const map_waypoints& wp) override;

};


class StraightPathPlanner : public PathPlanner {

public:

  pp_output plan(const pp_input& in, const map_waypoints& wp) override;

};

#endif //PATH_PLANNING_PATHPLANNER_H
