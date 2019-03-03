//
// Created by Oleksandr Semeniuta on 2019-02-23.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>

struct map_waypoints {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> dx;
  std::vector<double> dy;
};

class PathPlanner {

public:

  struct input {

    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    // Previous path data given to the planner
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;

    // Previous path's end s and d values
    double end_path_s;
    double end_path_d;

    // A list of all other cars on the same side of the road.
    std::vector<std::vector<double>> sensor_fusion;
  };

  struct output {
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
  };

  PathPlanner() = default;

  virtual output plan(const input& in, const map_waypoints& wp) = 0;

};


class PolynomialPathPlanner : public PathPlanner {

public:

  PathPlanner::output plan(const PathPlanner::input& in, const map_waypoints& wp) override;

};

class FrenetPathPlanner : public PathPlanner {

public:

  PathPlanner::output plan(const PathPlanner::input& in, const map_waypoints& wp) override;

};


class StraightPathPlanner : public PathPlanner {

public:

  PathPlanner::output plan(const PathPlanner::input& in, const map_waypoints& wp) override;

};

#endif //PATH_PLANNING_PATHPLANNER_H
