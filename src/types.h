//
// Created by Oleksandr Semeniuta on 2019-03-06.
//

#ifndef PATH_PLANNING_TYPES_H

const double MAX_SPEED_MPH = 49.5;

struct map_waypoints {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> dx;
  std::vector<double> dy;
};

struct sf_vehicle {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};

struct vehicle_info {
  int id;
  double s_dist;
  int lane;
};

enum class ego_state {
  start, keep_lane, prepare_to_change_lane, change_lane,
};

struct frenet_coord {
  double s;
  double d;
};

struct pp_input {

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
  std::vector<sf_vehicle> sensor_fusion;
};

struct pp_output {
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;
};

class PathPlanner {

public:

  PathPlanner() = default;

  virtual pp_output plan(const pp_input& in, const map_waypoints& wp) = 0;

};

#define PATH_PLANNING_TYPES_H

#endif //PATH_PLANNING_TYPES_H
