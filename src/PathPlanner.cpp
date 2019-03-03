//
// Created by Oleksandr Semeniuta on 2019-03-03.
//

#include "PathPlanner.h"
#include <iostream>
#include <cmath>
#include "helpers.h"
#include "Eigen-3.3/Eigen/Dense"
#include "geometry.h"


PathPlanner::output PolynomialPathPlanner::plan(const PathPlanner::input& in, const map_waypoints& wp) {

  const int TRAJ_SIZE = 50;
  const double DT = 0.02;

  double target_velocity = MPH2Metric(49.5);
  double lane_d = laneD(1);

  PathPlanner::output out{};

  printCarState(in);
  printPrevPathDetails(in);

  auto prev_path_size = in.previous_path_x.size();

  Eigen::VectorXd ref_state{3};
  double ref_yaw;

  std::vector<Eigen::VectorXd> points;

  if (prev_path_size < 2) {

    ref_state << in.car_x,
                 in.car_y,
                        1;

    ref_yaw = deg2rad(in.car_yaw);

  } else {

    ref_state << in.previous_path_x[prev_path_size - 1],
                 in.previous_path_y[prev_path_size - 1],
                                                     1;

    double prev_x = in.previous_path_x[prev_path_size - 2];
    double prev_y = in.previous_path_y[prev_path_size - 2];

    ref_yaw = atan2(ref_state(1) - prev_y, ref_state(0) - prev_x);

  }

  Eigen::MatrixXd ego_pose = createPose(ref_state(0), ref_state(1), ref_yaw);
  Eigen::MatrixXd to_ego = invertPose(ego_pose);

  points.push_back(ref_state);

  std::vector<double> s_increments = {30, 60, 90};

  for (double s_inc : s_increments) {

    auto xy = getXY(in.car_s + s_inc, lane_d, wp.s, wp.x, wp.y);
    Eigen::VectorXd p{3};
    p << xy[0], xy[1], 1;

    points.push_back(p);

  }

  Eigen::VectorXd anchor_x{points.size()};
  Eigen::VectorXd anchor_y{points.size()};

  for (unsigned int i = 0; i < points.size(); i++) {

    Eigen::VectorXd p = points[i];
    Eigen::VectorXd p_t = to_ego * p;

    anchor_x(i) = p_t(0) / p_t(2);
    anchor_y(i) = p_t(1) / p_t(2);
  }

  Eigen::VectorXd coeffs = polyfit(anchor_x, anchor_y, 3);

  if (!in.previous_path_x.empty()) {

    for (unsigned i = 0; i < prev_path_size; i++) {

      out.next_x_vals.push_back(in.previous_path_x[i]);
      out.next_y_vals.push_back(in.previous_path_y[i]);

    }

  }

  double dx = target_velocity * DT;
  auto n_poly_points = TRAJ_SIZE - prev_path_size;

  for (unsigned int i = 0; i < n_poly_points; i++) {

    double x = (i + 1) * dx;
    double y = polyeval(coeffs, x);

    Eigen::VectorXd p{3};
    p << x, y, 1;

    Eigen::VectorXd p_world = ego_pose * p;

    out.next_x_vals.push_back(p_world(0) / p_world(2));
    out.next_y_vals.push_back(p_world(1) / p_world(2));

  }

  std::cout << "x = ";
  printVector(out.next_x_vals);

  std::cout << "y = ";
  printVector(out.next_y_vals);

  std::cout << std::endl;

  return out;

}


PathPlanner::output FrenetPathPlanner::plan(const PathPlanner::input& in, const map_waypoints& wp) {

  PathPlanner::output out{};

  double s = in.car_s;
  double d = in.car_d;

  std::cout << "car = [" << in.car_x << ", " << in.car_y << "]\n";
  std::cout << "prev_path_size = " << in.previous_path_x.size() << std::endl;
  if (!in.previous_path_x.empty()) {
    std::cout << "prev_path_first = [" << in.previous_path_x[0] << ", " << in.previous_path_y[0] << "]\n";
  }

  double dist_inc = 0.2;
  for (int i = 1; i <= 50; ++i) {
    s += dist_inc;

    std::vector<double> xy = getXY(s, d, wp.s, wp.x, wp.y);
    out.next_x_vals.push_back(xy[0]);
    out.next_y_vals.push_back(xy[1]);

  }

  printNextXY(out);
  std::cout << std::endl;

  return out;


}


PathPlanner::output StraightPathPlanner::plan(const PathPlanner::input& in, const map_waypoints& wp) {

  PathPlanner::output out{};

  double s = in.car_s;
  double d = in.car_d;

  double dist_inc = 0.5;
  for (int i = 0; i < 50; ++i) {
    out.next_x_vals.push_back(in.car_x + (dist_inc * i) * cos(deg2rad(in.car_yaw)));
    out.next_y_vals.push_back(in.car_y + (dist_inc * i) * sin(deg2rad(in.car_yaw)));
  }

  printVector(out.next_x_vals);
  printVector(out.next_y_vals);

  return out;

}