//
// Created by Oleksandr Semeniuta on 2019-03-03.
//

#include "planners.h"
#include <iostream>
#include <cmath>
#include "Eigen-3.3/Eigen/Dense"
#include "geometry.h"
#include "steps.h"
#include "helpers.h"


pp_output TrafficAwarePathPlanner::plan(const pp_input& in, const map_waypoints& wp) {

  printCarState(in);
  printPrevPathDetails(in);

  too_close_ = checkForCarInFront(in, current_lane_, 40);
  target_velocity_ = updateTargetVelocity(too_close_, target_velocity_, 0.2, 30);

  ReferenceState ref = prepareReferenceState(in);
  ReferencePoses poses = createPoses(ref);

  auto vehiles_info = lookAround(in);
  if (too_close_) {

    auto neighbor_lanes = neighbors(current_lane_);

    for (int candidate_lane : neighbor_lanes) {
      if (safeInLane(candidate_lane, vehiles_info)) {
        current_lane_ = candidate_lane;
      }
    }

  }

  double lane_d = laneD(current_lane_);
  Eigen::VectorXd coeffs = fitPolynomial(in, wp, ref, poses, lane_d);

  pp_output out{};

  if (start_) {

    double accel_to_mph = 20;
    fillNextNYFirstTime(&out, accel_to_mph, poses, coeffs);

    start_ = false;
    target_velocity_ = accel_to_mph + 1;

  } else {

    fillNextXYFromPrevious(&out, in);
    fillNextXYTargetV(&out, in, target_velocity_, poses, coeffs);

  }

  printNextXY(out);
  std::cout << std::endl;

  return out;

}

pp_output PolynomialPathPlanner::plan(const pp_input& in, const map_waypoints& wp) {

  double target_velocity = 49.5;
  double lane_d = laneD(1);

  printCarState(in);
  printPrevPathDetails(in);

  ReferenceState ref = prepareReferenceState(in);
  ReferencePoses poses = createPoses(ref);
  Eigen::VectorXd coeffs = fitPolynomial(in, wp, ref, poses, lane_d);

  pp_output out{};
  fillNextXYFromPrevious(&out, in);
  fillNextXYTargetV(&out, in, target_velocity, poses, coeffs);

  printNextXY(out);
  std::cout << std::endl;

  return out;

}


pp_output FrenetPathPlanner::plan(const pp_input& in, const map_waypoints& wp) {

  pp_output out{};

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


pp_output StraightPathPlanner::plan(const pp_input& in, const map_waypoints& wp) {

  pp_output out{};

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