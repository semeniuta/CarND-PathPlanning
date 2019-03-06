//
// Created by Oleksandr Semeniuta on 2019-03-03.
//

#include "PathPlanner.h"
#include <iostream>
#include <cmath>
#include "Eigen-3.3/Eigen/Dense"
#include "geometry.h"
#include "steps.h"
#include "helpers.h"


pp_output TrafficAwarePathPlanner::plan(const pp_input& in, const map_waypoints& wp) {

  too_close_ = false;

  double lane_d = laneD(current_lane_);

  printCarState(in);
  printPrevPathDetails(in);

  for (const sf_vehicle& vehicle : in.sensor_fusion) {

    int v_lane = getCarLane(vehicle);

    if (v_lane == current_lane_ && vehicle.s > in.car_s) {

      double v_dist = vehicle.s - in.car_s;

      if (v_dist < 40) {
        too_close_ = true;
      }
    }

  }

  if (too_close_) {

    target_velocity_ -= 0.2;

    if (target_velocity_ <= 30) {
      target_velocity_ = 30;
    }

  } else {

    if (target_velocity_ < MAX_SPEED_MPH) {
      target_velocity_ += 0.2;
    }

  }

  ReferenceState ref = prepareReferenceState(in);
  ReferencePoses poses = createPoses(ref);
  Eigen::VectorXd coeffs = fitPolynomial(in, wp, ref, poses, lane_d);

  pp_output out{};
  fillNextXY(&out, in, MPH2Metric(target_velocity_), poses, coeffs);

  printNextXY(out);
  std::cout << std::endl;

  return out;

}

pp_output PolynomialPathPlanner::plan(const pp_input& in, const map_waypoints& wp) {

  double target_velocity = MPH2Metric(49.5);
  double lane_d = laneD(1);

  printCarState(in);
  printPrevPathDetails(in);

  ReferenceState ref = prepareReferenceState(in);
  ReferencePoses poses = createPoses(ref);
  Eigen::VectorXd coeffs = fitPolynomial(in, wp, ref, poses, lane_d);

  pp_output out{};
  fillNextXY(&out, in, target_velocity, poses, coeffs);

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