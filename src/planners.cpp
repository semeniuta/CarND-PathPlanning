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

  pp_output out{};

  too_close_ = checkForCarInFront(in, target_lane_, 40);
  target_velocity_ = updateTargetVelocity(too_close_, target_velocity_, 0.1, 30);

  ReferenceState ref = prepareReferenceState(in);
  ReferencePoses poses = createPoses(ref);

  double lane_d = laneD(target_lane_);
  double lane_d_0 = laneD(source_lane_);

  const int lc_counter_resolution = 80;
  double d_diff = lc_counter_ * ((lane_d - lane_d_0) / lc_counter_resolution);
  double target_d = lane_d - d_diff;

  std::vector<frenet_coord> next_frenet_points = {
      {30, target_d},
      {60, target_d},
      {90, target_d},
  };

  Eigen::VectorXd coeffs = fitPolynomial(in, wp, ref, next_frenet_points, poses);

  switch (state_) {

    case ego_state::start: {

      double accel_to_mph = 20;
      fillNextNYFirstTime(&out, accel_to_mph, poses, coeffs);

      start_ = false;
      target_velocity_ = accel_to_mph + 1;

      state_ = ego_state::keep_lane;

    }
    break;

    case ego_state::keep_lane: {

      if (too_close_) {
        state_ = ego_state::prepare_to_change_lane;
      }

      fillNextXYFromPrevious(&out, in);
      fillNextXYTargetV(&out, in, target_velocity_, poses, coeffs);

    }
    break;

    case ego_state::prepare_to_change_lane: {

      if (!too_close_) {

        fillNextXYFromPrevious(&out, in);
        state_ = ego_state::keep_lane;

      } else {

        int new_lane = checkIfSafeToChangeLane(in, target_lane_);
        bool safe_to_change_lane = (new_lane != -1) && (target_velocity_ < 40);
        //bool safe_to_change_lane = (new_lane != -1);
        if (safe_to_change_lane) {

          source_lane_ = target_lane_;
          target_lane_ = new_lane;

          lc_counter_ = lc_counter_resolution;

          fillNextXYFromPrevious(&out, in, 5);

          // --------------------------------------------------

//          double d_src = laneD(source_lane_);
//          double d_dst = (d_src + laneD(target_lane_)) / 2.;
//          out = JMTLaneChange(in, wp, d_src, d_dst, 20);
//          std::cout << "CHANGE\n";

          // --------------------------------------------------

          state_ = ego_state::change_lane;

        } else {

          fillNextXYFromPrevious(&out, in);
          fillNextXYTargetV(&out, in, target_velocity_, poses, coeffs);

        }

      }

    }
    break;

    case ego_state::change_lane: {

      //source_lane_ = target_lane_;

      if (lc_counter_ > 0) {
        lc_counter_--;
      }

      fillNextXYFromPrevious(&out, in);
      fillNextXYTargetV(&out, in, target_velocity_, poses, coeffs);

      if (fabs(in.car_d - lane_d) < 0.001) {
        source_lane_ = target_lane_;
        state_ = ego_state::keep_lane;
      }

    }
    break;

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

  std::vector<frenet_coord> next_frenet_points = {
      {30, lane_d},
      {60, lane_d},
      {90, lane_d}
  };

  Eigen::VectorXd coeffs = fitPolynomial(in, wp, ref, next_frenet_points, poses);

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