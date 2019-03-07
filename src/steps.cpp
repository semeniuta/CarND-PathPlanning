//
// Created by Oleksandr Semeniuta on 2019-03-04.
//

#include "steps.h"
#include <iostream>
#include "helpers.h"
#include "geometry.h"

ReferenceState prepareReferenceState(const pp_input& in) {

  // Prepare the reference state

  ReferenceState ref{};

  auto prev_path_size = in.previous_path_x.size();

  std::vector<Eigen::VectorXd> points;

  if (prev_path_size < 2) {

    ref.xy_h << in.car_x,
        in.car_y,
        1;

    ref.yaw = deg2rad(in.car_yaw);

  } else {

    ref.xy_h << in.previous_path_x[prev_path_size - 1],
        in.previous_path_y[prev_path_size - 1],
        1;

    double prev_x = in.previous_path_x[prev_path_size - 2];
    double prev_y = in.previous_path_y[prev_path_size - 2];

    ref.yaw = atan2(ref.xy_h(1) - prev_y, ref.xy_h(0) - prev_x);

  }

  return ref;

}

ReferencePoses createPoses(const ReferenceState& ref) {

  // Create homogeneous transformations (ego-in-world, world-in-ego)

  ReferencePoses poses{};

  poses.ego_in_world = createPose(ref.xy_h(0), ref.xy_h(1), ref.yaw);
  poses.world_in_ego = invertPose(poses.ego_in_world);

  return poses;

}

Eigen::VectorXd fitPolynomial(const pp_input& in,
                              const map_waypoints& wp,
                              const ReferenceState& ref,
                              const ReferencePoses& poses,
                              double lane_d) {
  // Fit polynomial

  std::vector<Eigen::VectorXd> points;
  points.push_back(ref.xy_h);

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
    Eigen::VectorXd p_t = poses.world_in_ego * p;

    anchor_x(i) = p_t(0) / p_t(2);
    anchor_y(i) = p_t(1) / p_t(2);
  }

  Eigen::VectorXd coeffs = polyfit(anchor_x, anchor_y, 3);

  return coeffs;

}

void fillNextXYFromPrevious(pp_output* out, const pp_input& in) {

  // Fill the next x/y values with the previous path

  auto prev_path_size = in.previous_path_x.size();

  if (!in.previous_path_x.empty()) {

    for (unsigned i = 0; i < prev_path_size; i++) {

      out->next_x_vals.push_back(in.previous_path_x[i]);
      out->next_y_vals.push_back(in.previous_path_y[i]);

    }

  }

}

void fillNextXYTargetV(pp_output* out,
                       const pp_input& in,
                       double target_velocity,
                       const ReferencePoses& poses,
                       const Eigen::VectorXd& coeffs) {

  // Fill the rest of the next x/y values with
  // the values based on the polynomial

  auto prev_path_size = in.previous_path_x.size();

  double dx = target_velocity * DT;
  auto n_poly_points = TRAJ_SIZE - prev_path_size;

  for (unsigned int i = 0; i < n_poly_points; i++) {

    double x = (i + 1) * dx;
    double y = polyeval(coeffs, x);

    Eigen::VectorXd p{3};
    p << x, y, 1;

    Eigen::VectorXd p_world = poses.ego_in_world * p;

    out->next_x_vals.push_back(p_world(0) / p_world(2));
    out->next_y_vals.push_back(p_world(1) / p_world(2));

  }

}

void fillNextNYFirstTime(pp_output* out,
                         double accel_to_mph,
                         const ReferencePoses& poses,
                         const Eigen::VectorXd& coeffs) {

  auto accel_increments = accel(0, MPH2Metric(accel_to_mph), 1.);

  double x = 0.;
  for (double dx : accel_increments) {

    x += dx;

    double y = polyeval(coeffs, x);

    Eigen::VectorXd p{3};
    p << x, y, 1;

    Eigen::VectorXd p_world = poses.ego_in_world * p;

    out->next_x_vals.push_back(p_world(0) / p_world(2));
    out->next_y_vals.push_back(p_world(1) / p_world(2));

  }

}

bool checkForCarInFront(const pp_input& in,
                        int current_lane,
                        double s_threshold) {

  for (const sf_vehicle& vehicle : in.sensor_fusion) {

    int v_lane = getCarLane(vehicle);

    if (v_lane == current_lane && vehicle.s > in.car_s) {

      double v_dist = vehicle.s - in.car_s;

      if (v_dist < s_threshold) {
        return true;
      }
    }

  }

  return false;

}

double updateTargetVelocity(bool too_close,
                            double target_velocity,
                            double velocity_increment,
                            double min_speed) {

  if (too_close) {

    target_velocity -= velocity_increment;

    if (target_velocity <= min_speed) {
      target_velocity = min_speed;
    }

  } else {

    if (target_velocity < MAX_SPEED_MPH) {
      target_velocity += velocity_increment;
    }

  }

  return target_velocity;

}

