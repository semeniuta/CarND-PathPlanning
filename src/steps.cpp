//
// Created by Oleksandr Semeniuta on 2019-03-04.
//

#include "steps.h"
#include <iostream>
#include "helpers.h"
#include "geometry.h"

ReferenceState prepareReferenceState(const PathPlanner::input& in) {

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

Eigen::VectorXd fitPolynomial(const PathPlanner::input& in,
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

void fillNextXY(PathPlanner::output* out,
                const PathPlanner::input& in,
                double target_velocity,
                const ReferencePoses& poses,
                const Eigen::VectorXd& coeffs) {

  // Fill the next x/y values with the previous path

  auto prev_path_size = in.previous_path_x.size();

  if (!in.previous_path_x.empty()) {

    for (unsigned i = 0; i < prev_path_size; i++) {

      out->next_x_vals.push_back(in.previous_path_x[i]);
      out->next_y_vals.push_back(in.previous_path_y[i]);

    }

  }

  // Fill the rest of the next x/y values with
  // the values based on the polynomial

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

