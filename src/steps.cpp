//
// Created by Oleksandr Semeniuta on 2019-03-04.
//

#include "steps.h"
#include <iostream>
#include "helpers.h"
#include "geometry.h"

ReferenceState prepareReferenceState(const pp_input& in, long index) {

  // Prepare the reference state

  ReferenceState ref{};

  auto prev_path_size = in.previous_path_x.size();
  if (index == -1) {
    index = prev_path_size - 1;
  }

  std::vector<Eigen::VectorXd> points;

  if (prev_path_size < 2) {

    ref.xy_h << in.car_x,
        in.car_y,
        1;

    ref.yaw = deg2rad(in.car_yaw);

  } else {

    ref.xy_h << in.previous_path_x[index],
        in.previous_path_y[prev_path_size - 1],
        1;

    double prev_x = in.previous_path_x[index - 1];
    double prev_y = in.previous_path_y[index - 1];

    ref.yaw = atan2(ref.getY() - prev_y, ref.getX() - prev_x);

  }

  return ref;

}

ReferencePoses createPoses(const ReferenceState& ref) {

  // Create homogeneous transformations (ego-in-world, world-in-ego)

  ReferencePoses poses{};

  poses.ego_in_world = createPose(ref.getX(), ref.getY(), ref.yaw);
  poses.world_in_ego = invertPose(poses.ego_in_world);

  return poses;

}

Eigen::VectorXd fitPolynomial(const pp_input& in,
                              const map_waypoints& wp,
                              const ReferenceState& ref,
                              const std::vector<frenet_coord>& next_frenet_points,
                              const ReferencePoses& poses,
                              bool use_car) {

  // when use_car is false,
  // start with ref as the first anchor point
  // otherwise, use car's current position

  std::vector<Eigen::VectorXd> points;
  points.push_back(ref.xy_h);

  double start_s = in.car_s;
  if (!use_car) {
    auto ref_frenet = getFrenet(ref.getX(), ref.getY(), ref.yaw, wp.x, wp.y);
    start_s = ref_frenet[0];
  }

  for (const frenet_coord& fp : next_frenet_points) {

    auto xy = getXY(start_s + fp.s, fp.d, wp.s, wp.x, wp.y);

    Eigen::VectorXd p{3};
    p << xy[0], xy[1], 1;

    points.push_back(p);

  }

  Eigen::VectorXd anchor_x{points.size()};
  Eigen::VectorXd anchor_y{points.size()};

  for (unsigned int i = 0; i < points.size(); i++) {

    Eigen::VectorXd p = points[i];
    Eigen::VectorXd p_t = poses.world_in_ego * p;

    anchor_x(i) = getXFromH(p_t);
    anchor_y(i) = getYFromH(p_t);
  }

  Eigen::VectorXd coeffs = polyfit(anchor_x, anchor_y, 3);

  return coeffs;

}

void fillNextXYFromPrevious(pp_output* out, const pp_input& in, long n_take) {

  // Fill the next x/y values with the previous path

  auto prev_path_size = in.previous_path_x.size();

  if (n_take == -1 || n_take > prev_path_size) { // Take all
    n_take = prev_path_size;
  }

  if (!in.previous_path_x.empty()) {

    for (unsigned i = 0; i < n_take; i++) {

      out->next_x_vals.push_back(in.previous_path_x[i]);
      out->next_y_vals.push_back(in.previous_path_y[i]);

    }

  }

}

void fillNextXYTargetV(pp_output* out,
                       const pp_input& in,
                       double target_velocity_mph,
                       const ReferencePoses& poses,
                       const Eigen::VectorXd& coeffs) {

  // Fill the rest of the next x/y values with
  // the values based on the polynomial

  //auto prev_path_size = in.previous_path_x.size();
  auto prev_path_size = out->next_x_vals.size();

  double target_velocity = MPH2Metric(target_velocity_mph);
  double dx = target_velocity * DT;
  auto n_poly_points = TRAJ_SIZE - prev_path_size;

  for (unsigned int i = 0; i < n_poly_points; i++) {

    double x = (i + 1) * dx;
    double y = polyeval(coeffs, x);

    Eigen::VectorXd p{3};
    p << x, y, 1;

    Eigen::VectorXd p_world = poses.ego_in_world * p;

    out->next_x_vals.push_back(getXFromH(p_world));
    out->next_y_vals.push_back(getYFromH(p_world));

  }

}

void fillNextNYFirstTime(pp_output* out,
                         double accel_to_mph,
                         const ReferencePoses& poses,
                         const Eigen::VectorXd& coeffs,
                         double accel_from_mph) {

  auto accel_increments = accel(MPH2Metric(accel_from_mph), MPH2Metric(accel_to_mph), 1.);

  double x = 0.;
  for (double dx : accel_increments) {

    x += dx;

    double y = polyeval(coeffs, x);

    Eigen::VectorXd p{3};
    p << x, y, 1;

    Eigen::VectorXd p_world = poses.ego_in_world * p;

    out->next_x_vals.push_back(getXFromH(p_world));
    out->next_y_vals.push_back(getYFromH(p_world));

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

std::vector<vehicle_info> lookAround(const pp_input& in) {

  std::vector<vehicle_info> res{};

  for (const sf_vehicle& vehicle : in.sensor_fusion) {

    vehicle_info info{};

    info.id = vehicle.id;
    info.lane = getCarLane(vehicle);
    info.s_dist = vehicle.s - in.car_s;

    res.push_back(info);
  }

  return res;
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

int checkIfSafeToChangeLane(const pp_input& in, int current_lane) {
  // If safe, return index of the lane
  // otherwise, return -1

  auto vehiles_info = lookAround(in);
  auto neighbor_lanes = neighbors(current_lane);

  for (int candidate_lane : neighbor_lanes) {

    if (safeInLane(candidate_lane, vehiles_info))
      return candidate_lane;

  }

  return -1;

}

pp_output JMTLaneChange(const pp_input& in,
                        const map_waypoints& wp,
                        double d0,
                        double d1,
                        double s_dist) {

  double s0 = in.car_s;
  double s1 = s0 + s_dist;
  double v0 = in.car_speed;
  double v1 = 30;
  double t = 1.;

  // position, velocity, acceleration
  std::vector<double> d_start = {d0, 0, 0};
  std::vector<double> d_end   = {d1, 0, 0};
  std::vector<double> s_start = {s0, v0, 0};
  std::vector<double> s_end   = {s1, v1, 0};

  auto jmt_d = JMT(d_start, d_end, t);
  auto jmt_s = JMT(s_start, s_end, t);

  pp_output out{};

  double t_now;
  double s;
  double d;
  for (int i = 1; i < 50; i++) {
    t_now = i * DT;

    s = QuinticPoly(jmt_s, t_now);
    d = QuinticPoly(jmt_d, t_now);

    auto xy = getXY(s, d, wp.s, wp.x, wp.y);
    out.next_x_vals.push_back(xy[0]);
    out.next_y_vals.push_back(xy[1]);

  }

  return out;

}
