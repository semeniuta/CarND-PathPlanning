//
// Created by Oleksandr Semeniuta on 2019-03-04.
//

#ifndef PATH_PLANNING_STEPS_H
#define PATH_PLANNING_STEPS_H

#include "Eigen-3.3/Eigen/Dense"
#include "planners.h"

const int TRAJ_SIZE = 50;
const double DT = 0.02;

struct ReferenceState {
  Eigen::VectorXd xy_h{3};
  double yaw;
};

struct ReferencePoses {
  Eigen::MatrixXd ego_in_world{3, 3};
  Eigen::MatrixXd world_in_ego{3, 3};
};

ReferenceState prepareReferenceState(const pp_input& in);

ReferencePoses createPoses(const ReferenceState& ref);

Eigen::VectorXd fitPolynomial(const pp_input& in,
                              const map_waypoints& wp,
                              const ReferenceState& ref,
                              const ReferencePoses& poses,
                              double lane_d);

void fillNextXYTargetV(pp_output* out,
                       const pp_input& in,
                       double target_velocity_mph,
                       const ReferencePoses& poses,
                       const Eigen::VectorXd& coeffs);

void fillNextNYFirstTime(pp_output* out,
                         double accel_to_mph,
                         const ReferencePoses& poses,
                         const Eigen::VectorXd& coeffs);

void fillNextXYFromPrevious(pp_output* out, const pp_input& in);

bool checkForCarInFront(const pp_input& in, int current_lane, double s_threshold);

double updateTargetVelocity(bool too_close,
                            double target_velocity,
                            double velocity_increment,
                            double min_speed);

std::vector<vehicle_info> lookAround(const pp_input& in);

#endif //PATH_PLANNING_STEPS_H
