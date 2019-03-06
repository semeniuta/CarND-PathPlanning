//
// Created by Oleksandr Semeniuta on 2019-03-04.
//

#ifndef PATH_PLANNING_STEPS_H
#define PATH_PLANNING_STEPS_H

#include "Eigen-3.3/Eigen/Dense"
#include "PathPlanner.h"

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

void fillNextXY(pp_output* out,
                const pp_input& in,
                double target_velocity,
                const ReferencePoses& poses,
                const Eigen::VectorXd& coeffs);

#endif //PATH_PLANNING_STEPS_H
