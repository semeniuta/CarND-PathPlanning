//
// Created by Oleksandr Semeniuta on 2019-03-02.
//

#ifndef PATH_PLANNING_GEOMETRY_H
#define PATH_PLANNING_GEOMETRY_H

#include "Eigen-3.3/Eigen/Dense"
#include "PathPlanner.h"

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

double distance(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);

unsigned int findClosestWaypoint(const map_waypoints& map, double car_x, double car_y);

std::pair<unsigned int, unsigned int> getSegment(
    const map_waypoints& map,
    unsigned int closest_idx,
    double car_s
);

#endif //PATH_PLANNING_GEOMETRY_H
