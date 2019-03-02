//
// Created by Oleksandr Semeniuta on 2019-03-02.
//

#include "geometry.h"


double distance(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) {

  auto n = x1.size();

  double ss{0.};

  for (unsigned int i = 0; i < n; i++) {

    double diff = x1(i) - x2(i);

    ss += diff * diff;
  }

  return sqrt(ss);

}


unsigned int findClosestWaypoint(const map_waypoints& map, double car_x, double car_y) {

  Eigen::VectorXd car{2};
  car << car_x, car_y;

  auto n = map.x.size();
  double min_dist = std::numeric_limits<double>::max();
  unsigned int closest_idx = 0;
  Eigen::VectorXd current{2};

  for (unsigned int i = 0; i < n; i++) {

    current << map.x[i], map.y[i];
    double dist = distance(current, car);

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;

}


std::pair<unsigned int, unsigned int> getSegment(
    const map_waypoints& map,
    unsigned int closest_idx,
    double car_s
) {

  auto n = map.x.size();

  bool closest_in_front{true};

  if (map.s[closest_idx] < car_s) {
    closest_in_front = false;
  }

  if (!closest_in_front) {

    if (closest_idx == n - 1) {
      return {closest_idx, 0};
    }

    return {closest_idx, closest_idx+1};
  }

  if (closest_idx == 0) {
    return {n - 1, closest_idx};
  }

  return {closest_idx - 1, closest_idx};

}
