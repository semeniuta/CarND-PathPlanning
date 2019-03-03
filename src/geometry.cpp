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

    return {closest_idx, closest_idx + 1};
  }

  if (closest_idx == 0) {
    return {n - 1, closest_idx};
  }

  return {closest_idx - 1, closest_idx};

}


Eigen::MatrixXd createPose(double x, double y, double theta) {

  double ct = cos(theta);
  double st = sin(theta);

  Eigen::MatrixXd pose{3, 3};

  pose << ct, -st, x,
          st,  ct, y,
           0,   0, 1;

  return pose;

}


Eigen::MatrixXd invertPose(const Eigen::MatrixXd& pose) {

  Eigen::MatrixXd pose_inv{3, 3};

  double ct = pose(0, 0);
  double st = pose(1, 0);

  double x = pose(0, 2);
  double y = pose(1, 2);

  pose_inv << ct, st, -x*ct - y*st,
             -st, ct,  x*st - y*ct,
               0,  0,            1;

  return pose_inv;

}


// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd& coeffs, double x) {

  double result = 0.0;

  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }

  return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order) {

  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;

}
