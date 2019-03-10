//
// Created by Oleksandr Semeniuta on 2019-03-02.
//

#include "planners.h"
#include <iostream>
#include "helpers.h"

int main() {

//  printVector( accel(0, 25) );
//  printVector( accel(26, 25) );
//  printVector( accel(20, 25) );

  std::string map_file = "../data/highway_map.csv";
  map_waypoints wp = readMap(map_file);

  std::vector<double> d_start = {6, 0, 0};
  std::vector<double> d_end = {2, 0, 0};

  std::vector<double> s_start = {50, 30, 0};
  std::vector<double> s_end = {100, 30, 0};

  auto jmt_d = JMT(d_start, d_end, 1);
  std::cout << "d_coeffs = ";
  printVector(jmt_d);

  auto jmt_s = JMT(s_start, s_end, 1);
  std::cout << "s_coeffs = ";
  printVector(jmt_s);

  std::vector<double> xs;
  std::vector<double> ys;

  double t;
  double s;
  double d;
  for (int i = 0; i < 50; i++) {
    t = i * 0.02;

    s = QuinticPoly(jmt_s, t);
    d = QuinticPoly(jmt_d, t);

    auto xy = getXY(s, d, wp.s, wp.x, wp.y);
    xs.push_back(xy[0]);
    ys.push_back(xy[1]);

  }

  std::cout << "x = ";
  printVector(xs);

  std::cout << "y = ";
  printVector(ys);

}