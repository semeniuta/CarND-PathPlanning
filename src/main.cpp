#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "init.h"
#include "PathPlanner.h"

class SimplePathPlanner : public PathPlanner {

public:

  PathPlanner::output plan(const PathPlanner::input& in, const map_waypoints& wp) override {

    PathPlanner::output out{};

    std::vector<double> s_vals;
    std::vector<double> d_vals;

    double s = in.car_s;
    double d = in.car_d;

    std::cout << "prev_path_size = " << in.previous_path_x.size() << std::endl;

    if (!in.previous_path_x.empty()) {
      std::cout << "prev_path_first = " << in.previous_path_x[0] << ", " << in.previous_path_y[1] << std::endl;
    }

//    std::vector<double> increments = accel(in.car_speed, 15);
//    std::cout << "inc = ";
//    printVector(increments);
//    for (double ds : increments) {
//      s += ds;

    double dist_inc = 0.2;
    for (int i = 1; i <= 50; ++i) {
      s += dist_inc;

      s_vals.push_back(s);
      d_vals.push_back(d);

      std::vector<double> xy = getXY(s, d, wp.s, wp.x, wp.y);
      out.next_x_vals.push_back(xy[0]);
      out.next_y_vals.push_back(xy[1]);

    }

    std::cout << "x = ";
    printVector(out.next_x_vals);

    std::cout << "y = ";
    printVector(out.next_y_vals);

    std::cout << "s = ";
    printVector(s_vals);

    std::cout << "d = ";
    printVector(d_vals);

    std::cout << std::endl;

    return out;

  }

};

class StraightPathPlanner : public PathPlanner {

public:

  PathPlanner::output plan(const PathPlanner::input& in, const map_waypoints& wp) override {

    PathPlanner::output out{};

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

};

int main() {

  // Waypoint map to read from
  std::string map_file = "../data/highway_map.csv";

  map_waypoints wp = readMap(map_file);

  uWS::Hub h;
  SimplePathPlanner planner;
  //StraightPathPlanner planner;

  initHub(h, planner, wp);

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}