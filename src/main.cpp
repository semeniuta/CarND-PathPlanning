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

class DummyPathPlanner : public PathPlanner {

    PathPlanner::output plan(const PathPlanner::input& in, const map_waypoints& wp) override {

      PathPlanner::output out{};

      return out;

    }

};

int main() {

  // Waypoint map to read from
  std::string map_file = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  map_waypoints wp = readMap(map_file);

  uWS::Hub h;
  DummyPathPlanner planner;

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