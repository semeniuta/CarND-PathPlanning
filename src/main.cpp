#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "PathPlanner.h"


int main() {

  // Waypoint map to read from
  std::string map_file = "../data/highway_map.csv";

  map_waypoints wp = readMap(map_file);

  uWS::Hub h;
  PolynomialPathPlanner planner;

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