//
// Created by Oleksandr Semeniuta on 2019-03-03.
//

#include "helpers.h"
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>

map_waypoints readMap(const std::string& map_file) {

  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  map_waypoints wp;

  std::ifstream in_map(map_file.c_str(), std::ifstream::in);

  string line;

  while (getline(in_map, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    wp.x.push_back(x);
    wp.y.push_back(y);
    wp.s.push_back(s);
    wp.dx.push_back(d_x);
    wp.dy.push_back(d_y);
  }

  return wp;
}

void initHub(uWS::Hub& h,
             PathPlanner& planner,
             map_waypoints& wp) {

  h.onMessage([&planner, &wp](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // ===========================================================================

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Parsing the sensor_fusion JSON

          std::vector<std::vector<double>> sf_data;

          for (auto it_vehicle = sensor_fusion.begin(); it_vehicle != sensor_fusion.end(); it_vehicle++) {

            auto vehicle_json = *it_vehicle;

            std::vector<double> vehicle;

            for (auto it_num = vehicle_json.begin(); it_num != vehicle_json.end(); it_num++) {
              vehicle.push_back(*it_num);
            }

            sf_data.push_back(vehicle);

          }

          // Parsing previous path (x, y) JSONs

          std::vector<double> prev_x_data;
          std::vector<double> prev_y_data;

          for (auto it_num = previous_path_x.begin(); it_num != previous_path_x.end(); it_num++) {
            prev_x_data.push_back(*it_num);
          }

          for (auto it_num = previous_path_y.begin(); it_num != previous_path_y.end(); it_num++) {
            prev_y_data.push_back(*it_num);
          }

          PathPlanner::input in{
              car_x,
              car_y,
              car_s,
              car_d,
              car_yaw,
              car_speed,
              prev_x_data,
              prev_y_data,
              end_path_s,
              end_path_d,
              sf_data
          };

          // Calling the planner

          PathPlanner::output out = planner.plan(in, wp);

          msgJson["next_x"] = out.next_x_vals;
          msgJson["next_y"] = out.next_y_vals;

          // ===========================================================================

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char* message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * M_PI / 180; }

double rad2deg(double x) { return x * 180 / M_PI; }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double>& maps_x,
                    const vector<double>& maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double>& maps_x,
                 const vector<double>& maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * M_PI - angle, angle);

  if (angle > M_PI / 2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double>& maps_x,
                         const vector<double>& maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double>& maps_s,
                     const vector<double>& maps_x,
                     const vector<double>& maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}

void printVector(const std::vector<double>& vec) {

  if (vec.empty()) {
    std::cout << "[]\n";
    return;
  }

  std::cout << "[";

  for (unsigned int i = 0; i < vec.size() - 1; i++) {
    std::cout << vec[i] << ", ";
  }

  std::cout << vec[vec.size() - 1] << "]\n";

}


std::vector<double> accel(double start_v, double target_v, double accel_t = 0.5) {

  const int TRAJ_SIZE = 50;
  const double DT = 0.02;

  double target_dx = target_v * DT;
  double start_dx = start_v * DT;

  int n_incr_const_speed = TRAJ_SIZE;
  std::vector<double> res;

  if (start_v < target_v) {

    int n_incr_zero_to_target = (int) (accel_t / DT); // e.g. 1. / 0.2 = 50
    double delta = target_dx / n_incr_zero_to_target;

    auto n_incr_accel = (int) ceil((target_dx - start_dx) / delta);
    n_incr_const_speed = TRAJ_SIZE - n_incr_accel;

    for (unsigned int i = 1; i <= n_incr_accel; i++) {
      res.push_back(start_dx + delta * i);
    }

  }

  for (unsigned int i = 1; i <= n_incr_const_speed; i++) {
    res.push_back(target_dx);
  }

  return res;

}