//
// Created by Oleksandr Semeniuta on 2019-02-23.
//

#ifndef PATH_PLANNING_INIT_H
#define PATH_PLANNING_INIT_H

#include <string>
#include "PathPlanner.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

#endif //PATH_PLANNING_INIT_H
