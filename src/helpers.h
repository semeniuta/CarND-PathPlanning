#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <vector>
#include "json.hpp"
#include <uWS/uWS.h>
#include "types.h"

// for convenience
using std::string;
using std::vector;
using nlohmann::json;

map_waypoints readMap(const std::string& map_file);

void initHub(uWS::Hub& h,
             PathPlanner& planner,
             map_waypoints& wp);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
double deg2rad(double x);

double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double>& maps_x,
                    const vector<double>& maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double>& maps_x,
                 const vector<double>& maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double>& maps_x,
                         const vector<double>& maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double>& maps_s,
                     const vector<double>& maps_x,
                     const vector<double>& maps_y);

void printVector(const std::vector<double>& vec);

std::vector<double> accel(double start_v, double target_v, double accel_t = 0.5);

void printCarState(const pp_input& in);

void printPrevPathDetails(const pp_input& in);

void printNextXY(const pp_output& out);

double MPH2Metric(double s);

double laneD(int lane_index);

int getCarLane(const sf_vehicle& vehicle);

#endif  // HELPERS_H