# CarND Path Planning Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Path planning project in the Udacity's Self-Driving Car NanoDegree. The original project template can be found [here](https://github.com/udacity/CarND-Path-Planning-Project).

This project involves the Term 3 Simulator (version 1.2) which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). 

## The project task

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides the car's localization and sensor fusion data on each cycle. There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. Other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

---

## Implementation summary

The path planning logic is implemented by a class realizing the `PathPlanner` interface, which specify the following method:

```cpp
pp_output plan(const pp_input& in, const map_waypoints& wp)
```

The `pp_input` and `pp_output` data types correspond to the input and output data communicated between the path planner program and the simulator. They, along with other data types, are defined in `types.h`. Particular implemntations of path planners can be found in `planners{.h/.cpp}`, with the most comprehensive being `TrafficAwarePathPlanner`. 

The logic of `TrafficAwarePathPlanner` is based on a finite state machine with 4 states/behaviors:

* `start`: accelerate smoothly from 0 to 20 MPH
* `keep_lane`: keep lane with speed up to 45 MPH
* `prepare_to_change_lane`: look for a safe possibility to change lane
* `change_lane`: in process of chaning lanes 

Apart from the FMS states, the common logic based on closeness to the car in front is executed in each cycle. If the car in front is as close as 40 m, the ego vehicle will slow down. 

To follow the road, a third degree polynomial is estimated from 4 points:

* the reference point: either the car's position or the last point of the previous path
* the point 30 m ahead of the car's current position in the target lane 
* the point 60 m ahead of the car's current position in the target lane
* the point 90 m ahead of the car's current position in the target lane

It was recommended by the instructors to use [the spline C++ library by Tino Kluge](http://kluge.in-chemnitz.de/opensource/spline). However, it was decided to go with polynomial estimation instead (in order to not include GPL code in this project). 

To realize smooth lane change, an additional counter field (`lc_counter_`) was used. It is set to `LC_COUNTER_RESOLUTION` (80) when the lane change is initiated, and further decremented down to zero on each successive cycle. In the common code, it governs setting of the target `d` coordinate further used for estimation of the polynomial:

```cpp
double lane_d = laneD(target_lane_);
double lane_d_0 = laneD(source_lane_);

double d_diff = lc_counter_ * ((lane_d - lane_d_0) / LC_COUNTER_RESOLUTION);
double target_d = lane_d - d_diff;

std::vector<frenet_coord> next_frenet_points = {
    {30, target_d},
    {60, target_d},
    {90, target_d},
};
```

The safeness of lane change is governed by a simple conservative logic: a neighbor lane is considered safe if there are no cars 40 m in front and 20 m in the back of the ego vehicle. 

---

## Map of the highway

The map of the highway is in `data/highway_map.txt`. Each waypoint in the list contains  `[x, y, s, dx, dy]` values. `x` and `y` are the waypoint's map coordinate position, the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet `s` value, distance along the road, goes from 0 to 6945.554.

## Build instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Data exchange with the simulator

The simulator provides the C++ path planner program with a JSON object containing the following data:

* main car's localization data (no noise)
  * `["x"]` The car's `x` position in map coordinates
  * `["y"]` The car's `y` position in map coordinates
  * `["s"]` The car's `s` position in frenet coordinates
  * `["d"]` The car's `d` position in frenet coordinates
  * `["yaw"]` The car's `yaw` angle in the map
  * `["speed"]` The car's speed in MPH

* Previous path data given to the planner: returns the previous list but with processed points removed

  * `["previous_path_x"]` The previous list of `x` points previously given to the simulator
  * `["previous_path_y"]` The previous list of `y` points previously given to the simulator

* Previous path's end `s` and `d` values 

  * `["end_path_s"]` The previous list's last point's frenet `s` value
  * `["end_path_d"]` The previous list's last point's frenet `d` value

* Sensor fusion data: a list of all other car's attributes on the same side of the road (no noise)

  * ["sensor_fusion"] An array of enties per car; each car is reprsented by an array of the following elements: 
    * car's unique ID
    * car's `x` position in map coordinates
    * car's `y` position in map coordinates
    * car's `x` velocity in m/s
    * car's `y` velocity in m/s
    * car's `s` position in frenet coordinates
    * car's `d` position in frenet coordinates

## More details

1. The car uses a perfect controller and will visit every `(x, y)` point it recieves in the list every .02 seconds. The units for the `(x, y)` points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The `(x, y)` point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. `previous_path_x`, and `previous_path_y` can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
* Eigen (bundled with the project)
