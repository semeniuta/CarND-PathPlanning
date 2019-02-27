//
// Created by Oleksandr Semeniuta on 2019-02-23.
//

#ifndef PATH_PLANNING_PATHPLANNER_H

#include <vector>

struct map_waypoints {
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<double> dx;
    vector<double> dy;
};

class PathPlanner {

public:

    struct input {

        double car_x;
        double car_y;
        double car_s;
        double car_d;
        double car_yaw;
        double car_speed;

        // Previous path data given to the planner
        std::vector<double> previous_path_x;
        std::vector<double> previous_path_y;

        // Previous path's end s and d values
        double end_path_s;
        double end_path_d;

        // A list of all other cars on the same side of the road.
        std::vector<std::vector<double>> sensor_fusion;
    };

    struct output {
        vector<double> next_x_vals;
        vector<double> next_y_vals;
    };

    PathPlanner() = default;

    virtual output plan(const input& in, const map_waypoints& wp) = 0;

};

#define PATH_PLANNING_PATHPLANNER_H

#endif //PATH_PLANNING_PATHPLANNER_H
