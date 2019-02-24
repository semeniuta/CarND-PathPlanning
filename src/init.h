//
// Created by Oleksandr Semeniuta on 2019-02-23.
//

#ifndef PATH_PLANNING_INIT_H

#include <string>

void readMap(
        const std::string& map_file,
        std::vector<double>& wp_x,
        std::vector<double>& wp_y,
        std::vector<double>& wp_s,
        std::vector<double>& wp_dx,
        std::vector<double>& wp_dy
        ) {

    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

    string line;

    while (getline(in_map_, line)) {
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
        wp_x.push_back(x);
        wp_y.push_back(y);
        wp_s.push_back(s);
        wp_dx.push_back(d_x);
        wp_dy.push_back(d_y);
    }
}


#define PATH_PLANNING_INIT_H

#endif //PATH_PLANNING_INIT_H
