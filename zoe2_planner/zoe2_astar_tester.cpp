//
// Created by Morgan Mayborne on 12/9/24.
//

#include "planner.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>

void logToFile(const std::vector<std::tuple<double, double, double, double, double, double, double>>& vec, const std::string& filename, const std::vector<double>& init_goal_write) {
    // Open the file for writing
    std::ofstream outFile(filename);

    if (!outFile) {
        std::cerr << "Failed to open the file!" << std::endl;
        return;
    }

    // Optionally set precision for floating point numbers
    outFile << std::fixed << std::setprecision(6);

    outFile
            << init_goal_write[0] << " "
            << init_goal_write[1] << " "
            << init_goal_write[2] << " "
            << init_goal_write[3] << " "
            << init_goal_write[4] << "\n";

    // Iterate over the vector and write each tuple as a line
    for (const auto& tup : vec) {
        outFile
            << std::get<0>(tup) << " "
            << std::get<1>(tup) << " "
            << std::get<2>(tup) << " "
            << std::get<3>(tup) << " "
            << std::get<4>(tup) << " "
            << std::get<5>(tup) << "\n";
    }

    // Close the file
    outFile.close();
}

int main() {
    double rad = 0.325;
    double width = 1.64;
    double wheelbase = 1.91;
    double wgt_heur = 10;
    double goal_radius = 0.0707;
    double th_gain = 1;
    double steer_angle_smooth = 20;
    std::vector<double> init = {0,0, 3*M_PI/2};
    std::vector<double> goal = {-4,2};
    std::vector<double> bounds = {-5.5,-5.5,5.5,5.5};

    std::vector<double> poss_R = {-10, 10, .5};
    std::vector<double> poss_dt = {.5, 6, 0.25};
    std::vector<double> poss_velo = {-1,1};

    std::vector<std::vector<double>> cost_map(std::ceil(bounds[2])-std::floor(bounds[0]), std::vector<double>(std::ceil(bounds[3])-std::floor(bounds[1]), 0.0));

    // Populate the cost map using the given formula
    // for (int i = 0; i < std::ceil(bounds[2])-std::floor(bounds[0]); ++i) {
    //     for (int j = 0; j < std::ceil(bounds[3])-std::floor(bounds[1]); ++j) {
    //         cost_map[i][j] = std::max(10 - 2 * std::abs(i - j), 0);
    //     }
    // }
    cost_map[0-bounds[1]][-2-bounds[0]] = 10000;

    auto start = std::chrono::high_resolution_clock::now();
    double last_plan = 0;  // Time of the last plan completion
    double completion_time = 1.00; // Completion time in seconds

    // Create the AStarPlanner object using constructor parameters
    AStarPlanner planner(
        {rad, width, wheelbase, wgt_heur, goal_radius, th_gain, steer_angle_smooth},
        bounds,
        poss_R,
        poss_dt,
        poss_velo,
        cost_map
    );

    // AStarPlanner planner;
    // auto path = planner.a_star();
    std::vector<std::tuple<double, double, double, double, double, double, double>> path;
    std::vector<std::tuple<double, double, double, double, double, double, double>> path_tmp;
    while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count() + 2 * last_plan <= completion_time) {
        auto plan_start = std::chrono::high_resolution_clock::now();

        if (last_plan != 0) {
            wgt_heur = std::max(wgt_heur / 1.5, 1.0);  // Adjust weight
        }
        path_tmp = planner.a_star(init[0], init[1], init[2], goal[0], goal[1]);

        std::cout << "Bounds: ";
        for (const auto& item : planner.getBounds()) {
            std::cout << item << " ";
        }
        std::cout << std::endl;
        std::cout << "Epsilon: " << planner.getEps() << std::endl;

        planner.setEps(std::max(planner.getEps() / 1.5, 1.5));

        if (path_tmp.size() > 0) {
            path = path_tmp;
        }

        for (const auto& step : path) {
            std::cout << std::get<0>(step) << " " << std::get<1>(step) << " "
                      << std::get<2>(step) << " " << std::get<3>(step) << " "
                      << std::get<4>(step) << " " << std::get<5>(step) << " "
                      << std::get<6>(step) <<
                          std::endl;
        }


        auto plan_end = std::chrono::high_resolution_clock::now();
        auto plan_duration = std::chrono::duration<double>(plan_end - plan_start).count();
        std::cout << "Time to Complete: " << plan_duration << " seconds for heuristic bias: " << wgt_heur << std::endl;

        last_plan = plan_duration;
    }

    std::cout << "Path (x, y, theta, radius, dt, velo):"  << " " << wgt_heur << std::endl;
    for (const auto& step : path) {
        std::cout << std::get<0>(step) << " " << std::get<1>(step) << " "
                  << std::get<2>(step) << " " << std::get<3>(step) << " "
                  << std::get<4>(step) << " " << std::get<5>(step) << " "
                  << std::get<6>(step) <<
                      std::endl;
    }

    std::cout << "Path Cost: " << std::get<6>(path[path.size()-1]);

    std::vector<double> init_goal_write = {init[0], init[1], init[2], goal[0], goal[1]};
    logToFile(path, "../output.txt",init_goal_write);
    return 0;
}
