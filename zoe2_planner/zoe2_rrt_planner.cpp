#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <tuple>
#include <chrono> 

class RRTNode {
public:
    double x, y, theta, vl, vr, arc_rad, v_comm;
    RRTNode* parent;

    RRTNode(double x, double y, double theta, double vl, double vr, double arc_rad, double v_comm, RRTNode* parent = nullptr)
        : x(x), y(y), theta(theta), vl(vl), vr(vr), arc_rad(arc_rad), v_comm(v_comm), parent(parent) {}
};

class RRTConnectPlanner {
public:
    double start_x, start_y, goal_x, goal_y;
    double dt, rad, width, wheelbase, goal_radius;
    int max_samples;
    std::vector<RRTNode*> tree_start;
    std::vector<RRTNode*> tree_goal;

    RRTConnectPlanner(double start_x, double start_y, double goal_x, double goal_y, const std::vector<double>& other_args)
        : start_x(start_x), start_y(start_y), goal_x(goal_x), goal_y(goal_y) {
        dt = other_args[0];
        rad = other_args[1];
        width = other_args[2];
        wheelbase = other_args[3];
        goal_radius = other_args[4];
        max_samples = static_cast<int>(other_args[5]);
    }

    double distance(double x1, double y1, double x2, double y2) const {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    std::vector<double> velocity_control(double rad_comm, double velo_comm) const {
        double th_comm = std::atan(wheelbase / (2 * rad_comm));
        double ang_velo = velo_comm / rad_comm;
        double vl = (1 / std::cos(th_comm)) * velo_comm - (width / 2) * ang_velo;
        double vr = (1 / std::cos(th_comm)) * velo_comm + (width / 2) * ang_velo;
        return {vl, vr};
    }

    std::tuple<double, double, double> random_sample() const {
        double x = static_cast<double>(rand()) / RAND_MAX * 5;  // Workspace dimensions
        double y = static_cast<double>(rand()) / RAND_MAX * 5;
        double theta = static_cast<double>(rand()) / RAND_MAX * (2 * M_PI);
        return {x, y, theta};
    }

    RRTNode* nearest_neighbor(const std::vector<RRTNode*>& tree, double sample_x, double sample_y) const {
        RRTNode* nearest_node = nullptr;
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& node : tree) {
            double dist = distance(node->x, node->y, sample_x, sample_y);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node = node;
            }
        }
        return nearest_node;
    }

    RRTNode* steer(RRTNode* nearest, double target_x, double target_y, double target_theta, double max_extend_dist = 0.5) const {
        std::vector<double> arc_rads = {-10, -7, -4, -1, 1, 4, 7, 10};
        std::vector<double> v_comms = {1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5};

        RRTNode* best_node = nullptr;
        double best_dist = std::numeric_limits<double>::infinity();

        for (double arc_rad : arc_rads) {
            for (double v_comm : v_comms) {
                auto comm_velos = velocity_control(arc_rad, v_comm);
                double vl = comm_velos[0];
                double vr = comm_velos[1];

                double velo = (rad / 2) * (vl + vr);
                double ang_velo = (rad / wheelbase) * (vr - vl);
                double next_theta = nearest->theta + ang_velo * dt;
                double next_x = nearest->x + velo * std::cos(next_theta) * dt;
                double next_y = nearest->y + velo * std::sin(next_theta) * dt;

                double dist_to_target = distance(next_x, next_y, target_x, target_y);
                if (dist_to_target > max_extend_dist) {
                    continue;
                }

                if (dist_to_target < best_dist) {
                    best_dist = dist_to_target;
                    best_node = new RRTNode(next_x, next_y, next_theta, vl, vr, arc_rad, v_comm, nearest);
                }
            }
        }

        return best_node;
    }

    RRTNode* connect(std::vector<RRTNode*>& tree, double target_x, double target_y, double target_theta, int max_attempts = 50) const {
        for (int i = 0; i < max_attempts; ++i) {
            RRTNode* nearest = nearest_neighbor(tree, target_x, target_y);
            RRTNode* new_node = steer(nearest, target_x, target_y, target_theta);
            if (new_node != nullptr) {
                tree.push_back(new_node);
                if (distance(new_node->x, new_node->y, target_x, target_y) <= goal_radius) {
                    return new_node;
                }
            }
        }
        return nullptr;
    }

    std::vector<std::tuple<double, double, double, double, double>> plan() {
    auto start_time = std::chrono::high_resolution_clock::now(); // Start timing
    RRTNode* start_node = new RRTNode(start_x, start_y, 0, 0, 0, 0, 0);
    RRTNode* goal_node = new RRTNode(goal_x, goal_y, 0, 0, 0, 0, 0);

        tree_start.push_back(start_node);
        tree_goal.push_back(goal_node);

        for (int i = 0; i < max_samples; ++i) {
            auto [sample_x, sample_y, sample_theta] = random_sample();


            RRTNode* nearest_start = nearest_neighbor(tree_start, sample_x, sample_y);
            RRTNode* new_start_node = steer(nearest_start, sample_x, sample_y, sample_theta);

            if (new_start_node != nullptr) {
                tree_start.push_back(new_start_node);

                RRTNode* connect_node = connect(tree_goal, new_start_node->x, new_start_node->y, new_start_node->theta);
                if (connect_node != nullptr) { 
                    std::vector<std::tuple<double, double, double, double, double>> path;

                    RRTNode* current = new_start_node;
                    while (current != nullptr) {
                        path.emplace_back(current->x, current->y, current->theta, current->arc_rad, current->v_comm);
                        current = current->parent;
                    }
                    std::reverse(path.begin(), path.end());
                    current = connect_node;
                    while (current != nullptr) {
                        path.emplace_back(current->x, current->y, current->theta, current->arc_rad, current->v_comm);
                        current = current->parent;
                    }

                    return path;
                }
            }
            std::swap(tree_start, tree_goal);
        }
        auto end_time = std::chrono::high_resolution_clock::now(); // End timing
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::cout << "Computation Time: " << elapsed.count() << " seconds" << std::endl;

        return {}; 
    }
};

int main() {
    double dt = 2;              
    double rad = 0.325;        
    double width = 1.64;       
    double wheelbase = 1.91;   
    double goal_radius = 0.1;  
    int max_samples = 1000;    

    double start_x = 0, start_y = 0; 
    double goal_x = 4, goal_y = 4;  

    std::vector<double> params = {dt, rad, width, wheelbase, goal_radius, static_cast<double>(max_samples)};
    RRTConnectPlanner planner(start_x, start_y, goal_x, goal_y, params);
    auto start_time = std::chrono::high_resolution_clock::now(); // Start timing
    auto plan = planner.plan();  
    auto end_time = std::chrono::high_resolution_clock::now(); // End timing
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::cout << "Computation Time: " << elapsed.count() << " seconds" << std::endl;

    for (const auto& p : plan) {
        std::cout << std::get<0>(p) << ", " << std::get<1>(p) << ", " << std::get<2>(p) << ", "
                  << std::get<3>(p) << ", " << std::get<4>(p) << std::endl;
    }

    return 0;
}
