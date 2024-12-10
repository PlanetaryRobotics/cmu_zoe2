#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <set>
#include <tuple>
#include <iomanip>
#include <fstream>
#include "planner.h"

// Function to log the contents of the vector to a file
class Node {
public:
    double x, y, theta, vl, vr, arc_radius, dt, sa, g, h, f, v;
    Node* parent;

    Node(double x, double y, double theta, double vl, double vr, double g, double h, double dt_comm, double velo, double arc_rad, double steer_angle, Node* parent = nullptr)
        : x(x), y(y), theta(theta), vl(vl), vr(vr), arc_radius(arc_rad), dt(dt_comm), v(velo),  g(g), h(h), sa(steer_angle), parent(parent) {
        f = g + h;
    }

    // Overload < for priority queue comparison
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

AStarPlanner::AStarPlanner(const std::vector<double>& other_args, const std::vector<double>& bounds, const std::vector<double>& pR, const std::vector<double>& pdt, const std::vector<double>& pv, const std::vector<std::vector<double>>& map)
    {
    rad = other_args[0];
    width = other_args[1];
    wheelbase = other_args[2];
    wgt_heur = other_args[3];
    goal_radius = other_args[4];
    th_gain = other_args[5];
    map_bounds = bounds;
    possR = pR;
    possdt = pdt;
    poss_v = pv;
    cost_map = map;
    steer_angle_smooth = other_args[6];
}

std::vector<std::tuple<double, double, double, double, double, double, double>> AStarPlanner::a_star(double start_x, double start_y, double start_th, double goal_x, double goal_y) {

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    std::set<std::tuple<int, int, int>> closed_list; // Discretized state space

    std::vector<double> poss_R = createRange(possR[0], -0.1, possR[2]);
    std::vector<double> poss_R_2 = createRange(0.1, possR[1], possR[2]);
    poss_R.insert( poss_R.end(), poss_R_2.begin(), poss_R_2.end());
    std::vector<double> poss_dt = createRange(possdt[0], possdt[1], possdt[2]);

    Node start_node(start_x, start_y, start_th, 0, 0, 0, heuristic(start_x, start_y, goal_x, goal_y), 0,0, 0,0, nullptr);
    open_list.push(start_node);

    while (!open_list.empty()) {
        Node curr = open_list.top();
        open_list.pop();

        // Goal condition
        if (heuristic(curr.x, curr.y, goal_x, goal_y)/wgt_heur <= goal_radius) {
            std::vector<std::tuple<double, double, double, double, double, double, double>> path;
            while (curr.parent) {
                path.emplace_back(curr.x, curr.y, curr.theta, curr.arc_radius, curr.dt, curr.v, curr.g);
                curr = *curr.parent;
            }
            path.emplace_back(curr.x, curr.y, curr.theta, curr.arc_radius, curr.dt, curr.v, curr.g);
            std::reverse(path.begin(), path.end());
            return path;
        }

        auto curr_state = std::make_tuple(curr.x, curr.y, curr.theta);
        closed_list.insert(curr_state);

        for (double arc_rad : poss_R) {
            for (double dt_comm : poss_dt) {
                for (double fixed_v : poss_v) {
                    double curr_sa = steer_angle(arc_rad);

                    auto comm_velos = velocity_control(arc_rad, fixed_v, curr.theta);
                    double vl = comm_velos[0];
                    double vr = comm_velos[1];

                    // double velo = fixed_v/arc_rad;
                    double ang_velo = fixed_v/arc_rad;
                    double next_th = normalize_angle(curr.theta-ang_velo*dt_comm);

                    double next_x, next_y;
                    if (arc_rad > 0) {
                        next_x = curr.x + arc_rad*(cos(-next_th)-cos(-curr.theta));
                        next_y = curr.y + arc_rad*(sin(-next_th)-sin(-curr.theta));
                    }
                    else {
                        next_x = curr.x + (-arc_rad)*(cos(M_PI-next_th)-cos(M_PI-curr.theta));
                        next_y = curr.y + (-arc_rad)*(sin(M_PI-next_th)-sin(M_PI-curr.theta));
                    }
                    if (!(map_bounds[0] <= next_x && next_x <= map_bounds[2] && map_bounds[1] <= next_y && next_y <= map_bounds[3])) continue;

                    bool in_bounds = true;
                    double max_g = 0;
                    for (double angv_step : createRange(0, ang_velo, ang_velo/10)) {
                        double next_th_;
                        next_th_ = curr.theta - angv_step * dt_comm;

                        double n_x, n_y;
                        if (arc_rad > 0) {
                            n_x = curr.x + arc_rad * (cos(-next_th_) - cos(-curr.theta));
                            n_y = curr.y + arc_rad * (sin(-next_th_) - sin(-curr.theta));
                        } else {
                            n_x = curr.x + (-arc_rad) * (cos(M_PI - next_th_) - cos(M_PI - curr.theta));
                            n_y = curr.y + (-arc_rad) * (sin(M_PI - next_th_) - sin(M_PI - curr.theta));
                        }

                        if (!(map_bounds[0] <= n_x && n_x < map_bounds[2] && map_bounds[1] <= n_y && n_y < map_bounds[3])) {
                            in_bounds = false;
                            break;
                        }

                        // Check the cost map and update max_g if necessary
                        int map_y = floor(n_y) - map_bounds[1];
                        int map_x = floor(n_x) - map_bounds[0];
                        if (cost_map[map_y][map_x] > max_g) {
                            max_g = cost_map[map_y][map_x];
                        }
                    }

                    if (!in_bounds) {
                        continue; // Skip the rest of the logic if out of bounds
                    }

                    auto next_state = std::make_tuple(next_x, next_y, next_th);
                    if (closed_list.count(next_state)) continue;

                    // std::cout << "Time to Complete: " << cost_map[(int)floor(next_x)][(int)floor(next_y)] << " seconds" << std::endl;
                    //double g_cost = curr.g + 5*(fabs(vl - curr.vl) + fabs(vr - curr.vr)) + cost_map[(int)floor(next_x)][(int)floor(next_y)];
                    double g_cost = curr.g + steer_angle_smooth*(fabs(curr_sa-curr.sa)) + max_g;

                    double h_cost = wgt_heur * heuristic(next_x, next_y, goal_x, goal_y);
                    if (arc_rad == -5 && fixed_v == 1 && dt_comm == 4.5 && curr.x == 0 && curr.y == 0 && curr.theta == 3*M_PI/2) {
                         std::cout << "Time to Complete: " << next_x << " " << next_y << " " << next_th << std::endl;
                    }
                    Node neighbor_node(next_x, next_y, next_th, vl, vr, g_cost, h_cost, dt_comm,fixed_v, arc_rad,curr_sa, new Node(curr));

                    open_list.push(neighbor_node);
                }
            }
        }
    }

    return {}; // Return empty path if no solution
}
std::vector<double> AStarPlanner::getBounds() const {
    return map_bounds;
}

void AStarPlanner::setBounds(const std::vector<double>& bounds) {
    map_bounds = bounds;
}

double AStarPlanner::getEps() const {
    return wgt_heur;
}

void AStarPlanner::setEps(const double& eps) {
    wgt_heur = eps;
}

// private:
double AStarPlanner::heuristic(double x, double y, double goal_x, double goal_y) {
    return pow(x - goal_x, 2) + pow(y - goal_y, 2);
}

std::array<double, 2> AStarPlanner::velocity_control(double rad_comm, double velo_comm, double th_prev) {
    double th_comm = atan(wheelbase / (2 * rad_comm));
    double ang_velo = velo_comm / rad_comm;
    double vl = velo_comm - width / 2 * ang_velo;
    double vr = velo_comm + width / 2 * ang_velo;

    // Calculate the physical term matrix multiplication
    double cos_th_comm = std::cos(th_comm);
    std::array<std::array<double, 2>, 2> phys_matrix = {
        {{1 / cos_th_comm, -width / 2}, {1 / cos_th_comm, width / 2}}
    };
    std::array<double, 2> input_vector = {velo_comm, ang_velo};
    std::array<double, 2> phys_term = {
        phys_matrix[0][0] * input_vector[0] + phys_matrix[0][1] * input_vector[1],
        phys_matrix[1][0] * input_vector[0] + phys_matrix[1][1] * input_vector[1]
    };

    // Calculate the gain term
    double th_diff = th_comm - th_prev;
    std::array<double, 2> gain_term = { -th_gain * (-th_diff), th_gain * th_diff };

    std::array<double, 2> velos;
    for (size_t i = 0; i < velos.size(); ++i) {
        velos[i] = phys_term[i] + gain_term[i];
    }
    return velos;

    // return {vl, vr};
}

std::vector<double> AStarPlanner::createRange(double start, double end, double step) {
    std::vector<double> range;
    if (end > start) {
        for (double value = start; value < end; value += step) {
            range.push_back(value);
            //std::cout << value << std::endl;
        }
    } else {
        for (double value = start; value > end; value += step) {
            range.push_back(value);
        }
    }
    return range;
}
// Normalize angle to the range [0, 2π)
double AStarPlanner::normalize_angle(double theta) {
    return fmod(theta + 2 * M_PI, 2 * M_PI);
}

double AStarPlanner::steer_angle(double arc_rad) {
    if (arc_rad <= 0) {
        return M_PI / 2 - std::asin(std::abs(arc_rad) / std::sqrt(arc_rad * arc_rad + (wheelbase / 2) * (wheelbase / 2)));
    } else {
        return -(M_PI / 2 - std::asin(std::abs(arc_rad) / std::sqrt(arc_rad * arc_rad + (wheelbase / 2) * (wheelbase / 2))));
    }

}
