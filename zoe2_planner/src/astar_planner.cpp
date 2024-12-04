#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <set>
#include <tuple>
#include <chrono>
#include <algorithm>
// Custom hash function for std::tuple<int, int, int>
struct TupleHash {
    std::size_t operator()(const std::tuple<int, int, int>& t) const {
        auto h1 = std::hash<int>()(std::get<0>(t));
        auto h2 = std::hash<int>()(std::get<1>(t));
        auto h3 = std::hash<int>()(std::get<2>(t));
        return h1 ^ (h2 << 1) ^ (h3 << 2); // Combine hashes
    }
};

// Normalize angle to the range [0, 2π)
double normalize_angle(double theta) {
    return fmod(theta + 2 * M_PI, 2 * M_PI);
}

class Node {
public:
    double x, y, theta, vl, vr, arc_radius, transl_velocity, g, h, f;
    Node* parent;

    Node(double x, double y, double theta, double vl, double vr, double g, double h, double v_comm, double arc_rad, Node* parent = nullptr)
        : x(x), y(y), theta(theta), vl(vl), vr(vr), arc_radius(arc_rad), transl_velocity(v_comm), g(g), h(h), parent(parent) {
        f = g + h;
    }

    // Overload < for priority queue comparison
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

class AStarPlanner {
public:
    double start_x, start_y, start_th, goal_x, goal_y;
    double fixed_v, rad, width, wheelbase, wgt_heur, goal_radius, th_gain;
    std::vector<double> map_bounds, possR, possdt;

    AStarPlanner(double start_x, double start_y, double start_th, double goal_x, double goal_y, const std::vector<double>& other_args, const std::vector<double>& bounds, const std::vector<double>& pR, const std::vector<double>& pdt)
        : start_x(start_x), start_y(start_y), start_th(start_th),goal_x(goal_x), goal_y(goal_y) {
        fixed_v = other_args[0];
        rad = other_args[1];
        width = other_args[2];
        wheelbase = other_args[3];
        wgt_heur = other_args[4];
        goal_radius = other_args[5];
        th_gain = other_args[6];
        map_bounds = bounds;
        possR = pR;
        possdt = pdt;
    }

    double heuristic(double x, double y) {
        return sqrt(pow(x - goal_x, 2) + pow(y - goal_y, 2));
    }

    std::array<double, 2> velocity_control(double rad_comm, double velo_comm, double th_prev) {
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

    std::vector<double> createRange(double start, double end, double step) {
        std::vector<double> range;
        for (double value = start; value < end; value += step) {
            range.push_back(value);
            }
        return range;
    }

    std::vector<std::tuple<double, double, double, double, double>> a_star() {

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
        std::set<std::tuple<int, int, int>> closed_list; // Discretized state space
        std::unordered_map<std::tuple<int, int, int>, double, TupleHash> g_cost_map;

        std::vector<double> poss_R = createRange(possR[0], possR[1], possR[2]);
        std::vector<double> poss_dt = createRange(possdt[0], possdt[1], possdt[2]);

        Node start_node(start_x, start_y, start_th, 0, 0, 0, heuristic(start_x, start_y), 0, 0, nullptr);
        open_list.push(start_node);
        g_cost_map[{start_x, start_y, start_th}] = 0;

        while (!open_list.empty()) {
            Node curr = open_list.top();
            open_list.pop();

            // Goal condition
            if (heuristic(curr.x, curr.y)/wgt_heur <= goal_radius) {
                std::vector<std::tuple<double, double, double, double, double>> path;
                while (curr.parent) {
                    path.emplace_back(curr.x, curr.y, curr.theta, curr.arc_radius, curr.transl_velocity);
                    curr = *curr.parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            auto curr_state = std::make_tuple(int(curr.x * 10), int(curr.y * 10), int(curr.theta * 10));
            closed_list.insert(curr_state);

            for (double arc_rad : poss_R) {
                for (double dt_comm : poss_dt) {
                    auto comm_velos = velocity_control(arc_rad, fixed_v, curr.theta);
                    double vl = comm_velos[0];
                    double vr = comm_velos[1];

                    double velo = fixed_v/arc_rad;
                    double ang_velo = (rad / wheelbase) * (vr - vl);
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

                    auto next_state = std::make_tuple(next_x, next_y, next_th);
                    if (closed_list.count(next_state)) continue;

                    double g_cost = curr.g + 5*(fabs(vl - curr.vl) + fabs(vr - curr.vr));
                    if (g_cost_map.count(next_state) && g_cost >= g_cost_map[next_state]) continue;

                    g_cost_map[next_state] = g_cost;
                    double h_cost = wgt_heur * heuristic(next_x, next_y);
                    Node neighbor_node(next_x, next_y, next_th, vl, vr, g_cost, h_cost, dt_comm, arc_rad, new Node(curr));

                    open_list.push(neighbor_node);
                }
            }
        }

        return {}; // Return empty path if no solution
    }


};

// int main() {
//     double dt = 2;
//     double rad = 0.325;
//     double width = 1.64;
//     double wheelbase = 1.91;
//     double wgt_heur = 10;
//     double goal_radius = 0.05;
//     double th_gain = 0.1;
//     double velocity = 1;
//     std::vector<double> init = {0,0, M_PI/4};
//     std::vector<double> goal = {5,2};
//     std::vector<double> bounds = {0,0,7,7};

//     std::vector<double> poss_R = {-10, 10, .5};
//     std::vector<double> poss_dt = {.5, 6, 0.25};

//     auto start = std::chrono::high_resolution_clock::now();
//     double last_plan = 0;  // Time of the last plan completion
//     double completion_time = 1.0; // Completion time in seconds

//     // AStarPlanner planner;
//     // auto path = planner.a_star();
//     std::vector<std::tuple<double, double, double, double, double>> path;
//     while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count() + 2 * last_plan <= completion_time) {
//         auto plan_start = std::chrono::high_resolution_clock::now();

//         if (last_plan != 0) {
//             wgt_heur = std::max(wgt_heur / 1.5, 1.0);  // Adjust weight
//         }
//         AStarPlanner planner(init[0], init[1], init[2], goal[0], goal[1], {velocity, rad, width, wheelbase, wgt_heur, goal_radius, th_gain}, bounds, poss_R, poss_dt);
//         path = planner.a_star();

//         // Print the time taken for this plan
//         auto plan_end = std::chrono::high_resolution_clock::now();
//         auto plan_duration = std::chrono::duration<double>(plan_end - plan_start).count();
//         std::cout << "Time to Complete: " << plan_duration << " seconds" << std::endl;

//         // Update last_plan with the time taken for this iteration
//         last_plan = plan_duration;
//     }
//     // Initialize the A_Star_Planner object and call a_star method
//     for (const auto& step : path) {
//         std::cout << std::get<0>(step) << " " << std::get<1>(step) << " "
//                   << std::get<2>(step) << " " << std::get<3>(step) << " "
//                   << std::get<4>(step) << std::endl;
//     }

//     return 0;
// }