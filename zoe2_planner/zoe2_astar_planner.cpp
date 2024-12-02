#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <set>
#include <tuple>

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
    double start_x, start_y, goal_x, goal_y;
    double dt, rad, width, wheelbase, wgt_heur, goal_radius, th_gain;

    AStarPlanner(double start_x, double start_y, double goal_x, double goal_y, const std::vector<double>& other_args)
        : start_x(start_x), start_y(start_y), goal_x(goal_x), goal_y(goal_y) {
        dt = other_args[0];
        rad = other_args[1];
        width = other_args[2];
        wheelbase = other_args[3];
        wgt_heur = other_args[4];
        goal_radius = other_args[5];
        th_gain = other_args[6];
    }

    double heuristic(double x, double y) {
        return sqrt(pow(x - goal_x, 2) + pow(y - goal_y, 2));
    }

    std::array<double, 2> velocity_control(double rad_comm, double velo_comm, double th_prev) {
        double th_comm = atan(wheelbase / (2 * rad_comm));
        double ang_velo = velo_comm / rad_comm;
        double vl = velo_comm - width / 2 * ang_velo;
        double vr = velo_comm + width / 2 * ang_velo;

        vl += th_gain * (th_comm - th_prev);
        vr += th_gain * (th_comm - th_prev);

        return {vl, vr};
    }

    std::vector<std::tuple<double, double, double, double, double>> a_star() {
        
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
        std::set<std::tuple<int, int, int>> closed_list; // Discretized state space
        std::unordered_map<std::tuple<int, int, int>, double, TupleHash> g_cost_map;

        std::vector<double> poss_R = {-10, -5, 0, 5, 10};
        std::vector<double> poss_vel = {1, 2, 3, 4, 5};

        Node start_node(start_x, start_y, 0, 0, 0, 0, heuristic(start_x, start_y), 0, 0, nullptr);
        open_list.push(start_node);
        g_cost_map[{int(start_x * 10), int(start_y * 10), int(start_node.theta * 10)}] = 0;

        while (!open_list.empty()) {
            Node curr = open_list.top();
            open_list.pop();

            // Goal condition
            if (heuristic(curr.x, curr.y) <= goal_radius) {
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
                for (double v_comm : poss_vel) {
                    auto comm_velos = velocity_control(arc_rad, v_comm, curr.theta);
                    double vl = comm_velos[0];
                    double vr = comm_velos[1];

                    double velo = (rad / 2) * (vl + vr);
                    double ang_velo = (rad / wheelbase) * (vr - vl);
                    double next_th = normalize_angle(curr.theta + ang_velo * dt);

                    double next_x = curr.x + velo * cos(curr.theta) * dt;
                    double next_y = curr.y + velo * sin(curr.theta) * dt;

                    // Discretize next state for hashing
                    auto next_state = std::make_tuple(int(next_x * 10), int(next_y * 10), int(next_th * 10));
                    if (closed_list.count(next_state)) continue;

                    double g_cost = curr.g + fabs(vl - curr.vl) + fabs(vr - curr.vr);
                    if (g_cost_map.count(next_state) && g_cost >= g_cost_map[next_state]) continue;

                    g_cost_map[next_state] = g_cost;
                    double h_cost = wgt_heur * heuristic(next_x, next_y);
                    Node neighbor_node(next_x, next_y, next_th, vl, vr, g_cost, h_cost, v_comm, arc_rad, new Node(curr));

                    open_list.push(neighbor_node);
                }
            }
        }
        


        return {}; // Return empty path if no solution
    }

    
};

int main() {
    double dt = 0.1;
    double rad = 0.325;
    double width = 1.64;
    double wheelbase = 1.91;
    double wgt_heur = 5.0;
    double goal_radius = 0.1;
    double th_gain = 0.1;

    AStarPlanner planner(0, 0, 4, 4, {dt, rad, width, wheelbase, wgt_heur, goal_radius, th_gain});
    auto start_time = std::chrono::high_resolution_clock::now(); // Start timing
    auto path = planner.a_star();
    auto end_time = std::chrono::high_resolution_clock::now(); // End timing
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::cout << "Computation Time: " << elapsed.count() << " seconds" << std::endl;


    for (const auto& step : path) {
        std::cout << std::get<0>(step) << " " << std::get<1>(step) << " "
                  << std::get<2>(step) << " " << std::get<3>(step) << " "
                  << std::get<4>(step) << std::endl;
    }

    return 0;
}
