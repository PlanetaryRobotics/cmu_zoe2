#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <set>
#include <tuple>
#include <iomanip>
#include <fstream>

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

double steer_angle(double arc_rad, double wheelbase) {
    if (arc_rad <= 0) {
        return M_PI / 2 - std::asin(std::abs(arc_rad) / std::sqrt(arc_rad * arc_rad + (wheelbase / 2) * (wheelbase / 2)));
    } else {
        return -(M_PI / 2 - std::asin(std::abs(arc_rad) / std::sqrt(arc_rad * arc_rad + (wheelbase / 2) * (wheelbase / 2))));
    }

}

// Function to log the contents of the vector to a file
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

class AStarPlanner {
public:
    double start_x, start_y, start_th, goal_x, goal_y;
    double rad, width, wheelbase, wgt_heur, goal_radius, th_gain, steer_angle_smooth;
    std::vector<double> map_bounds;
    std::vector<double> possR, possdt, poss_v;
    std::vector<std::vector<double>> cost_map;

    AStarPlanner(double start_x, double start_y, double start_th, double goal_x, double goal_y, const std::vector<double>& other_args, const std::vector<double>& bounds, const std::vector<double>& pR, const std::vector<double>& pdt, const std::vector<double>& pv, const std::vector<std::vector<double>>& map)
        : start_x(start_x), start_y(start_y), start_th(start_th),goal_x(goal_x), goal_y(goal_y) {
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

    double heuristic(double x, double y) {
        return pow(x - goal_x, 2) + pow(y - goal_y, 2);
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

    std::vector<std::tuple<double, double, double, double, double, double, double>> a_star() {

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
        std::set<std::tuple<int, int, int>> closed_list; // Discretized state space
        std::unordered_map<std::tuple<int, int, int>, double, TupleHash> g_cost_map;

        std::vector<double> poss_R = createRange(possR[0], -0.1, possR[2]);
        std::vector<double> poss_R_2 = createRange(0.1, possR[1], possR[2]);
        poss_R.insert( poss_R.end(), poss_R_2.begin(), poss_R_2.end());
        std::vector<double> poss_dt = createRange(possdt[0], possdt[1], possdt[2]);

        Node start_node(start_x, start_y, start_th, 0, 0, 0, heuristic(start_x, start_y), 0,0, 0,0, nullptr);
        open_list.push(start_node);
        g_cost_map[{start_x, start_y, start_th}] = 0;

        while (!open_list.empty()) {
            Node curr = open_list.top();
            open_list.pop();

            // Goal condition
            if (heuristic(curr.x, curr.y)/wgt_heur <= goal_radius) {
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
                if (arc_rad == 0) {
                    continue;
                }
                for (double dt_comm : poss_dt) {
                    for (double fixed_v : poss_v) {
                        double curr_sa = steer_angle(arc_rad, wheelbase);

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

                        auto next_state = std::make_tuple(next_x, next_y, next_th);
                        if (closed_list.count(next_state)) continue;

                        // std::cout << "Time to Complete: " << cost_map[(int)floor(next_x)][(int)floor(next_y)] << " seconds" << std::endl;
                        //double g_cost = curr.g + 5*(fabs(vl - curr.vl) + fabs(vr - curr.vr)) + cost_map[(int)floor(next_x)][(int)floor(next_y)];
                        double g_cost = curr.g + steer_angle_smooth*(fabs(curr_sa-curr.sa)) + cost_map[(int)floor(next_x)-floor(map_bounds[0])][(int)floor(next_y)-std::floor(map_bounds[1])];
                        //if (g_cost_map.count(next_state) && g_cost >= g_cost_map[next_state]) continue;

                        //g_cost_map[next_state] = g_cost;
                        double h_cost = wgt_heur * heuristic(next_x, next_y);
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


};

int main() {
    double rad = 0.325;
    double width = 1.64;
    double wheelbase = 1.91;
    double wgt_heur = 10;
    double goal_radius = 0.0707;
    double th_gain = 1;
    double steer_angle_smooth = 20;
    std::vector<double> init = {0,0, 3*M_PI/2};
    std::vector<double> goal = {-1,5};
    std::vector<double> bounds = {-5.5,-5.5,5.5,5.5};

    std::vector<double> poss_R = {-10, 10, .5};
    std::vector<double> poss_dt = {.5, 6, 0.25};
    std::vector<double> poss_velo = {-1,1};

    std::vector<std::vector<double>> cost_map(std::ceil(bounds[2])-std::floor(bounds[0]), std::vector<double>(std::ceil(bounds[3])-std::floor(bounds[1]), 0.0));

    // Populate the cost map using the given formula
    for (int i = 0; i < std::ceil(bounds[2])-std::floor(bounds[0]); ++i) {
        for (int j = 0; j < std::ceil(bounds[3])-std::floor(bounds[1]); ++j) {
            cost_map[i][j] = std::max(10 - 2 * std::abs(i - j), 0);
        }
    }

    auto start = std::chrono::high_resolution_clock::now();
    double last_plan = 0;  // Time of the last plan completion
    double completion_time = 1.00; // Completion time in seconds

    // AStarPlanner planner;
    // auto path = planner.a_star();
    std::vector<std::tuple<double, double, double, double, double, double, double>> path;
    std::vector<std::tuple<double, double, double, double, double, double, double>> path_tmp;
    while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count() + 2 * last_plan <= completion_time) {
        auto plan_start = std::chrono::high_resolution_clock::now();

        if (last_plan != 0) {
            wgt_heur = std::max(wgt_heur / 1.5, 1.0);  // Adjust weight
        }
        AStarPlanner planner(init[0], init[1], init[2], goal[0], goal[1], {rad, width, wheelbase, wgt_heur, goal_radius, th_gain, steer_angle_smooth}, bounds, poss_R, poss_dt, poss_velo, cost_map);
        path_tmp = planner.a_star();

        if (path_tmp.size() > 0) {
            path = path_tmp;
        }

        // Print the time taken for this plan
        auto plan_end = std::chrono::high_resolution_clock::now();
        auto plan_duration = std::chrono::duration<double>(plan_end - plan_start).count();
        std::cout << "Time to Complete: " << plan_duration << " seconds for heuristic bias: " << wgt_heur << std::endl;

        // Update last_plan with the time taken for this iteration
        last_plan = plan_duration;
    }
    // Initialize the A_Star_Planner object and call a_star method
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
