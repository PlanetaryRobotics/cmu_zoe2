#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <vector>
#include <tuple>
//
// // Forward declaration of the Node class since it's used by AStarPlanner
// class Node;

class AStarPlanner {
public:
    double rad, width, wheelbase, wgt_heur, goal_radius, th_gain, steer_angle_smooth;
    std::vector<double> map_bounds;
    std::vector<double> possR, possdt, poss_v;
    std::vector<std::vector<double>> cost_map;

    AStarPlanner(const std::vector<double>& other_args,
                 const std::vector<double>& bounds,
                 const std::vector<double>& pR,
                 const std::vector<double>& pdt,
                 const std::vector<double>& pv,
                 const std::vector<std::vector<double>>& map);

    std::vector<std::tuple<double, double, double, double, double, double, double>> a_star(double start_x,
                                                                                          double start_y,
                                                                                          double start_th,
                                                                                          double goal_x,
                                                                                          double goal_y);

    std::vector<double> getBounds() const;

    void setBounds(const std::vector<double>& bounds);

    double getEps() const;

    void setEps(const double& eps);
private:
    double heuristic(double x, double y, double goal_x, double goal_y);

    std::array<double, 2> velocity_control(double rad_comm, double velo_comm, double th_prev);

    std::vector<double> createRange(double start, double end, double step);

    double normalize_angle(double theta);

    double steer_angle(double arc_rad);
};

#endif // ASTAR_PLANNER_H