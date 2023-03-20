#include "pathPlanner.h"
#include "routePlanner.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

#include <iostream>

int main() {
    // Init ROS subscribers
    PathPlanner pp(1);
    RoutePlanner rp;

    std::vector<double> routeX(rp.size());
    std::vector<double> routeY(rp.size());
    FrenetPath* path = nullptr;

    while(true) { // ros::ok() ?
        double gps_x = 0;   // TODO
        double gps_y = 0;   // TODO
        int hole = 1;       // TODO
        int loc = 0;        // TODO

        //std::tie(routeX, routeY) = rp.ShortestRoute(gps_x, gps_y, hole, loc);

        std::tie(routeX, routeY) = rp.m.ShortestRoute(7208400605, 7220557245); // Hole 1 black tee to hole 9 green
    
        //std::cout << "Route:\n";
        //PrintRoute(routeX, routeY);

        // Path Planning Initial Conditions
        FrenetInitialConditions fot_ic = {
            0.0, //34.6, // S_0
            0.0, //7.10964962, // Speed
            0.0, //-1.35277168, 
            0.0, //-1.86,
            0.0,
            0.5, // Target speed
            routeX.data(), // Waypoints X
            routeY.data(), // Waypoints Y
            static_cast<int>(routeX.size()), // Number of waypoints
            nullptr, //o_llx
            nullptr, //o_lly
            nullptr, //o_urx
            nullptr, //o_ury
            0        // number of obstacles
        };

        path = pp.getPath(fot_ic);

        std::cout << "got path\n";

        // TODO: Convert path to clothoid

        return 1;
    }
}