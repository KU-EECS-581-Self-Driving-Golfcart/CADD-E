#include <iostream>

#include "frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h"
#include "frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

#include "Map.hpp"

int main() {
    LCC::Map M;
    M.Init();

	//M.PrintGraph();

    std::vector<double> routeX(M.Size());
    std::vector<double> routeY(M.Size());

    std::tie(routeX, routeY) = M.ShortestRoute(7208400631, 7208400684);

    //double o_llx[1] = {92.89};  // Obstacle lower left X
    //double o_lly[1] = {191.75}; // Obstacle lower left Y
    //double o_urx[1] = {92.89};  // Obstacle upper right X
    //double o_ury[1] = {191.75}; // Obstacle upper right Y

    // set up experiment
    FrenetInitialConditions fot_ic = {
        0.0, //34.6, // S_0
        0.0, //7.10964962, // Speed
        0.0, //-1.35277168, 
        0.0, //-1.86,
        0.0,
        10, // Target speed
        routeX.data(), // Waypoints X
        routeY.data(), // Waypoints Y
        static_cast<int>(routeX.size()), // Number of waypoints
        nullptr, //o_llx, // 
        nullptr, //o_lly,
        nullptr, //o_urx,
        nullptr, //o_ury,
        0
    };
    FrenetHyperparameters fot_hp = {
        25.0,	// max speed
        15.0,	// max accel
        15.0,	// max curvature
        5.0,	// max_road_width_l;
        5.0,	// max_road_width_r;
        0.5,	// d_road_w;
        0.1,	// dt; // WAS 0.2
        5.0,	// maxt;
        2.0,	// mint;
        0.5,	// d_t_s;
        2.0,	// n_s_sample;
        0.1,	// obstacle_clearance;
        1.0,	// kd;
        0.1,	// kv;
        0.1,	// ka;
        0.1,	// kj;
        0.1,	// kt;
        0.1,	// ko;
        1.0,	// klat;
        1.0,	// klon;
        1 // num thread // TODO: Increase to 4
    };

    // run experiment
    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
    FrenetPath* best_frenet_path = fot.getBestPath();
    if (best_frenet_path) {
		best_frenet_path->print_path();
        std::cout << "Success\n";
        return 1;
    }
    std::cout << "Failure\n";
    return 0;
}