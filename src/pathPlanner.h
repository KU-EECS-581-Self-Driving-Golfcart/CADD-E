#include <iostream>
#include <unistd.h>

#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

#ifndef PATH_PLANNER
#define PATH_PLANNER

class PathPlanner {

    private:
    FrenetHyperparameters fot_hp;
    const int NUM_ITER = 50;            // Number of attempts to try to find optimal path
    const int TEST_INTERVAL_TIME = 2;   // in microseconds
    int num_threads;

    public:
    PathPlanner(int num_threads) {
        fot_hp = {
            10.0,	// max speed
            5.0,	// max accel
            10.0,	// max curvature
            1.0,	// max_road_width_l;
            1.0,	// max_road_width_r;
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
            num_threads // num thread
        };
    }

    FrenetPath* getPath(FrenetInitialConditions fot_ic) {
        // run experiment
        AnytimeFrenetOptimalTrajectory fot = AnytimeFrenetOptimalTrajectory(&fot_ic, &fot_hp);
        
        fot.asyncPlan(); // start planning

        double prev_final_cf = INFINITY;
        double curr_cf;

        for (int i = 0; i < NUM_ITER; i ++) {
            FrenetPath* curr_frenet_path = fot.getBestPath();
            if (curr_frenet_path) {
                curr_cf = curr_frenet_path->cf;
            } else {
                curr_cf = INFINITY;
            }

            // anytime algo consistency test, cost should only reduce or stay unchanged
            // assert(curr_cf > prev_final_cf);
            if (prev_final_cf < curr_cf) {
                cout << "Not Consistent\n";
                return nullptr; 
            }

            prev_final_cf = curr_cf;
            usleep(TEST_INTERVAL_TIME);
        }
        

        return fot.getBestPath();;
    }
};

#endif // PATH_PLANNER