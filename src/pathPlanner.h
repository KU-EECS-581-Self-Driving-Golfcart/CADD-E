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

    FrenetInitialConditions generate_ic(Telemetry* telem, vector* wp_X, vector* wp_Y, int wp_no, FrenetPath previous_path) {
            //double s0, double x, double y, double vx,
            //double vy, double forward_speed, double* xp, double* yp, int np,
            //double* initial_conditions

        double s0 = 0.0; // TODO: Set s0

        FrenetInitialConditions fot_ic;

        vector<double> wx (wp_X, wp_X + wp_no);
        vector<double> wy (wp_Y, wp_Y + wp_no);
        CubicSpline2D* csp = new CubicSpline2D(wx, wy);

        // get distance from car to spline and projection
        double s = csp->find_s(telem->pos_x, telem->pos_y, s0);
        double distance = norm(csp->calc_x(s) - telem->pos_x, csp->calc_y(s) - telem->pos_y);
        tuple<double, double> bvec ((csp->calc_x(s) - telem->pos_x) / distance, (csp->calc_y(s) - telem->pos_y) / distance);

        // normal spline vector
        double x0 = csp->calc_x(s0);
        double y0 = csp->calc_y(s0);
        double x1 = csp->calc_x(s0 + 2);
        double y1 = csp->calc_y(s0 + 2);

        // unit vector orthog. to spline
        tuple<double, double> tvec (y1-y0, -(x1-x0));
        as_unit_vector(tvec);

        // compute tangent / normal car vectors
        tuple<double, double> fvec (telem->vel_x, telem->vel_y);
        as_unit_vector(fvec);

        // compute forward speed
        

        // get initial conditions in frenet frame
        initial_conditions[0] = s; // current longitudinal position s
        initial_conditions[1] = forward_speed; // speed [m/s]
        // lateral position c_d [m]
        initial_conditions[2] = copysign(distance, dot(tvec, bvec));
        // lateral speed c_d_d [m/s]
        initial_conditions[3] = -forward_speed * dot(tvec, fvec);
        initial_conditions[4] = 0.0; // lateral acceleration c_d_dd [m/s^2]
        // TODO: add lateral acceleration when CARLA 9.7 is patched (IMU)

        delete csp;
    }
};

#endif // PATH_PLANNER