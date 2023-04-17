#include <iostream>
#include <unistd.h>
#include <cmath>

#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

#ifndef PATH_PLANNER
#define PATH_PLANNER

class PathPlanner {

    private:
    FrenetHyperparameters fot_hp;
    const int NUM_ITER = 1000;            // Number of attempts to try to find optimal path
    const int TEST_INTERVAL_TIME = 2;   // in microseconds
    int num_threads;

    public:
    PathPlanner(int num_threads) {
        fot_hp = {
            4.0,	// max speed
            3.0,	// max accel
            7,	// max curvature
            0.75,	// max_road_width_l;
            0.75,	// max_road_width_r;
            0.05, //0.025,	// d_road_w;
            0.1,	// dt;
            5.0, //10.0,	// maxt;
            1.0,	// mint;
            0.1,	// d_t_s;
            2.0, //5.0,	// n_s_sample;
            0.1,	// obstacle_clearance;
            500, //85,	// kd;    75 - 100
            0.1,	// kv;
            0.1,	// ka;
            0.1,	// kj;
            -1000,	// kt;
            500,	// ko; 512.90906 - 512.909125
            1.0,	// klat;
            1.0,	// klon;
            num_threads // num thread
        };
    }

    FrenetPath* getPathAnytime(Telemetry telem, float* wp_X, float* wp_Y, int wp_no) {
        double s0 = 0.0; // Initial guess at longitudinal offset

        FrenetInitialConditions fot_ic;

        // std::cout << "Position: " << telem.pos_x << ", " << telem.pos_y << "\n";

        std::vector<double> wx(wp_X, wp_X + wp_no);
        std::vector<double> wy(wp_Y, wp_Y + wp_no);

        // std::cout << "PP pre route = [\n";
        // for(int i = 0; i < wx.size(); i++) {
        //     std::cout << "\t[" << wx[i] << ", " << wy[i] << "],\n";
        // }

        CubicSpline2D* csp = new CubicSpline2D(wx, wy);

        // get distance from car to spline and projection
        double s = csp->find_s(telem.pos_x, telem.pos_y, s0);
        double distance = norm(csp->calc_x(s) - telem.pos_x, csp->calc_y(s) - telem.pos_y);
        tuple<double, double> bvec ((csp->calc_x(s) - telem.pos_x) / distance, (csp->calc_y(s) - telem.pos_y) / distance);

        // normal spline vector
        double x0 = csp->calc_x(s0);
        double y0 = csp->calc_y(s0);
        double x1 = csp->calc_x(s0 + 2);
        double y1 = csp->calc_y(s0 + 2);

        // unit vector orthog. to spline
        tuple<double, double> tvec (y1-y0, -(x1-x0));
        as_unit_vector(tvec);

        // Compute X, Y components of velocity
        float heading_rad = 0.0174533 * telem.heading;

        double v_x = telem.speed_mps * cos(heading_rad);
        double v_y = telem.speed_mps * sin(heading_rad);

        // compute tangent / normal car vectors
        tuple<double, double> fvec (v_x, v_y);
        as_unit_vector(fvec);

        double o_llx[1] = {417};
        double o_lly[1] = {129};
        double o_urx[1] = {417.1};
        double o_ury[1] = {129.25};

        // get initial conditions in frenet frame
        fot_ic = {
            s, // Current longitudinal position s
            telem.speed_mps, // Speed [m/s]
            copysign(distance, dot(tvec, bvec)), // Lateral position c_d [m]
            -telem.speed_mps * dot(tvec, fvec), // Lateral speed c_d_d [m/s].
            telem.acc_x, // Lateral acceleration c_d_dd [m/s^2]            This assumes the car is facing in the direction of the spline
            1.5, // Target speed [m/s]
            wx.data(), // Waypoints X
            wy.data(), // Waypoints Y
            wp_no, // Number of waypoints
            o_llx, //o_llx
            o_lly, //o_lly
            o_urx, //o_urx
            o_ury, //o_ury
            1        // number of obstacles
        };

        delete csp;

        // std::cout << "PP set route = [\n";
        // for(int i = 0; i < fot_ic.nw; i++) {
        //     std::cout << "\t[" << *(fot_ic.wx + i) << ", " << *(fot_ic.wy + i) << "],\n";
        // }

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
        

        return fot.getBestPath();
    }

    FrenetPath* getPathRegular(Telemetry telem, float* wp_X, float* wp_Y, int wp_no) {
        double s0 = 0.0; // Initial guess at longitudinal offset

        FrenetInitialConditions fot_ic;

        // std::cout << "Position: " << telem.pos_x << ", " << telem.pos_y << "\n";

        std::vector<double> wx(wp_X, wp_X + wp_no);
        std::vector<double> wy(wp_Y, wp_Y + wp_no);

        // std::cout << "PP pre route = [\n";
        // for(int i = 0; i < wx.size(); i++) {
        //     std::cout << "\t[" << wx[i] << ", " << wy[i] << "],\n";
        // }

        CubicSpline2D* csp = new CubicSpline2D(wx, wy);

        // get distance from car to spline and projection
        double s = csp->find_s(telem.pos_x, telem.pos_y, s0);
        double distance = norm(csp->calc_x(s) - telem.pos_x, csp->calc_y(s) - telem.pos_y);
        tuple<double, double> bvec ((csp->calc_x(s) - telem.pos_x) / distance, (csp->calc_y(s) - telem.pos_y) / distance);

        // normal spline vector
        double x0 = csp->calc_x(s0);
        double y0 = csp->calc_y(s0);
        double x1 = csp->calc_x(s0 + 2);
        double y1 = csp->calc_y(s0 + 2);

        // unit vector orthog. to spline
        tuple<double, double> tvec (y1-y0, -(x1-x0));
        as_unit_vector(tvec);

        // Compute X, Y components of velocity
        float heading_rad = 0.0174533 * telem.heading;

        double v_x = telem.speed_mps * cos(heading_rad);
        double v_y = telem.speed_mps * sin(heading_rad);

        // compute tangent / normal car vectors
        tuple<double, double> fvec (v_x, v_y);
        as_unit_vector(fvec);

        int obs_count = 0;
        double o_llx[obs_count] = {}; //{415.5, 410.25, 412.75};
        double o_lly[obs_count] = {}; //{129, 131.25, 130.5};
        double o_urx[obs_count] = {}; //{416.3, 411.25, 413};
        double o_ury[obs_count] = {}; //{129.5, 131.85, 130.75};

        std::cout << "obstacles = [\n";
        for(int i = 0; i < obs_count; i++) {
            std::cout << "\t[" << o_llx[i] << ", " << o_lly[i] << ", " << o_urx[i] - o_llx[i] << ", " << o_ury[i] - o_lly[i] << "],\n";
        }
        std::cout << "]\n";

        // get initial conditions in frenet frame
        fot_ic = {
            s, // Current longitudinal position s
            telem.speed_mps, // Speed [m/s]
            copysign(distance, dot(tvec, bvec)), // Lateral position c_d [m]
            -telem.speed_mps * dot(tvec, fvec), // Lateral speed c_d_d [m/s].
            telem.acc_x, // Lateral acceleration c_d_dd [m/s^2]            This assumes the car is facing in the direction of the spline
            1.5, // Target speed [m/s]
            wx.data(), // Waypoints X
            wy.data(), // Waypoints Y
            wp_no, // Number of waypoints
            nullptr, //o_llx, //o_llx
            nullptr, //o_lly, //o_lly
            nullptr, //o_urx, //o_urx
            nullptr, //o_ury, //o_ury
            0// obs_count        // number of obstacles
        };

        delete csp;

        // std::cout << "PP set route = [\n";
        // for(int i = 0; i < fot_ic.nw; i++) {
        //     std::cout << "\t[" << *(fot_ic.wx + i) << ", " << *(fot_ic.wy + i) << "],\n";
        // }

        FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
        /*
        std::cout << "paths = [\n";
        for (FrenetPath *path : fot.frenet_paths) {
            std::cout << "\t[\n";
            for(size_t i = 0; i < path->x.size(); i++) {
                std::cout << "\t\t[" << path->x[i] << ", " << path->y[i] << "],\n";
            }
            std::cout << "\t],\n";
        }
        std::cout << "]\n";
        */
        return fot.getBestPath();
    }

    FrenetInitialConditions generate_ic(Telemetry telem, float* wp_X, float* wp_Y, int wp_no) {
            //double s0, double x, double y, double vx,
            //double vy, double forward_speed, double* xp, double* yp, int np,
            //double* initial_conditions
        double s0 = 0.0; // Initial guess at longitudinal offset

        FrenetInitialConditions fot_ic;

        // std::cout << "Position: " << telem.pos_x << ", " << telem.pos_y << "\n";

        std::vector<double> wx(wp_X, wp_X + wp_no);
        std::vector<double> wy(wp_Y, wp_Y + wp_no);

        // std::cout << "PP pre route = [\n";
        // for(int i = 0; i < wx.size(); i++) {
        //     std::cout << "\t[" << wx[i] << ", " << wy[i] << "],\n";
        // }

        CubicSpline2D* csp = new CubicSpline2D(wx, wy);

        // get distance from car to spline and projection
        double s = csp->find_s(telem.pos_x, telem.pos_y, s0);
        double distance = norm(csp->calc_x(s) - telem.pos_x, csp->calc_y(s) - telem.pos_y);
        tuple<double, double> bvec ((csp->calc_x(s) - telem.pos_x) / distance, (csp->calc_y(s) - telem.pos_y) / distance);

        // normal spline vector
        double x0 = csp->calc_x(s0);
        double y0 = csp->calc_y(s0);
        double x1 = csp->calc_x(s0 + 2);
        double y1 = csp->calc_y(s0 + 2);

        // unit vector orthog. to spline
        tuple<double, double> tvec (y1-y0, -(x1-x0));
        as_unit_vector(tvec);

        // Compute X, Y components of velocity
        float heading_rad = 0.0174533 * telem.heading;

        double v_x = telem.speed_mps * cos(heading_rad);
        double v_y = telem.speed_mps * sin(heading_rad);

        // compute tangent / normal car vectors
        tuple<double, double> fvec (v_x, v_y);
        as_unit_vector(fvec);

        // get initial conditions in frenet frame
        fot_ic = {
            s, // Current longitudinal position s
            telem.speed_mps, // Speed [m/s]
            copysign(distance, dot(tvec, bvec)), // Lateral position c_d [m]
            -telem.speed_mps * dot(tvec, fvec), // Lateral speed c_d_d [m/s].
            telem.acc_x, // Lateral acceleration c_d_dd [m/s^2]            This assumes the car is facing in the direction of the spline
            1.5, // Target speed [m/s]
            wx.data(), // Waypoints X
            wy.data(), // Waypoints Y
            wp_no, // Number of waypoints
            nullptr, //o_llx
            nullptr, //o_lly
            nullptr, //o_urx
            nullptr, //o_ury
            0        // number of obstacles
        };

        delete csp;

        // std::cout << "PP set route = [\n";
        // for(int i = 0; i < fot_ic.nw; i++) {
        //     std::cout << "\t[" << *(fot_ic.wx + i) << ", " << *(fot_ic.wy + i) << "],\n";
        // }

        return fot_ic;
    }

};

#endif // PATH_PLANNER