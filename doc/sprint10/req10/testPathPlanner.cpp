#include <iostream>
#include <ctime>

#include "frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h"
#include "frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/AnytimeFrenetOptimalTrajectory.h"
#include "frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

#include "Map.hpp"
#include "R_Tree/r_tree.h"
#include "R_Tree/point.h"

// SIG_LOC Return the node ID of a significant location based on the hole # and type of location ('black', 'bronze', 'silver', 'gold' for tee boxes and 'hole' for the hole)
long sig_loc(int hole, int loc) {
	switch(hole) {
		case 1:
			switch (loc) {
                case 0: //black
    				return 7208400605;
                case 1: //bronze
    				return 7208400608;
			    case 2: //silver
    				return 7208400617;
			    case 3: //gold
    				return 7208400631;
			    case 4: //hole
    				return 7208400684;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		case 2:
			switch (loc) {
                case 0: //black
    				return 7212354780;
                case 1: //bronze
    				return 7212354777;
			    case 2: //silver
    				return 7212354772;
			    case 3: //gold
    				return 7212354767;
			    case 4: //hole
    				return 7212354742;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		case 3:
			switch (loc) {
                case 0: //black
    				return 7212365140;
                case 1: //bronze
    				return 7212365144;
			    case 2: //silver
    				return 7212365158;
			    case 3: //gold
    				return 7212365180;
			    case 4: //hole
    				return 7212365300;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		case 4:
			switch (loc) {
                case 0: //black
    				return 8085848420;
                case 1: //bronze
    				return 7212365294;
			    case 2: //silver
    				return 8085841248;
			    case 3: //gold
    				return 7214033518;
			    case 4: //hole
    				return 7214033415;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		case 5:
			switch (loc) {
                case 0: //black
    				return 7214451284;
                case 1: //bronze
    				return 7214451287;
			    case 2: //silver
    				return 7214451291;
			    case 3: //gold
    				return 7214451305;
			    case 4: //hole
    				return 7214451415;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		case 6:
			switch (loc) {
                case 0: //black
    				return 7215346106;
                case 1: //bronze
    				return 7215346127;
			    case 2: //silver
    				return 7215346151;
			    case 3: //gold
    				return 7215346160;
			    case 4: //hole
    				return 7218855978;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		case 7:
			switch (loc) {
                case 0: //black
    				return 7218921089;
                case 1: //bronze
    				return 7218921094;
			    case 2: //silver
    				return 7218921104;
			    case 3: //gold
    				return 7218921112;
			    case 4: //hole
    				return 7218921175;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		case 8:
			switch (loc) {
                case 0: //black
    				return 7218922995;
                case 1: //bronze
    				return 7218923001;
			    case 2: //silver
    				return 7218923008;
			    case 3: //gold
    				return 7218923027;
			    case 4: //hole
    				return 7218923137;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		case 9:
			switch (loc) {
                case 0: //black
    				return 7218923162;
                case 1: //bronze
    				return 7218923167;
			    case 2: //silver
    				return 7218923179;
			    case 3: //gold
    				return 7220557139;
			    case 4: //hole
    				return 7220557245;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"black\", \"bronze\", \"silver\", \"gold\" or \"hole\"\n";
                    return -1;
			}
		default:
			std::cout << "Attempted to find a significant location of an invalid hole " << hole << ". Value must be in range [1,9]\n";
            return -1;
        }
}

R_Tree Init_RTree(LCC::Map m) {
    R_Tree RT;
    std::vector<std::pair<float, float>> XY = m.NodeXY();
    std::vector<long> ID = m.NodeID();

    for(size_t i = 0; i < XY.size(); i++) {
        RT.insert(TreePoint(XY[i].first, XY[i].second, ID[i]));
    }

    return RT;
}

void PrintRoute(std::vector<double> routeX, std::vector<double> routeY) {
    for(size_t i = 0; i < routeX.size(); i++) {
        std::cout << i << ") " << routeX[i] << "\t" << routeY[i] << "\n";
    }
}

int main() {
    clock_t start_init = clock();
    LCC::Map M;
    M.Init();

    int TEST_SIZE = 10;

    std::vector<float> x_positions(TEST_SIZE);
    std::vector<float> y_positions(TEST_SIZE);

    for(int i = 0; i < TEST_SIZE; i++) {
        float S_x, S_y;
        std::tie(S_x, S_y) = M.NodeXY()[i*10];
        x_positions[i] = S_x;
        y_positions[i] = S_y;
    }

    R_Tree RT = Init_RTree(M);

    std::vector<double> routeX(M.Size());
    std::vector<double> routeY(M.Size());

    int num_threads = 0;

    FrenetHyperparameters fot_hp = {
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
        num_threads // num thread // TODO: Increase to 4
    };

    //double o_llx[1] = {92.89};  // Obstacle lower left X
    //double o_lly[1] = {191.75}; // Obstacle lower left Y
    //double o_urx[1] = {92.89};  // Obstacle upper right X
    //double o_ury[1] = {191.75}; // Obstacle upper right Y

    double init_time = ((double) (clock() - start_init));

    std::cout << "Using " << num_threads << " threads\n";

    std::cout << "Init Time: " << init_time/CLOCKS_PER_SEC << " s\n";

    for(int i = 0; i < TEST_SIZE; i++) {
        clock_t start_it_time = clock();
        TreePoint q(x_positions[i], y_positions[i]);
        // std::cout << "x_pos[" << i << "] = " << x_positions[i] << "\n";
        // std::cout << "y_pos[" << i << "] = " << y_positions[i] << "\n";
        long s_id = RT.closest_point(q).id;
        clock_t closest_point_cp = clock();

        long t_id = M.NodeID()[i*10 + 10];
        std::tie(routeX, routeY) = M.ShortestRoute(s_id, t_id);
        clock_t shortest_route_cp = clock();

        if(routeX.size() == 0) {
            continue;
        }

        std::cout << "N: " << routeX.size() << "\n";
        std::cout << "S: " << s_id << "\tT: " << t_id << "\n";

        // set up experiment
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
            nullptr, //o_llx, // 
            nullptr, //o_lly,
            nullptr, //o_urx,
            nullptr, //o_ury,
            0
        };

        // run experiment
        AnytimeFrenetOptimalTrajectory fot = AnytimeFrenetOptimalTrajectory(&fot_ic, &fot_hp);
        
        fot.asyncPlan(); // start planning

        const int NUM_ITER = 50;
        const int TEST_INTERVAL_TIME = 2; // in microseconds

        double prev_final_cf = INFINITY;
        double curr_cf;

        for (int i = 0; i < NUM_ITER; i ++) {
        FrenetPath* curr_frenet_path = fot.getBestPath();
        if (curr_frenet_path) {
            curr_cf = curr_frenet_path->cf;
            //cout << "Found Valid Path with Cost: " << curr_cf << "\n";
        } else {
            curr_cf = INFINITY;
            //cout << "No Valid Path Found\n";
        }

        // anytime algo consistency test, cost should only reduce or stay unchanged
        // assert(curr_cf > prev_final_cf);
        if (prev_final_cf < curr_cf) {
            cout << "Not Consistent\n";
            return -1; 
        }

        prev_final_cf = curr_cf;
        usleep(TEST_INTERVAL_TIME);
    }
        
        FrenetPath* best_frenet_path = fot.getBestPath();
        clock_t best_path_cp = clock();

        if (best_frenet_path) {
            //best_frenet_path->print_path();
            std::cout << "Success\n";
            //return 1;
        } else {
            std::cout << "Failure\n";
        }

        std::cout << "Closest point " << i << " time: " << (((double) (closest_point_cp - start_it_time))/CLOCKS_PER_SEC)*1000 << " ms\n";
        std::cout << "Shortest route " << i << " time: " << (((double) (shortest_route_cp - closest_point_cp))/CLOCKS_PER_SEC)*1000 << " ms\n";
        std::cout << "Best Path " << i << " time: " << (((double) (best_path_cp - shortest_route_cp))/CLOCKS_PER_SEC)*1000 << " ms\n";
        std::cout << "Iteration " << i << " Time: " << (((double) (clock() - start_it_time))/CLOCKS_PER_SEC)*1000 << " ms\n\n";

    }
    return 0;
}