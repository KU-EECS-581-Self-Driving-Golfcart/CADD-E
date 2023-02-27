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
    std::vector<std::pair<double, double>> XY = m.NodeXY();
    std::vector<long> ID = m.NodeID();

    for(size_t i = 0; i < XY.size(); i++) {
        RT.insert(TreePoint(XY[i].first, XY[i].second, ID[i]));
    }

    return RT;
}

void PrintRoute(std::vector<double> routeX, std::vector<double> routeY) {
    for(int i = 0; i < routeX.size(); i++) {
        std::cout << i << ") " << routeX[i] << "\t" << routeY[i] << "\n";
    }
}

int main() {
    LCC::Map M;
    M.Init();

    std::vector<double> routeX(M.Size());
    std::vector<double> routeY(M.Size());

    std::tie(routeX, routeY) = M.ShortestRoute(7208400631, 7208400684);

	PrintRoute(routeX, routeY);

    //R_Tree RT = Init_RTree(M);
    //R_Tree RT = Init_RTree_TEST(route, M.NodeXY(), M.NodeID());
    
	R_Tree RT = Init_RTree(M);

	std::cout << "Plot time\n";

	RT.plot();

    //RT.print();


    // Read in map
    // Init R Tree with map nodes

    //while(ros::ok()) {
        // Read GPS
        // Check for updated target node
        // Find closest node to position

        //route = M.ShortestRoute(1, 2);

        // Update route
        // Send route with ROS
        // See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
    //}
}