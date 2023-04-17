#include "localization/Map.hpp"

#ifndef ROUTE_PLANNER
#define ROUTE_PLANNER

// SIG_LOC Return the node ID of a significant location based on the hole # and type of location ('black', 'bronze', 'silver', 'gold' for tee boxes and 'hole' for the hole)
long sig_loc(int hole, char loc) {
	switch(hole) {
		case 1:
			switch (loc) {
                case 'B': //black
    				return 7208400605;
                case 'R': //bronze
    				return 7208400608;
			    case 'S': //silver
    				return 7208400617;
			    case 'G': //gold
    				return 7208400631;
			    case 'H': //hole
    				return 7208400684;
			    default:
    				std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		case 2:
			switch (loc) {
                case 'B': //black
    				return 7212354780;
                case 'R': //bronze
    				return 7212354777;
			    case 'S': //silver
    				return 7212354772;
			    case 'G': //gold
    				return 7212354767;
			    case 'H': //hole
    				return 7212354742;
			    default:
					std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		case 3:
			switch (loc) {
                case 'B': //black
    				return 7212365140;
                case 'R': //bronze
    				return 7212365144;
			    case 'S': //silver
    				return 7212365158;
			    case 'G': //gold
    				return 7212365180;
			    case 'H': //hole
    				return 7212365300;
			    default:
					std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		case 4:
			switch (loc) {
                case 'B': //black
    				return 8085848420;
                case 'R': //bronze
    				return 7212365294;
			    case 'S': //silver
    				return 8085841248;
			    case 'G': //gold
    				return 7214033518;
			    case 'H': //hole
    				return 7214033415;
			    default:
					std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		case 5:
			switch (loc) {
                case 'B': //black
    				return 7214451284;
                case 'R': //bronze
    				return 7214451287;
			    case 'S': //silver
    				return 7214451291;
			    case 'G': //gold
    				return 7214451305;
			    case 'H': //hole
    				return 7214451415;
			    default:
					std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		case 6:
			switch (loc) {
                case 'B': //black
    				return 7215346106;
                case 'R': //bronze
    				return 7215346127;
			    case 'S': //silver
    				return 7215346151;
			    case 'G': //gold
    				return 7215346160;
			    case 'H': //hole
    				return 7218855978;
			    default:
					std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		case 7:
			switch (loc) {
                case 'B': //black
    				return 7218921089;
                case 'R': //bronze
    				return 7218921094;
			    case 'S': //silver
    				return 7218921104;
			    case 'G': //gold
    				return 7218921112;
			    case 'H': //hole
    				return 7218921175;
			    default:
					std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		case 8:
			switch (loc) {
                case 'B': //black
    				return 7218922995;
                case 'R': //bronze
    				return 7218923001;
			    case 'S': //silver
    				return 7218923008;
			    case 'G': //gold
    				return 7218923027;
			    case 'H': //hole
    				return 7218923137;
			    default:
					std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		case 9:
			switch (loc) {
                case 'B': //black
    				return 7218923162;
                case 'R': //bronze
    				return 7218923167;
			    case 'S': //silver
    				return 7218923179;
			    case 'G': //gold
    				return 7220557139;
			    case 'H': //hole
    				return 7220557245;
			    default:
					std::cout << "Invalid loc argument given to sig_loc. Value must be \"B\"lack, b\"R\"onze, \"S\"ilver, \"G\"old or \"H\"ole\n";
                    return -1;
			}
		default:
			std::cout << "Attempted to find a significant location of an invalid hole " << hole << ". Value must be in range [1,9]\n";
            return -1;
        }
}

void PrintRoute(std::vector<double> routeX, std::vector<double> routeY) {
    for(size_t i = 0; i < routeX.size(); i++) {
        std::cout << i << ") " << routeX[i] << "\t" << routeY[i] << "\n";
    }
}

class RoutePlanner {
	public:
	LCC::Map m;

	RoutePlanner() {
		// Read in map
		m.Init();
	}

	~RoutePlanner() {}

	std::vector<int> ShortestRouteIdxs(double lat, double lon, int hole, char loc) {
		// Localize coordinates
		float local_x, local_y;
		std::tie(local_x, local_y) = m.latlon2local(lat, lon);

		// Find closest graph node
		long s_id = m.closest_wp_id(local_x, local_y);
		int s_idx = m.nd_id_2_idx_map[s_id];
		std::cout << "#Closest point: ID = " << s_id<< " XY  = [" << m.NodeXY()[s_idx].first << ", " << m.NodeXY()[s_idx].second  << "]\n"; 
		std::cout << "closest_wp  = [" << m.NodeXY()[s_idx].first << ", " << m.NodeXY()[s_idx].second  << "]\n"; 

		// Find target node
		long t_id = sig_loc(hole, loc);

		// Return shortest route from source to target
		return m.ShortestRoute(s_id, t_id);
	}

	std::pair<std::vector<float>, std::vector<float>> LocalRoute(std::vector<int> RouteIdxs) {
		// Init return vectors
		std::vector<float> routeX, routeY;
        routeX.reserve(RouteIdxs.size());
        routeY.reserve(RouteIdxs.size());

		for(size_t i = 0; i < RouteIdxs.size(); i++) {
			std::pair<float, float> xy = m.NodeXY()[RouteIdxs[i]];
			routeX.push_back(xy.first);
        	routeY.push_back(xy.second);
		}

		return std::pair<std::vector<float>, std::vector<float>>(routeX, routeY);
	}

	std::vector<std::pair<double, double>> GlobalRoute(std::vector<int> RouteIdxs) {
		// Init return vectors
		std::vector<std::pair<double, double>> routeLatLon;
        routeLatLon.reserve(RouteIdxs.size());

		for(size_t i = 0; i < RouteIdxs.size(); i++) {
			routeLatLon.push_back(m.NodeLatLon()[RouteIdxs[i]]);
		}

		return routeLatLon;
	}

	// Convert a global coordinate point to a localized system
    std::pair<float, float> latlon2local(float lat, float lon) {
        return m.latlon2local(lat, lon);
    }

	int size(){
		return m.Size();
	}
};
#endif // ROUTE_PLANNER