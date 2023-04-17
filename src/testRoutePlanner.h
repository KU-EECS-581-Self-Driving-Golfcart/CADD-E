#include "localization/Map.hpp"
#include <cmath>

#ifndef TEST_ROUTE_PLANNER
#define TEST_ROUTE_PLANNER

#ifndef M_PI
#define M_PI           3.14159265358979323846  /* pi */
#endif

class TestRoutePlanner {
	public:
	double global_origin_lat;
	double global_origin_lon;
	float local_origin_x = 0.0;
	float local_origin_y = 0.0;
	int N = 0;
	int radius = 0;
	std::vector<std::pair<float, float>> node_xy; // Node local coordinates
    std::vector<std::pair<double, double>> node_latlon; // Node global coordinates
	std::vector<long> node_id; // Node IDs

	TestRoutePlanner(double origin_lat, double origin_lon, int N_points, int circ_radius) {
		global_origin_lat = origin_lat;
		global_origin_lon = origin_lon;
		// Read in map
		std::tie(local_origin_x, local_origin_y) = latlon2local(global_origin_lat, global_origin_lon);
		local_origin_x-=(radius*2);
		local_origin_y-=(radius);

		std::cout << "#global: " << global_origin_lat << ", " << global_origin_lon << "\n";
		std::cout << "#local: " << local_origin_x << ", " << local_origin_y << "\n";

		N = N_points;
		radius = circ_radius;

		node_xy.reserve(N);
		node_latlon.reserve(N);
		node_id.reserve(N);

		for(int i = 0; i < N; i++) {
			float angle = (2*M_PI/N)*i;
			float x = cos(angle)*radius + radius;
			float y = sin(angle)*radius + radius;

			node_xy[i] = std::pair<float, float>(x, y);
			node_id[i] = i;
		}
	}

	~TestRoutePlanner() {}

	std::vector<int> ShortestRouteIdxs(double lat, double lon, int hole, char loc) {
		// Localize coordinates
		float local_x, local_y;
		std::tie(local_x, local_y) = latlon2local(lat, lon);

		// Find closest graph node
		int s_id = closest_wp_id(local_x, local_y);
		std::cout << "closest_wp  = [" << node_xy[s_id].first << ", " << node_xy[s_id].second  << "]\n"; 
		std::vector<int> idxs(int(N/2));
		for(int i = 0; i < int(N/2); i++) {
			idxs[i] = (s_id + i) % N;
		}

		return idxs;
	}

	std::pair<std::vector<float>, std::vector<float>> LocalRoute(std::vector<int> RouteIdxs) {
		// Init return vectors
		std::vector<float> routeX, routeY;
        routeX.reserve(RouteIdxs.size());
        routeY.reserve(RouteIdxs.size());

		for(size_t i = 0; i < RouteIdxs.size(); i++) {
			std::pair<float, float> xy = node_xy[RouteIdxs[i]];
			routeX.push_back(xy.first);
        	routeY.push_back(xy.second);
		}

		return std::pair<std::vector<float>, std::vector<float>>(routeX, routeY);
	}

	/*
	std::vector<std::pair<double, double>> GlobalRoute(std::vector<int> RouteIdxs) {
		// Init return vectors
		std::vector<std::pair<double, double>> routeLatLon;
        routeLatLon.reserve(RouteIdxs.size());

		for(size_t i = 0; i < RouteIdxs.size(); i++) {
			routeLatLon.push_back(m.NodeLatLon()[RouteIdxs[i]]);
		}

		return routeLatLon;
	}
	*/

	int size(){
		return N;
	}

	// Convert a global coordinate point to a localized system
    std::pair<float, float> latlon2local(float lat, float lon) {
        // Convert degrees to radians (0.0174533 = PI/180 degrees)
        float lat_rad = lat * 0.0174533;
        float lon_rad = lon * 0.0174533;

        const int R = 6378100; // Radius of Earth in Meters

        float global_x_cart = R * cos(lat_rad) * cos(lon_rad);

        float global_y_cart = R * cos(lat_rad) * sin(lon_rad);

        return std::pair<float, float>(global_x_cart - local_origin_x, global_y_cart - local_origin_y);
    }

	private:
	

    float distance(float x1, float y1, float x2, float y2) {
        float x = x1 - x2;
        float y = y1 - y2;
        float dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
        dist = sqrt(dist);                  

        return dist;
    }

    long closest_wp_id(float x, float y) {
        float min_distance = std::numeric_limits<float>::max();
        long id = 0;

        for(size_t i = 0; i < node_xy.size(); i++) {
            std::pair<float, float> p_xy = node_xy[i];
            float dist_i = distance(x, y, p_xy.first, p_xy.second);
            if(dist_i < min_distance) {
                min_distance = dist_i;
                id = node_id[i];
            }
        }

        return id;
    }
};
#endif // TEST_ROUTE_PLANNER