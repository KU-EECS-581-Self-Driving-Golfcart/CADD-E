#include <matioCpp/matioCpp.h>
#include <iostream>
#include <map>
#include <limits>
#include <queue>
#include <cmath>

#ifndef MAP_H
#define MAP_H

typedef std::pair<float, int> edge_entry_pair;

namespace LCC{

class Map {

public:
    std::map<long, int> nd_id_2_idx_map;

    // Constructor
    Map(){
        N = 0;
        init = false;
    }

    // Destructor
    ~Map(){}

    // Initialize graph with .mat files
    void Init() {
        // Prevent initalizing twice
        if(init) {
            std::cout << "Init called twice. Not initializing again.\n";
            return;
        }

        std::tie(local_origin_x, local_origin_y) = latlon2local(global_origin_lat, global_origin_lon);

        // Create hash map mapping Node IDs to vector indexes
        InitNodeID2Idx();

        // Reserve space for entries for each node
        adj_list.resize(N);
        node_xy.resize(N);
        node_latlon.resize(N);
        node_id.resize(N);

        // Create adjacency list and store node IDs and coordinates in vectors
        InitGraph();

        adj_list[8][0].second = 9;
        adj_list[97][0].second = 98;
        adj_list[217][0].second = 218;
        adj_list[284][0].second = 285;
        adj_list[323][0].second = 324;
        adj_list[576][0].second = 577;
        adj_list[781][0].second = 782;
        adj_list[810][0].second = 811;
        adj_list[1254][0].second = 1255;
        adj_list[1255][1].second = 1256;
        adj_list[1304][0].second = 1305;

        init = true;
    }

    // Return # of nodes in graph
    int Size() {
        return N;
    }

    // Find shortest route from source node to destination node using Dijkstra's algorithm
    std::vector<int> ShortestRoute(long src_id, long dst_id) {
        // Check that graph has been initialized
        if(!init) {
            std::cout << "Map hasn't been initialized. Init with Map.Init()\n";
            std::vector<int> empty_return;
            // std::pair<std::vector<float>, std::vector<float>> empty_return;
            return empty_return;
        }

        // Convert node IDs to indexes
        int s_idx = nd_id_2_idx_map[src_id];
        int d_idx = nd_id_2_idx_map[dst_id];

        // Track distance to source node and predecessor node
        float dist[N];
        int prev[N];

        // Init distance to source node and predecessor nodes
        for(int i = 0; i < N; i++) {
            dist[i] = std::numeric_limits<float>::max();
            prev[i] = -1;
        }

        // Distance from source to source is 0
        dist[s_idx] = 0;

        // Init priority queue to order closer nodes first
        std::priority_queue<edge_entry_pair, std::vector<edge_entry_pair>, std::greater<edge_entry_pair>> Q;

        // Push source node
        Q.push(edge_entry_pair(0, s_idx));

        // Iterate and look for shorter paths to target node
        while(!Q.empty()) {
            edge_entry_pair U = Q.top();
            Q.pop();

            // Check if top of priority queue is destination node
            if(U.second == d_idx) {
                break;
            }

            // Iterate through U's edges
            std::vector<edge_entry_pair> E = adj_list[U.second];
            for(size_t i = 0; i < E.size(); i++) {
                int V = E[i].second;
                float W = E[i].first;

                //std::cout << "U = " << U.second << " V = " << V << "\n";

                // Check for improved path
                if(dist[V] > dist[U.second] + W) {
                    dist[V] = dist[U.second] + W;
                    Q.push(E[i]);
                    prev[E[i].second] = U.second;
                }
            }
        }

        // Create vector of waypoint coordinates from route indexes
        std::vector<int> routeIdxs;
        //std::vector<float> routeX, routeY;
        routeIdxs.reserve(N);
        //routeY.reserve(N);

        // Trace optimal route from target to source
        int U = d_idx;
        if(prev[U] != -1 || U == s_idx) {
            while(U > 0){
                routeIdxs.push_back(U);
                // routeX.push_back(node_xy[U].first);
                // routeY.push_back(node_xy[U].second);
                // Stop after adding source node
                if(U == s_idx) {
                    break;
                }
                U = prev[U];
            }
        }

        // Reverse route (D->S) -> (S->D)
        std::reverse(routeIdxs.begin(), routeIdxs.end());
        // std::reverse(routeX.begin(), routeX.end());
        // std::reverse(routeY.begin(), routeY.end());

        return routeIdxs;
        //return std::pair<std::vector<float>, std::vector<float>>(routeX, routeY);
    }

    // Print Adjacency List contents between (and including) upper and lower bounds
    void PrintGraphSlice(size_t lower_bound, size_t upper_bound) {
        // Prevent printing an empty map
        if(!init) {
            std::cout << "Map hasn't been initialized. Init with Map.Init()\n";
            return;
        }

        size_t adj_list_size = adj_list.size();
        if (upper_bound > adj_list_size) {
            upper_bound = adj_list_size;
        }

        if(lower_bound < 0) {
            lower_bound = 0;
        }

        // Print entries for each node        
        for(size_t i = lower_bound; i < upper_bound; i++){
            std::cout << "Node idx: " << i << "\n";
            // Print all entries
            for(size_t j = 0; j < adj_list[i].size(); j++) {
                std::cout << "\t" << j << ") " << adj_list[i][j].second << "\t" << adj_list[i][j].first << "\n";
            }
        }
        std::cout << "N = " << adj_list.size() << "\n";
    }

    // Print Adjacency List contents
    void PrintGraph() {
        // Prevent printing an empty map
        if(!init) {
            std::cout << "Map hasn't been initialized. Init with Map.Init()\n";
            return;
        }

        size_t adj_list_size = adj_list.size();

        // Print entries for each node        
        for(size_t i = 0; i < adj_list_size; i++){
            std::cout << "Node idx: " << i << "\n";
            // Print all entries
            for(size_t j = 0; j < adj_list[i].size(); j++) {
                std::cout << "\t" << j << ") " << adj_list[i][j].second << "\t" << adj_list[i][j].first << "\n";
            }
        }
        std::cout << "N = " << adj_list.size() << "\n";
    }

    // Return vector of node local coordinates
    std::vector<std::pair<float, float>> NodeXY() {
        return node_xy;
    }

    // Return vector of node global coordinates
    std::vector<std::pair<double, double>> NodeLatLon() {
        return node_latlon;
    }

    // Return vector of node IDs
    std::vector<long> NodeID() {
        return node_id;
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

private:
    // File names relative to build directory
    const std::string nodes_mat_file = "src/localization/maps/lcc_nodes.mat";
    const std::string nodes_mat_var = "nodes";
    const std::string connected_mat_file = "src/localization/maps/lcc_connected.mat";
    const std::string connected_mat_var = "connected";
    const std::string adj_list_mat_file = "src/localization/maps/lcc_adj_list.mat";
    const std::string adj_list_mat_var = "adjacency_list";

    std::vector<std::vector<edge_entry_pair>> adj_list; // Graph representation
    std::vector<std::pair<float, float>> node_xy; // Node local coordinates
    std::vector<std::pair<double, double>> node_latlon; // Node global coordinates
    std::vector<long> node_id; // Node IDs
    int N = 0; // Number of nodes
    bool init = false;
    float local_origin_x = 0.0;
    float local_origin_y = 0.0;
    float global_origin_lat = 38.9753690;
    float global_origin_lon = -95.2690418;

    // Build graph using adj_list_mat_file - an adjancency list created in MATLAB
    void InitGraph() {
        // Print all digits of a number
        std::cout << std::fixed;

        matioCpp::File file_handler;

        // Read in node struct
        file_handler.open(nodes_mat_file); // Open the .mat file
        matioCpp::Struct nodes_struct_ML = file_handler.read(nodes_mat_var).asStruct(); // Read the variable named hello_world
        file_handler.close();

        // Parse node IDs
        matioCpp::Vector<double> node_id_ML = nodes_struct_ML["id"].asVector<double>();
        matioCpp::MultiDimensionalArray<double> node_xy_ML = nodes_struct_ML["xy"].asMultiDimensionalArray<double>();

        // std::cout << "node_id_ML.size(): " << node_id_ML.size() << "\n";
        // std::cout << "node_xy_ML.size(): " << node_xy_ML.numberOfElements() << "\n";

        // Read in connected nodes
        file_handler.open(connected_mat_file); // Open the .mat file
        matioCpp::Vector<double> connected_nds_ML = file_handler.read(connected_mat_var).asVector<double>();
        file_handler.close();

        // Read in MATLAB adjacency list
        file_handler.open(adj_list_mat_file); // Open the .mat file
        matioCpp::CellArray adj_list_ML = file_handler.read(adj_list_mat_var).asCellArray();
        file_handler.close();

        // std::cout << "Process connected nodes\n";

        // std::cout << "connected_nds_ML.size(): " << connected_nds_ML.size() << "\n";

        // Process all connected nodes
        for(size_t i = 0; i < connected_nds_ML.size(); i++) {
            // std::cout << i << " ";
            int idx = static_cast<int>(connected_nds_ML[i]) - 1;
            // std::cout << "Node " << node_id_ML[idx] << "\t" << idx << "\n";

            // Store node coordinates
            // std::cout << "read node_xy\n";
            // std::cout << "x: " << node_xy_ML[idx*2] << "\n";
            // std::cout << "y: " << node_xy_ML[idx*2 + 1] << "\n";
            node_latlon[i] = std::pair<double, double>(node_xy_ML[idx*2 + 1], node_xy_ML[idx*2]);
            node_xy[i] = latlon2local(node_xy_ML[idx*2 + 1], node_xy_ML[idx*2]);
            // node_latlon[i] = latlon2local(node_xy_ML[idx*2 + 1], node_xy_ML[idx*2]);
            // std::cout << "read node_id\n";
            node_id[i] = static_cast<long>(node_id_ML[idx]);


            // Skip nodes without entries in MATLAB adjacency list
            if(idx == 7877 || idx == 11599) {
                continue;
            }

            // Load node's edges in MATLAB adjacency list
            matioCpp::CellArray adj_list_entry = adj_list_ML[idx].asCellArray();

            // Reserve space for entries
            adj_list[i].reserve(adj_list_entry.numberOfElements());
            for(size_t j = 0; j < adj_list_entry.numberOfElements(); j++) {
                // Get MATLAB adjacency list entry
                matioCpp::Vector<double> entry_item = adj_list_entry[j].asVector<double>();

                // Connected nd OLD IDX -> Node ID -> NEW IDX
                int connected_nd_idx = nd_id_2_idx_map[node_id_ML[static_cast<int>(entry_item[0] - 1)]];
                float weight = entry_item[1];

                // std::cout << "\t" << connected_nd_idx << "\t" << weight << "\n";

                // Add entry to new adjacency list
                adj_list[i].push_back(edge_entry_pair(weight, connected_nd_idx));
            }
        }
    }

    // Build a hashmap to made Node IDs to their indexes in the adjacency list
    void InitNodeID2Idx(){
        // Init file handler for Matlab IO
        matioCpp::File file_handler;

        // Read in node struct
        file_handler.open(nodes_mat_file); // Open the .mat file
        matioCpp::Struct nodes_struct_ML = file_handler.read(nodes_mat_var).asStruct(); // Read the variable named hello_world
        file_handler.close();

        // Parse node IDs
        matioCpp::Vector<double> node_id_ML = nodes_struct_ML["id"].asVector<double>();

        // Read in connected nodes
        file_handler.open(connected_mat_file); // Open the .mat file
        matioCpp::Vector<double> connected_nds_ML = file_handler.read(connected_mat_var).asVector<double>();
        file_handler.close();

        // Construct Map
        N = static_cast<int>(connected_nds_ML.size());

        for(int i = 0; i < N; i++) {
            int idx = static_cast<int>(connected_nds_ML[i]) - 1;
            nd_id_2_idx_map.insert({static_cast<long>(node_id_ML[idx]), i});
        }
    }

}; // Class Map
}; // Namespace LCC
#endif // MAP_H