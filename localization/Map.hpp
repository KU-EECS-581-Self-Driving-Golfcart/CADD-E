#include <matioCpp/matioCpp.h>
#include <iostream>
#include <map>
#include <limits>
#include <queue>

#ifndef MAP_H
#define MAP_H

//#include "r_tree.h" // TODO: include
//#include "waypoint.h" // TODO: include

typedef std::pair<float, int> edge_entry_pair;

class Map {

public:
    std::map<long, int> nd_id_2_idx_map;

    Map(){
        N = 0;
        init = false;
    }
    ~Map(){}

    void PrintGraph() {
        if(!init) {
            std::cout << "Map hasn't been initialized. Init with Map.Init()\n";
            return;
        }
        int adj_list_size = adj_list.size();
        // Print entries for each node        
        for(int i = 0; i < adj_list_size; i++){
            std::cout << "Node idx: " << i << "\n";
            // Print all entries
            for(int j = 0; j < adj_list[i].size(); j++) {
                std::cout << "\t" << adj_list[i][j].second << "\t" << adj_list[i][j].first << "\n";
            }
        }
        std::cout << "N = " << adj_list.size() << "\n";
    }

    std::vector<int> ShortestRoute(long src_id, long dst_id) {
        if(!init) {
            std::cout << "Map hasn't been initialized. Init with Map.Init()\n";
            std::vector<int> empty_return;
            return empty_return;
        }

        std::vector<int> route;
        route.reserve(N);
        //std::cout << "fetching S, D idxs... ";
        int s_idx = nd_id_2_idx_map[src_id];
        int d_idx = nd_id_2_idx_map[dst_id];
        //std::cout << "Done.\n";

        float dist[N];
        int prev[N];
        //std::cout << "Init dist and prev arrays... ";
        for(int i = 0; i < N; i++) {
            dist[i] = std::numeric_limits<float>::max();
            prev[i] = -1;
        }

        dist[s_idx] = 0;

        //std::cout << "Done.\n";

        std::priority_queue<edge_entry_pair, std::vector<edge_entry_pair>, std::greater<edge_entry_pair>> Q;

        //std::cout << "PQ made\n";

        Q.push(edge_entry_pair(0, s_idx));

        //std::cout << "Inserted (0, " << s_idx << ") to PQ\n";


        while(!Q.empty()) {
            edge_entry_pair U = Q.top();
            Q.pop();

            //std::cout << "U = " << U.second << "\n";

            if(U.second == d_idx) {
                //std::cout << U.second << " == " << d_idx << "... breaking\n";
                break;
            }

            std::vector<edge_entry_pair> E = adj_list[U.second];
            //std::cout << "Node " << U.second << " has " << E.size() << " edges.\n";
            for(int i = 0; i < E.size(); i++) {
                int V = E[i].second;
                float W = E[i].first;

                //std::cout << "\t" << i << ") V = " << V << " W = " << W << "\n";

                //std::cout << "\tdist[V] = " << dist[V] << " > dist[" << U.second << "] + W = " << dist[U.second] << " + " << W << " = " << dist[U.second] + W << "\n";
                //std::cout << "\t->" << (dist[V] > dist[U.second] + W) << "\n";
                if(dist[V] > dist[U.second] + W) {
                    dist[V] = dist[U.second] + W;
                    Q.push(E[i]);
                    prev[E[i].second] = U.second;
                }
            }
        }

        //std::cout << "Exit while loop\n";

        int U = d_idx;
        if(prev[U] != -1 || U == s_idx) {
            while(U > 0){
                route.push_back(U);
                if(U == s_idx) {
                    break;
                }
                U = prev[U];
            }
        }

        std::reverse(route.begin(), route.end());

        return route;
    }

    void PrintRoute(std::vector<int> route) {
        if(!init) {
            std::cout << "Map hasn't been initialized. Init with Map.Init()\n";
            return;
        }
        for(int i = 0; i < route.size(); i++) {
            std::cout << route[i] << "\t" << node_id[route[i]] << "\n";
        }
    }

    void Init() {
        if(init) {
            std::cout << "Init called twice. Not initializing again.\n";
            return;
        }

        InitNodeID2Idx();
        // Reserve space for entries for each node
        adj_list.resize(N);
        node_xy.resize(N);
        node_id.resize(N);
        InitGraph();

        init = true;
    }

private:
    // File names relative to build directory
    const std::string nodes_mat_file = "../maps/lcc_nodes.mat";
    const std::string nodes_mat_var = "nodes";
    const std::string connected_mat_file = "../maps/lcc_connected.mat";
    const std::string connected_mat_var = "connected";
    const std::string adj_list_mat_file = "../maps/lcc_adj_list.mat";
    const std::string adj_list_mat_var = "adjacency_list";

    std::vector<std::vector<edge_entry_pair>> adj_list; // Graph representation
    std::vector<std::pair<double, double>> node_xy; // Node coordinates
    std::vector<long> node_id; // Node coordinates
    int N; // Number of nodes
    bool init;

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
            node_xy[i] = std::pair<double, double>(node_xy_ML[idx*2], node_xy_ML[idx*2 + 1]);
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

};

#endif // MAP_H