#include "Map.hpp"

int main() {
    Map map;
    map.ShortestRoute(1, 2);
    std::vector<int> test_r;
    test_r.resize(3);
    test_r[0] = 100;
    test_r[1] = 200;
    test_r[2] = 300;
    map.PrintRoute(test_r);
    map.PrintGraph();
    map.Init();
    map.Init();

    //std::vector<int> route = map.ShortestPath(7208400631, 7208400684);
    //map.printRoute(route);
    //R_Tree r_tree = Init_RTree();
    
    // Read in map
    // Init R Tree with map nodes

    //while(true) {
        // Read GPS
        // Check for updated target node
        // Find closest node to position

        // Update route
        // Send route with ROS
    //}
}