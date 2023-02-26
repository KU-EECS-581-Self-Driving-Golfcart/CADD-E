# Requirement 5
## Read in map adjacency list and other data in C++

To satisfy this requirement, exported some variables from my MATLAB code as .mat files and imported the variables to C++ code using the [matio-cpp](https://github.com/ami-iit/matio-cpp) library. This library allows a programmer to read in .mat files as custom class types (e.g., CellArray, StructArray, Struct).

I then created a class I called Map which is outlined in [Map.hpp](https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E/blob/main/localization/Map.hpp). It reads in .mat files designated as constant private member variables and constructs an adjacency list of the nodes and a hash map which maps OpenStreetMap node ID #s to index #s in the hash map: ![Adjacency List Output](adj_list.png 'Adjacency List Output')

Using the hash map and adjacency list, I can take two node IDs (source and destination) and return the shortest route as a list of waypoint coordinates: ![Shortest Path Output](shortest_path.png 'Shortest Path Output')