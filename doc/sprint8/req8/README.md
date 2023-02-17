# Requirement 8
## Investigate C/C++ codegen

I followed an official MATLAB guide to C++ codegen [here](https://www.mathworks.com/help/dsp/ug/generate-c-code-from-matlab-code-1.html) but ran into issues because some portions of my code were not viable for codegen. Parts of the third party OSM parsing code I was using were not usable as was the entire [trajectoryOptimalFrenet](https://www.mathworks.com/help/nav/ref/trajectoryoptimalfrenet.html) object, which is the basis for the path planning procedure.

Because of this, I investigated alternative routes which could be taken to achieve the same goal which is being able to do route planning and path planning in C++.

Route planning - which is basically Dijkstra's algorithm - can be easily implemented in C++ as long as the graph data structure is present. Using the [matio-cpp](https://github.com/ami-iit/matio-cpp) library, I should be able to read in the adjacency list and map data generated in MATLAB into C++ code. I will do this in Sprint 9.

Path planning will be more difficult to implement in C++ from the ground up. I found an open source C++ repo, [erdos-project/frenet_optimal_trajectory_planner](https://github.com/erdos-project/frenet_optimal_trajectory_planner) which accomplishes the same path generation that my MATLAB code did. Assuming I can read in the data into C++ and generate a route with Dijkstra's algorithm, I should be able to generate an optimal path using this repo. I will do this in Sprint 9.