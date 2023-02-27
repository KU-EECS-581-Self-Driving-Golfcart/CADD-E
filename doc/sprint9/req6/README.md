# Requirement 6
## C++ Path Planning

I found an open source C++ repo, [erdos-project/frenet_optimal_trajectory_planner](https://github.com/erdos-project/frenet_optimal_trajectory_planner) which accomplishes the same path generation that my MATLAB code did. Using this code as the basis for the path planning module, I was able to make provide a list of GPS waypoints from the route planning module as the reference trajectory for the path planning. We can also provide more information like position, speed, acceleration, etc. from our GPS and IMU as well as the collision space from the collision space identifier to generate an optimal route which avoids obstacles.

We can then pass this path to the controller which can send the proper commands to the DeepRacer to control the car to follow the path.

My code for initially testing the C++ path planning module can be found [here](https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E/blob/main/localization/pathPlanner.cpp).