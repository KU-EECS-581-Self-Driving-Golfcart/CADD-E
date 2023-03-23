# Requirement 2
## Integrate GPS and IMU ROS subscribers into pathplanning and route planning modules

To satisfy this requirement, I first created a high-level [main.cpp](https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E/blob/main/src/main.cpp) to keep path planning and route planning modules abstracted from the main control loop.

From here, Alex and I created a new repo [cadd_e_interface](https://github.com/KU-EECS-581-Self-Driving-Golfcart/cadd_e_interface) to house the custom messages we'll be sending between the Raspberry Pi and main controller. 

I then integrated IMU and GPS subscribers to read in GPS and IMU data from two topics: "position" (for GPS data) and "telemetry" (for IMU data). The data will be published from the publishers I created in Sprint 10 (Requirement 1).

In addition to this, I created [build](https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E/blob/main/build.sh) and [install](https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E/blob/main/setup.sh) scripts to build the ROS packages in order to utilize the custom interfaces and setup the Frenet Frame submodule to be usable with the main control loop.

Here is a screenshot of test data being sent from a GPS publisher and an IMU publisher (left) to the main control loop (right):
![GPS/IMU pubsub output](imu_gps_pubsub_main.png 'GPS/IMU pubsub output')