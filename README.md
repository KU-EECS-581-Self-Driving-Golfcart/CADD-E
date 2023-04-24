# CADD-E: CAmera-Directed Driving Endeavor
## Overview
Computer Science capstone project at the University of Kansas - an autonomous golf cart system capable of driving the cart path for 9 holes of golf at Lawrence Country Club with minimal input from the driver.

<figure>
<img src=doc/assets/cadd_e.jpg>
<figcaption align = "center"><b>Figure 1: CADD-E</b></figcaption>
</figure><br>

## Getting Started
Please review the bill of materials [here](doc/BOM.pdf). This document outlines the components needed to replicate this project, noteably: AWS DeepRacer (the car's platform), Raspberry Pi 3B+ (computer for processing sensor data), a BNO-055 IMU (responsible for providing the car's heading and angular velocity), a ZED-F9P GPS kit (responsible for providing the car's position and ground speed), a USB camera, and two HC-SR04 ultrasonic sensors. The following sections detail the steps required to prepare the DeepRacer and Raspberry Pi to run CADD-E's software.

#### Getting Started: DeepRacer
First, make sure your DeepRacer is running Ubuntu 20. If not, follow [these steps](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-ubuntu-update.html) to upgrade it. Then run the following commands (these steps are also outlined in `scripts/deep_racer/deepracer_setup.sh`):
```console
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

echo "Setting up CADD-E repo"
git clone --recurse-submodules https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E
cd CADD-E
sudo chmod +x build.sh scripts/setup.sh scripts/deepracer/* scripts/pi/*
./scripts/setup.sh

echo "CADD-E repo dependencies setup"
echo "Setting up UI dependencies"
curl -sL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt-get install -y nodejs
echo "Node installed"
cd UI-App/UI
npm install react
echo "React installed"
echo "Building rosbridge system"
cd ~
git clone https://github.com/RobotWebTools/rosbridge_suite
cd rosbridge_suite
colcon build
. install/setup.bash

echo "Installing Python dependencies"
pip3 install pymongo tornado cvxpy
```

#### Getting Started: Raspberry Pi
First, make sure your Raspberry Pi 3B+ is running Ubuntu 20 Server Edition and has ROS 2 Foxy installed ([install steps](https://docs.ros.org/en/foxy/Installation.html)). Then run the following commands (these steps are also outlined in `scripts/pi/pi_setup.sh`):
```console
mkdir -p ros2_sensor_ws/src
cd ros2_sensor_ws/src
git clone https://github.com/KU-EECS-581-Self-Driving-Golfcart/cadd_e_interface
git clone https://github.com/KU-EECS-581-Self-Driving-Golfcart/gps_pubsub
git clone https://github.com/KU-EECS-581-Self-Driving-Golfcart/us_imu_pubsub
git clone --recurse-submodules https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E

sudo apt install libopencv-dev python3-opencv
```

### Getting Started: Hardware
CADD-E's hardware is fairly simple. It is comprised of a 3D printed mount which sits on top of the DeepRacer and holds the sensors. The sensors connect to the Raspberry Pi's USB ports. The Raspberry Pi and DeepRacer are connected by an Ethernet cable.

Print the following:
- [DeepRacer Mount](https://a360.co/3lBapGs) x1
- [Ultrasonic Sensor Mount](https://cults3d.com/:667627) x2

Drill holes into the mount to connect the Ultrasonic Sensor mounts and FT232H breakout board.

Setup the hardware according to these pictures:
<figure>
<img src=doc/assets/cadd_e_top.png>
<figcaption align = "center"><b>Figure 2: CADD-E Hardware Top View</b></figcaption>
<img src=doc/assets/cadd_e_left.png>
<img src=doc/assets/cadd_e_right.png>
<figcaption align = "center"><b>Figure 3: CADD-E Hardware Left and Right View</b></figcaption>
<img src=doc/assets/cadd_e_front.png>
<figcaption align = "center"><b>Figure 4: CADD-E Hardware Front View</b></figcaption>
</figure><br>

## Building and Executing
- Make sure all cables are connected correctly according to the "Getting Started: Hardware" section
- Power on the DeepRacer (make sure to connect the servo battery)
- Power on the Raspberry Pi
- Open 2 terminals on the DeepRacer
```console
##### DeepRacer #####
### Terminal One ###
# Build
cd CADD-E
./scripts/kill_jobs.sh # Kill unnecessary background jobs
./scripts/clean_logs.sh # Free up storage space
./build.sh # Build CADD-E packages
source cadd_e_interface/install/setup.bash
source src/control/install/setup.bash

### Terminal Two ###
cd CADD-E
source cadd_e_interface/install/setup.bash
source src/control/install/setup.bash
source install/setup.bash

##### Raspberry Pi #####
tmux # Use tmux to open two terminal windows. See https://tmuxcheatsheet.com for help
# Ctrl+B "
# Ctrl+B Up-Arrow
source /opt/ros/foxy/setup.bash
./CADD-E/scripts/pi/eth_setup.sh # Setup ethernet connection
# Ctrl+B Down-Arrow
source /opt/ros/setup.bash
./CADD-E/scripts/pi/run_sensors.sh # Run sensors
# Ctrl+B Up-Arrow
ros2 topic list # Verify topics /position, /ultrasonic, and /telemetry are present

##### DeepRacer #####
### Terminal One ###
./scripts/deep_racer/eth_setup.sh
ping 169.254.0.3 # Verify connection to Raspberry Pi. If this doesn't work, run ethernet setup scripts on both devices and try again. Also make sure Wi-Fi is disabled on the DeepRacer

### Terminal Two ###
ros2 run mpc control # Run controller

### Terminal One ###
./build/cadd-e/main # Run route planner and path planner
```

## Architecture
Our design adapts the self-driving car architecture presented by Badue et al. (2019) [1]. Figure 1 shows our adapted architecture. At the high level, we distinguish between the perception and decision making systems. The perception system receives information via the cart’s sensors (camera, radar, and GPS) in order to estimate the car’s state and build a representation of its surroundings. The decision-making system plans and carries out actions via the car’s actuators (throttle, brake, and steering) in order to navigate between a fixed initial point and destination while satisfying certain constraints, like passenger comfort and avoiding obstacles. Within those systems a number of subsystems can be identified.
<figure>
<img src=doc/assets/architecture.png>
<figcaption align = "center"><b>Figure 5: CADD-E Architecture</b></figcaption>
</figure><br>

### Offline Map
Holds static information about the environment, including drivable routes and available behavior at intersections and in unstructured environments. We plan to use OpenStreetMaps for the offline maps, like Mercedes’ Bertha self-driving car [2].

### Localizer
Identifies the cart’s location within the offline map using GPS data. Since sub-5cm precision is required for respecting cart path boundaries, we will use a GPS equiped with Real Time Kinematics (RTK) adjustment.

### Collision Space Identifier
Identifies regions the cart should avoid based on analysis of present and past camera and radar data. A computer vision system to detect objects and do semantic segmentation will be required for obstacle avoidance and motion planning. Our current plan of action is to mount a front-facing camera to the golf cart in order to process the environment in front of the golf cart.
Our object detection model will likely piggyback off of an open-source model (likely a YOLO model) which may then be further trained as needed to suit our needs. After receiving the detections of the objects, we may then further classify them in order to get a better understanding of their orientation. From an object’s class and orientation we will be able to predict potential areas that the object is likely move to in the future. This will allow us to stop when a person is walking across the cart path or slow down and pull over if another golf cart is headed towards our cart.
Semantic segmentation will be used in order to tell us where the cart path is located and more importantly if there are unexpected obstacles in the cart path. For example, if a tree branch has fallen in the path the segmentation model will understand that that branch is not the path. We may take this information and then proceed to avoid the obstacle.

### Behavior Selector
Determines the currect driving behavior by choosing a reference path to follow and a goal speed. The reference path extends a few seconds ahead of the cart, and the goal speed is the desired speed at the end of the reference path. A trajectory is a path associated with a goal speed. The available reference trajectories are
- cartPath: Follows the cart path as stored in the offline maps. Computed offline by modeling the path.
- softStop: Pull off the path and slow to a stop. Useful for stopping at a tee box or to play a ball.
- hardStop: Stop on the path. Parametrized by the distance to stop. Useful for avoiding pop-up obstacles.
<figure>
<img width="400" src=doc/assets/behavior.png>
<figcaption align = "center"><b>Figure 2: Finite State Machine for Cart Behavior</b></figcaption>
</figure><br><br>

Usually, the reference path will be the pre-planned cart path stored in the offline maps. But if the user inputs a stop command in the user interface, for example, the behavior selector will switch to the softStop trajectory which pulls off the path and slows to a stop. Such a stop trajectory will be a pre-defined curve. Once the user wants to continue along the path, the behavior selector will switch back to following the path reference at the path speed limit. The behavior selector can be programmed as a finite state machine, for example in [2][5][3].

### Motion Planner
Calculates an obstacle-free trajectory considering the reference path and desired speed which accounts for the cart’s dynamic constraints and passenger comfort. We assume the motion planner receives a representation of the online map which encodes the (safety-expanded) location of obstacles. The motion planner adjusts the reference path to maintain a safe distance from obstacle regions. This problem reduces to a nonlinear optimization problem which can be solved by applying nonlinear model predictive control (NMPC). MPC is an optimal control method which uses a model of the plant, in this case the golf cart, to optimize control inputs in a receding horizon fashion. Due to the driving dynamics, the optimization problem is nonlinear, which increases the computational demands. Regardless, NMPC is well-suited for autonomous vehicle planning in real time, as shown in [5][6]. Significant MPC design decisions are the choice of plant model and cost function, which we will determine from literature review and simulation.

### Obstacle Avoider
Slows the target speed to behave more cautiously around potential obstacles. Due to variations between the plant model and reality combined with real-time optimization constraints, the motion planner may compute a suboptimal trajectory with respect to the distance from potential obstacles. We will implement a final layer of obstacle avoidance to allow more cautious behavior by integrating probabilistic obstacle regions calculated by the collision space identifier. For example, we can simulate the potential paths of obstacles and slow down accordingly, as in [7], a test-car implementation which avoided all accidents in a year of operation.

### Controller
Generates actuator (brake, throttle, steering) commands to implement a given trajectory. Again, the controller can be framed as an optimization problem and solved using an NMPC approach. By decomposing the MPC controllers into a high-level path replanning module (the motion planner) and a low-level path following module (the controller), we are basically following [6]’s hierarchical controller model, which demonstrated real-time performance on icy roads when implemented on a test car. The controller will use a higher-fidelity vehicle model than the motion planner to fully account for dynamic constraints.

### User Interface
Lets the user start and stop the golf cart. The golf cart will feature an easily accessible touch screen that will serve as a User Interface. The UI will be a simple display split into two sections.
The left section will always provide any relevant information about the cart’s motion such as vehicle speed. Below this telemetry data will be a set of buttons that are shown depending on the state of the vehicle. If the vehicle is currently stopped, the user will be presented with a large “Go” button followed by two buttons that allow the user to choose where they would like to go. Once the user has selected either the “Next TeeBox” button or the “Next Green” button, they can press the “Go” button and the cart will switch states and begin transit. Once the golf cart is in transit, the user will be presented with a simple “Stop” button. Once pressed, this button will apply the brakes until the cart is back to the stop state. This simple system of stopping the cart will be necessary in case user input is needed to stop the cart. Similarly, if the user needs to apply corrective steering, they can do this directly through the steering wheel rather than interacting with the UI.
<figure>
<img width="400" src=doc/assets/initial_ui.png>
<figcaption align = "center"><b>Figure 3: Draft UI Design</b></figcaption>
</figure><br><br>

The right section of the UI will display a map of the golf course that the cart is currently navigating. This map will provide the user with a simple view of how the cart will navigate to its next destination according to the online map of the cart.

## References
[[1]](https://arxiv.org/abs/1901.04407)
C. Badue, R. Guidolini, R. V. Carneiro, P. Azevedo, V. B. Cardoso, A. Forechi, L. Jesus, R. Berriel, T. M. Paixao, F. Mutz, et al., “Self-Driving cars: A Survey,” Expert Systems with Applications, vol. 165, p. 113 816, 2021.

[[2]](https://ieeexplore.ieee.org/document/6803933)
J. Ziegler, P. Bender, M. Schreiber, H. Lategahn, T. Strauss, C. Stiller, T. Dang, U. Franke, N. Appenrodt, C. G. Keller, et al., “Making Bertha Drive—An Autonomous Journey on a Historic Route,” IEEE Intelligent transportation systems magazine, vol. 6, no. 2, pp. 8–20, 2014.

[[3]](https://ieeexplore.ieee.org/document/7056521)
K. Jo, J. Kim, D. Kim, C. Jang, and M. Sunwoo, “Development of Autonomous Car—Part II: A Case Study on the Implementation of an Autonomous Driving System Based on Distributed Architecture,” IEEE Transactions on Industrial Electronics, vol. 62, no. 8, pp. 5119–5132, 2015.

[[4]](https://ieeexplore.ieee.org/document/8489397)
R. Guidolini, L. G. Scart, L. F. Jesus, V. B. Cardoso, C. Badue, and T. Oliveira-Santos, “Handling Pedestrians in Crosswalks Using Deep Neural Networks in the IARA Autonomous Car,” in 2018 international joint conference on neural networks (ijcnn), IEEE, 2018, pp. 1–8.

[[5]](https://ieeexplore.ieee.org/document/6901109?arnumber=6901109)
M. A. Abbas, R. Milman, and J. M. Eklund, “Obstacle avoidance in real time with Nonlinear Model Predictive Control of autonomous vehicles,” Canadian journal of electrical and computer engineering, vol. 40, no. 1, pp. 12–22, 2017.

[[6]](https://escholarship.org/content/qt8xd0b56h/qt8xd0b56h_noSplash_6469c4e93fb947f62914e1d03570224f.pdf) 
Y. Gao, Model Predictive Control for Autonomous and Semiautonomous Vehicles. University of California, Berkeley, 2014.

[[7]](https://ieeexplore.ieee.org/document/7795866)
R. Guidolini, C. Badue, M. Berger, L. de Paula Veronese, and A. F. De Souza, “A simple yet effective obstacle avoider for the IARA autonomous car,” in 2016 IEEE 19th international conference on intelligent transportation systems (ITSC), IEEE, 2016, pp. 1914–1919.
