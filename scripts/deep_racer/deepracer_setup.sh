source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

echo "Setting up CADD-E repo"
git clone --recurse-submodules https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E
cd CADD-E
sudo chmod +x build.sh setup.sh
./setup.sh

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

echo "Install and setup Cyclone DDS"
sudo apt-get install ros-foxy-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

echo "Installing Python dependencies"
pip3 install pymongo tornado cvxpy