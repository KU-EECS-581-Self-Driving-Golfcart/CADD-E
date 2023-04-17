cd ~
echo "Sourcing ROS"
source /opt/ros/foxy/setup.bash

echo "Sourcing cadd_e_interface"
cd ~/CADD-E/cadd_e_interface
colcon build --packages-select cadd_e_interface
. install/setup.bash

echo "Launching rosbridge in background"
cd ~/rosbridge_suite
. install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

echo "Starting UI"
cd ~/CADD-E/UI-App/UI
npm start