# Source ROS
source /opt/ros/foxy/setup.bash

# Build sensor publishers and interfaces
cd ros2_sensor_ws
colcon build --packages-select cadd_e_interface gps_pubsub us_imu_pubsub
. install/setup.bash

# Run GPS publisher in background
ros2 run gps_pubsub talker &
ros2 run us_imu_pubsub talker &