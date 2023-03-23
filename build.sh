# Build interfaces
source /opt/ros/foxy/setup.bash

cd cadd_e_interface
colcon build --packages-select cadd_e_interface
. install/setup.bash

cd ..
colcon build --packages-select cadd-e
. install/setup.bash