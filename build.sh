# Build interfaces
source /opt/ros/foxy/setup.bash

# Build sensor interface
cd cadd_e_interface
colcon build --packages-select cadd_e_interface
. install/setup.bash
cd ..

# Build controller
cd src/control
colcon build
. install/setup.bash
cd .. && cd ..

# Build CADD-E
colcon build --packages-select cadd-e
. install/setup.bash