# Build interfaces
cd cadd_e_interfaces
colcon build --packages-select cadd_e_interfaces
. install/setup.bash

cd ..
colcon build --packages-select cadd-e
