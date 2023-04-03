From `CADD-E/src/control/`
```
colcon build
source install/setup.bash
ros2 run mpc control
```
The package name is `mpc` and the entry point is named `control`.

Topics are `/reference` and `controls`.