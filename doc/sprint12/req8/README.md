I wrote code to allow the controller to communicate via ROS2, which can be found in CADD-E/src/control.

The controller takes a reference trajectory as input, runs the control problem, and sends back the control suggestion over ROS.
