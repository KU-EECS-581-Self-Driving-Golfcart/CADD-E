# Requirement 3
## Test running control loop on Raspberry Pi

With Ubuntu 20 and ROS2 working on the Raspberry Pi, I moved the code for the control loop over and tested the performance.

In the tests I did, the control loop was able to complete a single iteration in ~100ms. These tests were conducted without running the sensor modules - I assume this would make the execution time of the control loop slower, although its possible that there would be no change because the Raspberry Pi's CPU has 4 cores.

Because we decided to run the control loop on the DeepRacer and the sensor modules on the Raspberry Pi, I did no further tests.