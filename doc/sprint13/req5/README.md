# Requirement 5
## Integrate Ultrasonic sensors to Raspberry Pi sensor system

Because the Raspberry Pi is running Ubuntu 20, I needed to utilize the FT232H breakout board in a similar fashion to when I setup the IMU. The FT232H can only be used by one program at a time so I had to create a single publisher to read and publish data from the IMU and both ultrasonic sensors.

The code for this can be found [here](https://github.com/KU-EECS-581-Self-Driving-Golfcart/us_imu_pubsub).