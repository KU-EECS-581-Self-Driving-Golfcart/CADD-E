# Requirement 1
## Interface ROS with Sensors

To satisfy this requirement, I reformatted a microSD with Ubuntu Server 20.04 (64-bit) to boot the Raspberry Pi with and then installed ROS 2 (Foxy release).

From here, I created Python publisher and C++ subscriber packages as well as custom messages to communicate GPS and IMU data on two topics: "position" (for GPS data) and "telemetry" (for IMU data).

I integrated the GPS polling functionality I developed in a past sprint into the GPS publisher so sensor data will be read and published on the "position" topic.

I was unable to use my code for reading IMU data because the Raspberry Pi GPIO ports aren't readily accessible on Ubuntu. I ordered a new [part](https://www.adafruit.com/product/2264) to take care of the issue. With this new part the IMU data may be read in through a USB port.

Here is a screenshot of test data being sent from the GPS publisher (left) to the GPS subscriber (right)
![GPS pubsub output](gps_pubsub_output.png 'GPS pubsub output')

Here is a screenshot of test data being sent from the IMU publisher (left) to the IMU subscriber (right)
![IMU pubsub output](imu_pubsub_output.png 'IMU pubsub output')