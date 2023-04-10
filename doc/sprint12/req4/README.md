# Requirement 4
## Read in IMU data through USB and pass through IMU publisher

The code for reading in sensor data from the BNO055 IMU sensor through the FT232H (which can be thought of as a USB adapter for the IMU) is found in qwiic_imu.py

Here is a sample output:

![Sensor data output](imu_output.png 'Sensor data output')

The code for reading in sensor data and passing through a ROS publisher is found in imu_publisher.py

Here is a sample output:

![IMU ROS message output](imu_pub_output.png 'IMU ROS message output')
