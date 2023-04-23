# Requirement 8
## Send sensor readings from Raspberry Pi to DeepRacer via Ethernet

The process of sending sensor data from the Raspberry Pi to the DeepRacer is essentially setting up both devices on the same wired network so ROS topics can be shared between the two.

In order to do this, I first had to buy a USB -> Ethernet adapter. I then created scripts for each device to assign an IP address to the device over the eth0 interface, disable the firewall, and turn off any wireless interfaces. Doing so made the ROS topics from the Raspberry Pi visible to the DeepRacer.

These scripts can be found here:
- [Raspberry Pi](https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E/blob/main/scripts/pi/eth_setup.sh)
- [DeepRacer](https://github.com/KU-EECS-581-Self-Driving-Golfcart/CADD-E/blob/main/scripts/deep_racer/eth_setup.sh)

Here is the DeepRacer script (the Raspberry Pi script is very similar):
```bash
echo "Setting eth0 IP to 169.254.0.3"
sudo ifconfig eth0 169.254.0.3
sleep 3
sudo ifconfig eth0 169.254.0.3

echo "Restarting ROS2 daemon"
ros2 daemon start
ros2 daemon stop
sleep 2
ros2 daemon start

echo "Disabling firewall"
sudo ufw disable

echo "Shutting down wlan0 interface"
sudo ifconfig wlan0 down
```