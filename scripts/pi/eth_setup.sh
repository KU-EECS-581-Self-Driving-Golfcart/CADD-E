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
