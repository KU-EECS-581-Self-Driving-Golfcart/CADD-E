#include <iostream>
#include <unistd.h>

// Locatization and planning includes
#include "pathPlanner.h"
#include "routePlanner.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "cadd_e_interface/msg/gps.hpp"
#include "cadd_e_interface/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

// Globals updated by ROS subscribers
float lat = 0.0;        // Y location (in meters)
float lon = 0.0;        // X location (in meters)
float heading = 0.0;    // Heading (in degrees)
float lin_acc_x = 0.0;  // Linear acceleration (X direction)
float lin_acc_y = 0.0;  // Linear acceleration (Y direction)
float lin_acc_z = 0.0;  // Linear acceleration (Z direction)
int tgt_hole = 0;   // Target hole (1-9)
char tgt_loc = '\0';    // Target location ("H"(ole), "B"(lack), (B)"R"(onze), "S"(ilver), "G"(old))

class GPSSubscriber : public rclcpp::Node {
  public:
  GPSSubscriber()
  : Node("gps_subscriber")
  {
    subscription_ = this->create_subscription<cadd_e_interface::msg::GPS>(
      "position", 1, std::bind(&GPSSubscriber::topic_callback, this, _1));
  }

  private:
  void topic_callback(const cadd_e_interface::msg::GPS::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f, %f'", msg->lat, msg->lon);
    lat = msg->lat;
    lon = msg->lon;
  }
  rclcpp::Subscription<cadd_e_interface::msg::GPS>::SharedPtr subscription_;
};

class IMUSubscriber : public rclcpp::Node {
  public:
  IMUSubscriber()
  : Node("imu_subscriber")
  {
    subscription_ = this->create_subscription<cadd_e_interface::msg::IMU>(
      "telemetry", 1, std::bind(&IMUSubscriber::topic_callback, this, _1));
  }

  private:
  void topic_callback(const cadd_e_interface::msg::IMU::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard [h: %f; l_a: %f, %f, %f]", msg->heading, msg->lin_acc_x, msg->lin_acc_y, msg->lin_acc_z);
    heading = msg->heading;
    lin_acc_x = msg->lin_acc_x;
    lin_acc_y = msg->lin_acc_y;
    lin_acc_z = msg->lin_acc_z;
  }
  rclcpp::Subscription<cadd_e_interface::msg::IMU>::SharedPtr subscription_;
};

class TargetLocationSubscriber : public rclcpp::Node
{
  public:
    TargetLocationSubscriber()
    : Node("target_loc_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "target_location", 10, std::bind(&TargetLocationSubscriber::topic_callback, this, _1));
    }
  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      tgt_hole = (int)(msg->data.at(0));
      tgt_loc = msg->data.at(1);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr gps_sub_node = std::make_shared<GPSSubscriber>();
    rclcpp::Node::SharedPtr imu_sub_node = std::make_shared<IMUSubscriber>();
    rclcpp::Node::SharedPtr tgt_loc_sub_node = std::make_shared<TargetLocationSubscriber>();
    rclcpp::executors::SingleThreadedExecutor ros_sub_executor;

    ros_sub_executor.add_node(gps_sub_node);
    ros_sub_executor.add_node(imu_sub_node);
    ros_sub_executor.add_node(tgt_loc_sub_node);

    ros_sub_executor.spin_once();

    // Init ROS subscribers
    PathPlanner pp(1);
    RoutePlanner rp;

    std::vector<float> routeX(rp.size());
    std::vector<float> routeY(rp.size());
    FrenetPath* path = nullptr;

    int i = 0;
    while(rclcpp::ok()) {
        std::cout << i++ << "\n";

        ros_sub_executor.spin_once();

        std::cout << "New coords: [" << lat << ", " << lon << "]\n";
        std::cout << "New telemetry: [h: " << heading << "; l_a: " << lin_acc_x << ", " << lin_acc_y << ", " << lin_acc_z << "]\n";

        std::tie(routeX, routeY) = rp.ShortestRoute(lat, lon, tgt_hole, tgt_loc);
        // std::tie(routeX, routeY) = rp.m.ShortestRoute(7208400605, 7220557245); // Hole 1 black tee to hole 9 green
    
        //PrintRoute(routeX, routeY);

        // Path Planning Initial Conditions
        FrenetInitialConditions fot_ic = {
            0.0, // Current longitudinal position s
            1.5, // Target speed [m/s]
            0.0, // Lateral position c_d [m]
            0.0, // Lateral speed c_d_d [m/s]
            0.0, // Lateral acceleration c_d_dd [m/s^2]
            0.5, // Target speed [m/s]
            routeX.data(), // Waypoints X
            routeY.data(), // Waypoints Y
            static_cast<int>(routeX.size()), // Number of waypoints
            nullptr, //o_llx
            nullptr, //o_lly
            nullptr, //o_urx
            nullptr, //o_ury
            0        // number of obstacles
        };

        path = pp.getPath(fot_ic);
        if(path) {
            std::cout << "got path\n";
        } else {
            std::cout << "no path found\n";
        }
        // TODO: Convert path to clothoid

        sleep(1);

    }
    rclcpp::shutdown();
    return 0;
}