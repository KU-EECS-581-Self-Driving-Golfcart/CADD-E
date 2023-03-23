#include "pathPlanner.h"
#include "routePlanner.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "cadd_e_interface/msg/gps.hpp"
#include "cadd_e_interface/msg/imu.hpp"

#include <unistd.h>

using std::placeholders::_1;

float lat = 0.0;
float lon = 0.0;
float heading = 0.0;
float lin_acc_x = 0.0;
float lin_acc_y = 0.0;
float lin_acc_z = 0.0;

class GPSSubscriber : public rclcpp::Node
{
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
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f, %f'", msg->lat, msg->lon);
    lat = msg->lat;
    lon = msg->lon;
  }
  rclcpp::Subscription<cadd_e_interface::msg::GPS>::SharedPtr subscription_;
};

class IMUSubscriber : public rclcpp::Node
{
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
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f, %f'", msg->lat, msg->lon);
    heading = msg->heading;
    lin_acc_x = msg->lin_acc_x;
    lin_acc_y = msg->lin_acc_y;
    lin_acc_z = msg->lin_acc_z;
  }
  rclcpp::Subscription<cadd_e_interface::msg::IMU>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr gps_sub_node = std::make_shared<GPSSubscriber>();
    rclcpp::Node::SharedPtr imu_sub_node = std::make_shared<IMUSubscriber>();
    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(gps_sub_node);
    executor.add_node(imu_sub_node);

    std::cout << "initial spin...\n";
    executor.spin_once();

    // Init ROS subscribers
    PathPlanner pp(1);
    RoutePlanner rp;

    std::vector<double> routeX(rp.size());
    std::vector<double> routeY(rp.size());
    FrenetPath* path = nullptr;

    int i = 0;
    while(rclcpp::ok()) {
        std::cout << ++i << "\n";
        //int hole = 1;       // TODO
        //int loc = 0;        // TODO

        //std::tie(routeX, routeY) = rp.ShortestRoute(lat, lon, hole, loc);

        std::cout << "spinning...\n";
        executor.spin_once();

        std::cout << "New coords: [" << lat << ", " << lon << "]\n";
        std::cout << "New telemetry: [h: " << heading << "; l_a: " << lin_acc_x << ", " << lin_acc_y << ", " << lin_acc_z << "]\n";

        std::tie(routeX, routeY) = rp.m.ShortestRoute(7208400605, 7220557245); // Hole 1 black tee to hole 9 green
    
        //std::cout << "Route:\n";
        //PrintRoute(routeX, routeY);

        // Path Planning Initial Conditions
        FrenetInitialConditions fot_ic = {
            0.0, //34.6, // S_0
            0.0, //7.10964962, // Speed
            0.0, //-1.35277168, 
            0.0, //-1.86,
            0.0,
            0.5, // Target speed
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