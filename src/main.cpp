#include "pathPlanner.h"
#include "routePlanner.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "cadd_e_interface/msg/gps.hpp"

#include <unistd.h>

using std::placeholders::_1;

class GPSSubscriber : public rclcpp::Node
{
public:
  GPSSubscriber()
  : Node("gps_subscriber")
  {
    subscription_ = this->create_subscription<cadd_e_interface::msg::GPS>(          // CHANGE
      "position", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const cadd_e_interface::msg::GPS::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f, %f'", msg->lat, msg->lon);              // CHANGE
  }
  rclcpp::Subscription<cadd_e_interface::msg::GPS>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr sub_node = std::make_shared<GPSSubscriber>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sub_node);
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
        std::cout << i << "\n";
        //double gps_x = 0;   // TODO
        //double gps_y = 0;   // TODO
        //int hole = 1;       // TODO
        //int loc = 0;        // TODO

        //std::tie(routeX, routeY) = rp.ShortestRoute(gps_x, gps_y, hole, loc);

        std::cout << "spinning...\n";
        executor.spin_once();

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

    }
    rclcpp::shutdown();
    return 0;
}