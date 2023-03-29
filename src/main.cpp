#include <iostream>
#include <unistd.h>
#include <chrono>
#include <pthread.h>

// Locatization and planning includes
#include "telemetry.h"
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
double lat = 0.0;        // GPS Latitude
double lon = 0.0;        // GPS Longitude
float heading = 0.0;    // Heading (in degrees)
int tgt_hole = 1;   // Target hole (1-9)
char tgt_loc = 'H';    // Target location ("H"(ole), "B"(lack), (B)"R"(onze), "S"(ilver), "G"(old))
Telemetry telem;

class GPSSubscriber : public rclcpp::Node {
	public:
  	GPSSubscriber() : Node("gps_subscriber") {
		subscription_ = this->create_subscription<cadd_e_interface::msg::GPS>(
    	"position", 1, std::bind(&GPSSubscriber::topic_callback, this, _1));
	}

	private:
  	void topic_callback(const cadd_e_interface::msg::GPS::SharedPtr msg) const {
		RCLCPP_INFO(this->get_logger(), "I heard: '%f, %f'", msg->lat, msg->lon);
        lat = msg->lat;
        lon = msg->lon;
    }
    rclcpp::Subscription<cadd_e_interface::msg::GPS>::SharedPtr subscription_;
};

class IMUSubscriber : public rclcpp::Node {
	public:
	IMUSubscriber() : Node("imu_subscriber") {
		subscription_ = this->create_subscription<cadd_e_interface::msg::IMU>(
		"telemetry", 1, std::bind(&IMUSubscriber::topic_callback, this, _1));
	}

  	private:
  	void topic_callback(const cadd_e_interface::msg::IMU::SharedPtr msg) const {
		RCLCPP_INFO(this->get_logger(), "I heard [h: %f; l_a: %f, %f, %f]", msg->heading, msg->lin_acc_x, msg->lin_acc_y, msg->lin_acc_z);
        std::chrono::milliseconds time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		telem.update_imu(msg->lin_acc_x, msg->lin_acc_y, msg->lin_acc_z, msg->heading, time);
	}
  	rclcpp::Subscription<cadd_e_interface::msg::IMU>::SharedPtr subscription_;
};

class TargetLocationSubscriber : public rclcpp::Node
{
	public:
    TargetLocationSubscriber() : Node("target_loc_subscriber") {
    	subscription_ = this->create_subscription<std_msgs::msg::String>(
    	"teeInfo", 10, std::bind(&TargetLocationSubscriber::topic_callback, this, _1));
    }
  	private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    	//RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    	tgt_hole = stoi(std::string(1, msg->data.at(0)));
    	tgt_loc = msg->data.at(1);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

void* spin_executor(void* exec_ptr) {
    std::cout << "Created ROS subscriber thread\n";
    rclcpp::executors::SingleThreadedExecutor* exec = (rclcpp::executors::SingleThreadedExecutor*) exec_ptr;
    exec->spin();
    return NULL;
}

int main(int argc, char* argv[]) {
    // Init ROS
    rclcpp::init(argc, argv);

    // Create subscriber nodes
    rclcpp::Node::SharedPtr gps_sub_node = std::make_shared<GPSSubscriber>();
    rclcpp::Node::SharedPtr imu_sub_node = std::make_shared<IMUSubscriber>();
    rclcpp::Node::SharedPtr tgt_loc_sub_node = std::make_shared<TargetLocationSubscriber>();

    // Add subscriber nodes to executor
    rclcpp::executors::SingleThreadedExecutor ros_sub_executor;
    ros_sub_executor.add_node(gps_sub_node);
    ros_sub_executor.add_node(imu_sub_node);
    ros_sub_executor.add_node(tgt_loc_sub_node);

    // Run subscriber executor on separate thread
    pthread_t subscriber_thread;
    pthread_create(&subscriber_thread, nullptr, spin_executor, (void*) &ros_sub_executor);

    // Init route planning and path planning modules
    PathPlanner pp(1);
    RoutePlanner rp;

    // Init route and path variables
    std::vector<float> routeX(rp.size());
    std::vector<float> routeY(rp.size());
    FrenetPath* path = nullptr;

    // Init IMU telemetry tracker
    telem.start();

    // TODO: Remove dummy variable
    // Hole 1 Gold tee box
    lat = 38.9777938333;
    lon = -95.2642973333;

    tgt_hole = 3;     // TODO: Remove dummy variable
    tgt_loc = 'H';    // TODO: Remove dummy variable

    int i = 0;
    while(rclcpp::ok()) {
        if(i == 1) {            // TODO: Remove test start
            rclcpp::shutdown(); // TODO: Remove test start
            return 0;           // TODO: Remove test start
        }                       // TODO: Remove test start
        std::cout << "Iteration (" << i++ << ")\n";

        // Localize position from GPS
        std::tie(telem.pos_x, telem.pos_y) = rp.m.latlon2local(lat, lon);

        std::cout << "Position:   [" << telem.pos_x << ", " << telem.pos_y << "] (local)\n";
        std::cout << "            [" << lat << ", " << lon << "] (global)\n";
        std::cout << "Telemetry:  [h: " << telem.heading << "; l_a: " << telem.acc_x << ", " << telem.acc_y << ", " << telem.acc_z << "]\n";
        std::cout << "Target loc: [" << tgt_hole << tgt_loc << "]\n";

        // Generate route
        std::tie(routeX, routeY) = rp.ShortestRoute(lat, lon, tgt_hole, tgt_loc);

        std::cout << "route = [\n";
        for(size_t i = 0; i < routeX.size(); i++) {
            std::cout << "\t[" << routeX[i] << ", " << routeY[i] << "],\n";
        }
        std::cout << "]\n";

        // Generate path
        path = pp.getPathAnytime(telem, routeX.data(), routeY.data(), routeX.size());
        //path = pp.getPathRegular(telem, routeX.data(), routeY.data(), routeX.size());

        if(path) {
            //std::cout << "got path\n";
            // TODO: Remove
            std::cout << "path = [\n";
            for(size_t i = 0; i < path->x.size(); i++) {
                std::cout << "\t[" << path->x[i] << ", " << path->y[i] << "],\n";
            }
            std::cout << "]\n";
        } else {
            std::cout << "no path found\n";
        }
        // TODO: Convert path to clothoid
    }
    rclcpp::shutdown();
    return 0;
}