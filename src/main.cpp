#include <iostream>
#include <vector>
#include <unistd.h>
#include <chrono>
#include <pthread.h>
#include <cmath>
#include <cstdlib>

// Locatization and planning includes
#include "telemetry.h"
#include "pathPlanner.h"
#include "routePlanner.h"
#include "testRoutePlanner.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/FrenetPath.h"
#include "localization/frenet_optimal_trajectory_planner/src/FrenetOptimalTrajectory/py_cpp_struct.h"

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "cadd_e_interface/msg/gps.hpp"
#include "cadd_e_interface/msg/route.hpp"
#include "cadd_e_interface/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include "interfaces/msg/reference.hpp"
#include "interfaces/msg/state.hpp"
#include "interfaces/msg/controls.hpp"

using std::placeholders::_1;

bool gps_init = false;
bool imu_init = false;

// Globals updated by ROS subscribers
double lat = 0.0;        // GPS Latitude
double lon = 0.0;        // GPS Longitude
float speed_mps = 0.0;   // Ground speed
float heading = 0.0;    // Heading (in degrees)
int tgt_hole = 9;   // Target hole (1-9)
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
		//RCLCPP_INFO(this->get_logger(), "I heard: '%f, %f'", msg->lat, msg->lon);
        lat = msg->lat;
        lon = msg->lon;
        speed_mps = msg->speed_mps;
        telem.update_gps(speed_mps);
        gps_init = true;
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
		//RCLCPP_INFO(this->get_logger(), "I heard [h: %f; l_a: %f, %f, %f]", msg->heading, msg->lin_acc_x, msg->lin_acc_y, msg->lin_acc_z);
        std::chrono::milliseconds time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		telem.update_imu(msg->lin_acc_x, msg->lin_acc_y, msg->lin_acc_z, msg->heading, msg->ang_vel[2], time);
        imu_init = true;
	}
  	rclcpp::Subscription<cadd_e_interface::msg::IMU>::SharedPtr subscription_;
};

class TargetLocationSubscriber : public rclcpp::Node
{
	public:
    TargetLocationSubscriber() : Node("target_loc_subscriber") {
    	subscription_ = this->create_subscription<std_msgs::msg::String>("teeInfo", 10, std::bind(&TargetLocationSubscriber::topic_callback, this, _1));
    }
  	private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    	RCLCPP_INFO(this->get_logger(), "# I heard: '%s'", msg->data.c_str());
    	tgt_hole = stoi(std::string(1, msg->data.at(0)));
    	tgt_loc = msg->data.at(1);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class RoutePublisher : public rclcpp::Node
{
    private:
        size_t count_;
        rclcpp::Publisher<cadd_e_interface::msg::Route>::SharedPtr publisher_;
        std::vector<double> lat_vec;
        std::vector<double> lon_vec;

    public:
        RoutePublisher() : Node("route_publisher"), count_(0)
        {
            this->publisher_ = this->create_publisher<cadd_e_interface::msg::Route>("route", 1);
        }

        void publish(int size, std::pair<double, double>* latlon)
        {
            // Populate message
            auto message = cadd_e_interface::msg::Route();

            for(int i = 0; i < size; i++) {
                message.lat.push_back((latlon + i)->first);
                message.lon.push_back((latlon + i)->second);
            }
            message.route_size = size;
            std::cout << "# Publishing route of size " << message.route_size << "\n";
            RCLCPP_INFO(this->get_logger(), "# Publishing: route of size %d'", message.route_size);
            this->publisher_->publish(message);
        }
};

class StatePublisher : public rclcpp::Node
{
    private:
        rclcpp::Publisher<interfaces::msg::State>::SharedPtr publisher_;

    public:
        StatePublisher() : Node("state_publisher")
        {
            this->publisher_ = this->create_publisher<interfaces::msg::State>("state", 1);
        }

        void publish(float X, float Y, float xdot, float ydot, float psi, float psidot)
        {
            auto statemsg = interfaces::msg::State();
            statemsg.x = X;
            statemsg.y = Y;
            statemsg.xdot = xdot;
            statemsg.ydot = ydot;
            statemsg.psi = psi;
            statemsg.psidot = psidot;

            std::cout << "# Publishing state (" << X << ','
                << Y << ',' << xdot << ',' << ydot << ','
                << psi << ',' << psidot << ")\n";
            this->publisher_->publish(statemsg);
        }
};

class ReferencePublisher : public rclcpp::Node
{
    private:
        rclcpp::Publisher<interfaces::msg::Reference>::SharedPtr publisher_;

    public:
        ReferencePublisher() : Node("reference_publisher")
        {
            this->publisher_ = this->create_publisher<interfaces::msg::Reference>("reference", 1);
        }

        void publish(vector<double> x,
                     vector<double> y,
                     vector<double> vel,
                     vector<double> yaw,
                     vector<double> curvatures)
        {
            auto refmsg = interfaces::msg::Reference();
            for(size_t i=0; i < x.size(); i++) {
                refmsg.x.push_back(x[i]);
                refmsg.y.push_back(y[i]);
                refmsg.vel.push_back(vel[i]);
                refmsg.yaw.push_back(yaw[i]);
                refmsg.curvatures.push_back(curvatures[i]);
            }
            refmsg.length = x.size();

            std::cout << "# Publishing reference: first entry(" << x[0] << ','
                << y[0] << ',' << vel[0] << ',' << yaw[0] << ','
                << curvatures[0] << ")\n";
            this->publisher_->publish(refmsg);
        }
};

void* spin_sub_executor(void* exec_ptr) {
    //std::cout << "Created ROS subscriber thread\n";
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

    // Create publisher nodes.
    std::shared_ptr<RoutePublisher> route_pub_node = std::make_shared<RoutePublisher>();
    std::shared_ptr<StatePublisher> state_pub_node = std::make_shared<StatePublisher>();
    std::shared_ptr<ReferencePublisher> ref_pub_node = std::make_shared<ReferencePublisher>();

    // Add subscriber nodes to executor
    rclcpp::executors::SingleThreadedExecutor ros_sub_executor;
    ros_sub_executor.add_node(gps_sub_node);
    ros_sub_executor.add_node(imu_sub_node);
    ros_sub_executor.add_node(tgt_loc_sub_node);

    // Run subscriber executor on separate thread
    pthread_t subscriber_thread;
    pthread_create(&subscriber_thread, nullptr, spin_sub_executor, (void*) &ros_sub_executor);

    // Add publisher node to executor
    rclcpp::executors::SingleThreadedExecutor ros_pub_executor;
    ros_pub_executor.add_node(route_pub_node);
    ros_pub_executor.add_node(state_pub_node);
    ros_pub_executor.add_node(ref_pub_node);

    // Init IMU telemetry tracker
    telem.start();

    // TODO: Remove dummy variable
    // Hole 1 Gold tee box
    // lat = 38.9777938333;
    // lon = -95.2642973333;

    std::cout << "#Waiting for sensors... \n";
    while(!(imu_init && gps_init)) {}
    std::cout << "#\t\tDone!\n";

    // Init route planning and path planning modules
    PathPlanner pp(2);
    RoutePlanner rp;

    // CIRCLE TEST - Test route planner to generate circular waypoints
        //int radius = 5;
        //TestRoutePlanner rp(lat, lon, 24, radius);
        //std::cout << "radius = " << radius << "\n";

    telem.heading = 0.0;

    // Init route and path variables
    std::vector<float> routeX(rp.size());
    std::vector<float> routeY(rp.size());

    int runtime_ms = 0;  // Track total program runtime in ms. Used for average iteration time calculation
    int path_misses = 0; // Track # of iterations where path planner fails to generate path

    int i = 0; // Iteration count
    // UPDATED RANDOM POSITION TEST - Random seed for simulating random offsets from position.
        // srand((unsigned) time(NULL)); Testing only
    while(rclcpp::ok()) {
        // UPDATED RANDOM POSITION TEST - Set car position to i'th waypoint.
            // std::tie(lat, lon) = rp.m.NodeLatLon()[i];

        auto start_time = std::chrono::high_resolution_clock::now();
        std::cout << "# Iteration (" << i++ << ")\n";

        // Localize position from GPS
        std::tie(telem.pos_x, telem.pos_y) = rp.latlon2local(lat, lon);
        
        // UPDATED RANDOM POSITION TEST - Offset position of car by some random value [-0.75, 0.75] in X and Y direction.
            // float offset_x = 0.15*(rand() % 10) - 0.75;
            // float offset_y = 0.15*(rand() % 10) - 0.75;

            // std::cout << "pos_offset =   [" << offset_x << ", " << offset_y << "]\n";

            // telem.pos_x += offset_x;
            // telem.pos_y += offset_y;

        // CIRCLE TEST - Offset position for circular route testing.
            //telem.pos_x += radius*2;
            //telem.pos_y += radius;

        std::cout << "pos_actual =   [" << telem.pos_x << ", " << telem.pos_y << "] #(local)\n";
        std::cout << "#             [" << lat << ", " << lon << "] (global)\n";
        std::cout << "# Telemetry:  [h: " << telem.heading << "; l_a: " << telem.acc_x << ", " << telem.acc_y << ", " << telem.acc_z << "; speed: [" << telem.speed_mps << "]\n";
        std::cout << "# Target loc: [" << tgt_hole << tgt_loc << "]\n";

        // Generate route
        std::vector<int> routeIdxs = rp.ShortestRouteIdxs(lat, lon, tgt_hole, tgt_loc);

        std::tie(routeX, routeY) = rp.LocalRoute(routeIdxs);

        // CIRCLE TEST - Route publisher must be commented
        route_pub_node->publish(int(routeIdxs.size()), rp.GlobalRoute(routeIdxs).data());

        std::cout << "route = [\n";
        if (routeX.size() >= 7 || false) {
            for(size_t i = 0; i < 7; i++) {
                std::cout << "\t[" << routeX[i] << ", " << routeY[i] << "],\n";
            }
        } else {
            for(size_t i = 0; i < routeX.size(); i++) {
                std::cout << "\t[" << routeX[i] << ", " << routeY[i] << "],\n";
            }
        }
        
        std::cout << "]\n";

        auto rp_split = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(rp_split - start_time);
        std::cout << "# RP Duration: " << duration_ms.count() << " ms\n";

        if(routeX.size() < 2) {
            std::cout << "# No route\n";
            continue;
        }
        // Generate path
        FrenetPath* path = pp.getPathRegular(telem, routeX.data(), routeY.data(), routeX.size());

        auto path_gen_split = std::chrono::high_resolution_clock::now();
        duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(path_gen_split - rp_split);
        std::cout << "# PP Pathgen Duration: " << duration_ms.count() << " ms\n";

        path = pp.cleanPath(path);

        if(path) {
            std::cout << "best_path = [\n";
            for(size_t i = 0; i < path->x.size(); i++) {
                std::cout << "\t[" << path->x[i] << ", " << path->y[i] << "],\n";
            }
            std::cout << "]\n";
            /*
            * Generate controls.
            */
            state_pub_node->publish(telem.pos_x,
                telem.pos_y,
                telem.speed_mps*cos(telem.heading),
                telem.speed_mps*sin(telem.heading),
                telem.heading,
                telem.headingdot);
            ref_pub_node->publish(
                path->x,
                path->y,
                path->s_d,
                path->yaw,
                path->c
            );
        } else {
            path_misses++;
            std::cout << "# no path found\n";
            std::cout << "best_path = []\n";
        }
        
        auto stop_time = std::chrono::high_resolution_clock::now();
        duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - path_gen_split);
        std::cout << "# PP Path Cleaning Duration: " << duration_ms.count() << " ms\n";

        duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
        std::cout << "# Duration: " << duration_ms.count() << " ms\n";
        runtime_ms += duration_ms.count();
        std::cout << "# Average Duration: " << runtime_ms/i << " ms\n";
        std::cout << "# Path Misses=" << path_misses << "\n";
        std::cout << "# Path Success Rate = " << float((float(i) - float(path_misses))/(float(i))) << "\n";
    }
    rclcpp::shutdown();
    return 0;
}
