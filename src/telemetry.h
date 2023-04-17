#include <cmath>
#include <chrono>

#ifndef TELEMETRY_H
#define TELEMETRY_H

class Telemetry {
    public:
    float acc_x = 0.0;
    float acc_y = 0.0;
    float acc_z = 0.0;

    //float vel_x = 0.0;  // Longitudinal speed in the car-frame.
    //float vel_y = 0.0;  // Lateral speed in the car-frame.
    //float vel_z = 0.0;

    float speed_mps = 0.0;

    float pos_x = 0.0;  // Global position on east-west axis.
    float pos_y = 0.0;  // Global position on north-south axis.
     
    float heading = 0.0;  // Yaw: angle of car's center line, positive CC from the east-west axis.
    float headingdot = 0.0;  // Time derivative of yaw.


    std::chrono::milliseconds imu_last_updated;

    Telemetry() {}
    ~Telemetry() {}

    void start() {
        imu_last_updated = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    }
    void update_gps(float new_speed) {
        speed_mps = new_speed;
    }

    void update_imu(float new_acc_x, float new_acc_y, float new_acc_z, float new_heading, float new_heading_dot, std::chrono::milliseconds record_time) {
        //vel_x = vel_x+ new_acc_x*1000*(record_time - imu_last_updated).count();
        //vel_y = vel_y+ new_acc_y*1000*(record_time - imu_last_updated).count();
        //vel_z = vel_z+ new_acc_z*1000*(record_time - imu_last_updated).count();
        acc_x = new_acc_x;
        acc_y = new_acc_y;
        acc_z = new_acc_z;
        heading = new_heading;
        headingdot = new_heading_dot;
        imu_last_updated = record_time;
    }

};

#endif // TELEMETRY_H