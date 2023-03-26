#include <cmath>
#include <chrono>

#ifndef TELEMETRY_H
#define TELEMETRY_H

class Telemetry {
    public:
    float acc_x = 0.0;
    float acc_y = 0.0;
    float acc_z = 0.0;

    float vel_x = 0.0;
    float vel_y = 0.0;
    float vel_z = 0.0;

    float pos_x = 0.0;
    float pos_y = 0.0;

    float heading = 0.0;

    std::chrono::milliseconds imu_last_updated;

    Telemetry() {}

    start() {
        imu_last_updated = std::chrono::system_clock::now().time_since_epoch();
    }

    ~Telemetry() {}

    update_imu(float new_acc_x, float new_acc_y, float new_acc_z, float new_heading, std::chrono::milliseconds record_time) {
        vel_x = vel_x + new_acc_x*1000*(record_time - imu_last_updated);
        vel_y = vel_y + new_acc_y*1000*(record_time - imu_last_updated);
        vel_z = vel_z + new_acc_z*1000*(record_time - imu_last_updated);
        acc_x = new_acc_x;
        acc_y = new_acc_y;
        acc_z = new_acc_z;
        heading = new_heading;
        imu_last_updated = record_time;
    }

};

class Acceleration {
    public:
    time 
}

#endif // TELEMETRY_H