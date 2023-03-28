# ROS Imports
import rclpy
from rclpy.node import Node
from cadd_e_interface.msg import IMU

# BNO055 (IMU) Imports
import board
import busio
import adafruit_bno055

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(IMU, 'telemetry', 10)
        timer_period = 0.5 # seconds # TODO: Update
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=2000)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.euler = [0.0, 0.0, 0.0]
        self.lin_acc = [0.0, 0.0, 0.0]

    def timer_callback(self):
        msg = IMU()
        # Read sensor values
        lin_acc_read = self.sensor.linear_acceleration
        if lin_acc_read[0]:
            self.lin_acc = lin_acc_read

        euler_read = self.sensor.euler
        if euler_read[0]:
            self.euler = euler_read

        # Set message values
        msg.heading = self.euler[0]
        msg.lin_acc_x = self.lin_acc[0] # TODO: Add calibration offset - sensor.offsets_accelerometer[0]
        msg.lin_acc_y = self.lin_acc[1] # TODO: Add calibration offset - sensor.offsets_accelerometer[1]
        msg.lin_acc_z = self.lin_acc[2] # TODO: Add calibration offset - sensor.offsets_accelerometer[2]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "[H: {}; LA: {}, {}, {}]"'.format(msg.heading, msg.lin_acc_x, msg.lin_acc_y, msg.lin_acc_z))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()