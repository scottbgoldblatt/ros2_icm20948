import math

import qwiic_icm20948
import rclpy
import sensor_msgs.msg
from rclpy.node import Node


class ICM20948Node(Node):
    def __init__(self):
        super().__init__("icm20948_node")

        # Logger
        self.logger = self.get_logger()

        # Parameters
        self.declare_parameter("i2c_address", 0x69)
        i2c_addr = self.get_parameter("i2c_address").get_parameter_value().integer_value
        self.i2c_addr = i2c_addr

        self.declare_parameter("frame_id", "imu_icm20948")
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.frame_id = frame_id

        self.declare_parameter("pub_rate", 50)
        pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.pub_rate = pub_rate

        # IMU instance
        self.imu = qwiic_icm20948.QwiicIcm20948(address=self.i2c_addr)
        if not self.imu.connected:
            self.logger.info(
                "The Qwiic ICM20948 device isn't connected to the system. Please check your connection."
            )
        self.imu.begin()
        self.imu.setFullScaleRangeGyro(qwiic_icm20948.dps2000)
        self.imu.setFullScaleRangeAccel(qwiic_icm20948.gpm16)
        
        self.imu.enableDlpfGyro(True)
        self.imu.setDLPFcfgGyro(3)       # moderate cutoff (0..7)

        self.imu.enableDlpfAccel(True)
        self.imu.setDLPFcfgAccel(3)

        # Publishers
        self.imu_pub_ = self.create_publisher(sensor_msgs.msg.Imu, "/imu/data_raw", 10)
        self.mag_pub_ = self.create_publisher(
            sensor_msgs.msg.MagneticField, "/imu/mag_raw", 10
        )
        self.pub_clk_ = self.create_timer(1 / self.pub_rate, self.publish_cback)

    def publish_cback(self):
        imu_msg = sensor_msgs.msg.Imu()
        mag_msg = sensor_msgs.msg.MagneticField()
        if self.imu.dataReady():
            try:
                self.imu.getAgmt()
            except Exception as e:
                self.logger.info(str(e))

            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            imu_msg.linear_acceleration.x = self.imu.axRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.y = self.imu.ayRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.z = self.imu.azRaw * 9.81 / 2048.0
            imu_msg.angular_velocity.x = self.imu.gxRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.y = self.imu.gyRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.z = self.imu.gzRaw * math.pi / (16.4 * 180)
            imu_msg.orientation_covariance[0] = -1

            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = self.imu.mxRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.y = self.imu.myRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.z = self.imu.mzRaw * 1e-6 / 0.15

        self.imu_pub_.publish(imu_msg)
        self.mag_pub_.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    icm20948_node = ICM20948Node()
    rclpy.spin(icm20948_node)

    icm20948_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
ICM-20948 Gyroscope Configuration (Table 16)

GYRO_FCHOICE | GYRO_DLPFCFG | 3dB BW (Hz) | NBW (Hz) | Output Rate (Hz)
-----------------------------------------------------------------------
     0       |      x       |   12106    |  12316  | 9000

     1       |      0       |   196.6    |  229.8  | 1125 / (1 + GYRO_SMPLRT_DIV)
     1       |      1       |   151.8    |  187.6  | 1125 / (1 + GYRO_SMPLRT_DIV)
     1       |      2       |   119.5    |  154.3  | 1125 / (1 + GYRO_SMPLRT_DIV)
     1       |      3       |    51.2    |   73.3  | 1125 / (1 + GYRO_SMPLRT_DIV)
     1       |      4       |    23.9    |   35.9  | 1125 / (1 + GYRO_SMPLRT_DIV)
     1       |      5       |    11.6    |   17.8  | 1125 / (1 + GYRO_SMPLRT_DIV)
     1       |      6       |     5.7    |    8.9  | 1125 / (1 + GYRO_SMPLRT_DIV)
     1       |      7       |   361.4    |  376.5  | 1125 / (1 + GYRO_SMPLRT_DIV)

Notes:
- GYRO_SMPLRT_DIV is an 8-bit value (0–255)
- Output rate formula applies when GYRO_FCHOICE = 1
"""
"""
ICM-20948 Accelerometer Configuration (Table 18)

ACCEL_FCHOICE | ACCEL_DLPFCFG | 3dB BW (Hz) | NBW (Hz) | Output Rate (Hz)
-------------------------------------------------------------------------
      0       |       x       |   1209     |  1248   | 4500

      1       |       0       |   246.0    |  265.0  | 1125 / (1 + ACCEL_SMPLRT_DIV)
      1       |       1       |   246.0    |  265.0  | 1125 / (1 + ACCEL_SMPLRT_DIV)
      1       |       2       |   111.4    |  136.0  | 1125 / (1 + ACCEL_SMPLRT_DIV)
      1       |       3       |    50.4    |   68.8  | 1125 / (1 + ACCEL_SMPLRT_DIV)
      1       |       4       |    23.9    |   34.4  | 1125 / (1 + ACCEL_SMPLRT_DIV)
      1       |       5       |    11.5    |   17.0  | 1125 / (1 + ACCEL_SMPLRT_DIV)
      1       |       6       |     5.7    |    8.3  | 1125 / (1 + ACCEL_SMPLRT_DIV)
      1       |       7       |   473      |  499    | 1125 / (1 + ACCEL_SMPLRT_DIV)

Notes:
- ACCEL_SMPLRT_DIV is a 12-bit value (0–4095)
- Data rate = 1.125 kHz / (1 + ACCEL_SMPLRT_DIV)
- Output rate applies after DLPF block
"""
