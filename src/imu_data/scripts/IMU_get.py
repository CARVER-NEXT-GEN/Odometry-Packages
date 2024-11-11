#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, MagneticField

class IMUgetNode(Node):
    def __init__(self):
        super().__init__('imu_get_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Publishers for IMU and MagneticField data for each sensor
        self.imu_086_pub = self.create_publisher(Imu, "imu_086", 10)
        self.magf_086_pub = self.create_publisher(MagneticField, "magf_086", 10)

        self.imu_055_pub = self.create_publisher(Imu, "imu_055", 10)
        self.magf_055_pub = self.create_publisher(MagneticField, "magf_055", 10)

        # Initialize a structured dictionary to hold sensor data
        self.sensor_data = {
            'imu_086': {
                'acceleration': [0, 0, 0],
                'linear_acceleration': [0, 0, 0],
                'angular_velocity': [0, 0, 0],
                'magnetic_field': [0, 0, 0],
                'euler_angles': [0, 0, 0]
            },
            'imu_055': {
                'acceleration': [0, 0, 0],
                'linear_acceleration': [0, 0, 0],
                'angular_velocity': [0, 0, 0],
                'magnetic_field': [0, 0, 0],
                'euler_angles': [0, 0, 0]
            }
        }

        # Subscription to the cubemx_imu_data topic
        self.create_subscription(Float32MultiArray, "cubemx_imu_data", self.imu_data_callback, qos_profile)

    def imu_data_callback(self, msg):
        # Populate the sensor_data structure with data from the message
        # IMU 086 data
        self.sensor_data['imu_086']['acceleration'] = msg.data[0:3]
        self.sensor_data['imu_086']['linear_acceleration'] = msg.data[3:6]
        self.sensor_data['imu_086']['angular_velocity'] = msg.data[6:9]
        self.sensor_data['imu_086']['magnetic_field'] = msg.data[9:12]
        self.sensor_data['imu_086']['euler_angles'] = msg.data[12:15]

        # IMU 055 data
        self.sensor_data['imu_055']['acceleration'] = msg.data[15:18]
        self.sensor_data['imu_055']['linear_acceleration'] = msg.data[18:21]
        self.sensor_data['imu_055']['angular_velocity'] = msg.data[21:24]
        self.sensor_data['imu_055']['magnetic_field'] = msg.data[24:27]
        self.sensor_data['imu_055']['euler_angles'] = msg.data[27:30]

        # Instantly publish data for each sensor
        self.publish_bno086_data()
        self.publish_bno055_data()


    def publish_bno086_data(self):
        # Publish Imu data for bno086
        imu_086_msg = Imu()
        imu_086_msg.linear_acceleration.x = self.sensor_data['imu_086']['linear_acceleration'][0]
        imu_086_msg.linear_acceleration.y = self.sensor_data['imu_086']['linear_acceleration'][1]
        imu_086_msg.linear_acceleration.z = self.sensor_data['imu_086']['linear_acceleration'][2]

        imu_086_msg.angular_velocity.x = self.sensor_data['imu_086']['angular_velocity'][0]
        imu_086_msg.angular_velocity.y = self.sensor_data['imu_086']['angular_velocity'][1]
        imu_086_msg.angular_velocity.z = self.sensor_data['imu_086']['angular_velocity'][2]

        self.imu_086_pub.publish(imu_086_msg)

        # Publish MagneticField data for bno086
        magf_086_msg = MagneticField()
        magf_086_msg.magnetic_field.x = self.sensor_data['imu_086']['magnetic_field'][0]
        magf_086_msg.magnetic_field.y = self.sensor_data['imu_086']['magnetic_field'][1]
        magf_086_msg.magnetic_field.z = self.sensor_data['imu_086']['magnetic_field'][2]

        self.magf_086_pub.publish(magf_086_msg)

    def publish_bno055_data(self):
        # Publish Imu data for bno055
        imu_055_msg = Imu()
        imu_055_msg.linear_acceleration.x = self.sensor_data['imu_055']['linear_acceleration'][0]
        imu_055_msg.linear_acceleration.y = self.sensor_data['imu_055']['linear_acceleration'][1]
        imu_055_msg.linear_acceleration.z = self.sensor_data['imu_055']['linear_acceleration'][2]

        imu_055_msg.angular_velocity.x = self.sensor_data['imu_055']['angular_velocity'][0]
        imu_055_msg.angular_velocity.y = self.sensor_data['imu_055']['angular_velocity'][1]
        imu_055_msg.angular_velocity.z = self.sensor_data['imu_055']['angular_velocity'][2]

        self.imu_055_pub.publish(imu_055_msg)

        # Publish MagneticField data for bno055
        magf_055_msg = MagneticField()
        magf_055_msg.magnetic_field.x = self.sensor_data['imu_055']['magnetic_field'][0]
        magf_055_msg.magnetic_field.y = self.sensor_data['imu_055']['magnetic_field'][1]
        magf_055_msg.magnetic_field.z = self.sensor_data['imu_055']['magnetic_field'][2]

        self.magf_055_pub.publish(magf_055_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUgetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
