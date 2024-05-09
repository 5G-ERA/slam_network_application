#!/usr/bin/env python

import rclpy
from rclpy.node import Node 
from point_cloud_interfaces.msg import CompressedPointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import qos_profile_sensor_data

class TopicSynchronizer(Node):
    
    def __init__(self):
        super().__init__('topic_synchronizer')
   
        points_sub = Subscriber(self, CompressedPointCloud2, '/robot/top_laser/point_cloud/draco')
        odom_sub = Subscriber(self, Odometry, '/robot/odometry/filtered')
        imu_sub = Subscriber(self, Imu, '/robot/imu/data', qos_profile=qos_profile_sensor_data)
    
        self.synchronizer = ApproximateTimeSynchronizer([points_sub, odom_sub, imu_sub], queue_size=50, slop=0.05)
        self.synchronizer.registerCallback(self.callback)
        
        self.points_pub = self.create_publisher(CompressedPointCloud2, '/synchronized/draco', 10)
        self.odom_pub = self.create_publisher(Odometry, '/synchronized/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/synchronized/imu', 10)
        

    def callback(self, points_msg, odom_msg, imu_msg):
        # Your synchronization logic here
        self.points_pub.publish(points_msg)
        self.odom_pub.publish(odom_msg)
        self.imu_pub.publish(imu_msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    synchronizer = TopicSynchronizer()
    rclpy.spin(synchronizer)
    synchronizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()