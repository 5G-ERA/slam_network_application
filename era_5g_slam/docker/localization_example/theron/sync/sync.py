#!/usr/bin/env python

import rclpy
from rclpy.node import Node 
#from point_cloud_interfaces.msg import CompressedPointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import qos_profile_sensor_data

class TopicSynchronizer(Node):
    
    def __init__(self):
        super().__init__('topic_synchronizer')
        self.last_stamp = 0
        print("start")
   
        #points_sub = Subscriber(self, CompressedPointCloud2, '/robot/top_laser/point_cloud/draco')
        #odom_sub = Subscriber(self, Odometry, '/robot/odometry/filtered')
        scan_front = Subscriber(self, LaserScan, '/robot/front_laser/scan')
        scan_rear = Subscriber(self, LaserScan, '/robot/rear_laser/scan')
        odom_sub = Subscriber(self, Odometry, '/robot/robotnik_base_control/odom/fixed')
        imu_sub = Subscriber(self, Imu, '/robot/imu/data', qos_profile=qos_profile_sensor_data)
    
        self.synchronizer = ApproximateTimeSynchronizer([scan_front, scan_rear, odom_sub, imu_sub], queue_size=4, slop=0.025)
        self.synchronizer.registerCallback(self.callback)
        
        #self.points_pub = self.create_publisher(CompressedPointCloud2, '/synchronized/draco', 10)
        self.scan_front = self.create_publisher(LaserScan, '/synchronized/scan_front', 10)
        self.scan_rear = self.create_publisher(LaserScan, '/synchronized/scan_rear', 10)
        self.odom_pub = self.create_publisher(Odometry, '/synchronized/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/synchronized/imu', 10)
        

    def callback(self, scan_front_msg, scan_rear_msg, odom_msg, imu_msg):  #points_msg
        # Your synchronization logic here
        #self.points_pub.publish(points_msg)
	
        #stmp = odom_msg.header.stamp
        #scan_front_msg.header.stamp = stmp
        #scan_rear_msg.header.stamp = stmp
        #imu_msg.header.stamp = stmp
        
        stmp = scan_front_msg.header.stamp
        
        # continuity check
        test_stmp = stmp.sec * 10**9 + stmp.nanosec
        if test_stmp < self.last_stamp:
            print("continuity check failed")
            return
            
        self.last_stamp = test_stmp
        
        #print(f"{odom_msg.header.stamp.nanosec:10}")
        #print(f"{scan_front_msg.header.stamp.nanosec:10}")
        #print(f"{scan_rear_msg.header.stamp.nanosec:10}")
        #print(f"{imu_msg.header.stamp.nanosec:10}")
        #print()
        
        self.scan_front.publish(scan_front_msg)
        self.scan_rear.publish(scan_rear_msg)
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
