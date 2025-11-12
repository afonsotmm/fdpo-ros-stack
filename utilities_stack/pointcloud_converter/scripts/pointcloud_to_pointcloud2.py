#!/usr/bin/env python3
"""
Simple converter node: PointCloud (v1) -> PointCloud2
Converts laser_scan_point_cloud to laser_scan_point_cloud2
"""

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

class PointCloudConverter:
    def __init__(self):
        rospy.init_node('pointcloud_converter', anonymous=False)
        
        # Parameters
        self.input_topic = rospy.get_param('~input_topic', 'laser_scan_point_cloud')
        self.output_topic = rospy.get_param('~output_topic', 'laser_scan_point_cloud2')
        
        # Publisher and Subscriber
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud, self.callback, queue_size=10)
        
        rospy.loginfo(f"[PointCloudConverter] Converting {self.input_topic} -> {self.output_topic}")
    
    def callback(self, msg_in):
        """Convert PointCloud to PointCloud2"""
        # Create PointCloud2 message
        msg_out = PointCloud2()
        msg_out.header = msg_in.header
        
        # Convert points from PointCloud to PointCloud2 format
        # PointCloud has geometry_msgs/Point32[] points
        # PointCloud2 uses a more compact binary format
        points_list = []
        for pt in msg_in.points:
            points_list.append([pt.x, pt.y, pt.z])
        
        # Create PointCloud2 with xyz fields
        msg_out = pc2.create_cloud_xyz32(msg_in.header, points_list)
        
        # Publish
        self.pub.publish(msg_out)

if __name__ == '__main__':
    try:
        converter = PointCloudConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

