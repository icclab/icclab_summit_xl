#!/usr/bin/python
#
# Send a value to change the opening of the Robotiq gripper using an action
#

import argparse
import rospy
import copy
import geometry_msgs.msg
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from tf.transformations import *
from copy import deepcopy

markerArray = MarkerArray()
topic = 'visualization_marker_array'

def place_marker_at_pose(publisher, poseStamped):
  marker_x = Marker(
    type=Marker.ARROW,
    id=0,
    lifetime=rospy.Duration(180),
    pose=poseStamped.pose,
    scale=Vector3(0.1, 0.01, 0.01),
    header=poseStamped.header,
    color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
  markerArray.markers.append(marker_x)
  # RPY to convert: -90deg around z -> we get y from x
  quat_tf = poseStamped.pose.orientation
  quat_x = [quat_tf.x, quat_tf.y, quat_tf.z, quat_tf.w]
  quat_rot = quaternion_from_euler(0, 0, -1.5707)
  quat_y = quaternion_multiply(quat_rot, quat_x)
  marker_y = Marker(
    type=Marker.ARROW,
    id=1,
    lifetime=rospy.Duration(180),
    pose=copy.deepcopy(poseStamped.pose),
    scale=Vector3(0.1, 0.01, 0.01),
    header=poseStamped.header,
    color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
  marker_y.pose.orientation.x = quat_y[0]
  marker_y.pose.orientation.y = quat_y[1]
  marker_y.pose.orientation.z = quat_y[2]
  marker_y.pose.orientation.w = quat_y[3]
  markerArray.markers.append(marker_y)
  # RPY to convert: +90deg around y -> we get z from x
  quat_rot = quaternion_from_euler(0, 1.5707, 0)
  quat_z = quaternion_multiply(quat_rot, quat_x)
  marker_z = Marker(
    type=Marker.ARROW,
    id=2,
    lifetime=rospy.Duration(180),
    pose=copy.deepcopy(poseStamped.pose),
    scale=Vector3(0.1, 0.01, 0.01),
    header=poseStamped.header,
    color=ColorRGBA(0.0, 0.0, 1.0, 0.8))
  marker_z.pose.orientation.x = quat_z[0]
  marker_z.pose.orientation.y = quat_z[1]
  marker_z.pose.orientation.z = quat_z[2]
  marker_z.pose.orientation.w = quat_z[3]
  markerArray.markers.append(marker_z)
  # Publish the MarkerArray
  publisher.publish(markerArray)

if __name__ == '__main__':
  try:
    # Start the ROS node
    rospy.init_node('pose_marker_visualizer')
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    poseStamped = geometry_msgs.msg.PoseStamped();
    poseStamped.header.frame_id = "map"
    poseStamped.pose.position.x = 0.1;
    poseStamped.pose.position.y = 0.1;
    poseStamped.pose.position.z = 0.1;
    poseStamped.pose.orientation.w = 1;
    place_marker_at_pose(publisher, poseStamped)
    rospy.spin()
    
  except rospy.ROSInterruptException:
      print ("Program interrupted before completion")
