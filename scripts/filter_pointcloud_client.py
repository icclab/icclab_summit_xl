#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Trigger

def call_pointcloud_filter_service():
    print "Requesting point cloud filtering"
    rospy.wait_for_service('/summit_xl/filter_pointcloud')
    try:
        service_proxy = rospy.ServiceProxy('/summit_xl/filter_pointcloud', Trigger)
        resp1 = service_proxy()
        print "Service invoked"
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    call_pointcloud_filter_service()
