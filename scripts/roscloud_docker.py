#!/usr/bin/python
import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from roscloud_base import push_launch
import rospy

if __name__ == "__main__":
    rospy.init_node('roscloud')
    print(rospy.get_param_names())
    launch_file = rospy.get_param('~launch_file')
    push_launch(launch_file)

