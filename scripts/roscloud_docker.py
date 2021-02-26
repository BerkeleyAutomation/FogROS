#!/usr/bin/python
import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from roscloud_base import push_docker
import rospy

if __name__ == "__main__":
    rospy.init_node('roscloud')
    print(rospy.get_param_names())
    docker_image = rospy.get_param('~docker_image')
    instance_type = rospy.get_param('~instance_type')
    push_docker(docker_image, instance_type)

