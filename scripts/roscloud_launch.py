#!/usr/bin/python
import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from roscloud_base import push_launch
import rospy

if __name__ == "__main__":
    rospy.init_node('roscloud')
    print(rospy.get_param_names())
    launch_file = rospy.get_param('~launch_file')
    instance_type = rospy.get_param("~instance_type")
    env_script = rospy.get_param("~env_script", "")
    if not env_script:
        with open("/tmp/empty_bash_env", "w") as f:
            f.write("echo Hello")
        env_script = "/tmp/empty_bash_env"
    push_launch(launch_file, instance_type, env_script)

