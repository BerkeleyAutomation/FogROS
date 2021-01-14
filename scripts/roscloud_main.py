#!/home/keplerc/anaconda3/bin/python

import rospy

if __name__ == '__main__':
    rospy.init_node('roscloud')
    print(rospy.get_param('~instance_nodes', None))
