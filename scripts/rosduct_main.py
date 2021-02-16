#!/usr/bin/env python3

import rospy
from rosduct.rosduct_impl import ROSduct

if __name__ == '__main__':
    rospy.init_node('rosduct')
    r = ROSduct()
    r.spin()
