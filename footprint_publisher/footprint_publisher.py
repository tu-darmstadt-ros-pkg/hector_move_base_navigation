#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('footprint_publisher')

import rospy
import sys
import math

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

flipper_radius = 0.37 / 2
axis_diff = 0.41
ctr_2_flipper_end = 0.665 # 0.365
ctr_2_robot_edge = 0.165 # = radius?


# ------------------#
# plt--prt          #
#  |    |           #
# plb--prb          #
#                   #
# [plt plb prb prt] #
# ------------------#

def initial_footprint():
    plt = Point32() # left right correct? <== TODO!
    plb = Point32()
    prb = Point32()
    prt = Point32()

    plt.x = 0.4
    plt.y = 0.2
    plt.z = 0

    plb.x = -0.4
    plb.y = 0.2
    plb.z = 0

    prb.x = -0.4
    prb.y = -0.2
    prb.z = 0

    prt.x = 0.4
    prt.y = -0.2
    prt.z = 0

    return [plt, plb, prb, prt]

def compute_extension(angle):
#    print angle
#    print ctr_2_flipper_end * math.cos(angle)
#    print flipper_radius * abs(math.sin(angle))
#    print '--'
    return ctr_2_flipper_end * math.cos(angle) + flipper_radius * abs(math.sin(angle)) - ctr_2_robot_edge


def joint_state_callback(js):
    ifront = js.name.index('front_flipper_joint')
    irear = js.name.index('rear_flipper_joint')

    rear_cutted = max(min(js.position[irear], math.pi / 2), -math.pi / 2)
    front_cutted = max(min(js.position[ifront], math.pi / 2), -math.pi / 2)

    rear_ext = compute_extension(rear_cutted)
    front_ext = compute_extension(front_cutted)

    footprint = initial_footprint()

    footprint[0].x += front_ext
    footprint[3].x += front_ext
    footprint[1].x -= rear_ext
    footprint[2].x -= rear_ext
    pub.publish(footprint)


def run_subscriber():
    rospy.Subscriber('joint_states', JointState, joint_state_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('footprint_publisher')
    rospy.loginfo('>> Footprint publisher running! <<')

    pub = rospy.Publisher('/move_base/global_costmap/footprint', Polygon, queue_size = 10)
    run_subscriber()
