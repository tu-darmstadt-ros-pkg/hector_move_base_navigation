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
from copy import deepcopy

flipper_radius = 0.37 / 2
axis_diff = 0.41
ctr_2_flipper_end = 0.665 # 0.365
ctr_2_robot_edge = 0.165 # = radius?

def compute_extension(angle):
    #    print angle
    #    print ctr_2_flipper_end * math.cos(angle)
    #    print flipper_radius * abs(math.sin(angle))
    #    print '--'
    return ctr_2_flipper_end * math.cos(angle) + flipper_radius * abs(math.sin(angle)) - ctr_2_robot_edge

def polygon_l2_diff(pl, pr):
    if len(pl) != len(pr):
        return float('inf')
    return sum(map(lambda e1,e2: pow(e1.x - e2.x, 2) + pow(e1.y - e2.y, 2) + pow(e1.z - e2.z, 2), pl, pr), 0)

def list_2_point32(a):
    p = Point32()
    p.x = a[0]
    p.y = a[1]
    p.z = 0
    return p

class footprint_info(object):
    initial = []
    current = []

    # ------------------#
    # plt--prt          #
    #  |    |           #
    # plb--prb          #
    #                   #
    # [plt plb prb prt] #
    # ------------------#
    def set_initial_footprint(self):

        if rospy.has_param('~global_costmap/footprint'):
            polygons = rospy.get_param('~global_costmap/footprint')
            self.initial = map(list_2_point32, polygons)
            self.current = deepcopy(self.initial)

        else:
            plt = Point32() # left right correct? <== TODO!
            plb = Point32()
            prb = Point32()
            prt = Point32()

            plt.x = 0.6
            plt.y = 0.2
            plt.z = 0

            plb.x = -0.6
            plb.y = 0.2
            plb.z = 0

            prb.x = -0.6
            prb.y = -0.2
            prb.z = 0

            prt.x = 0.6
            prt.y = -0.2
            prt.z = 0

            self.initial = [plt, plb, prb, prt]



    def joint_state_callback(self,js):
        ifront = js.name.index('front_flipper_joint')
        irear = js.name.index('rear_flipper_joint')

        rear_cutted = max(min(js.position[irear], math.pi / 2), -math.pi / 2)
        front_cutted = max(min(js.position[ifront], math.pi / 2), -math.pi / 2)

        rear_ext = compute_extension(rear_cutted)
        front_ext = compute_extension(front_cutted)

        footprint = deepcopy(self.initial)

        footprint[0].x += front_ext
        footprint[3].x += front_ext
        footprint[1].x -= rear_ext
        footprint[2].x -= rear_ext

        if polygon_l2_diff(footprint, self.current) > 0.05:
            print '--'
            print polygon_l2_diff(footprint, self.current)
            print '--'

            self.current = deepcopy(footprint)
            pub.publish(footprint)

            print self.initial
            print '--'
            print footprint


    def __init__(self):
        self.set_initial_footprint()


#---------------------
# Subscriber loop
#---------------------
def run_subscriber(fpi):
    rospy.Subscriber('joint_states', JointState, lambda js: fpi.joint_state_callback(js))
    rospy.spin()

#---------------------
# Entry point
#---------------------
if __name__ == '__main__':
    rospy.init_node('footprint_publisher')
    rospy.loginfo('>> Footprint publisher running! <<')

    fpi = footprint_info()

    pub = rospy.Publisher('/move_base/global_costmap/footprint', Polygon, queue_size = 10)
    run_subscriber(fpi)
