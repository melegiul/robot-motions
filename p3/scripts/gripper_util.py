#!/usr/bin/env python2
from copy import *
import rospy
from geometry_msgs.msg import Pose, Point
import tf
import math
from gazebo_ros_link_attacher.srv import Attach
from gazebo_msgs.srv import *

ATTACH_SRV = "/link_attacher_node/attach"
DETACH_SRV = "/link_attacher_node/detach"
WORLD_SRV = "/gazebo/get_world_properties"
LINK_SRV = "/gazebo/get_link_state"
CUBE_PRFX = "cube"

#Gripping area in ref to prbt_flange
GRIP_MAX_X = 0.0075
GRIP_MIN_X = -0.0075
GRIP_MAX_Y = 0.04
GRIP_MIN_Y = -0.04
GRIP_MAX_Z = 0.17
GRIP_MIN_Z = 0.145

CUBE_SIZE = 0.05

class Gripper:
    def __init__(self):
        self.__attach = rospy.ServiceProxy(ATTACH_SRV, Attach)
        self.__detach = rospy.ServiceProxy(DETACH_SRV, Attach)
        self.__get_world_props = rospy.ServiceProxy(WORLD_SRV, GetWorldProperties)
        self.__get_link_state = rospy.ServiceProxy(LINK_SRV, GetLinkState)
        self.__attached = []
        self.closed = False

    def close(self):
        if self.closed:
            print "gripper already closed, doing nothing"
            return
        for cube in self.__getGrippableCubes():
            self.__attach('prbt', 'prbt_flange', cube, 'link')
            self.__attached.append(cube)
            print "grabbed " + cube
            self.closed = True

    def open(self):
        for cube in self.__attached:
            self.__detach('prbt', 'prbt_flange', cube, 'link')
            print "released " + cube
        self.__attached = []
        self.closed = False

    def __inGrippingArea(self, position):
        res = True
        c = CUBE_SIZE/2.0
        res = res and (position.x-c <= GRIP_MAX_X and position.x+c >= GRIP_MIN_X)
        res = res and (position.y-c <= GRIP_MAX_Y and position.y+c >= GRIP_MIN_Y)
        res = res and (position.z-c <= GRIP_MAX_Z and position.z+c >= GRIP_MIN_Z)
        return res

    def __getGrippableCubes(self):
        model_names  = self.__get_world_props().model_names
        cubes = [m for m in model_names if m.startswith(CUBE_PRFX)]
        ret = []
        for cube in cubes:
            state = self.__get_link_state(cube+"::link", "prbt::prbt_flange")
            if self.__inGrippingArea(state.link_state.pose.position):
                ret.append(cube)
        return ret
