#!/usr/bin/env python2
from copy import *
import rospy
from geometry_msgs.msg import Pose, Point
import tf
from pilz_robot_programming import *
import math
import gripper_util

__REQUIRED_API_VERSION__ = "1"
ROBOT_NAME="prbt"
HOME_POSITION = [0,0,0,0,0,0] #Achswerte der Home-Stellung

WORLD_FRAME = "/world"
FIRST_CUBE_FRAME = "/first_cube"

#Das Programm sollte mit allen Belegungen der beiden untenstehenden Variablen funktionieren!
ROW_FRAME = "/A" #"/B", "/C"
N_CUBES = 6 #Laenge der Wuerfel-Reihe, 0-6

class CubeRow:
    def __init__(self, robot):
        self.robot = robot
        self.gripper = gripper_util.Gripper()
        #self.gripper.close() #Oeffnen und schliessen des Greifers
        #self.gripper.open()
        self.row_pose = Pose()
        self.first_cube_pose = Pose()
        self.__lookupFrames()

    def __lookupFrames(self):
        listener = tf.TransformListener()
        listener.waitForTransform(WORLD_FRAME, ROW_FRAME, rospy.Time(0), rospy.Duration(10))

        #Auslesen der Position der Reihe
        (trans, rot) = listener.lookupTransform(WORLD_FRAME, ROW_FRAME, rospy.Time(0))
        self.row_pose.position.x = trans[0]
        self.row_pose.position.y = trans[1]
        self.row_pose.position.z = trans[2]

        self.row_pose.orientation.x = rot[0] #Orientierung in Quaternionen. Mit from_euler(a,b,c) koennen Eulerwinkel in Quaternionen umgerechnet werden
        self.row_pose.orientation.y = rot[1]
        self.row_pose.orientation.z = rot[2]
        self.row_pose.orientation.w = rot[3]

        #Auslesen der Startposition des ersten Wuerfels
        listener.waitForTransform(WORLD_FRAME, FIRST_CUBE_FRAME, rospy.Time(0), rospy.Duration(10))
        (trans, rot) = listener.lookupTransform(WORLD_FRAME, FIRST_CUBE_FRAME, rospy.Time(0))
        self.first_cube_pose.position.x = trans[0]
        self.first_cube_pose.position.y = trans[1]
        self.first_cube_pose.position.z = trans[2]
        self.first_cube_pose.orientation.x = rot[0]
        self.first_cube_pose.orientation.y = rot[1]
        self.first_cube_pose.orientation.z = rot[2]
        self.first_cube_pose.orientation.w = rot[3]

    def main(self):
        #TODO: your code
        pass

if __name__=='__main__':
    rospy.init_node('p3_program_node')
    robot = Robot(__REQUIRED_API_VERSION__)
    cube_row = CubeRow(robot)
    cube_row.main()
