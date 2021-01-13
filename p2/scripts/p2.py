#!/usr/bin/env python2
import rospy
import state_storage
from geometry_msgs.msg import Point
from pilz_robot_programming import *
import math

__REQUIRED_API_VERSION__ = "1"
REFERENCE_FRAME = "workpiece"
ROBOT_NAME="prbt"
HOME_POSITION = [0,0,0,0,0,0] #Achswerte der Home-Stellung

#Bewegt den Roboter in die Home-Stellung
#Vorbedingung: Roboter kann sich kollisionsfrei in die Nullstellung bewegen
#Nachbedingung: Roboter befindet sich in der Nullstellung
def home(robot):
    robot.move(Ptp(goal=HOME_POSITION))

def main():
    robot = Robot(__REQUIRED_API_VERSION__)
    state_store = state_storage.StateStorage(ROBOT_NAME)
    print "Stored states: ", state_store.listStateNames()
    #to retrieve state (array of joint positions): state_store.getState("state_name")
    home(robot)
    #TODO: your code
    home(robot)

if __name__=='__main__':
    # Init a ros node
    rospy.init_node('p2_program_node')
    main()
