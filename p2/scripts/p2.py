#!/usr/bin/env python2
import rospy
import state_storage
from geometry_msgs.msg import Point
from pilz_robot_programming import *
import math
import collections

__REQUIRED_API_VERSION__ = "1"
REFERENCE_FRAME = "workpiece"
ROBOT_NAME="prbt"
HOME_POSITION = [0,0,0,0,0,0] #Achswerte der Home-Stellung

#Bewegt den Roboter in die Home-Stellung
#Vorbedingung: Roboter kann sich kollisionsfrei in die Nullstellung bewegen
#Nachbedingung: Roboter befindet sich in der Nullstellung
def home(robot):
    robot.move(Ptp(goal=HOME_POSITION))

def get_states(state_store):
    """Retrieve goal positions from database

    Precondition: database contains all eight positions
    Postcondition: dictionary set in order of insertion

    Args:
        state_store ([StateStorage]): [robot positions storage]

    Returns:
        [dict]: [key, values for all saved robot positions]
    """
    state_dic = collections.OrderedDict()
    state_dic["start_position"] = state_store.getState("bottom_end")
    state_dic["bottom_corner"] = state_store.getState("bottom_start")
    state_dic["bottom_to_top"] = state_store.getState("bottom_to_top")
    state_dic["top_reached"] = state_store.getState("top_reached")
    state_dic["top_corner"] = state_store.getState("top_corner")
    state_dic["top_next_corner"] = state_store.getState("top_next_corner")
    state_dic["top_to_bottom"] = state_store.getState("top_to_bottom")
    state_dic["bottom_reached"] = state_store.getState("bottom_reached")
    return state_dic

def move_robot(robot, state_dic):
    """compresses position transition to a single sequence

    Precondition: robot can reach start position and traverse according to goven pattern
    Postcondition: start position of robot

    Args:
        robot ([Robot]): [instance, which joints should be moved]
        state_dic ([dict]): [intermediate robot positions]
    """
    sequence = Sequence()
    for k,v in state_dic.iteritems():
        if k == "start_position":
            sequence.append(Ptp(goal=v))
        elif k != "bottom_corner" and k != "top_next_corner":
            sequence.append(Lin(goal=v))
        else:
            sequence.append(Lin(goal=v), blend_radius=0.1)
    sequence.append(Lin(goal=state_dic["start_position"]))
    robot.move(sequence)

def main():
    robot = Robot(__REQUIRED_API_VERSION__)
    state_store = state_storage.StateStorage(ROBOT_NAME)
    print "Stored states: ", state_store.listStateNames()
    #to retrieve state (array of joint positions): state_store.getState("state_name")
    home(robot)
    state_dic = get_states(state_store)
    move_robot(robot, state_dic)
    home(robot)

if __name__=='__main__':
    # Init a ros node
    rospy.init_node('p2_program_node')
    main()
