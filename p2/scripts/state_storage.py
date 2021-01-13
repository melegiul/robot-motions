#!/usr/bin/env python2
from moveit_msgs.srv import GetRobotStateFromWarehouse, ListRobotStatesInWarehouse
import rospy

class StateStorage:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        rospy.wait_for_service('/get_robot_state')
        rospy.wait_for_service('/list_robot_states')
        self.__get_state = rospy.ServiceProxy('/get_robot_state', GetRobotStateFromWarehouse)
        self.__list_states = rospy.ServiceProxy('/list_robot_states', ListRobotStatesInWarehouse)

    def getState(self, name):
        res = self.__get_state(name, self.robot_name)
        return res.state.joint_state.position[:6] #return only arm joint positions

    def listStateNames(self):
        return self.__list_states("", self.robot_name).states
