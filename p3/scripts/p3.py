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
#Achswerte der Home-Stellung
HOME_POSITION = [0,0,0,0,0,0]

WORLD_FRAME = "/world"
FIRST_CUBE_FRAME = "/first_cube"

#Das Programm sollte mit allen Belegungen der beiden untenstehenden Variablen funktionieren!
# ROW_FRAME = "/A"
# ROW_FRAME = "/B"
ROW_FRAME = "/C"
#Laenge der Wuerfel-Reihe, 0-6
N_CUBES = 6

#Rotationsmatrix, die die Z-Achse nach unten dreht
UPSIDE_DOWN=from_euler(0, math.radians(180), 0)

TARGET_CUBE_DISTANCE = 0.08
START_CUBE_DISTANCE = 0.1
CUBE_COL_NUM = 3
CUBE_ROW_NUM = 2
OFFSET = 0
LIN_VELOCITY = 0.3
LIN_ACCELERATION = 0.3
PTP_VELOCITY = 0.7
PTP_ACCELERATION = 0.4
ANGLE_STRETCH = 1.5
BLEND_RADIUS = 0.05

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

        # Auslesen der Position der Reihe
        (trans, rot) = listener.lookupTransform(WORLD_FRAME, ROW_FRAME, rospy.Time(0))
        self.row_pose.position.x = trans[0]
        self.row_pose.position.y = trans[1]
        self.row_pose.position.z = trans[2]

        # Orientierung in Quaternionen. Mit from_euler(a,b,c) koennen Eulerwinkel in Quaternionen umgerechnet werden
        self.row_pose.orientation.x = rot[0]
        self.row_pose.orientation.y = rot[1]
        self.row_pose.orientation.z = rot[2]
        self.row_pose.orientation.w = rot[3]

        # Auslesen der Startposition des ersten Wuerfels
        listener.waitForTransform(WORLD_FRAME, FIRST_CUBE_FRAME, rospy.Time(0), rospy.Duration(10))
        (trans, rot) = listener.lookupTransform(WORLD_FRAME, FIRST_CUBE_FRAME, rospy.Time(0))
        self.first_cube_pose.position.x = trans[0]
        self.first_cube_pose.position.y = trans[1]
        self.first_cube_pose.position.z = trans[2]
        self.first_cube_pose.orientation.x = rot[0]
        self.first_cube_pose.orientation.y = rot[1]
        self.first_cube_pose.orientation.z = rot[2]
        self.first_cube_pose.orientation.w = rot[3]

    def home(self, robot):
        """Return robot to home position

        Args:
            robot (pilz_robot_programming.Robot): robot to move
        """
        try:
            robot.move(
                Ptp(
                    goal=HOME_POSITION,
                    vel_scale=PTP_VELOCITY,
                    acc_scale=PTP_ACCELERATION
                )
            )
        except RobotMoveFailed as e:
            rospy.loginfo(e)


    def get_cubes_targets_pairs(self):
        """Connect cube and target poses

        Returns:
            list: list of cube/target pose pairs
        """
        cubes_targets_pairs = []
        for cube_id in range(N_CUBES - OFFSET):
            cube_pose = self.get_cube_pose(cube_id)
            target_pose = self.get_target_pose(cube_id)
            euler1 = tf.transformations.euler_from_quaternion(
                [
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w,
                ]
            )
            euler2 = tf.transformations.euler_from_quaternion(
                [
                    cube_pose.orientation.x,
                    cube_pose.orientation.y,
                    cube_pose.orientation.z,
                    cube_pose.orientation.w,
                ]
            )
            # import pdb; pdb.set_trace()
            cubes_targets_pairs.append((cube_pose, target_pose))
        return cubes_targets_pairs


    def get_cube_pose(self, cube_id):
        """Set cube pose according to first cube and cube distances

        Additionally turn round z-axis orientation by quaternion multiplication
        as preparation for setting TCP pose

        Args:
            cube_id (int): cube id

        Returns:
            geometry_msgs.msg.Pose: start pose of cube
        """
        cube_pose = Pose(
            position=Point(
                self.first_cube_pose.position.x + (cube_id / CUBE_COL_NUM * START_CUBE_DISTANCE),
                self.first_cube_pose.position.y + (cube_id % CUBE_COL_NUM * START_CUBE_DISTANCE),
                self.first_cube_pose.position.z
            )
        )
        # turn around z-axis
        p = tf.transformations.quaternion_multiply(
            [
                self.first_cube_pose.orientation.x,
                self.first_cube_pose.orientation.y,
                self.first_cube_pose.orientation.z,
                self.first_cube_pose.orientation.w
            ],
            [
                UPSIDE_DOWN.x,
                UPSIDE_DOWN.y,
                UPSIDE_DOWN.z,
                UPSIDE_DOWN.w
            ]
        )
        cube_pose.orientation.x = p[0]
        cube_pose.orientation.y = p[1]
        cube_pose.orientation.z = p[2]
        cube_pose.orientation.w = p[3]
        return cube_pose


    def get_target_pose(self, cube_id):
        """Sets cube target pose taking into account cube id

        Sets orientation of row according to the angle of x/y-coordinates of row start position
        Row propagates always in direction of the cube depot
        Due to turning around the z-axis, x/y-coordinates must be mirrored

        Args:
            cube_id (int): cube id

        Returns:
            geometry_msgs.msg.Pose: Pose of specified cube
        """
        # set row orientation orthogonal to line between row-origin and world-origin
        row_angle = math.atan(self.row_pose.position.y / self.row_pose.position.x)
        row_angle *= ANGLE_STRETCH
        row_direction = self.get_row_direction()
        target_pose = Pose(
            # x_sign, y_sign control the rows direction
            position=Point(
                self.row_pose.position.x + row_direction['x'] * (cube_id * TARGET_CUBE_DISTANCE * math.sin(row_angle)),
                self.row_pose.position.y + row_direction['y'] * (cube_id * TARGET_CUBE_DISTANCE * math.cos(row_angle)),
                self.row_pose.position.z
            )
        )
        pose_orientation = from_euler(0, 0, row_angle)
        # turn around z-axis
        p = tf.transformations.quaternion_multiply(
            [
                pose_orientation.x,
                pose_orientation.y,
                pose_orientation.z,
                pose_orientation.w
            ],
            [
                UPSIDE_DOWN.x,
                UPSIDE_DOWN.y,
                UPSIDE_DOWN.z,
                UPSIDE_DOWN.w
            ]
        )
        switch = from_euler(math.radians(180), 0, 0)
        p = tf.transformations.quaternion_multiply(
            [
                p[0],
                p[1],
                p[2],
                p[3]
            ],
            [
                switch.x,
                switch.y,
                switch.z,
                switch.w
            ]
        )
        target_pose.orientation.x = p[0]
        target_pose.orientation.y = p[1]
        target_pose.orientation.z = p[2]
        target_pose.orientation.w = p[3]
        return target_pose


    def get_row_direction(self):
        if self.row_pose.position.x < 0 and self.row_pose.position.y < 0:
            # third quadrant propagation rule
            x_sign = 1 
            y_sign = -1 
        elif self.row_pose.position.x < 0 and self.row_pose.position.y > 0:
            # second quadrant propagation rule
            x_sign = -1 
            y_sign = 1
        else:
            #TODO other quadrants if necessary
            pass
        return {'x': x_sign, 'y': y_sign}



    def get_cube(self, robot, pick_pose):
        approach_point = Point(pick_pose.position.x, pick_pose.position.y, pick_pose.position.z+0.15)
        # Nicht mit dem Greifer in den Wuerfel fahren, sondern 5cm darueber, damit er zwischen den Greiferbacken ist.
        grasp_point = Point(pick_pose.position.x, pick_pose.position.y, pick_pose.position.z+0.05)
        # Bewegungen zu einer Sequenz zusammenfassen
        #Zuerst: PTP zu Position direkt ueber den Wuerfel
        try:
            sequence = Sequence()
            sequence.append(
                Ptp(
                    goal=Pose
                    (
                        position=approach_point,
                        orientation=pick_pose.orientation
                    ),
                    vel_scale=PTP_VELOCITY,
                    acc_scale=PTP_ACCELERATION,
                ),
                blend_radius=BLEND_RADIUS
            )
            # Greifer oeffnen
            # self.gripper.open()
            #LIN zum Wuerfel
            sequence.append(
                Lin(
                    goal=Pose(
                        position=grasp_point,
                        orientation=pick_pose.orientation
                    ),
                    vel_scale=LIN_VELOCITY,
                    acc_scale=LIN_ACCELERATION,
                )
            )
            robot.move(sequence)
            #Greifer schliessen
            self.gripper.close()
            #mit LIN gerade nach oben anheben
            robot.move(
                Lin(
                    goal=Pose(
                        position=approach_point,
                        orientation=pick_pose.orientation
                    ),
                    vel_scale=LIN_VELOCITY,
                    acc_scale=LIN_ACCELERATION,
                )
            )
        except RobotMoveFailed as e:
            rospy.loginfo(e)

    #Ablegen eines Wuerfels, der sich schon im Greifer befinden muss, an der gegebenen Position
    def place_cube(self, robot, target_pose):
        approach_point = Point(target_pose.position.x, target_pose.position.y, target_pose.position.z+0.15)
        place_point = Point(target_pose.position.x, target_pose.position.y, target_pose.position.z+0.05)
        try:
            sequence = Sequence()
            #PTP ueber die Ablageposition. blend_radius: Ueberschleifen
            sequence.append(
                Ptp(
                    goal=Pose(
                        position=approach_point,
                        orientation=target_pose.orientation
                    ),
                    vel_scale=PTP_VELOCITY,
                    acc_scale=PTP_ACCELERATION,
                ),
                blend_radius=BLEND_RADIUS
            )
            #LIN Gerade nach unten
            sequence.append(
                Lin(
                    goal=Pose(
                        position=place_point,
                        orientation=target_pose.orientation
                    ),
                    vel_scale=LIN_VELOCITY,
                    acc_scale=LIN_ACCELERATION,
                )
            )
            #Ausfuehren der Sequenz
            robot.move(sequence)
            #Greifer oeffnen
            self.gripper.open()
            #Nach dem Ablegen wieder gerade nach oben fahren
            robot.move(
                Lin(
                    goal=Pose
                    (
                        position=approach_point,
                        orientation=target_pose.orientation
                    ),
                    vel_scale=LIN_VELOCITY,
                    acc_scale=LIN_ACCELERATION,
                )
            )
        except RobotMoveFailed as e:
            rospy.loginfo(e)

    def main(self):
        cubes_targets_pairs = self.get_cubes_targets_pairs()
        self.gripper.open()
        for n in range(N_CUBES-OFFSET):
            # place cubes at targets
            self.get_cube(robot, cubes_targets_pairs[n][0])
            self.place_cube(robot, cubes_targets_pairs[n][1])
        for m in range(N_CUBES-OFFSET):
            # reset cubes positions
            self.get_cube(robot, cubes_targets_pairs[m][1])
            self.place_cube(robot, cubes_targets_pairs[m][0])
        self.home(robot)

if __name__=='__main__':
    rospy.init_node('p3_program_node')
    robot = Robot(__REQUIRED_API_VERSION__)
    cube_row = CubeRow(robot)
    cube_row.main()
