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

TABLE_FRAME = "/table" #Vordere Ecke der Ablageflaeche
#Hoehe der Tuerme, 0-4
N_CUBES = 4
TOTAL_CUBES = 6

# cube depot settings
START_CUBE_DISTANCE = 0.1
CUBE_COL_NUM = 2

# table and tower settings
TABLE_CUBE_HEIGHT = 0.05
TABLE_CUBE_DISTANCE = 0.08
TABLE_OFFSET_X = 0.18
TABLE_OFFSET_Y = 0.08
START_TOWER_X = TABLE_OFFSET_X
MID_TOWER_X = TABLE_OFFSET_X + TABLE_CUBE_DISTANCE
END_TOWER_X = TABLE_OFFSET_X + 2 * TABLE_CUBE_DISTANCE

# settings about velocity and acceleration
LIN_VELOCITY = 0.3
LIN_ACCELERATION = 0.3
PTP_VELOCITY = 0.7
PTP_ACCELERATION = 0.4
BLEND_RADIUS = 0.01
HOME_PTP_VELOCITY = 0.7
HOME_PTP_ACCELERATION = 0.4
HOME_BLEND_RADIUS = 0

# genral picking and placing
UPSIDE_DOWN=from_euler(0, math.radians(180), 0)
APPROACH_HEIGHT = 0.08

# tower states after table setup
SOURCE = Pose(position=Point(START_TOWER_X, TABLE_OFFSET_Y, TABLE_CUBE_HEIGHT * N_CUBES), orientation=UPSIDE_DOWN)
AUXILIARY = Pose(position=Point(MID_TOWER_X, TABLE_OFFSET_Y, 0), orientation=UPSIDE_DOWN)
DESTINATION = Pose(position=Point(END_TOWER_X, TABLE_OFFSET_Y, 0), orientation=UPSIDE_DOWN)

class Hanoi:
    def __init__(self, robot):
        self.robot = robot
        self.gripper = gripper_util.Gripper()
        #self.gripper.close() #Oeffnen und schliessen des Greifers
        #self.gripper.open()
        self.table_pose = Pose()
        self.first_cube_pose = Pose()
        self.__lookupFrames()

    def __lookupFrames(self):
        listener = tf.TransformListener()
        listener.waitForTransform(WORLD_FRAME, TABLE_FRAME, rospy.Time(0), rospy.Duration(10))

        #Auslesen der Position der Reihe
        (trans, rot) = listener.lookupTransform(WORLD_FRAME, TABLE_FRAME, rospy.Time(0))
        self.table_pose.position.x = trans[0]
        self.table_pose.position.y = trans[1]
        self.table_pose.position.z = trans[2]

        self.table_pose.orientation.x = rot[0] #Orientierung in Quaternionen. Mit from_euler(a,b,c) koennen Eulerwinkel in Quaternionen umgerechnet werden
        self.table_pose.orientation.y = rot[1]
        self.table_pose.orientation.z = rot[2]
        self.table_pose.orientation.w = rot[3]

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


    def home(self, robot):
        """Return robot to home position
        pre: no obstacles on route to home position
        post: home position reached

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


    def get_cube_poses(self):
        """get all cube poses in the range of N_CUBES

        Returns:
            list: list of cube poses
        """
        return [self.get_cube_pose(cube_id) for cube_id in range(N_CUBES)]


    def get_cube_pose(self, cube_id):
        """get cube pose depending on first cubes position and cube distances/formation

        Additionally turn round z-axis orientation by quaternion multiplication
        as preparation for setting TCP pose

        Args:
            cube_id (int): cube id

        Returns:
            geometry_msgs.msg.Pose: start pose of cube
        """
        cube_pose = Pose(
            position=Point(
                0 + (cube_id / CUBE_COL_NUM * START_CUBE_DISTANCE),
                0 + (cube_id % CUBE_COL_NUM * START_CUBE_DISTANCE),
                0
            )
        )
        # turn around z-axis
        pose_orientation = from_euler(0, 0, 0)
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
        cube_pose.orientation.x = p[0]
        cube_pose.orientation.y = p[1]
        cube_pose.orientation.z = p[2]
        cube_pose.orientation.w = p[3]
        return cube_pose

    
    def get_table_poses(self, x_offset):
        """depending on x_offset this funtion returns either cube formation for start or end tower

        Args:
            x_offset (float): x-coordinate of start or end tower

        Returns:
            list: list of all tower poses
        """
        return [self.get_table_pose(cube_id, x_offset) for cube_id in range(N_CUBES)]


    def get_table_pose(self, cube_id, x_offset):
        """get single cube pose for either start or end tower depending on x_offset

        Args:
            cube_id (integer): cubes id to get the tower pose
            x_offset (float): either start or end tower coordinate

        Returns:
            geometry_msgs.msg.Pose: a single cubes tower pose
        """
        target_pose = Pose(
            position=Point(
                0 + x_offset,
                0 + TABLE_OFFSET_Y,
                0 + (cube_id * TABLE_CUBE_HEIGHT)
            )
        )
        pose_orientation = from_euler(0, 0, 0)
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
        target_pose.orientation.x = p[0]
        target_pose.orientation.y = p[1]
        target_pose.orientation.z = p[2]
        target_pose.orientation.w = p[3]
        return target_pose


    def get_cube(self, robot, pick_pose, frame, to_home=True, operation_height=0):
        """move gripper above cube position and pick cube up
        pre: cubes pose is in robots range
        post: cube is lifted by gripper

        Args:
            robot (pilz_robot_programming.Robot): robot to move
            pick_pose (geometry_msgs.msg.Pose): pose of cube to pick up
            frame (string): target frame
            to_home (bool, optional): whether to set home as inermediate or not. Defaults to True.
            operation_height (int, optional): fixed approach hight independent from cube pose. Defaults to 0.
        """
        if operation_height != 0:
            approach_height = operation_height
        else:
            approach_height = pick_pose.position.z + APPROACH_HEIGHT
        approach_point = Point(pick_pose.position.x, pick_pose.position.y, approach_height)
        # Nicht mit dem Greifer in den Wuerfel fahren, sondern 5cm darueber, damit er zwischen den Greiferbacken ist.
        grasp_point = Point(pick_pose.position.x, pick_pose.position.y, pick_pose.position.z + 0.05)
        try:
            self.approach_cube(robot, pick_pose, approach_point, grasp_point, frame, to_home)
            self.gripper.close()
            # lift cube with LIN
            robot.move(
                Lin(
                    goal=Pose(
                        position=approach_point,
                        orientation=pick_pose.orientation
                    ),
                    vel_scale=LIN_VELOCITY,
                    acc_scale=LIN_ACCELERATION,
                    reference_frame=frame
                )
            )
        except RobotMoveFailed as e:
            rospy.loginfo(e)


    def approach_cube(self, robot, pick_pose, approach_point, grasp_point, frame, to_home):
        """approach cube depending on current TCP position
        pre: approach point is in robots range
        post: cube is fixed by gripper

        Args:
            robot (pilz_robot_programming.Robot): robot to move
            pick_pose (geometry_msgs.msg.Pose): pose of cube to pick up
            approach_point (geometry_msgs.msg.Position): point above cube
            grasp_point (geometry_msgs.msg.Position): point for gripping cube
            frame (string): target frame
            to_home (bool, optional): whether to set home as inermediate or not. Defaults to True.
        """
        try:
            current_pose = robot.get_current_pose(base=frame)
            current_position = Point(
                round(current_pose.position.x, 2),
                round(current_pose.position.y, 2),
                round(current_pose.position.z, 2)
            )
            approach_point = Point(
                round(approach_point.x, 2),
                round(approach_point.y, 2),
                round(approach_point.z, 2)
            )
            if (
                current_position.x != approach_point.x
                or current_position.y != approach_point.y
                or current_position.z != approach_point.z
            ):
            # if TCP is not right above cube
                sequence = Sequence()
                if to_home:
                    # if there are obstacles between TCP and cube position
                    # PTP motion to home
                    sequence.append(
                        Ptp(
                            goal=HOME_POSITION,
                            vel_scale=HOME_PTP_VELOCITY,
                            acc_scale=HOME_PTP_ACCELERATION,
                            reference_frame=WORLD_FRAME.strip('/')
                        ),
                        blend_radius=HOME_BLEND_RADIUS
                    )
                # PTP motion to cube
                sequence.append(
                    Ptp(
                        goal=Pose
                        (
                            position=approach_point,
                            orientation=pick_pose.orientation
                        ),
                        vel_scale=PTP_VELOCITY,
                        acc_scale=PTP_ACCELERATION,
                        reference_frame=frame
                    ),
                    blend_radius=BLEND_RADIUS
                )
                # LIN to pick up cube
                sequence.append(
                    Lin(
                        goal=Pose(
                            position=grasp_point,
                            orientation=pick_pose.orientation
                        ),
                        vel_scale=LIN_VELOCITY,
                        acc_scale=LIN_ACCELERATION,
                        reference_frame=frame
                    )
                )
                robot.move(sequence)
            else:
                # LIN to pick up cube
                robot.move(
                    Lin(
                        goal=Pose(
                            position=grasp_point,
                            orientation=pick_pose.orientation
                        ),
                        vel_scale=LIN_VELOCITY,
                        acc_scale=LIN_ACCELERATION,
                        reference_frame=frame
                    )
                )
        except RobotMoveFailed as e:
            rospy.loginfo(e)


    def place_cube(self, robot, target_pose, frame, to_home=True, operation_height=0):
        """place cube at target avoiding obstacles
        pre: gripper contains cube
        post: cube set on right place

        Args:
            robot (pilz_robot_programming.Robot): robot to move
            target_pose (geometry_msgs.msg.Pose): cubes target pose
            frame (string): target frame
            to_home (bool, optional): whether to set home as inermediate or not. Defaults to True.
            operation_height (int, optional): fixed approach hight independent from cube pose. Defaults to 0.
        """
        if operation_height != 0:
            # if fixed approach hight is set
            approach_height = operation_height
        else:
            approach_height = target_pose.position.z + APPROACH_HEIGHT
        approach_point = Point(target_pose.position.x, target_pose.position.y, approach_height)
        place_point = Point(target_pose.position.x, target_pose.position.y, target_pose.position.z + 0.05)
        try:
            sequence = Sequence()
            if to_home:
                # if there are obstacles between TCP and cube position
                # PTP motion to home
                sequence.append(
                    Ptp(
                        goal=HOME_POSITION,
                        vel_scale=HOME_PTP_VELOCITY,
                        acc_scale=HOME_PTP_ACCELERATION,
                        reference_frame=WORLD_FRAME.strip('/')
                    ),
                    blend_radius=HOME_BLEND_RADIUS
                )
            # PTP to cube location
            sequence.append(
                Ptp(
                    goal=Pose(
                        position=approach_point,
                        orientation=target_pose.orientation
                    ),
                    vel_scale=PTP_VELOCITY,
                    acc_scale=PTP_ACCELERATION,
                    reference_frame=frame
                ),
                blend_radius=BLEND_RADIUS
            )
            # LIN to place cube
            sequence.append(
                Lin(
                    goal=Pose(
                        position=place_point,
                        orientation=target_pose.orientation
                    ),
                    vel_scale=LIN_VELOCITY,
                    acc_scale=LIN_ACCELERATION,
                    reference_frame=frame
                )
            )
            robot.move(sequence)
            self.gripper.open()
            # TCP moves straight up
            robot.move(
                Lin(
                    goal=Pose
                    (
                        position=approach_point,
                        orientation=target_pose.orientation
                    ),
                    vel_scale=LIN_VELOCITY,
                    acc_scale=LIN_ACCELERATION,
                    reference_frame=frame
                )
            )
        except RobotMoveFailed as e:
            rospy.loginfo(e)




    def tower_of_hanoi(self, robot, n, source, destination, auxiliary):
        """moves cubes from source to destination tower according to "Tower of Hanoi" rules
        pre: source tower was build correctly
        post: moved tower from source to destination

        Args:
            robot (pilz_robot_programming.Robot): robot to move
            n (integer): number of remaining cubes
            source (geometry_msgs.msg.Pose): top level cubes pose of source tower
            destination (geometry_msgs.msg.Pose): top level cubes pose of mid tower
            auxiliary (geometry_msgs.msg.Pose): top level cubes pose of destination tower
        """
        operation_height = (N_CUBES - 1) * TABLE_CUBE_HEIGHT + APPROACH_HEIGHT
        if n == 1:
            source.position.z -= TABLE_CUBE_HEIGHT
            self.get_cube(robot, source, TABLE_FRAME.strip('/'), False, operation_height)
            self.place_cube(robot, destination, TABLE_FRAME.strip('/'), False, operation_height)
            destination.position.z += TABLE_CUBE_HEIGHT
            return
        self.tower_of_hanoi(robot, n-1, source, auxiliary, destination)
        source.position.z -= TABLE_CUBE_HEIGHT
        self.get_cube(robot, source, TABLE_FRAME.strip('/'), False, operation_height)
        self.place_cube(robot, destination, TABLE_FRAME.strip('/'), False, operation_height)
        destination.position.z += TABLE_CUBE_HEIGHT
        self.tower_of_hanoi(robot, n-1, auxiliary, destination, source)


    def main(self):
        """
        set up tables cubes for "Tower of Hanoi" and clears table
        """
        cube_depot = self.get_cube_poses()
        start_tower = self.get_table_poses(START_TOWER_X)
        end_tower = self.get_table_poses(END_TOWER_X)
        self.gripper.open()
        for n in range(N_CUBES):
            # place cubes at targets
            to_home = True
            if n == 0:
                to_home = False
            self.get_cube(robot, cube_depot[n], FIRST_CUBE_FRAME.strip('/'), to_home)
            self.place_cube(robot, start_tower[n], TABLE_FRAME.strip('/'))
        if N_CUBES != 0:
            self.tower_of_hanoi(robot, N_CUBES, SOURCE, DESTINATION, AUXILIARY)
        for m in range(N_CUBES):
            # reset cubes positions
            to_home = True
            if m == 0:
                to_home = False
            self.get_cube(robot, end_tower[N_CUBES - 1 - m], TABLE_FRAME.strip('/'), to_home)
            self.place_cube(robot, cube_depot[N_CUBES - 1 - m], FIRST_CUBE_FRAME.strip('/'))
        self.home(robot)

if __name__=='__main__':
    rospy.init_node('p4_program_node')
    robot = Robot(__REQUIRED_API_VERSION__)
    hanoi = Hanoi(robot)
    hanoi.main()
