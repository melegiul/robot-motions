#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker

rospy.init_node( 'marker_pub')
pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
while not rospy.is_shutdown():
    marker = Marker()
    marker.header.frame_id = 'workpiece'
    marker.id = 0
    marker.type = 10 #mesh
    marker.pose = Pose(position=Point(0, 0, 0), orientation=Quaternion(0,0,0,1))
    marker.scale = Vector3(0.3, 0.1, 0.25)
    marker.color.r = 1.0
    marker.color.g = 0.3
    marker.color.b = 0.1
    marker.color.a = 1.0
    marker.mesh_resource = 'package://p2/meshes/pipe.dae'
    pub.publish(marker)

