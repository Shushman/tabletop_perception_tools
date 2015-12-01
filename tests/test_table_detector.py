import herbpy
import rospy
import os
import numpy
import math
import openravepy
import tabletop_perception_tools.msg
from tabletop_perception_tools.srv import *
import tf
import geometry_msgs.msg
from tf.transformations import quaternion_matrix,euler_from_matrix,euler_matrix
from IPython import embed

def find_table_plane(service_name = "tools_server/detect_planes",
                        cloud_topic="/head/kinect2/qhd/points"):
    #Assumes single major table plane

    print "waiting for service..."
    rospy.wait_for_service(service_name);
    print "Calling service..."
    try:
        service = rospy.ServiceProxy(service_name, DetectPlane);
        response = service(cloud_topic);
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

rospy.init_node('table_detector',anonymous=True)

from catkin.find_in_workspaces import find_in_workspaces
objects_path = find_in_workspaces(
    search_dirs=['share'],
    project='pr_ordata',
    path='data/test',
    first_match_only=True)[0]

env, robot = herbpy.initialize(sim=True,attach_viewer='interactivemarker')

plane_file = os.path.join(objects_path, 'plane.xml')
plane = env.ReadKinBodyXMLFile(plane_file)

plane_resp = find_table_plane()

plane_pose = numpy.array(quaternion_matrix([
				plane_resp.pose.orientation.x,
				plane_resp.pose.orientation.y,
				plane_resp.pose.orientation.z,
				plane_resp.pose.orientation.w]))

#plane_pose = numpy.eye(4)
plane_pose[0,3] = plane_resp.pose.position.x
plane_pose[1,3] = plane_resp.pose.position.y
plane_pose[2,3] = plane_resp.pose.position.z

timeout = 10
destination_frame = '/map'
detection_frame = '/head/kinect2_rgb_optical_frame'
listener = tf.TransformListener()

listener.waitForTransform(detection_frame,destination_frame,rospy.Time(0),rospy.Duration(timeout))
frame_trans,frame_rot = listener.lookupTransform(destination_frame,detection_frame,rospy.Time(0))
frame_offset = numpy.matrix(quaternion_matrix(frame_rot))
frame_offset[0,3] = frame_trans[0]
frame_offset[1,3] = frame_trans[1]
frame_offset[2,3] = frame_trans[2]

plane_in_world = numpy.array(numpy.dot(frame_offset,plane_pose))

#Add pi/2 to az
curr_rot = plane_in_world[0:3,0:3]
ax,ay,az = euler_from_matrix(curr_rot)
curr_rot = euler_matrix(ax,ay,az+numpy.pi/2)
plane_in_world[0:3,0:3] = curr_rot[0:3,0:3]


env.AddKinBody(plane)
plane.SetTransform(plane_in_world)
