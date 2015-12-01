#!/usr/bin/env python

from tabletop_perception_tools.srv import *
import rospy
import sys
import geometry_msgs.msg

def extract_clusters(service_name="tools_server/extract_clusters", 
                     cloud_topic="/head/kinect2/qhd/points",
                     segment_planes=True, 
                     num_planes=1, 
                     plane_distance=0.01, 
                     segment_depth=True, 
                     min_depth=0.8, 
                     max_depth=1.0, 
                     cluster_tolerance=0.01, 
                     min_cluster_size=100, 
                     max_cluster_size=10000):
    rospy.wait_for_service(service_name);
    try:
        service = rospy.ServiceProxy(service_name, ExtractClusters);
        response = service(cloud_topic, segment_planes, num_planes, plane_distance, segment_depth, min_depth, max_depth, cluster_tolerance, min_cluster_size, max_cluster_size);
        return response.clusters;
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def find_blocks(service_name="tools_server/find_blocks", 
                     cloud_topic="/head/kinect2/qhd/points",
                     segment_planes=True, 
                     num_planes=1, 
                     plane_distance=0.015, 
                     segment_depth=True, 
                     min_depth=0.9, 
                     max_depth=1.5, 
                     cluster_tolerance=0.005, 
                     min_cluster_size=50, 
                     max_cluster_size=300,
                     segment_box=True,
                     box_min = [-0.5, 0.1, 0.6],
                     box_max = [0.5, 0.8, 1.5]):
    print "waiting for service..."
    rospy.wait_for_service(service_name);
    print "Calling service..."
    try:
        box_min_pt = geometry_msgs.msg.Point();
        box_min_pt.x = box_min[0];
        box_min_pt.y = box_min[1];
        box_min_pt.z = box_min[2];
        box_max_pt = geometry_msgs.msg.Point();
        box_max_pt.x = box_max[0];
        box_max_pt.y = box_max[1];
        box_max_pt.z = box_max[2];
        
        service = rospy.ServiceProxy(service_name, FindBlocks);
        response = service(cloud_topic, segment_planes, num_planes, plane_distance, segment_depth, min_depth, max_depth, cluster_tolerance, min_cluster_size, max_cluster_size, segment_box, box_min_pt, box_max_pt);
        return response.blocks;
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


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



if __name__ == "__main__":
    print find_blocks();
