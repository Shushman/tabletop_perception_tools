#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <tabletop_perception_tools/PCLHelpers.h>
#include <tabletop_perception_tools/ExtractClusters.h>
#include <tabletop_perception_tools/FindBlocks.h>
#include <tabletop_perception_tools/DetectPlane.h>
#include <visualization_msgs/MarkerArray.h>

using namespace tabletop_perception_tools;

ros::Publisher vis_publisher;
ros::Publisher cloud_pub;
ros::NodeHandle* nodeHandle = 0x0;
typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
PointCloud::ConstPtr lastPointCloud;
bool pointCloudDirty = true;

void PointCloudCallback(PointCloud::ConstPtr pointCloud)
{
    lastPointCloud = pointCloud;
    pointCloudDirty = false;
}



bool ExtractClustersService(ExtractClusters::Request& request, ExtractClusters::Response& response)
{
    std::string topic = request.point_cloud_topic;
    pointCloudDirty = true;
    ros::Subscriber sub = nodeHandle->subscribe(topic, 1, &PointCloudCallback);
    ros::Duration timeout(5);
    ros::Time begin = ros::Time::now();
    ROS_INFO("Waiting for point cloud %s", topic.c_str());
    while (pointCloudDirty)
    {
        usleep(10000);
        ROS_INFO("Waiting...");
        if (ros::Time::now() - begin > timeout)
        {
            ROS_WARN("Timeout exceeded waiting for point cloud %s", topic.c_str());
            return false;
        }
    }

    ROS_INFO("Got point cloud. Extracting");
    PointCloud::Ptr pointCloud(new PointCloud(*lastPointCloud));

    if (request.segment_depth)
    {
        ROS_INFO("Segmenting depth");
        pointCloud = pcl_helpers::FilterBoundingBox<Point>(pointCloud, pcl_helpers::Vec3(-1000, -1000, request.min_depth), pcl_helpers::Vec3(1000, 1000, request.max_depth));
    }

    if (request.segment_planes)
    {
        ROS_INFO("Removing a plane");
        pointCloud = pcl_helpers::RemoveLargePlanes<Point>(pointCloud, request.num_planes, request.plane_distance);

    }

    ROS_INFO("Removing the clusters");
    std::vector<pcl_helpers::Cluster<Point> > clusters;
    pcl_helpers::ExtractClusters<Point>(pointCloud, &clusters, request.cluster_tolerance, request.min_cluster_size, request.max_cluster_size);

    ROS_INFO("Extracted %lu clusters", clusters.size());
    visualization_msgs::MarkerArray markers;

    for (size_t i = 0; i < clusters.size(); i++)
    {
        pcl_helpers::Cluster<Point>& cluster = clusters.at(i);

        tabletop_perception_tools::Cluster cluster_msg;
        cluster_msg.header.stamp = ros::Time::now(); // TODO: get the right timestamp
        cluster_msg.header.frame_id = pointCloud->header.frame_id;
        cluster_msg.max.x = cluster.bounds.max.x();
        cluster_msg.max.y = cluster.bounds.max.y();
        cluster_msg.max.z = cluster.bounds.max.z();

        cluster_msg.min.x = cluster.bounds.min.x();
        cluster_msg.min.y = cluster.bounds.min.y();
        cluster_msg.min.z = cluster.bounds.min.z();

        cluster_msg.pose.position.x = cluster.bounds.transform.translation().x();
        cluster_msg.pose.position.y = cluster.bounds.transform.translation().y();
        cluster_msg.pose.position.z = cluster.bounds.transform.translation().z();

        pcl_helpers::Quaternion quat(cluster.bounds.transform.linear());

        cluster_msg.pose.orientation.x = quat.x();
        cluster_msg.pose.orientation.y = quat.y();
        cluster_msg.pose.orientation.z = quat.z();
        cluster_msg.pose.orientation.w = quat.w();

        visualization_msgs::Marker marker;
        marker.color.r = 1;
        marker.color.g = 0.8;
        marker.color.b = 0;
        marker.color.a = 0.5;
        marker.header.frame_id = cluster_msg.header.frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose = cluster_msg.pose;
        marker.scale.x = (cluster_msg.max.x - cluster_msg.min.x);
        marker.scale.y = (cluster_msg.max.y - cluster_msg.min.y);
        marker.scale.z = (cluster_msg.max.z - cluster_msg.min.z);
        marker.header.stamp = ros::Time::now();
        markers.markers.push_back(marker);

        pcl::toROSMsg(*cluster.points, cluster_msg.points);
        response.clusters.push_back(cluster_msg);
    }

    vis_publisher.publish(markers);

    ROS_INFO("Shutting down...");
    sub.shutdown();

    ROS_INFO("Done.");
    return true;
}

bool FindBlocksService(FindBlocks::Request& request, FindBlocks::Response& response)
{
    std::string topic = request.point_cloud_topic;
    pointCloudDirty = true;
    ros::Subscriber sub = nodeHandle->subscribe(topic, 1, &PointCloudCallback);
    ros::Duration timeout(20);
    ros::Time begin = ros::Time::now();
    ROS_INFO("Waiting for point cloud %s", topic.c_str());
    while (pointCloudDirty)
    {
        usleep(10000);
        ROS_INFO("Waiting...");
        if (ros::Time::now() - begin > timeout)
        {
            ROS_WARN("Timeout exceeded waiting for point cloud %s", topic.c_str());
            return false;
        }
    }

    ROS_INFO("Got point cloud. Extracting");
    PointCloud::Ptr pointCloud(new PointCloud(*lastPointCloud));

    if (request.segment_depth)
    {
        ROS_INFO("Segmenting depth");
        pointCloud = pcl_helpers::FilterBoundingBox<Point>(pointCloud, pcl_helpers::Vec3(-1000, -1000, request.min_depth), pcl_helpers::Vec3(1000, 1000, request.max_depth));
        ROS_INFO("Filtered cloud has %lu points\n", pointCloud->size());
    }

    if (request.segment_box)
    {
        ROS_INFO("Segmenting box");
        pointCloud = pcl_helpers::FilterBoundingBox<Point>(pointCloud, pcl_helpers::Vec3(request.box_min.x, request.box_min.y, request.box_min.z), pcl_helpers::Vec3(request.box_max.x, request.box_max.y, request.box_max.z));
        ROS_INFO("Filtered cloud has %lu points\n", pointCloud->size());
    }

    if (request.segment_planes)
    {
        ROS_INFO("Removing planes");
        pointCloud = pcl_helpers::RemoveLargePlanes<Point>(pointCloud, request.num_planes, request.plane_distance);
        ROS_INFO("Filtered cloud has %lu points\n", pointCloud->size());
    }

    ROS_INFO("Filtered cloud has %lu points\n", pointCloud->size());
    cloud_pub.publish(pointCloud);

    ROS_INFO("Removing the clusters");
    std::vector<pcl_helpers::Cluster<Point> > clusters;
    pcl_helpers::ExtractClusters<Point>(pointCloud, &clusters, request.cluster_tolerance, request.min_cluster_size, request.max_cluster_size);

    ROS_INFO("Extracted %lu clusters", clusters.size());
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        pcl_helpers::Cluster<Point>& cluster = clusters.at(i);

        tabletop_perception_tools::Block block_msg;
        block_msg.header.stamp = ros::Time::now(); // TODO: get the right timestamp
        block_msg.header.frame_id = pointCloud->header.frame_id;


        block_msg.pose.position.x = cluster.bounds.transform.translation().x();
        block_msg.pose.position.y = cluster.bounds.transform.translation().y();
        block_msg.pose.position.z = cluster.bounds.transform.translation().z();

        pcl_helpers::Quaternion quat(cluster.bounds.transform.linear());

        block_msg.pose.orientation.x = quat.x();
        block_msg.pose.orientation.y = quat.y();
        block_msg.pose.orientation.z = quat.z();
        block_msg.pose.orientation.w = quat.w();

        float avg_r = 0.0f;
        float avg_g = 0.0f;
        float avg_b = 0.0f;
        float avg_a = 1.0f;
        for (size_t j = 0; j < cluster.points->size(); j++)
        {
            const Point& p = cluster.points->at(j);
            uint32_t rgb = *((int*)(&p.rgb));
            uint8_t r = (rgb >> 16) & 0x0000ff;
            uint8_t g = (rgb >> 8)  & 0x0000ff;
            uint8_t b = (rgb)       & 0x0000ff;
            avg_r += r;
            avg_g += g;
            avg_b += b;
        }

        avg_r /= cluster.points->size();
        avg_g /= cluster.points->size();
        avg_b /= cluster.points->size();

        avg_r /= 255.0f;
        avg_g /= 255.0f;
        avg_b /= 255.0f;

        block_msg.avg_color.r = avg_r;
        block_msg.avg_color.g = avg_g;
        block_msg.avg_color.b = avg_b;
        block_msg.avg_color.a = avg_a;

        visualization_msgs::Marker marker;
        marker.color.r = avg_r;
        marker.color.g = avg_g;
        marker.color.b = avg_b;
        marker.color.a = avg_a;
        marker.header.frame_id = block_msg.header.frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose = block_msg.pose;
        marker.scale.x = 0.025f;
        marker.scale.y = 0.025f;
        marker.scale.z = 0.025f;
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        markers.markers.push_back(marker);

        response.blocks.push_back(block_msg);
    }

    vis_publisher.publish(markers);

    ROS_INFO("Shutting down...");
    sub.shutdown();

    ROS_INFO("Done.");
    return true;
}


bool DetectPlaneService(DetectPlane::Request& request, DetectPlane::Response& response)
{
    std::string topic = request.point_cloud_topic;
    pointCloudDirty = true;
    ros::Subscriber sub = nodeHandle->subscribe(topic, 1, &PointCloudCallback);
    ros::Duration timeout(15);
    ros::Time begin = ros::Time::now();
    ROS_INFO("Waiting for point cloud %s", topic.c_str());
    while (pointCloudDirty)
    {
        usleep(10000);
        ROS_INFO("Waiting...");
        if (ros::Time::now() - begin > timeout)
        {
            ROS_WARN("Timeout exceeded waiting for point cloud %s", topic.c_str());
            return false;
        }
    }

    ROS_INFO("Got point cloud. Extracting");
    PointCloud::Ptr pointCloud(new PointCloud(*lastPointCloud));
    std::vector<float> plane_params;
    pcl_helpers::Vec4 centroid;

    //Try to detect plane
    bool plane_detected = pcl_helpers::DetectPlane<Point>(pointCloud,plane_params,centroid);

    if(plane_detected)
    {

        response.a = plane_params[0];
        response.b = plane_params[1];
        response.c = plane_params[2];
        response.d = plane_params[3];
        
        //Set translation
        response.pose.position.x = centroid[0];
        response.pose.position.y = centroid[1];
        response.pose.position.z = centroid[2];

        //Set orientation
        Eigen::Vector3d normal(plane_params[0], plane_params[1], plane_params[2]);
        normal = normal / sqrt(normal.dot(normal));

        
        Eigen::Vector3d unit_std(0, 0, 1);
        Eigen::Quaterniond R_q;
        R_q.setFromTwoVectors(unit_std, normal);
        
        /*
        pcl_helpers::Vec3 cam_z(0,0,-1);
        pcl_helpers::Vec3 table_x(cam_z.cross(normal));
        pcl_helpers::Vec3 table_y(normal.cross(table_x));
        pcl_helpers::Mat4x4 table_pose;
        table_pose << table_x,table_y,normal,Eigen::ArrayXf::Zero(3,1),
                      0, 0, 0, 1;

        pcl_helpers::Quaternion R_q(table_pose);
        */

        response.pose.orientation.x = R_q.x();
        response.pose.orientation.y = R_q.y();
        response.pose.orientation.z = R_q.z();     
        response.pose.orientation.w = R_q.w();

        response.ok = true;

        ROS_INFO_STREAM("Found plane! Coefficients: ["<<response.a<<" "<<response.b<<" "<<response.c<<" "<<response.d<<"]");
    }
    else
    {
        response.ok = false;
        ROS_INFO("Did not find plane!");
    }

    

    ROS_INFO("Shutting down...");
    sub.shutdown();

    ROS_INFO("Done.");

    return true;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "tools_server");
    ros::NodeHandle node("~");
    nodeHandle = &node;
    ros::ServiceServer server = node.advertiseService(std::string("extract_clusters"), &ExtractClustersService);
    ros::ServiceServer block_server = node.advertiseService(std::string("find_blocks"), &FindBlocksService);
    ros::ServiceServer plane_server = node.advertiseService(std::string("detect_planes"), &DetectPlaneService);
    vis_publisher = node.advertise<visualization_msgs::MarkerArray>(std::string("cluster_markers"), 1);
    cloud_pub = node.advertise<PointCloud>(std::string("filtered_cloud"), 1);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
