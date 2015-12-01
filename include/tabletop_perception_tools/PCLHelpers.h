#ifndef PCLHELPERS_H_
#define PCLHELPERS_H_

#include <iostream>
#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Geometry.h"

namespace pcl_helpers
{
    template<typename PointT> struct Cluster
    {
            typename pcl::PointCloud<PointT>::Ptr points;
            OBB bounds;
    };

    template<typename PointT> typename pcl::PointCloud<PointT>::Ptr FilterBoundingBox(const typename pcl::PointCloud<PointT>::Ptr& points, const Vec3& min, const Vec3& max)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>(*points));

        //calculate bounding box
        float xmin = min.x();
        float xmax = max.x();
        float ymin = min.y();
        float ymax = max.y();
        float zmin = min.z();
        float zmax = max.z();

        // Create the filtering object x
        pcl::PassThrough<PointT> pass_x;
        pass_x.setInputCloud (points);
        pass_x.setFilterFieldName ("x");
        pass_x.setFilterLimits (xmin, xmax);
        pass_x.filter (*cloud_filtered);

        // Create the filtering object y
        pcl::PassThrough<PointT> pass_y;
        pass_y.setInputCloud (cloud_filtered);
        pass_y.setFilterFieldName ("y");
        pass_y.setFilterLimits (ymin, ymax);
        pass_y.filter (*cloud_filtered);

        // Create the filtering object z
        pcl::PassThrough<PointT> pass_z;
        pass_z.setInputCloud (cloud_filtered);
        pass_z.setFilterFieldName ("z");
        pass_z.setFilterLimits (zmin, zmax);
        pass_z.filter (*cloud_filtered);

        return cloud_filtered;
    }


    template<typename PointT> OBB ComputeBounds(const typename pcl::PointCloud<PointT>::Ptr& points)
    {
        // compute principal direction
        Vec4 centroid;
        pcl::compute3DCentroid(*points, centroid);
        Mat3x3 covariance;
        computeCovarianceMatrixNormalized(*points, centroid, covariance);

        Eigen::SelfAdjointEigenSolver<Mat3x3> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Mat3x3 eigDx = eigen_solver.eigenvectors();
        eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

        // move the points to the that reference frame
        Mat4x4 p2w(Mat4x4::Identity());
        p2w.block<3,3>(0,0) = eigDx.transpose();
        p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
        pcl::PointCloud<PointT> cPoints;
        pcl::transformPointCloud(*points, cPoints, p2w);

        PointT min_pt, max_pt;
        pcl::getMinMax3D(cPoints, min_pt, max_pt);
        const Vec3 mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

        // final transform
        const Quaternion qfinal(eigDx);
        const Vec3 tfinal = eigDx*mean_diag + centroid.head<3>();


        OBB toReturn;
        toReturn.transform.linear() = qfinal.toRotationMatrix();
        toReturn.transform.translation() = centroid.head<3>();
        toReturn.min = min_pt.getVector3fMap();
        toReturn.max = max_pt.getVector3fMap();

        return toReturn;

    }

    template <typename PointT> bool DetectPlane(const typename pcl::PointCloud<PointT>::Ptr& points, std::vector<float>& plane_params,
             Vec4& centroid, float distanceThreshold = 0.0005f, int methodType = pcl::SAC_RANSAC, int modelType = pcl::SACMODEL_PLANE)
    {

        //Return plane normal and plane pose
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(modelType);
        seg.setMethodType(methodType);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(distanceThreshold);

        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>(*points));

        std::vector<int> nanIndex;
        pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, nanIndex);
        size_t nr_points = cloud_filtered->points.size();


        while(true){
            //Get largest plane component params
            inliers->indices.clear();
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);

            

            if (inliers->indices.size() == 0)
            {
                //No inliers -> plane not detected
                return false;
            }
            else
            {
                typename pcl::PointCloud<PointT>::Ptr inlier_pts(new pcl::PointCloud<PointT>(*cloud_filtered,inliers->indices));
                pcl::compute3DCentroid(*inlier_pts,centroid);
                ROS_INFO_STREAM(inliers->indices.size()<<" inliers, "<<" centroid - "<<centroid[0]<<","<<centroid[1]<<","<<centroid[2]);
                
                /*
                for (size_t i = 0; i < inliers->indices.size (); ++i){
                    std::cerr << inliers->indices[i] << "    " << cloud_filtered->points[inliers->indices[i]].x << " "
                                               << cloud_filtered->points[inliers->indices[i]].y << " "
                                               << cloud_filtered->points[inliers->indices[i]].z << std::endl;
                }*/

                if(centroid[2] > 2.0){
                    pcl::ExtractIndices<PointT> extract;
                    extract.setInputCloud(cloud_filtered);
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    extract.filter(*cloud_filtered);
                }
                else{
                    
                    plane_params = coefficients->values;
                    return true;
                }
                
            }
        }

        return false;


    }   


    template <typename PointT> typename pcl::PointCloud<PointT>::Ptr RemoveLargePlanes(const typename pcl::PointCloud<PointT>::Ptr& points,
            int numPlanes = 1, float distanceThreshold = 0.015f, int methodType = pcl::SAC_RANSAC, int modelType = pcl::SACMODEL_PLANE)
    {

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(modelType);
        seg.setMethodType(methodType);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(distanceThreshold);


        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>(*points));
        std::vector<int> nanIndex;
        pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, nanIndex);
        size_t nr_points = cloud_filtered->points.size();
        for (int k = 0; k < numPlanes; k++)
        {
            inliers->indices.clear();
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                return cloud_filtered;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_filtered);
        }

        pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, nanIndex);

        return cloud_filtered;

    }

    float randf()
    {
        return (float)(rand()) / (float)(RAND_MAX);
    }

    template <typename PointT> void ExtractClusters(const typename pcl::PointCloud<PointT>::Ptr& points, std::vector<Cluster<PointT> >* clusters, float tolerance = 0.2f, int minClusterSize = 100, int maxClusterSize = 25000)
    {
        // Creating the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(points);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(minClusterSize);
        ec.setMaxClusterSize(maxClusterSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(points);
        ec.extract(cluster_indices);

        /*
        pcl::visualization::PCLVisualizer* viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        viewer->addPointCloud(points, "FullPoints");
        */
        //int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            {
                cloud_cluster->points.push_back(points->points[*pit]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            Cluster<PointT> cluster;
            cluster.points = cloud_cluster;
            cluster.bounds = ComputeBounds<PointT>(cloud_cluster);
            clusters->push_back(cluster);
            /*
            std::stringstream ss;
            ss << j;

            pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (points, randf() * 255, randf() * 255, randf() * 255);
            viewer->addPointCloud<PointT> (cloud_cluster, single_color, "cluster " + ss.str());
            j++;
            */
        }

        /*
        while (!viewer->wasStopped ())
        {
          viewer->spinOnce (100);
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
        delete viewer;
        */

    }
}


#endif // PCLHELPERS_H_ 
