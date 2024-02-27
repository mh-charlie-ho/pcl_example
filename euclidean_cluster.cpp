#include "pcl_reader.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, setfill


int main()
{    
    // Read in the cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudReader reader("table_scene_lms400.pcd", cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; // 顯示是否讀取成功

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg; // filter obj
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; // filtering

    // Create the segmentation object for the planar model and set all the parameters
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02); // the distance from the plane

    int nr_points = (int)cloud_filtered->size();
    while (cloud_filtered->size() > 0.3 * nr_points) // stop until the number of point cloud is under the 0.3 times.
    {

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients); // do filtering

        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false); // extract the point in terms of the index.

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);

        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered); // use these points to build a tree based on the kdtree
    std::vector<pcl::PointIndices> cluster_indices; // cluster_indices[0] represent the point index of first cluster

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    pcl::PCDWriter writer;

    int j = 0;
    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud_filtered)[idx]); // store the specified point
        }

        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        
        // store
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;

        writer.write<pcl::PointXYZ>("cloud_cluster_" + ss.str() + ".pcd", *cloud_cluster, false); //*
        j++;
    }

    return (0);
}