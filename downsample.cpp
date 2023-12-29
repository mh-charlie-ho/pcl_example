#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

int main()
{

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredXYZ(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;

    // Replace the path below with the path where you saved your file
    reader.read("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
    
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filteredXYZ);
    pcl::fromPCLPointCloud2(*cloud, *cloudXYZ);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered,
                 Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloudXYZ, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    pcl::visualization::PCLVisualizer::Ptr viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer for filtered"));
    viewer2->setBackgroundColor (0, 0, 0);
    viewer2->addPointCloud<pcl::PointXYZ> (cloud_filteredXYZ, "sample cloud filered");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer2->addCoordinateSystem (1.0);
    viewer2->initCameraParameters ();
    // pcl::visualization::CloudViewer viewer("watch");
    // viewer->getRenderWindow()->GlobalWarningDisplayOff(); // Add This Line
    // viewer.showCloud(cloudXYZ);
     
    viewer->spin(); viewer2->spin();  // This is a key line.

    return (0);
}