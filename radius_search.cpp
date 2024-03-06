#include "pcl_read_file.h"
#include "util.h"

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/gpu/octree/octree.hpp>

#include <iomanip> // for setw, setfill

int main()
{
    // Read in the cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudReader reader("table_scene_lms400.pcd", cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; // 顯示是否讀取成功

    for (int i; i < cloud->points.size(); i++)
    {
        cloud->points[i].r = 0;
        cloud->points[i].g = 0;
        cloud->points[i].b = 255;
    }

    // 選擇點
    pcl::PointXYZRGB searchPoint;
    searchPoint.x = cloud->points[150000].x;
    searchPoint.y = cloud->points[150000].y;
    searchPoint.z = cloud->points[150000].z;


    for (float i = 0; i < 2.4; i = i + 0.1)
    {
        std::cout << "octree" << std::endl;
        float radius = i;

        // octree
        float resolution = 0.01f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        std::cout << "Neighbors within radius search at ("
                  << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z
                  << ") with radius:  " << radius << std::endl;

        util::Timer timeroctree("radiussearch_octree");
        auto tmpoc = octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        timeroctree.stop();

        std::cout << ":   " << pointIdxRadiusSearch.size() << std::endl;
        std::cout << std::endl;

        if (radius == 0.1f)
        {
            for (int j : pointIdxRadiusSearch)
            {
                cloud->points[j].r = 255;
                cloud->points[j].g = 0;
                cloud->points[j].b = 0;
            }

            pcl::PCLPointCloud2 pcl_pc2;
            pcl::toPCLPointCloud2(*cloud, pcl_pc2);

            pcl::PCDWriter writer;
            writer.write("table_scene_lms400_redpoint.pcd", pcl_pc2, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
        }
    }

    for (float i = 0; i < 2.4; i = i + 0.1)
    {
        std::cout << "kdtree" << std::endl;
        float radius = i;

        // kdtree
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(cloud);

        std::vector<int> kdtree_pointIdxRadiusSearch;
        std::vector<float> kdtree_pointRadiusSquaredDistance;

        std::cout << "Neighbors within radius search at ("
                  << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z
                  << ") with radius:  " << radius << std::endl;

        util::Timer timerkdtree("radiussearch_kdtree");
        auto tmpkd = kdtree.radiusSearch(searchPoint, radius, kdtree_pointIdxRadiusSearch, kdtree_pointRadiusSquaredDistance);
        timerkdtree.stop();

        std::cout << ":   " << kdtree_pointIdxRadiusSearch.size() << std::endl;
        std::cout << std::endl;
    }
    return (0);
}