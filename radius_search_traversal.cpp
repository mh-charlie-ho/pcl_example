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

static void GPUOctreeRadiusSearch(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

    int n = 25008;
    std::vector<float> radius(n, 0.5f);
    // radius.push_back(0.1f);

    // std::cout << "HI" << std::endl;
    pcl::gpu::Octree::Radiuses radiuses_device;
    radiuses_device.upload(radius);

    util::Timer timeroctree("radiussearch_gpu_octree");

    // std::cout << "HI" << std::endl;
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(cloud->points);

    // std::cout << "HI" << std::endl;
    pcl::gpu::Octree octree_device;
    // std::cout << "HI" << std::endl;
    octree_device.setCloud(cloud_device);
    // std::cout << "HI" << std::endl;
    octree_device.build();

    std::vector<pcl::PointXYZ> query_host;
    for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i * n + j > 25008)
                continue;

            query_host.resize(n);
            query_host[j].x = cloud->points[i * n + j].x;
            query_host[j].y = cloud->points[i * n + j].y;
            query_host[j].z = cloud->points[i * n + j].z;

            if (i * n + j == 25007)
            {
                std::cout << i * n + j << std::endl;
            }
        }
        pcl::gpu::Octree::Queries queries_device;
        queries_device.upload(query_host);

        const int max_answers = 20000;
        pcl::gpu::NeighborIndices result_device(queries_device.size(), max_answers);

        octree_device.radiusSearch(queries_device, radiuses_device, max_answers, result_device);

        // ==== Save data ====
        // std::vector<int> sizes, data;
        // result_device.sizes.download(sizes);
        // result_device.data.download(data);

        // std::cout << "INFO: Data generated" << std::endl;
        // std::cout << "INFO: found : " << data.size() << " data.size" << std::endl;
        // std::cout << "INFO: found : " << sizes.size() << " sizes.size" << std::endl;

        // std::vector<int> record_amount;
        // for (std::size_t i = 0; i < sizes.size(); ++i)
        // {
        //     record_amount.push_back(sizes[i]);
        // }

        // std::ofstream outfile("gpuoctree.txt");
        // if (outfile.is_open())
        // {
        //     for (const auto &r : record_amount)
        //     {
        //         outfile << r << ", "; // 将向量中的每个元素写入文件中，每个元素占据一行
        //     }
        //     outfile.close(); // 关闭文件
        //     std::cout << "Data has been written to data.txt" << std::endl;
        // }
        // else
        // {
        //     std::cerr << "Unable to open file" << std::endl;
        // }
    }

    timeroctree.stop();
}

static void OctreeRadiusSearch(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // std::vector<int> record_amount;

    util::Timer timeroctree("radiussearch_octree");

    float resolution = 0.01f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    for (int i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZRGB searchPoint;
        searchPoint.x = cloud->points[i].x;
        searchPoint.y = cloud->points[i].y;
        searchPoint.z = cloud->points[i].z;

        float radius = 0.5;

        octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        // record_amount.push_back(pointIdxRadiusSearch.size());
    }
    timeroctree.stop();

    // ==== Save data ====
    // std::ofstream outfile("cpuoctree.txt");
    // if (outfile.is_open())
    // {
    //     for (const auto &r : record_amount)
    //     {
    //         outfile << r << ", "; // 将向量中的每个元素写入文件中，每个元素占据一行
    //     }
    //     outfile.close(); // 关闭文件
    //     std::cout << "Data has been written to data.txt" << std::endl;
    // }
    // else
    // {
    //     std::cerr << "Unable to open file" << std::endl;
    // }
}

static void KdtreeRadiusSearch(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // std::vector<int> record_amount;

    util::Timer timerkdtree("radiussearch_kdtree");

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);

    for (int i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZRGB searchPoint;
        searchPoint.x = cloud->points[i].x;
        searchPoint.y = cloud->points[i].y;
        searchPoint.z = cloud->points[i].z;

        float radius = 0.5;

        kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        // record_amount.push_back(pointIdxRadiusSearch.size());
    }
    timerkdtree.stop();

    // ==== Save data ====
    // std::ofstream outfile("cpukdtree.txt");
    // if (outfile.is_open())
    // {
    //     for (const auto &r : record_amount)
    //     {
    //         outfile << r << ", "; // 将向量中的每个元素写入文件中，每个元素占据一行
    //     }
    //     outfile.close(); // 关闭文件
    //     std::cout << "Data has been written to data.txt" << std::endl;
    // }
    // else
    // {
    //     std::cerr << "Unable to open file" << std::endl;
    // }
}

int main()
{
    // Read in the cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudReader reader("check_redpoint.pcd", cloud);

    std::cout << "kdtree radius search" << std::endl;
    std::cout << "total points: " << cloud->size() << " data points." << std::endl; // 顯示是否讀取成功

    OctreeRadiusSearch(cloud);
    KdtreeRadiusSearch(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNOCOLOR(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudReader readerNOCOLOR("check_redpoint.pcd", cloudNOCOLOR);
    GPUOctreeRadiusSearch(cloudNOCOLOR);

    return (0);
}