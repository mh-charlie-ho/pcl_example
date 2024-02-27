#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class Cluster
{
public:
    Cluster();

    void Downsample(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud,
        bool check = false)
    {
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg; // filter obj
        vg.setInputCloud(in_cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*out_cloud);

        if (check)
            std::cout << "PointCloud after filtering has: " << out_cloud->size() << " data points." << std::endl; // filtering
    }
};