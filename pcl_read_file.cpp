#include "pcl_read_file.h"

PointCloudReader::PointCloudReader(
    std::string filename,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    : mCloud(cloud)
{
    Read();
}

PointCloudReader::PointCloudReader(
    std::string filename,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB)
    : mCloudRGB(cloudRGB)
{
    ReadRGB();
}

void PointCloudReader::Read()
{
    mReader.read("table_scene_lms400.pcd", *mCloud);
}

void PointCloudReader::ReadRGB()
{
    mReader.read("table_scene_lms400.pcd", *mCloudRGB);
}