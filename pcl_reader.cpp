#include "pcl_reader.h"

PointCloudReader::PointCloudReader(
    std::string filename,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    : mCloud(cloud)
{
    Read();
}

void PointCloudReader::Read()
{
    mReader.read("table_scene_lms400.pcd", *mCloud);
}