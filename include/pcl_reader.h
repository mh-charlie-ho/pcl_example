#include <pcl/io/pcd_io.h>
#include <string>

class PointCloudReader
{
public:
    PointCloudReader(
        std::string filename,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    
    void Read();

private:
    pcl::PCDReader mReader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
};