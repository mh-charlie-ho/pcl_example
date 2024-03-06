#include <pcl/io/pcd_io.h>
#include <string>

class PointCloudReader
{
public:
    PointCloudReader(
        std::string filename,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    PointCloudReader(
        std::string filename,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB);
    
    void Read();
    void ReadRGB();

private:
    pcl::PCDReader mReader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mCloudRGB;
};