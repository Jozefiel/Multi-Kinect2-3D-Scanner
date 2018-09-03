#ifndef PCL_H
#define PCL_H

#include <memory>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

class pclCloud
{
public:
    pclCloud(int cam_id);
    ~pclCloud();

    void pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int);
    void pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    int id=0;

};

/***********************************************************************************************************************************************************/
class pclViewer
{
public:
    pclViewer(const int connectedDevices);
    ~pclViewer();

    void pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int);
    void spinOnce();
    void pclCopyCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_cloud);


private:
    pcl::visualization::PCLVisualizer *viewer =nullptr;
    std::vector<int> viewPortsId;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    int id=0;

};

#endif // PCL_H
