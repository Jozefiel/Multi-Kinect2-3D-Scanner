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

class pclKinect
{
public:
    pclKinect(const int connectedDevices);
    ~pclKinect();

    void pclAddCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int);
    void spinOnce();


private:
    pcl::visualization::PCLVisualizer *viewer =nullptr;
    std::vector<int> viewPortsId;
};

#endif // PCL_H
