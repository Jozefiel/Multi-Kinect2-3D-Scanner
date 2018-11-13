#ifndef SUPPORT_H
#define SUPPORT_H

#include "camera.h"
#include "kinect.h"
#include "pcl.h"

#define mutexTimeDelay 10


class support
{
public:
    support();

    void cameraInit();
    void kinectInit();
    void realsenseInit();
    int connectedCameras();
    std::vector<Camera*> cameras();

    void threadsInit();
    void threadCameraSnapping();
    void threadComputePointCloud();

    void cloudInit();
    void camera2cloudDataTransfer();
    void transformCloud();
    void transformCloud(std::vector<Eigen::Matrix4d> transform_matrix);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudData(int id);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTransformedCloudData(int id);
    std::vector<pclCloud> getClouds();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mergeClouds(bool transformed);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mergeClouds(bool transformed, std::vector<Eigen::Matrix4d> transform_matrix);

private:

    std::vector<Camera*>        connected_cams;                                            // vector of Camera objects
    std::vector<pclCloud>       clouds;

    std::vector<std::thread>    cam_threads;                                           // vector of threads for image snapping
    std::vector<std::thread>    cloud_threads;
    std::atomic<bool> snap_running {true};
    std::atomic<bool> compute_cloud_style {true};
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> merged_clouds;

};

#endif // SUPPORT_H
