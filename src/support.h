#ifndef SUPPORT_H
#define SUPPORT_H

#include "camera.h"
#include "kinect.h"
#include "pcl.h"

#include <QObject>
#include <QImage>
#include <QPixmap>


#define mutexTimeDelay 10
#define imshow_32to8 2048

class support : public QObject
{
    Q_OBJECT
public:
    support(QObject *parent = nullptr);

    void cameraInit();
    void kinectInit();
    void realsenseInit();
    int connectedCameras();
    std::vector<Camera*> cameras();

    void threadsInit();
    void threadCameraSnapping();
    void threadComputePointCloud();
    void threadFrameUpdater();
    void closeThreads();

    void cloudInit();
    void camera2cloudDataTransfer();
    void transformCloud();
    void transformCloud(std::vector<Eigen::Matrix4d> transform_matrix);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudData(int id);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTransformedCloudData(int id);
    std::vector<pclCloud> getClouds();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mergeClouds(bool transformed);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mergeClouds(bool transformed, std::vector<Eigen::Matrix4d> transform_matrix);

    void viewerUpdater(std::atomic<bool> & snap_running);
    void pclUpdater(std::atomic<bool> &snap_running);

    std::string  IntToStr(int n);

    std::atomic<bool> snap_running {true};
    pcl::PointCloud<pcl::PointXYZRGB> cloudik;

private:

    std::vector<Camera*>        connected_cams;                                            // vector of Camera objects
    std::vector<pclCloud>       clouds;

    std::vector<std::thread>    cam_threads;                                           // vector of threads for image snapping
    std::vector<std::thread>    cloud_threads;
    std::vector<std::thread>    viewer_threads;


    std::atomic<bool> compute_cloud_style {false};
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> merged_clouds;
    pclCloud * merged_cloud;

signals:
    void newRGBD(QPixmap pix,int id);
    void newDepth(QPixmap pix,int id);
    void newIR(QPixmap pix,int id);
    void newCloud();

};



#endif // SUPPORT_H

