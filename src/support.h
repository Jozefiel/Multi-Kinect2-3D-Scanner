#ifndef SUPPORT_H
#define SUPPORT_H

#include "camera.h"
#include "kinect.h"
#include "pcl.h"

#include <queue>

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

    void camera2framesDataTransfer();


    void cloudInit();
    void camera2cloudDataTransfer();
    void transformCloud();
    void transformCloud(std::vector<Eigen::Matrix4d> transform_matrix);
    pcl::PointCloud<pcl::PointXYZRGB> getCloudData(int id);
    pcl::PointCloud<pcl::PointXYZRGB> getTransformedCloudData(int id);
    std::vector<pclCloud> getClouds();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> mergeClouds(bool transformed);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> mergeClouds(bool transformed, std::vector<Eigen::Matrix4d> transform_matrix);

    std::vector<Camera *> getConnectedCams();

    void viewerUpdater(std::atomic<bool> & snap_running);
    void pclUpdater(std::atomic<bool> &snap_running);
    void frameUpdater(std::atomic<bool> &snap_running);
    std::string IntToStr(int n);
    void changeComputeStyle(int);
    void saveLUT(cv::Mat depth, cv::Mat rgbd, std::string filename, int counter);
    std::atomic<bool> snap_running {true};
    pcl::PointCloud<pcl::PointXYZRGB> cloudik;
    pclCloud * merged_cloud;
    void saveSequence();

private:

    std::vector<Camera*>        connected_cams;                                            // vector of Camera objects
    std::vector<pclCloud>       clouds;
    std::vector<std::thread>    cam_threads;                                           // vector of threads for image snapping
    std::vector<std::thread>    cloud_threads;
    std::vector<std::thread>    viewer_threads;

    std::vector<std::queue<Camera::camera_frames>> * cam_frames=nullptr;

    std::atomic<bool> compute_cloud_style {false};
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> merged_clouds;

signals:
    void newRGBD(QPixmap pix,int id);
    void newDepth(QPixmap pix,int id);
    void newIR(QPixmap pix,int id);
    void newRangedRGBD(QPixmap pix,int id);
    void newRangedDepth(QPixmap pix,int id);
    void newHist(QPixmap pix,int id);
    void newCloud();

};



#endif // SUPPORT_H

